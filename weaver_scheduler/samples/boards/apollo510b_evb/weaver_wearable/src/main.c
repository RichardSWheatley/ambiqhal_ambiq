/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Weaver wearable demo on Apollo510 Blue EVB at 96 MHz LP mode using
 * real sensors via mikroBUS Click boards:
 *
 *   IOM0 / I2C0  ->  6DOF IMU 14 Click   (BMI270)  @ 0x68
 *   IOM1 / I2C1  ->  Heart Rate 4 Click  (MAX30101) @ 0x57
 *   IOM6 / SPI6  ->  BLE controller (built into apollo510b_evb DTS)
 *   ap510_disp shield -> AMOLED + touch
 *
 * Warp threads:
 *   - IMU sample thread:  reads BMI270 at 50 Hz (period 20 ms)
 *   - PPG sample thread:  reads MAX30101 at 25 Hz (period 40 ms)
 *   - BLE LL connection event handler (Zephyr Bluetooth host owns this)
 *
 * Weft threads (compete via Weaver pressure):
 *   - sensor_fusion: consumes IMU samples, runs step counter
 *   - hr_algorithm:  consumes PPG samples, runs HR estimator
 *   - gatt_tx:       drains GATT notification ring
 *   - display:       updates AMOLED watch face on change
 *   - nvm_logger:    persists hourly summaries
 *
 * Each producer calls weaver_set_buffer_fill_predictive_q16() so the
 * scheduler reacts to where the FIFO will be next tick, not where it
 * is now. That is the "Predictive" half of the Weaver design and
 * costs ~6 cycles per call. No ML, no FPU.
 */

#include <zephyr/kernel.h>
#include <zephyr/kernel/weaver_sched.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(weaver_wearable, LOG_LEVEL_INF);

#define IMU_PERIOD_MS  20    /* 50 Hz */
#define PPG_PERIOD_MS  40    /* 25 Hz */
#define DEMO_DURATION_MS 5000

#define IMU_PRIO        3
#define PPG_PRIO        4
#define FUSION_PRIO     8
#define HR_PRIO         8
#define GATT_TX_PRIO    9
#define DISPLAY_PRIO    11
#define NVM_PRIO        12

/* IMU FIFO holds 6 axes * N samples; we model the high-water mark of a
 * 32-sample software ring as 1.0 in Q16.16. */
#define FIFO_SAMPLES 32

struct ring_sim {
	atomic_t depth;
};

static struct ring_sim imu_ring  = ATOMIC_INIT(0);
static struct ring_sim ppg_ring  = ATOMIC_INIT(0);
static struct ring_sim gatt_ring = ATOMIC_INIT(0);

static const struct device *const bmi270_dev   = DEVICE_DT_GET_ANY(bosch_bmi270_i2c);
static const struct device *const max30101_dev = DEVICE_DT_GET_ANY(maxim_max30101);

#define STACK_SIZE 2048

static K_THREAD_STACK_DEFINE(s_imu,    STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_ppg,    STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_fusion, STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_hr,     STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_gatt,   STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_disp,   STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_nvm,    STACK_SIZE);

static struct k_thread t_imu, t_ppg, t_fusion, t_hr, t_gatt, t_disp, t_nvm;

static struct weaver_thread_data wd_imu, wd_ppg, wd_fusion, wd_hr,
				 wd_gatt, wd_disp, wd_nvm;

static inline uint32_t depth_to_q16(int depth)
{
	if (depth <= 0) {
		return 0;
	}
	if (depth >= FIFO_SAMPLES) {
		return WEAVER_Q16_ONE;
	}
	return ((uint32_t)depth * WEAVER_Q16_ONE) / FIFO_SAMPLES;
}

/* ---------- Warp threads ---------- */

static void imu_sample_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	if (!device_is_ready(bmi270_dev)) {
		LOG_WRN("BMI270 not ready; IMU thread idling");
	}

	while (1) {
		if (device_is_ready(bmi270_dev)) {
			(void)sensor_sample_fetch(bmi270_dev);
			/* Real driver would push 6 axes into a ring buffer
			 * here. We model the FIFO as a counter. */
			int d = atomic_inc(&imu_ring.depth) + 1;
			if (d > FIFO_SAMPLES) {
				atomic_set(&imu_ring.depth, FIFO_SAMPLES);
			}
			weaver_set_buffer_fill_predictive_q16(
				&wd_fusion, depth_to_q16(d));
		}
		k_sleep(K_MSEC(IMU_PERIOD_MS));
	}
}

static void ppg_sample_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	if (!device_is_ready(max30101_dev)) {
		LOG_WRN("MAX30101 not ready; PPG thread idling");
	}

	while (1) {
		if (device_is_ready(max30101_dev)) {
			(void)sensor_sample_fetch(max30101_dev);
			int d = atomic_inc(&ppg_ring.depth) + 1;
			if (d > FIFO_SAMPLES) {
				atomic_set(&ppg_ring.depth, FIFO_SAMPLES);
			}
			weaver_set_buffer_fill_predictive_q16(
				&wd_hr, depth_to_q16(d));
		}
		k_sleep(K_MSEC(PPG_PERIOD_MS));
	}
}

/* ---------- Weft consumers ---------- */

static void fusion_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		int d = atomic_get(&imu_ring.depth);
		if (d > 0) {
			/* Drain a batch; real fusion would do orientation,
			 * step count, gesture detection. */
			int new_d = MAX(0, d - 8);
			atomic_set(&imu_ring.depth, new_d);
			weaver_set_buffer_fill_predictive_q16(
				&wd_fusion, depth_to_q16(new_d));
		}
		k_sleep(K_MSEC(5));
	}
}

static void hr_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		int d = atomic_get(&ppg_ring.depth);
		if (d > 0) {
			int new_d = MAX(0, d - 4);
			atomic_set(&ppg_ring.depth, new_d);
			weaver_set_buffer_fill_predictive_q16(
				&wd_hr, depth_to_q16(new_d));
		}
		k_sleep(K_MSEC(10));
	}
}

static void gatt_tx_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		/* Mock: producer side bumps the ring, consumer drains. */
		int d = atomic_inc(&gatt_ring.depth) + 1;
		if (d > FIFO_SAMPLES) {
			atomic_set(&gatt_ring.depth, FIFO_SAMPLES);
		}
		weaver_set_buffer_fill_predictive_q16(
			&wd_gatt, depth_to_q16(d));

		k_sleep(K_MSEC(15));

		int new_d = MAX(0, atomic_get(&gatt_ring.depth) - 3);
		atomic_set(&gatt_ring.depth, new_d);
		weaver_set_buffer_fill_predictive_q16(
			&wd_gatt, depth_to_q16(new_d));
	}
}

static void display_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		k_sleep(K_MSEC(33));   /* 30 fps watch face */
	}
}

static void nvm_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		k_sleep(K_MSEC(1000));
	}
}

/* ---------- Tick driver ---------- */

static void tick_handler(struct k_timer *t) { ARG_UNUSED(t); weaver_tick(); }
static K_TIMER_DEFINE(weaver_timer, tick_handler, NULL);

int main(void)
{
	printk("Weaver wearable demo on Apollo510B EVB (LP @ 96 MHz, mikroBUS Click)\n");
	printk("BMI270  ready: %s\n",
	       (bmi270_dev   && device_is_ready(bmi270_dev))   ? "yes" : "no");
	printk("MAX30101 ready: %s\n",
	       (max30101_dev && device_is_ready(max30101_dev)) ? "yes" : "no");

	k_thread_create(&t_imu,    s_imu,    STACK_SIZE, imu_sample_fn,
			NULL, NULL, NULL, IMU_PRIO,     0, K_NO_WAIT);
	k_thread_create(&t_ppg,    s_ppg,    STACK_SIZE, ppg_sample_fn,
			NULL, NULL, NULL, PPG_PRIO,     0, K_NO_WAIT);
	k_thread_create(&t_fusion, s_fusion, STACK_SIZE, fusion_fn,
			NULL, NULL, NULL, FUSION_PRIO,  0, K_NO_WAIT);
	k_thread_create(&t_hr,     s_hr,     STACK_SIZE, hr_fn,
			NULL, NULL, NULL, HR_PRIO,      0, K_NO_WAIT);
	k_thread_create(&t_gatt,   s_gatt,   STACK_SIZE, gatt_tx_fn,
			NULL, NULL, NULL, GATT_TX_PRIO, 0, K_NO_WAIT);
	k_thread_create(&t_disp,   s_disp,   STACK_SIZE, display_fn,
			NULL, NULL, NULL, DISPLAY_PRIO, 0, K_NO_WAIT);
	k_thread_create(&t_nvm,    s_nvm,    STACK_SIZE, nvm_fn,
			NULL, NULL, NULL, NVM_PRIO,     0, K_NO_WAIT);

	weaver_register(&wd_imu, &t_imu, WEAVER_TO_Q16(15), true);
	weaver_set_warp_deadline(&wd_imu, IMU_PERIOD_MS);

	weaver_register(&wd_ppg, &t_ppg, WEAVER_TO_Q16(15), true);
	weaver_set_warp_deadline(&wd_ppg, PPG_PERIOD_MS);

	weaver_register(&wd_fusion, &t_fusion, WEAVER_TO_Q16(8), false);
	weaver_register(&wd_hr,     &t_hr,     WEAVER_TO_Q16(8), false);
	weaver_register(&wd_gatt,   &t_gatt,   WEAVER_TO_Q16(7), false);
	weaver_register(&wd_disp,   &t_disp,   WEAVER_TO_Q16(4), false);
	weaver_register(&wd_nvm,    &t_nvm,    WEAVER_TO_Q16(2), false);

	k_timer_start(&weaver_timer, K_MSEC(1), K_MSEC(1));

	uint32_t t0 = k_uptime_get_32();
	while ((k_uptime_get_32() - t0) < DEMO_DURATION_MS) {
		k_sleep(K_MSEC(500));

		struct weaver_stats s;
		weaver_get_stats(&s);

		printk("[t=%4ums] sys_p=0x%06x throttle=%d level=%3u next_warp=%u "
		       "ticks=%u promo=%u clear=%u idle=%u\n",
		       k_uptime_get_32() - t0,
		       weaver_system_pressure(),
		       weaver_should_throttle() ? 1 : 0,
		       weaver_throttle_level(),
		       weaver_ticks_to_next_warp(),
		       s.total_ticks, s.weft_promotions,
		       s.pre_warp_clears, s.skipped_idle_ticks);
	}

	k_timer_stop(&weaver_timer);

	weaver_unregister(&wd_imu);
	weaver_unregister(&wd_ppg);
	weaver_unregister(&wd_fusion);
	weaver_unregister(&wd_hr);
	weaver_unregister(&wd_gatt);
	weaver_unregister(&wd_disp);
	weaver_unregister(&wd_nvm);

	printk("Weaver Wearable Demo Complete\n");
	return 0;
}
