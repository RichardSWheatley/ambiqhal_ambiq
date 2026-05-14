/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Same wearable workload as samples/.../weaver_wearable, but using
 * the floating-point Weaver variant. See weaver_sched_fp.h header for
 * when this variant is preferable to the fixed-point one.
 */

#include <zephyr/kernel.h>
#include <zephyr/kernel/weaver_sched_fp.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(weaver_wearable_fp, LOG_LEVEL_INF);

#define IMU_PERIOD_MS  20
#define PPG_PERIOD_MS  40
#define DEMO_DURATION_MS 5000

#define IMU_PRIO        3
#define PPG_PRIO        4
#define FUSION_PRIO     8
#define HR_PRIO         8
#define GATT_TX_PRIO    9
#define DISPLAY_PRIO    11
#define NVM_PRIO        12

#define FIFO_SAMPLES 32

static atomic_t imu_ring  = ATOMIC_INIT(0);
static atomic_t ppg_ring  = ATOMIC_INIT(0);
static atomic_t gatt_ring = ATOMIC_INIT(0);

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
static struct weaver_fp_thread_data wd_imu, wd_ppg, wd_fusion, wd_hr,
				    wd_gatt, wd_disp, wd_nvm;

static inline float depth_to_fill(int depth)
{
	if (depth <= 0)             return 0.0f;
	if (depth >= FIFO_SAMPLES)  return 1.0f;
	return (float)depth / (float)FIFO_SAMPLES;
}

static void imu_sample_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		if (device_is_ready(bmi270_dev)) {
			(void)sensor_sample_fetch(bmi270_dev);
			int d = atomic_inc(&imu_ring) + 1;
			if (d > FIFO_SAMPLES) atomic_set(&imu_ring, FIFO_SAMPLES);
			weaver_fp_set_buffer_fill_predictive(
				&wd_fusion, depth_to_fill(d));
		}
		k_sleep(K_MSEC(IMU_PERIOD_MS));
	}
}

static void ppg_sample_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		if (device_is_ready(max30101_dev)) {
			(void)sensor_sample_fetch(max30101_dev);
			int d = atomic_inc(&ppg_ring) + 1;
			if (d > FIFO_SAMPLES) atomic_set(&ppg_ring, FIFO_SAMPLES);
			weaver_fp_set_buffer_fill_predictive(
				&wd_hr, depth_to_fill(d));
		}
		k_sleep(K_MSEC(PPG_PERIOD_MS));
	}
}

static void fusion_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		int d = atomic_get(&imu_ring);
		if (d > 0) {
			int new_d = MAX(0, d - 8);
			atomic_set(&imu_ring, new_d);
			weaver_fp_set_buffer_fill_predictive(
				&wd_fusion, depth_to_fill(new_d));
		}
		k_sleep(K_MSEC(5));
	}
}

static void hr_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		int d = atomic_get(&ppg_ring);
		if (d > 0) {
			int new_d = MAX(0, d - 4);
			atomic_set(&ppg_ring, new_d);
			weaver_fp_set_buffer_fill_predictive(
				&wd_hr, depth_to_fill(new_d));
		}
		k_sleep(K_MSEC(10));
	}
}

static void gatt_tx_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		int d = atomic_inc(&gatt_ring) + 1;
		if (d > FIFO_SAMPLES) atomic_set(&gatt_ring, FIFO_SAMPLES);
		weaver_fp_set_buffer_fill_predictive(
			&wd_gatt, depth_to_fill(d));

		k_sleep(K_MSEC(15));

		int new_d = MAX(0, atomic_get(&gatt_ring) - 3);
		atomic_set(&gatt_ring, new_d);
		weaver_fp_set_buffer_fill_predictive(
			&wd_gatt, depth_to_fill(new_d));
	}
}

static void display_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) { k_sleep(K_MSEC(33)); }
}

static void nvm_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) { k_sleep(K_MSEC(1000)); }
}

static void tick_handler(struct k_timer *t) { ARG_UNUSED(t); weaver_fp_tick(); }
static K_TIMER_DEFINE(weaver_timer, tick_handler, NULL);

int main(void)
{
	printk("Weaver FP wearable demo on Apollo510B EVB (LP @ 96 MHz)\n");
	printk("BMI270   ready: %s\n",
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

	weaver_fp_register(&wd_imu, &t_imu, 15.0f, true);
	weaver_fp_set_warp_deadline(&wd_imu, IMU_PERIOD_MS);

	weaver_fp_register(&wd_ppg, &t_ppg, 15.0f, true);
	weaver_fp_set_warp_deadline(&wd_ppg, PPG_PERIOD_MS);

	weaver_fp_register(&wd_fusion, &t_fusion, 8.0f, false);
	weaver_fp_register(&wd_hr,     &t_hr,     8.0f, false);
	weaver_fp_register(&wd_gatt,   &t_gatt,   7.0f, false);
	weaver_fp_register(&wd_disp,   &t_disp,   4.0f, false);
	weaver_fp_register(&wd_nvm,    &t_nvm,    2.0f, false);

	k_timer_start(&weaver_timer, K_MSEC(1), K_MSEC(1));

	uint32_t t0 = k_uptime_get_32();
	while ((k_uptime_get_32() - t0) < DEMO_DURATION_MS) {
		k_sleep(K_MSEC(500));

		struct weaver_fp_stats s;
		weaver_fp_get_stats(&s);

		/* %f formatting requires printk float support; fall back to
		 * printing the pressure * 1000 as an integer for portability.
		 */
		uint32_t sysp_milli = (uint32_t)(weaver_fp_system_pressure() * 1000.0f);

		printk("[t=%4ums] sys_p_x1000=%u throttle=%d level=%3u next_warp=%u "
		       "ticks=%u promo=%u clear=%u idle=%u tick_cyc=%u\n",
		       k_uptime_get_32() - t0,
		       sysp_milli,
		       weaver_fp_should_throttle() ? 1 : 0,
		       weaver_fp_throttle_level(),
		       weaver_fp_ticks_to_next_warp(),
		       s.total_ticks, s.weft_promotions,
		       s.pre_warp_clears, s.skipped_idle_ticks,
		       weaver_fp_get_last_tick_cycles());
	}

	k_timer_stop(&weaver_timer);

	weaver_fp_unregister(&wd_imu);
	weaver_fp_unregister(&wd_ppg);
	weaver_fp_unregister(&wd_fusion);
	weaver_fp_unregister(&wd_hr);
	weaver_fp_unregister(&wd_gatt);
	weaver_fp_unregister(&wd_disp);
	weaver_fp_unregister(&wd_nvm);

	printk("Weaver FP Wearable Demo Complete\n");
	return 0;
}
