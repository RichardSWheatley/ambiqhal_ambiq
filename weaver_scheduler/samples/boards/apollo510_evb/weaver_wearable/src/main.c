/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Apollo510 LP @ 96 MHz wearable workload demo for Weaver scheduler.
 *
 * Models a typical fitness/health wearable:
 *
 *   Warp (deterministic, fixed deadlines):
 *     - BLE LL  : connection event every 50 ms (CONFIG_BLE_INTERVAL_MS)
 *     - IMU     : 6-axis sample @ 50 Hz   -> period 20 ms
 *     - PPG/HR  : optical sample  @ 25 Hz -> period 40 ms
 *
 *   Weft (opportunistic, pressure scales with FIFO depth):
 *     - Sensor fusion / step counter (consumes IMU FIFO)
 *     - HR algorithm                  (consumes PPG FIFO)
 *     - GATT TX queue                 (consumes notification ring)
 *     - Activity classifier           (periodic ML inference)
 *     - Display update                (frame buffer presents on change)
 *     - NVM/log flush                 (persists hourly summaries)
 *
 * The mocked sensors push synthetic FIFO-depth data into Weaver via
 * weaver_set_buffer_fill_q16(). The dispatcher boosts whichever Weft
 * is closest to overflow. Pre-Warp guard ticks suppress promotion when
 * a BLE/IMU/PPG deadline is imminent.
 */

#include <zephyr/kernel.h>
#include <zephyr/kernel/weaver_sched.h>
#include <zephyr/sys/printk.h>
#include <zephyr/random/random.h>

/* ---------- Wearable workload constants ---------- */

#define TICK_MS                 1
#define DEMO_DURATION_MS        2000

#define BLE_INTERVAL_MS         50
#define IMU_INTERVAL_MS         20
#define PPG_INTERVAL_MS         40
#define CLASSIFIER_INTERVAL_MS  500
#define DISPLAY_INTERVAL_MS     33   /* 30 fps */
#define NVM_INTERVAL_MS         1000

/* Cooperative range is negative; we run all of these in preemptible
 * space so Weaver can adjust priorities. Lower number = higher prio.
 */
#define BLE_LL_PRIO     2  /* Warp */
#define IMU_PRIO        3  /* Warp */
#define PPG_PRIO        4  /* Warp */
#define FUSION_PRIO     8  /* Weft */
#define HR_PRIO         8  /* Weft */
#define GATT_TX_PRIO    9  /* Weft */
#define CLASSIFIER_PRIO 10 /* Weft */
#define DISPLAY_PRIO    11 /* Weft */
#define NVM_PRIO        12 /* Weft */

/* ---------- Mocked sensor FIFOs ---------- */

struct fifo_sim {
	uint32_t depth;     /* Q16.16 fill: 0 = empty, ONE = full */
	uint32_t prod_step; /* Per-tick increment in Q16.16. */
	uint32_t cons_step; /* Per-consumer-run drain in Q16.16. */
};

static struct fifo_sim imu_fifo  = { 0, WEAVER_Q16_ONE / 200, WEAVER_Q16_ONE / 4 };
static struct fifo_sim ppg_fifo  = { 0, WEAVER_Q16_ONE / 400, WEAVER_Q16_ONE / 4 };
static struct fifo_sim gatt_tx   = { 0, WEAVER_Q16_ONE / 300, WEAVER_Q16_ONE / 3 };

/* ---------- Stacks + thread structs ---------- */

#define STACK_SIZE 1024

static K_THREAD_STACK_DEFINE(s_ble,        STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_imu,        STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_ppg,        STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_fusion,     STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_hr,         STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_gatt_tx,    STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_classifier, STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_display,    STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_nvm,        STACK_SIZE);

static struct k_thread t_ble, t_imu, t_ppg, t_fusion, t_hr, t_gatt_tx,
		       t_classifier, t_display, t_nvm;

static struct weaver_thread_data wd_ble, wd_imu, wd_ppg, wd_fusion, wd_hr,
				 wd_gatt_tx, wd_classifier, wd_display, wd_nvm;

/* ---------- Warp threads ---------- */

static void ble_ll_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		/* Connection event: minimal CPU, but the deadline is sacred. */
		(void)k_uptime_get();
		k_sleep(K_MSEC(BLE_INTERVAL_MS));
	}
}

static void imu_sample_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		/* "Read" 6 axes -> push to FIFO. */
		imu_fifo.depth = MIN(imu_fifo.depth + imu_fifo.prod_step, WEAVER_Q16_ONE);
		weaver_set_buffer_fill_q16(&wd_fusion, imu_fifo.depth);
		k_sleep(K_MSEC(IMU_INTERVAL_MS));
	}
}

static void ppg_sample_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		ppg_fifo.depth = MIN(ppg_fifo.depth + ppg_fifo.prod_step, WEAVER_Q16_ONE);
		weaver_set_buffer_fill_q16(&wd_hr, ppg_fifo.depth);
		k_sleep(K_MSEC(PPG_INTERVAL_MS));
	}
}

/* ---------- Weft threads ---------- */

static void fusion_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		if (imu_fifo.depth > 0) {
			imu_fifo.depth = (imu_fifo.depth > imu_fifo.cons_step)
				? imu_fifo.depth - imu_fifo.cons_step : 0;
			weaver_set_buffer_fill_q16(&wd_fusion, imu_fifo.depth);
		}
		k_sleep(K_MSEC(5));
	}
}

static void hr_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		if (ppg_fifo.depth > 0) {
			ppg_fifo.depth = (ppg_fifo.depth > ppg_fifo.cons_step)
				? ppg_fifo.depth - ppg_fifo.cons_step : 0;
			weaver_set_buffer_fill_q16(&wd_hr, ppg_fifo.depth);
		}
		k_sleep(K_MSEC(10));
	}
}

static void gatt_tx_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		gatt_tx.depth = MIN(gatt_tx.depth + gatt_tx.prod_step, WEAVER_Q16_ONE);
		weaver_set_buffer_fill_q16(&wd_gatt_tx, gatt_tx.depth);
		if (gatt_tx.depth > gatt_tx.cons_step) {
			gatt_tx.depth -= gatt_tx.cons_step;
		}
		k_sleep(K_MSEC(15));
	}
}

static void classifier_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		k_sleep(K_MSEC(CLASSIFIER_INTERVAL_MS));
	}
}

static void display_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		k_sleep(K_MSEC(DISPLAY_INTERVAL_MS));
	}
}

static void nvm_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		k_sleep(K_MSEC(NVM_INTERVAL_MS));
	}
}

/* ---------- Weaver tick driver ---------- */

static void tick_handler(struct k_timer *t) { ARG_UNUSED(t); weaver_tick(); }
static K_TIMER_DEFINE(weaver_timer, tick_handler, NULL);

int main(void)
{
	printk("Weaver wearable demo on Apollo510 LP @ 96 MHz\n");

	/* Create all threads. */
	k_thread_create(&t_ble,        s_ble,        STACK_SIZE, ble_ll_fn,     NULL, NULL, NULL, BLE_LL_PRIO,     0, K_NO_WAIT);
	k_thread_create(&t_imu,        s_imu,        STACK_SIZE, imu_sample_fn, NULL, NULL, NULL, IMU_PRIO,        0, K_NO_WAIT);
	k_thread_create(&t_ppg,        s_ppg,        STACK_SIZE, ppg_sample_fn, NULL, NULL, NULL, PPG_PRIO,        0, K_NO_WAIT);
	k_thread_create(&t_fusion,     s_fusion,     STACK_SIZE, fusion_fn,     NULL, NULL, NULL, FUSION_PRIO,     0, K_NO_WAIT);
	k_thread_create(&t_hr,         s_hr,         STACK_SIZE, hr_fn,         NULL, NULL, NULL, HR_PRIO,         0, K_NO_WAIT);
	k_thread_create(&t_gatt_tx,    s_gatt_tx,    STACK_SIZE, gatt_tx_fn,    NULL, NULL, NULL, GATT_TX_PRIO,    0, K_NO_WAIT);
	k_thread_create(&t_classifier, s_classifier, STACK_SIZE, classifier_fn, NULL, NULL, NULL, CLASSIFIER_PRIO, 0, K_NO_WAIT);
	k_thread_create(&t_display,    s_display,    STACK_SIZE, display_fn,    NULL, NULL, NULL, DISPLAY_PRIO,    0, K_NO_WAIT);
	k_thread_create(&t_nvm,        s_nvm,        STACK_SIZE, nvm_fn,        NULL, NULL, NULL, NVM_PRIO,        0, K_NO_WAIT);

	/* Register Warp threads with their deadlines. */
	weaver_register(&wd_ble, &t_ble, WEAVER_TO_Q16(20), true);
	weaver_set_warp_deadline(&wd_ble, BLE_INTERVAL_MS / TICK_MS);

	weaver_register(&wd_imu, &t_imu, WEAVER_TO_Q16(15), true);
	weaver_set_warp_deadline(&wd_imu, IMU_INTERVAL_MS / TICK_MS);

	weaver_register(&wd_ppg, &t_ppg, WEAVER_TO_Q16(15), true);
	weaver_set_warp_deadline(&wd_ppg, PPG_INTERVAL_MS / TICK_MS);

	/* Register Weft threads. */
	weaver_register(&wd_fusion,     &t_fusion,     WEAVER_TO_Q16(8), false);
	weaver_register(&wd_hr,         &t_hr,         WEAVER_TO_Q16(8), false);
	weaver_register(&wd_gatt_tx,    &t_gatt_tx,    WEAVER_TO_Q16(7), false);
	weaver_register(&wd_classifier, &t_classifier, WEAVER_TO_Q16(5), false);
	weaver_register(&wd_display,    &t_display,    WEAVER_TO_Q16(4), false);
	weaver_register(&wd_nvm,        &t_nvm,        WEAVER_TO_Q16(2), false);

	k_timer_start(&weaver_timer, K_MSEC(TICK_MS), K_MSEC(TICK_MS));

	/* Run for DEMO_DURATION_MS, log fabric stats every 250 ms. */
	uint32_t t0 = k_uptime_get_32();
	while ((k_uptime_get_32() - t0) < DEMO_DURATION_MS) {
		k_sleep(K_MSEC(250));

		struct weaver_stats s;
		weaver_get_stats(&s);

		printk("[t=%4ums] sys_p=0x%06x throttle=%d next_warp=%u "
		       "ticks=%u promo=%u clear=%u idle=%u\n",
		       k_uptime_get_32() - t0,
		       weaver_system_pressure(),
		       weaver_should_throttle() ? 1 : 0,
		       weaver_ticks_to_next_warp(),
		       s.total_ticks, s.weft_promotions,
		       s.pre_warp_clears, s.skipped_idle_ticks);
	}

	k_timer_stop(&weaver_timer);

	weaver_unregister(&wd_ble);
	weaver_unregister(&wd_imu);
	weaver_unregister(&wd_ppg);
	weaver_unregister(&wd_fusion);
	weaver_unregister(&wd_hr);
	weaver_unregister(&wd_gatt_tx);
	weaver_unregister(&wd_classifier);
	weaver_unregister(&wd_display);
	weaver_unregister(&wd_nvm);

	printk("Weaver Wearable Demo Complete\n");
	return 0;
}
