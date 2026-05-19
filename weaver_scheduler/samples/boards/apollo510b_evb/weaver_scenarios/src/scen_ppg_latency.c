/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Scenario 3: PPG -> HR sensor-to-process latency tail.
 *
 * PPG producer pushes a sample every 10 ms (100 Hz, 4x nominal).
 * HR consumer wakes when samples are available, does 8 ms FFT work
 * per batch of 4 samples. A "noise" thread does 6 ms of CPU work
 * every 10 ms - just enough to delay HR if scheduled equally.
 *
 * For each PPG sample we record the timestamp delta between
 * production and the start of the HR processing pass that consumed
 * it. The P99 of these deltas is the metric.
 *
 * Stock Zephyr: HR + noise at the same priority round-robin. HR's
 *               long 8 ms work period creates head-of-line blocking;
 *               P99 latency tails into the tens of ms.
 *
 * Weaver:       Predictive buffer-fill helper (fill + (fill - last_fill))
 *               anticipates the queue filling and boosts HR one tick
 *               BEFORE the overflow. P99 stays bounded.
 *
 * PASS threshold (M2): Weaver P99 latency <= 0.7 * stock P99.
 */

#include "scen_common.h"
#include <zephyr/sys/printk.h>

#define DUR_MS         12000
#define FIFO_DEPTH     16
#define PPG_PERIOD_MS  10
#define HR_BATCH       4
#define HR_WORK_US     8000
#define NOISE_PERIOD_MS 10
#define NOISE_WORK_US  6000

#define PPG_PRIO    4
#define HR_PRIO     8
#define NOISE_PRIO  8

static atomic_t s3_depth;
static atomic_t s3_stop;
static uint32_t s3_ts_ring[FIFO_DEPTH];
static atomic_t s3_head, s3_tail;
static struct lat_hist s3_lat = { .bucket_us = LAT_BUCKETS_DEFAULT };

static K_THREAD_STACK_DEFINE(s3_ppg_stack,   SCEN_STACK_SIZE);
static K_THREAD_STACK_DEFINE(s3_hr_stack,    SCEN_STACK_SIZE);
static K_THREAD_STACK_DEFINE(s3_noise_stack, SCEN_STACK_SIZE);
static struct k_thread s3_ppg_t, s3_hr_t, s3_noise_t;

#ifdef CONFIG_WEAVER_SCHED
static struct weaver_thread_data s3_ppg_wd, s3_hr_wd, s3_noise_wd;
static void s3_tick(struct k_timer *t) { ARG_UNUSED(t); weaver_tick(); }
static K_TIMER_DEFINE(s3_timer, s3_tick, NULL);

static inline uint32_t depth_to_q16(int d) {
	if (d <= 0) return 0;
	if (d >= FIFO_DEPTH) return WEAVER_Q16_ONE;
	return ((uint32_t)d * WEAVER_Q16_ONE) / FIFO_DEPTH;
}
#endif

static void s3_ppg(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (!atomic_get(&s3_stop)) {
		int h = atomic_inc(&s3_head);
		s3_ts_ring[h % FIFO_DEPTH] = k_cyc_to_us_floor32(k_cycle_get_32());
		int d = atomic_inc(&s3_depth) + 1;
		if (d > FIFO_DEPTH) atomic_set(&s3_depth, FIFO_DEPTH);
#ifdef CONFIG_WEAVER_SCHED
		weaver_set_buffer_fill_predictive_q16(&s3_hr_wd, depth_to_q16(d));
#endif
		k_sleep(K_MSEC(PPG_PERIOD_MS));
	}
}

static void s3_hr(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (!atomic_get(&s3_stop)) {
		int d = atomic_get(&s3_depth);
		if (d >= HR_BATCH) {
			uint32_t now = k_cyc_to_us_floor32(k_cycle_get_32());
			for (int i = 0; i < HR_BATCH; i++) {
				int t = atomic_inc(&s3_tail);
				uint32_t prod_us = s3_ts_ring[t % FIFO_DEPTH];
				lat_add(&s3_lat, now - prod_us);
			}
			atomic_sub(&s3_depth, HR_BATCH);
			k_busy_wait(HR_WORK_US);
#ifdef CONFIG_WEAVER_SCHED
			weaver_set_buffer_fill_predictive_q16(
				&s3_hr_wd, depth_to_q16(atomic_get(&s3_depth)));
#endif
		}
		k_sleep(K_MSEC(2));
	}
}

static void s3_noise(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (!atomic_get(&s3_stop)) {
		k_busy_wait(NOISE_WORK_US);
		k_sleep(K_MSEC(NOISE_PERIOD_MS));
	}
}

void scen_ppg_latency(struct scen_result *out)
{
	atomic_set(&s3_depth, 0);
	atomic_set(&s3_head, 0);
	atomic_set(&s3_tail, 0);
	atomic_set(&s3_stop, 0);
	for (int i = 0; i < HIST_BUCKETS; i++) atomic_set(&s3_lat.buckets[i], 0);
	s3_lat.count = 0; s3_lat.max_us = 0;

	k_thread_create(&s3_ppg_t,   s3_ppg_stack,   SCEN_STACK_SIZE,
			s3_ppg,   NULL, NULL, NULL, PPG_PRIO,   0, K_NO_WAIT);
	k_thread_create(&s3_hr_t,    s3_hr_stack,    SCEN_STACK_SIZE,
			s3_hr,    NULL, NULL, NULL, HR_PRIO,    0, K_NO_WAIT);
	k_thread_create(&s3_noise_t, s3_noise_stack, SCEN_STACK_SIZE,
			s3_noise, NULL, NULL, NULL, NOISE_PRIO, 0, K_NO_WAIT);

#ifdef CONFIG_WEAVER_SCHED
	weaver_register(&s3_ppg_wd,   &s3_ppg_t,   WEAVER_TO_Q16(15), true);
	weaver_set_warp_deadline(&s3_ppg_wd, PPG_PERIOD_MS);
	weaver_register(&s3_hr_wd,    &s3_hr_t,    WEAVER_TO_Q16(8),  false);
	weaver_register(&s3_noise_wd, &s3_noise_t, WEAVER_TO_Q16(5),  false);
	k_timer_start(&s3_timer, K_MSEC(1), K_MSEC(1));
#endif

	k_sleep(K_MSEC(DUR_MS));
	atomic_set(&s3_stop, 1);
	k_sleep(K_MSEC(50));

#ifdef CONFIG_WEAVER_SCHED
	k_timer_stop(&s3_timer);
	weaver_unregister(&s3_ppg_wd);
	weaver_unregister(&s3_hr_wd);
	weaver_unregister(&s3_noise_wd);
#endif

	k_thread_abort(&s3_ppg_t);
	k_thread_abort(&s3_hr_t);
	k_thread_abort(&s3_noise_t);

	out->name = "ppg_latency";
	out->latency_p50_us = lat_percentile(&s3_lat, 50);
	out->latency_p99_us = lat_percentile(&s3_lat, 99);
	out->latency_max_us = s3_lat.max_us;
	out->misc_label = "samples";
	out->misc       = s3_lat.count;
	out->pass = true;
}
