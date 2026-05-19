/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Scenario 1: IMU FIFO overrun under sustained 200 Hz production.
 *
 * IMU producer runs at 200 Hz (1 sample per 5 ms). Fusion consumer
 * normally drains 8 samples per pass but the work takes 1.5 ms so
 * the ring fills if fusion doesn't get enough CPU. A "noise" Weft
 * thread does aggressive 3 ms of work every 4 ms - just barely
 * dominant enough at equal priority to starve fusion under stock
 * Zephyr.
 *
 * Stock Zephyr:  fusion + noise are both at preemptible priority 8.
 *                Round-robin gives them equal time. Fusion can't
 *                keep up with 200 Hz, FIFO overruns.
 *
 * Weaver:        fusion's buffer_fill_q16 rises as the ring fills,
 *                which boosts fusion above noise via Weft promotion.
 *                FIFO stays bounded.
 *
 * PASS threshold (M1): Weaver overruns <= 0.5 * stock overruns.
 */

#include "scen_common.h"
#include <zephyr/sys/printk.h>

#define DUR_MS          20000   /* 20 s run */
#define FIFO_DEPTH      32
#define IMU_PERIOD_MS   5       /* 200 Hz */
#define FUSION_DRAIN    8       /* samples per fusion pass */
#define FUSION_WORK_US  1500
#define NOISE_PERIOD_MS 4
#define NOISE_WORK_US   3000

#define IMU_PRIO    3
#define FUSION_PRIO 8
#define NOISE_PRIO  8

static atomic_t s1_ring;
static atomic_t s1_overruns;
static atomic_t s1_stop;

static K_THREAD_STACK_DEFINE(s1_imu_stack,    SCEN_STACK_SIZE);
static K_THREAD_STACK_DEFINE(s1_fusion_stack, SCEN_STACK_SIZE);
static K_THREAD_STACK_DEFINE(s1_noise_stack,  SCEN_STACK_SIZE);
static struct k_thread s1_imu_t, s1_fusion_t, s1_noise_t;

#ifdef CONFIG_WEAVER_SCHED
static struct weaver_thread_data s1_imu_wd, s1_fusion_wd, s1_noise_wd;

static void s1_tick(struct k_timer *t) { ARG_UNUSED(t); weaver_tick(); }
static K_TIMER_DEFINE(s1_timer, s1_tick, NULL);

static inline uint32_t depth_to_q16(int d) {
	if (d <= 0) return 0;
	if (d >= FIFO_DEPTH) return WEAVER_Q16_ONE;
	return ((uint32_t)d * WEAVER_Q16_ONE) / FIFO_DEPTH;
}
#endif

static void s1_imu(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (!atomic_get(&s1_stop)) {
		int d = atomic_inc(&s1_ring) + 1;
		if (d > FIFO_DEPTH) {
			atomic_set(&s1_ring, FIFO_DEPTH);
			atomic_inc(&s1_overruns);
		}
#ifdef CONFIG_WEAVER_SCHED
		weaver_set_buffer_fill_predictive_q16(&s1_fusion_wd, depth_to_q16(d));
#endif
		k_sleep(K_MSEC(IMU_PERIOD_MS));
	}
}

static void s1_fusion(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (!atomic_get(&s1_stop)) {
		int d = atomic_get(&s1_ring);
		if (d > 0) {
			int drain = MIN(d, FUSION_DRAIN);
			atomic_sub(&s1_ring, drain);
			k_busy_wait(FUSION_WORK_US);
#ifdef CONFIG_WEAVER_SCHED
			weaver_set_buffer_fill_predictive_q16(
				&s1_fusion_wd,
				depth_to_q16(atomic_get(&s1_ring)));
#endif
		}
		k_sleep(K_MSEC(2));
	}
}

static void s1_noise(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (!atomic_get(&s1_stop)) {
		k_busy_wait(NOISE_WORK_US);
		k_sleep(K_MSEC(NOISE_PERIOD_MS));
	}
}

void scen_imu_overrun(struct scen_result *out)
{
	atomic_set(&s1_ring, 0);
	atomic_set(&s1_overruns, 0);
	atomic_set(&s1_stop, 0);

	k_thread_create(&s1_imu_t,    s1_imu_stack,    SCEN_STACK_SIZE,
			s1_imu,    NULL, NULL, NULL, IMU_PRIO,    0, K_NO_WAIT);
	k_thread_create(&s1_fusion_t, s1_fusion_stack, SCEN_STACK_SIZE,
			s1_fusion, NULL, NULL, NULL, FUSION_PRIO, 0, K_NO_WAIT);
	k_thread_create(&s1_noise_t,  s1_noise_stack,  SCEN_STACK_SIZE,
			s1_noise,  NULL, NULL, NULL, NOISE_PRIO,  0, K_NO_WAIT);

#ifdef CONFIG_WEAVER_SCHED
	weaver_register(&s1_imu_wd,    &s1_imu_t,    WEAVER_TO_Q16(15), true);
	weaver_set_warp_deadline(&s1_imu_wd, IMU_PERIOD_MS);
	weaver_register(&s1_fusion_wd, &s1_fusion_t, WEAVER_TO_Q16(8),  false);
	weaver_register(&s1_noise_wd,  &s1_noise_t,  WEAVER_TO_Q16(5),  false);
	k_timer_start(&s1_timer, K_MSEC(1), K_MSEC(1));
#endif

	k_sleep(K_MSEC(DUR_MS));
	atomic_set(&s1_stop, 1);
	k_sleep(K_MSEC(50));

#ifdef CONFIG_WEAVER_SCHED
	k_timer_stop(&s1_timer);
	weaver_unregister(&s1_imu_wd);
	weaver_unregister(&s1_fusion_wd);
	weaver_unregister(&s1_noise_wd);
#endif

	k_thread_abort(&s1_imu_t);
	k_thread_abort(&s1_fusion_t);
	k_thread_abort(&s1_noise_t);

	out->name     = "imu_overrun";
	out->overruns = atomic_get(&s1_overruns);
	out->misc_label = "duration_ms";
	out->misc       = DUR_MS;
	/* PASS is judged by the harness after capturing both stock and
	 * weaver runs; we just emit the number. */
	out->pass = true;
}
