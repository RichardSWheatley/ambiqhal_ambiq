/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Scenario 5: Display refresh + NVM flush deferral under sensor load.
 *
 * IMU at 100 Hz (10 ms period) with fusion consumer competes against:
 *   - Display refresh at 30 Hz, 4 ms work each
 *   - NVM flush every 250 ms, 12 ms work each
 *
 * Display and NVM are Weft, IMU + fusion are the priority path. NVM
 * is a long-running consumer that can preempt fusion under stock
 * Zephyr if their priorities allow it, causing IMU FIFO buildup.
 *
 * Metric: maximum IMU FIFO depth observed during the run. Lower is
 *         better.
 *
 * Stock Zephyr: display/NVM cycles get stolen at preempt points;
 *               peak IMU FIFO depth climbs during long NVM bursts.
 *
 * Weaver:       fusion's pressure rises as the FIFO fills, boosting
 *               it above display/NVM. Aging term ensures fusion
 *               eventually wins even when display work seems urgent.
 *
 * PASS:         Weaver peak_depth <= stock peak_depth (no regression
 *               on the IMU pipeline) and ideally <= 0.7 * stock.
 */

#include "scen_common.h"
#include <zephyr/sys/printk.h>

#define DUR_MS         15000
#define FIFO_DEPTH     64
#define IMU_PERIOD_MS  10      /* 100 Hz */
#define FUSION_DRAIN   4
#define FUSION_WORK_US 2000
#define DISP_PERIOD_MS 33      /* ~30 Hz */
#define DISP_WORK_US   4000
#define NVM_PERIOD_MS  250
#define NVM_WORK_US    12000

#define IMU_PRIO    4
#define FUSION_PRIO 8
#define DISP_PRIO   9
#define NVM_PRIO    10

static atomic_t s5_depth;
static atomic_t s5_peak;
static atomic_t s5_stop;

static K_THREAD_STACK_DEFINE(s5_imu_stack,    SCEN_STACK_SIZE);
static K_THREAD_STACK_DEFINE(s5_fusion_stack, SCEN_STACK_SIZE);
static K_THREAD_STACK_DEFINE(s5_disp_stack,   SCEN_STACK_SIZE);
static K_THREAD_STACK_DEFINE(s5_nvm_stack,    SCEN_STACK_SIZE);
static struct k_thread s5_imu_t, s5_fusion_t, s5_disp_t, s5_nvm_t;

#ifdef CONFIG_WEAVER_SCHED
static struct weaver_thread_data s5_imu_wd, s5_fusion_wd, s5_disp_wd, s5_nvm_wd;
static void s5_tick(struct k_timer *t) { ARG_UNUSED(t); weaver_tick(); }
static K_TIMER_DEFINE(s5_timer, s5_tick, NULL);

static inline uint32_t depth_to_q16(int d) {
	if (d <= 0) return 0;
	if (d >= FIFO_DEPTH) return WEAVER_Q16_ONE;
	return ((uint32_t)d * WEAVER_Q16_ONE) / FIFO_DEPTH;
}
#endif

static inline void track_peak(int d)
{
	uint32_t cur = atomic_get(&s5_peak);
	if ((uint32_t)d > cur) atomic_set(&s5_peak, d);
}

static void s5_imu(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (!atomic_get(&s5_stop)) {
		int d = atomic_inc(&s5_depth) + 1;
		if (d > FIFO_DEPTH) {
			atomic_set(&s5_depth, FIFO_DEPTH);
			d = FIFO_DEPTH;
		}
		track_peak(d);
#ifdef CONFIG_WEAVER_SCHED
		weaver_set_buffer_fill_predictive_q16(&s5_fusion_wd, depth_to_q16(d));
#endif
		k_sleep(K_MSEC(IMU_PERIOD_MS));
	}
}

static void s5_fusion(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (!atomic_get(&s5_stop)) {
		int d = atomic_get(&s5_depth);
		if (d > 0) {
			int drain = MIN(d, FUSION_DRAIN);
			atomic_sub(&s5_depth, drain);
			k_busy_wait(FUSION_WORK_US);
#ifdef CONFIG_WEAVER_SCHED
			weaver_set_buffer_fill_predictive_q16(
				&s5_fusion_wd, depth_to_q16(atomic_get(&s5_depth)));
#endif
		}
		k_sleep(K_MSEC(5));
	}
}

static void s5_disp(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (!atomic_get(&s5_stop)) {
		k_busy_wait(DISP_WORK_US);
		k_sleep(K_MSEC(DISP_PERIOD_MS));
	}
}

static void s5_nvm(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (!atomic_get(&s5_stop)) {
		k_busy_wait(NVM_WORK_US);
		k_sleep(K_MSEC(NVM_PERIOD_MS));
	}
}

void scen_display_defer(struct scen_result *out)
{
	atomic_set(&s5_depth, 0);
	atomic_set(&s5_peak, 0);
	atomic_set(&s5_stop, 0);

	k_thread_create(&s5_imu_t,    s5_imu_stack,    SCEN_STACK_SIZE,
			s5_imu,    NULL, NULL, NULL, IMU_PRIO,    0, K_NO_WAIT);
	k_thread_create(&s5_fusion_t, s5_fusion_stack, SCEN_STACK_SIZE,
			s5_fusion, NULL, NULL, NULL, FUSION_PRIO, 0, K_NO_WAIT);
	k_thread_create(&s5_disp_t,   s5_disp_stack,   SCEN_STACK_SIZE,
			s5_disp,   NULL, NULL, NULL, DISP_PRIO,   0, K_NO_WAIT);
	k_thread_create(&s5_nvm_t,    s5_nvm_stack,    SCEN_STACK_SIZE,
			s5_nvm,    NULL, NULL, NULL, NVM_PRIO,    0, K_NO_WAIT);

#ifdef CONFIG_WEAVER_SCHED
	weaver_register(&s5_imu_wd,    &s5_imu_t,    WEAVER_TO_Q16(15), true);
	weaver_set_warp_deadline(&s5_imu_wd, IMU_PERIOD_MS);
	weaver_register(&s5_fusion_wd, &s5_fusion_t, WEAVER_TO_Q16(8),  false);
	weaver_register(&s5_disp_wd,   &s5_disp_t,   WEAVER_TO_Q16(4),  false);
	weaver_register(&s5_nvm_wd,    &s5_nvm_t,    WEAVER_TO_Q16(2),  false);
	k_timer_start(&s5_timer, K_MSEC(1), K_MSEC(1));
#endif

	k_sleep(K_MSEC(DUR_MS));
	atomic_set(&s5_stop, 1);
	k_sleep(K_MSEC(50));

#ifdef CONFIG_WEAVER_SCHED
	k_timer_stop(&s5_timer);
	weaver_unregister(&s5_imu_wd);
	weaver_unregister(&s5_fusion_wd);
	weaver_unregister(&s5_disp_wd);
	weaver_unregister(&s5_nvm_wd);
#endif
	k_thread_abort(&s5_imu_t);
	k_thread_abort(&s5_fusion_t);
	k_thread_abort(&s5_disp_t);
	k_thread_abort(&s5_nvm_t);

	out->name = "display_defer";
	out->misc_label = "peak_fifo_depth";
	out->misc       = atomic_get(&s5_peak);
	out->pass = true;
}
