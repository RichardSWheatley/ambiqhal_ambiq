/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Scenario 2: BLE LL deadline integrity under heavy Weft contention.
 *
 * Models a 50 ms BLE connection event (3 ms work window) competing
 * with several Weft threads that each do bursty CPU work. The BLE
 * thread is Warp under Weaver; its deadline is registered.
 *
 * Each BLE wakeup records the timestamp delta vs. its expected 50 ms
 * cadence. A miss is defined as >1 ms late.
 *
 * Stock Zephyr: BLE wins on static priority alone. The Weft threads
 *               can still steal cycles right up to the BLE wake
 *               instant via timeslicing, occasionally pushing the
 *               BLE start past its window.
 *
 * Weaver:       Pre-Warp guard window suppresses Weft promotion when
 *               BLE deadline is within CONFIG_WEAVER_PREWARP_GUARD_TICKS
 *               ticks; BLE sees a quiet system.
 *
 * PASS threshold (M3 hard gate): Weaver miss count <= stock miss count.
 * No regression is acceptable. Improvement is bonus.
 */

#include "scen_common.h"
#include <zephyr/sys/printk.h>

#define DUR_MS          15000
#define BLE_PERIOD_MS   50
#define BLE_WORK_US     3000
#define MISS_THRESH_US  1000   /* >1 ms late counts as miss */
#define N_NOISE         3
#define NOISE_PERIOD_MS 8
#define NOISE_WORK_US   5000

#define BLE_PRIO    2
#define NOISE_PRIO  6

static atomic_t s2_misses;
static atomic_t s2_wakes;
static atomic_t s2_stop;

static K_THREAD_STACK_DEFINE(s2_ble_stack, SCEN_STACK_SIZE);
static K_THREAD_STACK_ARRAY_DEFINE(s2_noise_stacks, N_NOISE, SCEN_STACK_SIZE);
static struct k_thread s2_ble_t;
static struct k_thread s2_noise_t[N_NOISE];

#ifdef CONFIG_WEAVER_SCHED
static struct weaver_thread_data s2_ble_wd, s2_noise_wd[N_NOISE];
static void s2_tick(struct k_timer *t) { ARG_UNUSED(t); weaver_tick(); }
static K_TIMER_DEFINE(s2_timer, s2_tick, NULL);
#endif

static void s2_ble(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	uint32_t expected = k_uptime_get_32();
	while (!atomic_get(&s2_stop)) {
		expected += BLE_PERIOD_MS;
		k_sleep(K_TIMEOUT_ABS_MS(expected));

		uint32_t now = k_uptime_get_32();
		int32_t late_ms = (int32_t)(now - expected);
		if (late_ms * 1000 > (int32_t)MISS_THRESH_US) {
			atomic_inc(&s2_misses);
		}
		atomic_inc(&s2_wakes);
		k_busy_wait(BLE_WORK_US);
	}
}

static void s2_noise(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (!atomic_get(&s2_stop)) {
		k_busy_wait(NOISE_WORK_US);
		k_sleep(K_MSEC(NOISE_PERIOD_MS));
	}
}

void scen_ble_deadline(struct scen_result *out)
{
	atomic_set(&s2_misses, 0);
	atomic_set(&s2_wakes, 0);
	atomic_set(&s2_stop, 0);

	k_thread_create(&s2_ble_t, s2_ble_stack, SCEN_STACK_SIZE,
			s2_ble, NULL, NULL, NULL, BLE_PRIO, 0, K_NO_WAIT);
#ifdef CONFIG_WEAVER_SCHED
	weaver_register(&s2_ble_wd, &s2_ble_t, WEAVER_TO_Q16(20), true);
	weaver_set_warp_deadline(&s2_ble_wd, BLE_PERIOD_MS);
#endif

	for (int i = 0; i < N_NOISE; i++) {
		k_thread_create(&s2_noise_t[i], s2_noise_stacks[i], SCEN_STACK_SIZE,
				s2_noise, NULL, NULL, NULL, NOISE_PRIO, 0, K_NO_WAIT);
#ifdef CONFIG_WEAVER_SCHED
		weaver_register(&s2_noise_wd[i], &s2_noise_t[i],
				WEAVER_TO_Q16(5), false);
#endif
	}

#ifdef CONFIG_WEAVER_SCHED
	k_timer_start(&s2_timer, K_MSEC(1), K_MSEC(1));
#endif

	k_sleep(K_MSEC(DUR_MS));
	atomic_set(&s2_stop, 1);
	k_sleep(K_MSEC(50));

#ifdef CONFIG_WEAVER_SCHED
	k_timer_stop(&s2_timer);
	weaver_unregister(&s2_ble_wd);
	for (int i = 0; i < N_NOISE; i++) weaver_unregister(&s2_noise_wd[i]);
#endif

	k_thread_abort(&s2_ble_t);
	for (int i = 0; i < N_NOISE; i++) k_thread_abort(&s2_noise_t[i]);

	out->name = "ble_deadline";
	out->deadline_misses = atomic_get(&s2_misses);
	out->misc_label = "ble_wakes";
	out->misc       = atomic_get(&s2_wakes);
	out->pass = (atomic_get(&s2_misses) == 0);  /* hard gate */
}
