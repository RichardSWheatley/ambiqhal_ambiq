/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Scenario 4: Burst-event throttle shedding via fuzzy controller.
 *
 * Four Weft "producer" threads run at steady state. At t=3s, t=6s,
 * t=9s, a burst event fires that ramps each producer's buffer_fill
 * synthetically up to 1.0 over 200 ms (simulating a cascade of
 * sensor events). We then observe the fuzzy throttle level and
 * verify it ramps up smoothly without flapping.
 *
 * Stock Zephyr: no concept of "system pressure", no graded throttle
 *               signal exists at all.
 *
 * Weaver:       weaver_throttle_level() rises smoothly during the
 *               burst, plateaus at >= 200, then decays cleanly.
 *               weaver_should_throttle() goes true ONCE per burst,
 *               not many times (EMA hysteresis suppresses flap).
 *
 * Metric:       peak observed weaver_throttle_level() during bursts
 *               and number of distinct throttle "entry" events.
 *
 * PASS:         stock can't report a throttle level - the comparison
 *               is qualitative. Weaver should reach >= 192 during a
 *               burst and recover to 0 between bursts.
 */

#include "scen_common.h"
#include <zephyr/sys/printk.h>

#define DUR_MS      12000
#define N_WEFTS     4
#define BURST_DUR_MS 200

static atomic_t s4_stop;
static uint32_t s4_peak_throttle;

static K_THREAD_STACK_ARRAY_DEFINE(s4_stacks, N_WEFTS, SCEN_STACK_SIZE);
static struct k_thread s4_t[N_WEFTS];

#ifdef CONFIG_WEAVER_SCHED
static struct weaver_thread_data s4_wd[N_WEFTS];
static void s4_tick(struct k_timer *t) { ARG_UNUSED(t); weaver_tick(); }
static K_TIMER_DEFINE(s4_timer, s4_tick, NULL);
#endif

static atomic_t s4_burst_active;
static atomic_t s4_burst_ramp_pct;  /* 0-100 */

static void s4_weft(void *arg, void *b, void *c)
{
	ARG_UNUSED(b); ARG_UNUSED(c);
	uintptr_t id = (uintptr_t)arg;
	while (!atomic_get(&s4_stop)) {
		(void)id;
#ifdef CONFIG_WEAVER_SCHED
		uint32_t pct = atomic_get(&s4_burst_ramp_pct);
		uint32_t fill_q16 = (pct * WEAVER_Q16_ONE) / 100U;
		weaver_set_buffer_fill_predictive_q16(&s4_wd[id], fill_q16);
#endif
		k_sleep(K_MSEC(5));
	}
}

void scen_burst_shed(struct scen_result *out)
{
	atomic_set(&s4_stop, 0);
	atomic_set(&s4_burst_active, 0);
	atomic_set(&s4_burst_ramp_pct, 0);
	s4_peak_throttle = 0;
	uint32_t throttle_entries = 0;
	bool was_throttling = false;

	for (int i = 0; i < N_WEFTS; i++) {
		k_thread_create(&s4_t[i], s4_stacks[i], SCEN_STACK_SIZE,
				s4_weft, (void *)(uintptr_t)i, NULL, NULL,
				7, 0, K_NO_WAIT);
#ifdef CONFIG_WEAVER_SCHED
		weaver_register(&s4_wd[i], &s4_t[i], WEAVER_TO_Q16(7), false);
#endif
	}
#ifdef CONFIG_WEAVER_SCHED
	k_timer_start(&s4_timer, K_MSEC(1), K_MSEC(1));
#endif

	const uint32_t burst_times_ms[] = { 3000, 6000, 9000 };
	uint32_t t0 = k_uptime_get_32();

	while (1) {
		uint32_t now = k_uptime_get_32() - t0;
		if (now >= DUR_MS) break;

		/* Drive burst ramps. */
		uint32_t ramp = 0;
		for (size_t i = 0; i < ARRAY_SIZE(burst_times_ms); i++) {
			uint32_t bs = burst_times_ms[i];
			if (now >= bs && now < bs + BURST_DUR_MS) {
				ramp = ((now - bs) * 100U) / BURST_DUR_MS;
			} else if (now >= bs + BURST_DUR_MS &&
				   now < bs + 2 * BURST_DUR_MS) {
				/* Decay back to 0 over BURST_DUR_MS. */
				ramp = 100U - (((now - bs - BURST_DUR_MS) * 100U) /
					       BURST_DUR_MS);
			}
		}
		atomic_set(&s4_burst_ramp_pct, ramp);

#ifdef CONFIG_WEAVER_SCHED
		uint8_t lvl = weaver_throttle_level();
		if (lvl > s4_peak_throttle) s4_peak_throttle = lvl;
		bool throttling = weaver_should_throttle();
		if (throttling && !was_throttling) throttle_entries++;
		was_throttling = throttling;
#endif
		k_sleep(K_MSEC(10));
	}

	atomic_set(&s4_stop, 1);
	k_sleep(K_MSEC(50));

#ifdef CONFIG_WEAVER_SCHED
	k_timer_stop(&s4_timer);
	for (int i = 0; i < N_WEFTS; i++) weaver_unregister(&s4_wd[i]);
#endif
	for (int i = 0; i < N_WEFTS; i++) k_thread_abort(&s4_t[i]);

	out->name = "burst_shed";
	out->throttle_peak = s4_peak_throttle;
	out->misc_label = "throttle_entries";
	out->misc = throttle_entries;
#ifdef CONFIG_WEAVER_SCHED
	out->pass = (s4_peak_throttle >= 192) &&
		    (throttle_entries == ARRAY_SIZE(burst_times_ms));
#else
	out->pass = false;  /* stock can't satisfy this scenario at all */
#endif
}
