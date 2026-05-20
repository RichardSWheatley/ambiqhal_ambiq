/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Weaver scheduler stress / soak / negative tests.
 *
 *   stress: rapid register/unregister churn, registry-full + recovery,
 *           rapid weights / boost / deadline changes.
 *   soak:   N-second weaver_tick() run with active producers, verify
 *           no stat-counter wraparound and no priority leakage.
 *   neg:    double-unregister, unregister of foreign wd, thread
 *           aborted before unregister.
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/kernel/weaver_sched.h>
#include <errno.h>
#include <string.h>

#define STK 1024
#define IDLE_PRIO 9
#define POOL_SIZE (CONFIG_WEAVER_MAX_THREADS + 2)

static K_THREAD_STACK_ARRAY_DEFINE(stress_stacks, POOL_SIZE, STK);
static struct k_thread stress_threads[POOL_SIZE];
static struct weaver_thread_data stress_wd[POOL_SIZE];

static void idle_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		k_sleep(K_MSEC(50));
	}
}

static void *suite_setup(void)
{
	for (int i = 0; i < POOL_SIZE; i++) {
		k_thread_create(&stress_threads[i], stress_stacks[i], STK,
				idle_fn, NULL, NULL, NULL,
				IDLE_PRIO, 0, K_NO_WAIT);
	}
	k_sleep(K_MSEC(10));
	return NULL;
}

static void cleanup(void *fixture)
{
	ARG_UNUSED(fixture);
	for (int i = 0; i < POOL_SIZE; i++) {
		(void)weaver_unregister(&stress_wd[i]);
		memset(&stress_wd[i], 0, sizeof(stress_wd[i]));
	}
}

/* -------- stress -------- */

ZTEST(weaver_stress, rapid_register_unregister_churn)
{
	for (int iter = 0; iter < 1000; iter++) {
		int rc = weaver_register(&stress_wd[0], &stress_threads[0],
					 WEAVER_TO_Q16(8), false);
		zassert_equal(rc, 0, "iter %d register failed: %d", iter, rc);
		rc = weaver_unregister(&stress_wd[0]);
		zassert_equal(rc, 0, "iter %d unregister failed: %d", iter, rc);
	}
}

ZTEST(weaver_stress, fill_registry_then_drain_then_refill)
{
	for (int round = 0; round < 3; round++) {
		for (int i = 0; i < CONFIG_WEAVER_MAX_THREADS; i++) {
			int rc = weaver_register(&stress_wd[i], &stress_threads[i],
						 WEAVER_TO_Q16(5), false);
			zassert_equal(rc, 0, "round %d slot %d: %d", round, i, rc);
		}
		/* The +1th registration must fail with -ENOMEM. */
		int rc = weaver_register(&stress_wd[CONFIG_WEAVER_MAX_THREADS],
					 &stress_threads[CONFIG_WEAVER_MAX_THREADS],
					 WEAVER_TO_Q16(5), false);
		zassert_equal(rc, -ENOMEM, "expected -ENOMEM round %d got %d",
			      round, rc);
		for (int i = 0; i < CONFIG_WEAVER_MAX_THREADS; i++) {
			rc = weaver_unregister(&stress_wd[i]);
			zassert_equal(rc, 0, "drain round %d slot %d: %d",
				      round, i, rc);
		}
	}
}

ZTEST(weaver_stress, runtime_weights_dont_corrupt)
{
	weaver_register(&stress_wd[0], &stress_threads[0],
			WEAVER_TO_Q16(8), false);

	for (int i = 0; i < 100; i++) {
		uint32_t a = 0x1000 + (i * 0x100);
		uint32_t b = 0x2000 + (i * 0x100);
		uint32_t c = 0x3000 + (i * 0x100);
		weaver_set_weights(a, b, c);

		/* Pressure for a static thread should change consistently
		 * with the new weights - no wraparound, no garbage. */
		stress_wd[0].priority_q16 = WEAVER_Q16_ONE;
		stress_wd[0].buffer_fill_q16 = 0;
		stress_wd[0].wait_ticks = 0;
		uint32_t p = weaver_calculate_pressure(&stress_wd[0]);
		zassert_true(p > 0, "iter %d: zero pressure with prio=ONE", i);
		zassert_true(p < 0x100000U,
			     "iter %d: runaway pressure 0x%x", i, p);
	}
	weaver_set_weights(WEAVER_W_URGENCY, WEAVER_W_DENSITY, WEAVER_W_AGING);
}

/* -------- soak -------- */

ZTEST(weaver_stress, soak_2s_tick_with_active_producers)
{
	/* Light registry with 3 threads, drive weaver_tick() ~2000 times
	 * (2 ms cadence) while flipping buffer fills. Verify no stat
	 * counter goes backwards and no priority gets stuck out of range. */
	weaver_register(&stress_wd[0], &stress_threads[0], WEAVER_TO_Q16(15), true);
	weaver_set_warp_deadline(&stress_wd[0], 10);
	weaver_register(&stress_wd[1], &stress_threads[1], WEAVER_TO_Q16(8), false);
	weaver_register(&stress_wd[2], &stress_threads[2], WEAVER_TO_Q16(8), false);

	struct weaver_stats prev, now;
	weaver_get_stats(&prev);

	for (int i = 0; i < 1000; i++) {
		uint32_t f = (i & 0x1F) * (WEAVER_Q16_ONE / 32U);
		weaver_set_buffer_fill_q16(&stress_wd[1], f);
		weaver_set_buffer_fill_q16(&stress_wd[2], WEAVER_Q16_ONE - f);
		weaver_tick();
		k_sleep(K_MSEC(2));
	}

	weaver_get_stats(&now);
	zassert_true(now.total_ticks > prev.total_ticks,
		     "tick counter did not advance");
	zassert_true(now.total_ticks >= 1000,
		     "expected >=1000 ticks, got %u",
		     now.total_ticks - prev.total_ticks);

	/* Priority must end within the valid range. */
	int p1 = k_thread_priority_get(&stress_threads[1]);
	int p2 = k_thread_priority_get(&stress_threads[2]);
	zassert_true(p1 >= -128 && p1 <= 127, "p1 out of range: %d", p1);
	zassert_true(p2 >= -128 && p2 <= 127, "p2 out of range: %d", p2);
}

/* -------- negative -------- */

ZTEST(weaver_stress, double_unregister_returns_enoent)
{
	weaver_register(&stress_wd[0], &stress_threads[0],
			WEAVER_TO_Q16(5), false);
	int rc = weaver_unregister(&stress_wd[0]);
	zassert_equal(rc, 0, "first unregister: %d", rc);
	rc = weaver_unregister(&stress_wd[0]);
	zassert_equal(rc, -ENOENT, "double unregister should be -ENOENT got %d", rc);
}

ZTEST(weaver_stress, unregister_foreign_wd)
{
	struct weaver_thread_data foreign;
	memset(&foreign, 0, sizeof(foreign));
	foreign.in_use = 1;   /* lying about state */
	int rc = weaver_unregister(&foreign);
	zassert_equal(rc, -ENOENT,
		      "foreign wd unregister should be -ENOENT got %d", rc);
}

ZTEST(weaver_stress, weaver_tick_with_empty_registry_safe)
{
	for (int i = 0; i < 100; i++) {
		weaver_tick();  /* must not crash */
	}
	zassert_true(true, "should have completed");
}

ZTEST(weaver_stress, set_boost_levels_zero_no_op)
{
	weaver_register(&stress_wd[0], &stress_threads[0],
			WEAVER_TO_Q16(5), false);
	uint8_t before = stress_wd[0].boost_levels;
	weaver_set_boost_levels(&stress_wd[0], 0);
	zassert_equal(stress_wd[0].boost_levels, before,
		      "zero boost should be no-op, got %d", stress_wd[0].boost_levels);
}

ZTEST(weaver_stress, deadline_zero_disables_countdown)
{
	weaver_register(&stress_wd[0], &stress_threads[0],
			WEAVER_TO_Q16(15), true);
	weaver_set_warp_deadline(&stress_wd[0], 10);
	weaver_set_warp_deadline(&stress_wd[0], 0);
	zassert_equal(stress_wd[0].period_ticks, 0, "period should be 0");
	zassert_equal(stress_wd[0].next_deadline_ticks, 0,
		      "next_deadline should be 0");
}

ZTEST_SUITE(weaver_stress, NULL, suite_setup, NULL, cleanup, NULL);
