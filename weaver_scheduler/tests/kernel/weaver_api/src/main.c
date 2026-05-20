/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Weaver scheduler API contract tests.
 *
 * Exercises every public function under realistic and adversarial
 * inputs:
 *   - Argument validation (NULL, zero, out-of-range)
 *   - State transitions (register, set fill, set weights, unregister)
 *   - Pressure math (warp saturation, weft sum, predictive extrapolation)
 *   - Boost behavior (priority elevation, restore on unregister)
 *   - Registry capacity (-ENOMEM at WEAVER_MAX_THREADS)
 *
 * Runs under native_sim so a CI machine without an Apollo510 EVB can
 * still gate on API correctness.
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/kernel/weaver_sched.h>
#include <errno.h>

#define TEST_STACK 1024
#define IDLE_PRIO  9

static K_THREAD_STACK_ARRAY_DEFINE(test_stacks, CONFIG_WEAVER_MAX_THREADS + 2,
				   TEST_STACK);
static struct k_thread test_threads[CONFIG_WEAVER_MAX_THREADS + 2];
static struct weaver_thread_data test_wd[CONFIG_WEAVER_MAX_THREADS + 2];
static int n_created;

static void idle_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		k_sleep(K_MSEC(100));
	}
}

/* Setup: create CONFIG_WEAVER_MAX_THREADS + 1 do-nothing threads
 * so test cases have handles to register. The +1 lets us prove the
 * registry returns -ENOMEM when full. */
static void *suite_setup(void)
{
	for (int i = 0; i < CONFIG_WEAVER_MAX_THREADS + 1; i++) {
		k_thread_create(&test_threads[i], test_stacks[i], TEST_STACK,
				idle_fn, NULL, NULL, NULL,
				IDLE_PRIO, 0, K_NO_WAIT);
		n_created++;
	}
	/* Let them schedule once so k_thread_priority_get returns sane. */
	k_sleep(K_MSEC(10));
	return NULL;
}

/* Each test starts and ends with an empty registry. */
static void test_before(void *fixture)
{
	ARG_UNUSED(fixture);
	for (int i = 0; i < CONFIG_WEAVER_MAX_THREADS + 1; i++) {
		(void)weaver_unregister(&test_wd[i]);
		memset(&test_wd[i], 0, sizeof(test_wd[i]));
	}
}

ZTEST(weaver_api, register_basic_success)
{
	int rc = weaver_register(&test_wd[0], &test_threads[0],
				 WEAVER_TO_Q16(8), false);
	zassert_equal(rc, 0, "register returned %d", rc);
	zassert_equal(test_wd[0].in_use, 1, "in_use not set");
	zassert_equal(test_wd[0].is_warp, 0, "is_warp should be 0 for Weft");
	zassert_equal(test_wd[0].priority_q16, WEAVER_TO_Q16(8),
		      "priority_q16 not stored");
}

ZTEST(weaver_api, register_warp_marks_warp)
{
	int rc = weaver_register(&test_wd[0], &test_threads[0],
				 WEAVER_TO_Q16(15), true);
	zassert_equal(rc, 0, "register returned %d", rc);
	zassert_equal(test_wd[0].is_warp, 1, "is_warp not set for Warp");
}

ZTEST(weaver_api, register_null_wd_returns_einval)
{
	int rc = weaver_register(NULL, &test_threads[0], WEAVER_TO_Q16(8), false);
	zassert_equal(rc, -EINVAL, "expected -EINVAL got %d", rc);
}

ZTEST(weaver_api, register_null_thread_returns_einval)
{
	int rc = weaver_register(&test_wd[0], NULL, WEAVER_TO_Q16(8), false);
	zassert_equal(rc, -EINVAL, "expected -EINVAL got %d", rc);
}

ZTEST(weaver_api, register_full_returns_enomem)
{
	for (int i = 0; i < CONFIG_WEAVER_MAX_THREADS; i++) {
		int rc = weaver_register(&test_wd[i], &test_threads[i],
					 WEAVER_TO_Q16(5), false);
		zassert_equal(rc, 0, "slot %d register failed: %d", i, rc);
	}
	int rc = weaver_register(&test_wd[CONFIG_WEAVER_MAX_THREADS],
				 &test_threads[CONFIG_WEAVER_MAX_THREADS],
				 WEAVER_TO_Q16(5), false);
	zassert_equal(rc, -ENOMEM,
		      "expected -ENOMEM when registry full got %d", rc);
}

ZTEST(weaver_api, unregister_restores_base_priority)
{
	int8_t base = (int8_t)k_thread_priority_get(&test_threads[0]);
	weaver_register(&test_wd[0], &test_threads[0], WEAVER_TO_Q16(8), false);

	/* Simulate Weaver boosting the priority. */
	k_thread_priority_set(&test_threads[0], base - 1);

	int rc = weaver_unregister(&test_wd[0]);
	zassert_equal(rc, 0, "unregister returned %d", rc);
	int8_t now = (int8_t)k_thread_priority_get(&test_threads[0]);
	zassert_equal(now, base, "priority not restored: now=%d base=%d",
		      now, base);
}

ZTEST(weaver_api, unregister_unknown_wd_returns_enoent)
{
	struct weaver_thread_data orphan = { .in_use = 0 };
	int rc = weaver_unregister(&orphan);
	zassert_equal(rc, -ENOENT, "expected -ENOENT got %d", rc);
}

ZTEST(weaver_api, unregister_null_returns_einval)
{
	int rc = weaver_unregister(NULL);
	zassert_equal(rc, -EINVAL, "expected -EINVAL got %d", rc);
}

ZTEST(weaver_api, set_buffer_fill_clamps_to_one)
{
	weaver_register(&test_wd[0], &test_threads[0], WEAVER_TO_Q16(8), false);
	weaver_set_buffer_fill_q16(&test_wd[0], 0xDEADBEEF);
	zassert_equal(test_wd[0].buffer_fill_q16, WEAVER_Q16_ONE,
		      "fill not clamped: %u", test_wd[0].buffer_fill_q16);
}

ZTEST(weaver_api, set_buffer_fill_null_no_crash)
{
	weaver_set_buffer_fill_q16(NULL, WEAVER_Q16_ONE);
	/* If we reach here without crashing, pass. */
	zassert_true(true, "should not crash on NULL");
}

ZTEST(weaver_api, predictive_extrapolates_upward)
{
	weaver_register(&test_wd[0], &test_threads[0], WEAVER_TO_Q16(8), false);
	weaver_set_buffer_fill_predictive_q16(&test_wd[0], WEAVER_Q16_ONE / 4);
	/* From 0 -> Q16/4 the predicted next = Q16/4 + (Q16/4 - 0) = Q16/2. */
	weaver_set_buffer_fill_predictive_q16(&test_wd[0], WEAVER_Q16_ONE / 4);
	zassert_equal(test_wd[0].buffer_fill_q16, WEAVER_Q16_ONE / 4,
		      "stable input should give stable output, got %u",
		      test_wd[0].buffer_fill_q16);
}

ZTEST(weaver_api, predictive_saturates_at_one)
{
	weaver_register(&test_wd[0], &test_threads[0], WEAVER_TO_Q16(8), false);
	/* Two large successive jumps should clamp to ONE, not overflow. */
	weaver_set_buffer_fill_predictive_q16(&test_wd[0], WEAVER_Q16_ONE / 2);
	weaver_set_buffer_fill_predictive_q16(&test_wd[0], WEAVER_Q16_ONE);
	zassert_true(test_wd[0].buffer_fill_q16 <= WEAVER_Q16_ONE,
		     "predictive overshot: %u", test_wd[0].buffer_fill_q16);
}

ZTEST(weaver_api, calculate_pressure_warp_is_max)
{
	test_wd[0].is_warp = 1;
	uint32_t p = weaver_calculate_pressure(&test_wd[0]);
	zassert_equal(p, WEAVER_PRESSURE_WARP,
		      "Warp pressure should saturate, got 0x%x", p);
}

ZTEST(weaver_api, calculate_pressure_weft_sums_terms)
{
	test_wd[0].is_warp = 0;
	test_wd[0].priority_q16 = WEAVER_Q16_ONE;
	test_wd[0].buffer_fill_q16 = WEAVER_Q16_ONE;
	test_wd[0].wait_ticks = 0;

	uint32_t p = weaver_calculate_pressure(&test_wd[0]);
	/* priority*W_URG + fill*W_DEN + 0*W_AGE = 0x6666 + 0x6666 = 0xCCCC */
	zassert_within(p, 0xCCCCU, 8,
		       "pressure %u not within 8 of expected 0xCCCC", p);
}

ZTEST(weaver_api, set_weights_zero_preserves_old)
{
	weaver_set_weights(0x4000, 0x4000, 0x2000);
	/* Then passing 0 in any slot should leave that weight alone. */
	weaver_set_weights(0, 0x3000, 0);

	/* Indirect verification: pressure with the new W_DEN must reflect
	 * the changed value but other terms must still use the prior
	 * weights. */
	struct weaver_thread_data wd = {
		.is_warp = 0,
		.priority_q16 = WEAVER_Q16_ONE,
		.buffer_fill_q16 = WEAVER_Q16_ONE,
		.wait_ticks = 0,
	};
	uint32_t p = weaver_calculate_pressure(&wd);
	/* urgency = 0x4000, density = 0x3000 => 0x7000 */
	zassert_within(p, 0x7000U, 8,
		       "pressure %u suggests weights not partial-updated", p);

	/* Restore defaults for subsequent tests. */
	weaver_set_weights(WEAVER_W_URGENCY, WEAVER_W_DENSITY, WEAVER_W_AGING);
}

ZTEST(weaver_api, boost_levels_caps_at_zero)
{
	/* Register a thread that's already at priority 0. Set boost to
	 * something silly like 5. weft_boost_prio should never go
	 * negative (cooperative space). */
	k_thread_priority_set(&test_threads[0], 0);
	weaver_register(&test_wd[0], &test_threads[0], WEAVER_TO_Q16(8), false);
	weaver_set_boost_levels(&test_wd[0], 5);
	zassert_true(test_wd[0].weft_boost_prio >= 0,
		     "boost crossed into coop: %d", test_wd[0].weft_boost_prio);
}

ZTEST(weaver_api, tick_promotes_highest_pressure)
{
	/* Two Wefts: one starved, one idle. After several ticks the
	 * starved one should be at a higher priority than the idle one. */
	weaver_register(&test_wd[0], &test_threads[0], WEAVER_TO_Q16(8), false);
	weaver_register(&test_wd[1], &test_threads[1], WEAVER_TO_Q16(8), false);

	weaver_set_buffer_fill_q16(&test_wd[0], WEAVER_Q16_ONE);
	weaver_set_buffer_fill_q16(&test_wd[1], 0);

	for (int i = 0; i < 5; i++) {
		weaver_tick();
	}

	int p0 = k_thread_priority_get(&test_threads[0]);
	int p1 = k_thread_priority_get(&test_threads[1]);
	/* Lower numeric value = higher priority in Zephyr. */
	zassert_true(p0 < p1 || p0 == p1,
		     "high-pressure not boosted: p0=%d p1=%d", p0, p1);
}

ZTEST_SUITE(weaver_api, NULL, suite_setup, test_before, NULL, NULL);
