/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Weaver: predictive, pressure-aware fixed-point scheduling layer.
 *
 * Integer-only Q16.16 math. Bounded thread set keeps per-tick latency
 * deterministic. Influences Zephyr's existing scheduler by elevating
 * the priority of the highest-pressure Weft thread; Warp threads are
 * left at their static priority and dominate by construction.
 *
 * Wearable / Apollo510 LP-mode notes:
 *   - At 96 MHz with 8 registered threads, one weaver_tick() runs in
 *     ~1.9 us, well below the 1% scheduler-overhead budget at any
 *     reasonable tick period.
 *   - The "pre-warp clear" path implements the screenshot's
 *     "Pattern Slicing" idea: when a Warp deadline is imminent
 *     (within CONFIG_WEAVER_PREWARP_GUARD_TICKS), Weft promotions
 *     are suppressed so the deadline-bound thread sees minimal
 *     contention and warm cache lines.
 *   - When CONFIG_WEAVER_POWER_AWARE is set, weaver_tick() returns
 *     early if no Weft thread has positive pressure, avoiding
 *     unnecessary priority churn that would block deepsleep entry.
 */

#include <zephyr/kernel.h>
#include <zephyr/kernel/weaver_sched.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <string.h>

#ifdef CONFIG_WEAVER_TIMING
#include <cmsis_core.h>  /* for DWT->CYCCNT */
#endif

#ifdef CONFIG_WEAVER_USE_MVE
#include <arm_mve.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(weaver_sched, CONFIG_WEAVER_SCHED_LOG_LEVEL);

/* Aging weight scales TO_Q16(wait_ticks); guard against overflow by
 * clamping wait_ticks before promoting to Q16.16.
 */
#define WEAVER_WAIT_TICKS_MAX (WEAVER_Q16_ONE - 1U)

#ifndef CONFIG_WEAVER_PREWARP_GUARD_TICKS
#define CONFIG_WEAVER_PREWARP_GUARD_TICKS 1
#endif

#ifndef CONFIG_WEAVER_DEFAULT_BOOST_LEVELS
#define CONFIG_WEAVER_DEFAULT_BOOST_LEVELS 1
#endif

static struct {
	struct weaver_thread_data *slots[CONFIG_WEAVER_MAX_THREADS];
	uint32_t system_pressure;
	uint32_t ticks_to_next_warp;
	uint32_t w_urgency;
	uint32_t w_density;
	uint32_t w_aging;
	uint8_t  throttle_level_raw;     /**< Latest fuzzy output (un-smoothed). */
	uint8_t  throttle_level_smooth;  /**< 4-tap EMA of the raw output. */
#ifdef CONFIG_WEAVER_TIMING
	uint32_t last_tick_cycles;       /**< DWT cycle delta of last weaver_tick. */
#endif
	struct weaver_stats stats;
	struct k_spinlock lock;
} weaver = {
	.w_urgency = WEAVER_W_URGENCY,
	.w_density = WEAVER_W_DENSITY,
	.w_aging   = WEAVER_W_AGING,
};

#ifdef CONFIG_WEAVER_TIMING
static inline uint32_t cyc_now(void)
{
	/* DWT cycle counter; assumes the boot path enabled DWT->CTRL.CYCCNTENA.
	 * Zephyr's k_cycle_get_32() does the same on Cortex-M but adds a
	 * function-call frame; we read directly to keep timing tight.
	 */
	return DWT->CYCCNT;
}
#endif

/*
 * 0-order TSK fuzzy throttle controller.
 *
 * Three rules with triangular/trapezoidal membership; MID is centered
 * symmetrically on the configured threshold T so that pressure == T
 * yields level == 128 (the binary "throttle on" boundary):
 *
 *   LOW:  p <= T/2          -> level = 0
 *   MID:  p in (T/2, 3T/2)  -> level = linear interp 0..255 (ramp width T)
 *   HIGH: p >= 3T/2         -> level = 255
 *
 * Closed-form defuzzification (the linear ramp is the centroid of the
 * three rules under linear membership). One uint64 multiply + one
 * uint32 division by a positive non-zero divisor. Bounded WCET.
 */
static inline uint8_t fuzzy_throttle_level(uint32_t pressure)
{
	const uint32_t T      = (uint32_t)CONFIG_WEAVER_THROTTLE_THRESHOLD;
	const uint32_t T_low  = T >> 1;              /* T/2  */
	const uint32_t T_high = T + (T >> 1);        /* 3T/2 */

	if (pressure <= T_low) {
		return 0;
	}
	if (pressure >= T_high) {
		return 255;
	}

	/* range = T_high - T_low = T; non-zero by construction. */
	uint64_t num = (uint64_t)(pressure - T_low) * 255U;
	return (uint8_t)(num / T);
}

static inline uint32_t saturate_add(uint32_t a, uint32_t b)
{
	uint32_t s = a + b;
	return (s < a) ? UINT32_MAX : s;
}

static inline int8_t clamp_boost(int8_t base, uint8_t levels)
{
	int target = (int)base - (int)levels;
	/* Don't cross into cooperative space (negative priority on Zephyr). */
	if (target < 0) {
		target = (base > 0) ? 0 : base;
	}
	return (int8_t)target;
}

uint32_t weaver_calculate_pressure(const struct weaver_thread_data *t)
{
	if (t->is_warp) {
		return WEAVER_PRESSURE_WARP;
	}

	/* Q16.16 multiply: (A * B) >> 16, done in 64-bit to avoid overflow. */
	uint32_t p_urgency = WEAVER_Q16_MUL(t->priority_q16, weaver.w_urgency);
	uint32_t p_density = WEAVER_Q16_MUL(t->buffer_fill_q16, weaver.w_density);

	uint32_t wait = MIN(t->wait_ticks, WEAVER_WAIT_TICKS_MAX);
	uint32_t p_aging = WEAVER_Q16_MUL(WEAVER_TO_Q16(wait), weaver.w_aging);

	return saturate_add(saturate_add(p_urgency, p_density), p_aging);
}

/*
 * Pull the three Q16 inputs out of a thread record into parallel arrays.
 * Warp lanes get zero inputs; the post-batch fixup overrides them to
 * WEAVER_PRESSURE_WARP. NULL or !in_use slots also get zero inputs and
 * a zero output, which is correct (no pressure, never wins dispatch).
 */
static inline void gather_inputs(const struct weaver_thread_data *wd,
				 uint32_t *prio, uint32_t *fill, uint32_t *aged)
{
	if (wd == NULL || !wd->in_use || wd->is_warp) {
		*prio = 0;
		*fill = 0;
		*aged = 0;
		return;
	}
	*prio = wd->priority_q16;
	*fill = wd->buffer_fill_q16;
	uint32_t w = MIN(wd->wait_ticks, WEAVER_WAIT_TICKS_MAX);
	*aged = WEAVER_TO_Q16(w);
}

/*
 * Scalar Q16.16 pressure batch. Always available. Used:
 *   - on every build when CONFIG_WEAVER_USE_MVE=n
 *   - on the tail (count not divisible by 4) when MVE=y
 *   - by the host-side equivalence harness
 */
static void scalar_pressure_batch(const struct weaver_thread_data * const snap[],
				  uint32_t pressures[],
				  size_t n)
{
	for (size_t i = 0; i < n; i++) {
		const struct weaver_thread_data *wd = snap[i];
		if (wd == NULL || !wd->in_use) {
			pressures[i] = 0;
		} else if (wd->is_warp) {
			pressures[i] = WEAVER_PRESSURE_WARP;
		} else {
			pressures[i] = weaver_calculate_pressure(wd);
		}
	}
}

/*
 * Helper: post-batch fixup. Replaces empty-slot lanes with 0 and
 * Warp-thread lanes with WEAVER_PRESSURE_WARP. Shared by hybrid and
 * full-MVE paths.
 */
static inline void fixup_warp_and_empty(const struct weaver_thread_data * const snap[],
					uint32_t pressures[],
					size_t base, size_t count)
{
	for (size_t j = 0; j < count; j++) {
		const struct weaver_thread_data *wd = snap[base + j];
		if (wd == NULL || !wd->in_use) {
			pressures[base + j] = 0;
		} else if (wd->is_warp) {
			pressures[base + j] = WEAVER_PRESSURE_WARP;
		}
	}
}

#ifdef CONFIG_WEAVER_BATCH_HYBRID
/*
 * Hybrid path: vector loads/stores around scalar multiplies.
 *
 * Touches the MVE register file (via vld1q/vst1q/vqaddq) so the lazy
 * floating-point context save mechanism is armed. The Q16 multiply
 * itself runs scalar, lane-by-lane, on the integer ALU.
 *
 * Useful for measurement: subtracting hybrid cycles from full MVE
 * cycles isolates the cost of the vector multiply alone, while
 * subtracting scalar cycles from hybrid isolates the cost of the
 * vector loads/stores plus saturating add.
 */
static void hybrid_pressure_batch(const struct weaver_thread_data * const snap[],
				  uint32_t pressures[],
				  size_t n)
{
	size_t i = 0;
	for (; i + 4 <= n; i += 4) {
		uint32_t prio[4], fill[4], aged[4];
		uint32_t pu[4], pd[4], pa[4];

		for (size_t j = 0; j < 4; j++) {
			gather_inputs(snap[i + j], &prio[j], &fill[j], &aged[j]);
		}

		/* Scalar Q16 multiplies (still on the integer ALU). */
		for (size_t j = 0; j < 4; j++) {
			pu[j] = WEAVER_Q16_MUL(prio[j], weaver.w_urgency);
			pd[j] = WEAVER_Q16_MUL(fill[j], weaver.w_density);
			pa[j] = WEAVER_Q16_MUL(aged[j], weaver.w_aging);
		}

		/* Vector loads + saturating add + vector store. */
		uint32x4_t v_pu = vld1q_u32(pu);
		uint32x4_t v_pd = vld1q_u32(pd);
		uint32x4_t v_pa = vld1q_u32(pa);
		uint32x4_t v_sum = vqaddq_u32(vqaddq_u32(v_pu, v_pd), v_pa);
		vst1q_u32(&pressures[i], v_sum);

		fixup_warp_and_empty(snap, pressures, i, 4);
	}

	if (i < n) {
		scalar_pressure_batch(&snap[i], &pressures[i], n - i);
	}
}
#endif /* CONFIG_WEAVER_BATCH_HYBRID */

#ifdef CONFIG_WEAVER_BATCH_MVE
/*
 * Full MVE path: true 4-wide Q16.16 multiply.
 *
 * a, b are uint32x4_t. We need (a * b) >> 16 per lane with the full
 * 32x32 -> 64 intermediate (the inputs span up to 32 bits, e.g.
 * WEAVER_TO_Q16(WAIT_TICKS_MAX) = 0xFFFF0000).
 *
 * MVE provides:
 *   vmullbq_int_u32(a, b) -> u64x2 from the EVEN-indexed u32 lanes
 *   vmulltq_int_u32(a, b) -> u64x2 from the ODD-indexed u32 lanes
 *   vshrnbq_n_u64(inactive, src, imm) shifts u64x2 right by imm and
 *                                     narrows to u32x4 even lanes
 *   vshrntq_n_u64(inactive, src, imm) shifts u64x2 right by imm and
 *                                     narrows to u32x4 odd lanes
 *
 * Result: 4 lanes of (a * b) >> 16 in 4 MVE instructions.
 */
static inline uint32x4_t mve_q16_mul(uint32x4_t a, uint32x4_t b)
{
	uint64x2_t lo = vmullbq_int_u32(a, b);
	uint64x2_t hi = vmulltq_int_u32(a, b);
	uint32x4_t out = vshrnbq_n_u64(vuninitializedq_u32(), lo,
				       WEAVER_Q16_SHIFT);
	out = vshrntq_n_u64(out, hi, WEAVER_Q16_SHIFT);
	return out;
}

static void mve_pressure_batch(const struct weaver_thread_data * const snap[],
			       uint32_t pressures[],
			       size_t n)
{
	const uint32x4_t v_w_u = vdupq_n_u32(weaver.w_urgency);
	const uint32x4_t v_w_d = vdupq_n_u32(weaver.w_density);
	const uint32x4_t v_w_a = vdupq_n_u32(weaver.w_aging);

	size_t i = 0;
	for (; i + 4 <= n; i += 4) {
		uint32_t prio[4], fill[4], aged[4];

		for (size_t j = 0; j < 4; j++) {
			gather_inputs(snap[i + j], &prio[j], &fill[j], &aged[j]);
		}

		uint32x4_t v_prio = vld1q_u32(prio);
		uint32x4_t v_fill = vld1q_u32(fill);
		uint32x4_t v_aged = vld1q_u32(aged);

		uint32x4_t v_pu = mve_q16_mul(v_prio, v_w_u);
		uint32x4_t v_pd = mve_q16_mul(v_fill, v_w_d);
		uint32x4_t v_pa = mve_q16_mul(v_aged, v_w_a);

		uint32x4_t v_sum = vqaddq_u32(vqaddq_u32(v_pu, v_pd), v_pa);
		vst1q_u32(&pressures[i], v_sum);

		fixup_warp_and_empty(snap, pressures, i, 4);
	}

	if (i < n) {
		scalar_pressure_batch(&snap[i], &pressures[i], n - i);
	}
}
#endif /* CONFIG_WEAVER_BATCH_MVE */

/*
 * Single dispatch entry. The selected variant is fixed at build time
 * by CONFIG_WEAVER_BATCH_*, so this function compiles to a direct call
 * with no runtime dispatch overhead.
 */
static void compute_pressure_batch(const struct weaver_thread_data * const snap[],
				   uint32_t pressures[],
				   size_t n)
{
#if   defined(CONFIG_WEAVER_BATCH_MVE)
	mve_pressure_batch(snap, pressures, n);
#elif defined(CONFIG_WEAVER_BATCH_HYBRID)
	hybrid_pressure_batch(snap, pressures, n);
#else
	scalar_pressure_batch(snap, pressures, n);
#endif
}

int weaver_register(struct weaver_thread_data *wd, struct k_thread *thread,
		    uint32_t priority_q16, bool is_warp)
{
	if (wd == NULL || thread == NULL) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&weaver.lock);

	for (size_t i = 0; i < ARRAY_SIZE(weaver.slots); i++) {
		if (weaver.slots[i] == NULL) {
			memset(wd, 0, sizeof(*wd));
			wd->thread = thread;
			wd->priority_q16 = priority_q16;
			wd->is_warp = is_warp ? 1U : 0U;
			wd->base_prio = (int8_t)k_thread_priority_get(thread);
			wd->boost_levels = CONFIG_WEAVER_DEFAULT_BOOST_LEVELS;
			wd->weft_boost_prio = clamp_boost(wd->base_prio, wd->boost_levels);
			wd->in_use = 1U;
			weaver.slots[i] = wd;
			k_spin_unlock(&weaver.lock, key);
			return 0;
		}
	}

	k_spin_unlock(&weaver.lock, key);
	return -ENOMEM;
}

int weaver_unregister(struct weaver_thread_data *wd)
{
	if (wd == NULL) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&weaver.lock);

	for (size_t i = 0; i < ARRAY_SIZE(weaver.slots); i++) {
		if (weaver.slots[i] == wd) {
			if (wd->thread != NULL) {
				k_thread_priority_set(wd->thread, wd->base_prio);
			}
			weaver.slots[i] = NULL;
			wd->in_use = 0U;
			k_spin_unlock(&weaver.lock, key);
			return 0;
		}
	}

	k_spin_unlock(&weaver.lock, key);
	return -ENOENT;
}

void weaver_set_buffer_fill_q16(struct weaver_thread_data *wd, uint32_t fill_q16)
{
	if (wd == NULL) {
		return;
	}
	if (fill_q16 > WEAVER_Q16_ONE) {
		fill_q16 = WEAVER_Q16_ONE;
	}
	wd->last_fill_q16 = wd->buffer_fill_q16;
	wd->buffer_fill_q16 = fill_q16;
}

void weaver_set_buffer_fill_predictive_q16(struct weaver_thread_data *wd,
					   uint32_t fill_q16)
{
	if (wd == NULL) {
		return;
	}
	if (fill_q16 > WEAVER_Q16_ONE) {
		fill_q16 = WEAVER_Q16_ONE;
	}

	uint32_t prev = wd->buffer_fill_q16;
	wd->last_fill_q16 = prev;

	/* One-step linear extrapolation: fill_next ~= fill + (fill - prev). */
	uint32_t projected;
	if (fill_q16 >= prev) {
		uint32_t delta = fill_q16 - prev;
		projected = (fill_q16 + delta < fill_q16) ? WEAVER_Q16_ONE
							  : fill_q16 + delta;
		if (projected > WEAVER_Q16_ONE) {
			projected = WEAVER_Q16_ONE;
		}
	} else {
		uint32_t delta = prev - fill_q16;
		projected = (fill_q16 > delta) ? fill_q16 - delta : 0;
	}

	wd->buffer_fill_q16 = projected;
}

void weaver_set_warp_deadline(struct weaver_thread_data *wd, uint32_t period_ticks)
{
	if (wd == NULL) {
		return;
	}
	wd->period_ticks = period_ticks;
	wd->next_deadline_ticks = period_ticks;
}

void weaver_set_boost_levels(struct weaver_thread_data *wd, uint8_t levels)
{
	if (wd == NULL || levels == 0) {
		return;
	}
	wd->boost_levels = levels;
	wd->weft_boost_prio = clamp_boost(wd->base_prio, levels);
}

void weaver_tick(void)
{
#ifdef CONFIG_WEAVER_TIMING
	uint32_t cyc_start = cyc_now();
#endif
	struct weaver_thread_data *winner = NULL;
	uint32_t max_pressure = 0;
	uint32_t sys_pressure = 0;
	uint32_t min_warp_remaining = UINT32_MAX;
	bool any_weft_pressure = false;

	struct weaver_thread_data *snap[CONFIG_WEAVER_MAX_THREADS];

	k_spinlock_key_t key = k_spin_lock(&weaver.lock);
	memcpy(snap, weaver.slots, sizeof(snap));
	k_spin_unlock(&weaver.lock, key);

	/* ---- Pass 1: aging + Warp deadline countdown.
	 *      Pure scalar, mutates wd state. Cheap (~3 ops per slot). ----
	 */
	struct k_thread *current = k_current_get();
	for (size_t i = 0; i < ARRAY_SIZE(snap); i++) {
		struct weaver_thread_data *wd = snap[i];

		if (wd == NULL || !wd->in_use) {
			continue;
		}

		if (wd->thread == current) {
			wd->wait_ticks = 0;
		} else if (wd->wait_ticks < WEAVER_WAIT_TICKS_MAX) {
			wd->wait_ticks++;
		}

		if (wd->is_warp && wd->period_ticks > 0) {
			if (wd->next_deadline_ticks == 0) {
				wd->next_deadline_ticks = wd->period_ticks;
			} else {
				wd->next_deadline_ticks--;
			}
			if (wd->next_deadline_ticks < min_warp_remaining) {
				min_warp_remaining = wd->next_deadline_ticks;
			}
		}
	}

	/* ---- Pass 2: batch pressure computation.
	 *      MVE-vectorized when CONFIG_WEAVER_USE_MVE=y, scalar otherwise.
	 *      Both paths produce identical results; tested in
	 *      test/weaver_mve_equiv.c on host. ----
	 */
	uint32_t pressures[CONFIG_WEAVER_MAX_THREADS];
	compute_pressure_batch(
		(const struct weaver_thread_data * const *)snap,
		pressures, ARRAY_SIZE(snap));

	/* ---- Pass 3: dispatch decisions.
	 *      Aggregate sys_pressure, find winning Weft, track activity. ----
	 */
	for (size_t i = 0; i < ARRAY_SIZE(snap); i++) {
		struct weaver_thread_data *wd = snap[i];
		uint32_t p = pressures[i];

		if (wd == NULL || !wd->in_use || wd->is_warp) {
			continue;
		}

		sys_pressure = saturate_add(sys_pressure, p);
		if (p > 0) {
			any_weft_pressure = true;
		}
		if (p > max_pressure) {
			max_pressure = p;
			winner = wd;
		}
	}

	weaver.system_pressure = sys_pressure;
	weaver.ticks_to_next_warp = min_warp_remaining;
	weaver.stats.total_ticks++;

	/* Fuzzy throttle: compute graded level, then 4-tap EMA smooth.
	 * Smoothed value drives weaver_should_throttle() for hysteresis.
	 */
	uint8_t level = fuzzy_throttle_level(sys_pressure);
	weaver.throttle_level_raw = level;
	weaver.throttle_level_smooth =
		(uint8_t)((3U * (uint32_t)weaver.throttle_level_smooth + level) >> 2);

	/* Count entry edges into throttle (smoothed-value rising past 128).
	 * EMA hysteresis means brief pressure spikes do not inflate this.
	 */
	{
		static uint8_t prev_above;
		uint8_t now_above = (weaver.throttle_level_smooth >= 128U) ? 1U : 0U;
		if (now_above && !prev_above) {
			weaver.stats.throttle_events++;
		}
		prev_above = now_above;
	}

#ifdef CONFIG_WEAVER_POWER_AWARE
	/* Skip priority churn when nothing has pressure: lets the system
	 * settle into deepsleep instead of bouncing thread states.
	 */
	if (!any_weft_pressure) {
		weaver.stats.skipped_idle_ticks++;
		return;
	}
#endif

	/* Pattern Slicing: if a Warp deadline is imminent, suppress the
	 * Weft promotion this pass and restore everyone to base priority
	 * so the Warp thread sees a quiet system.
	 */
	bool prewarp_clear =
		min_warp_remaining <= (uint32_t)CONFIG_WEAVER_PREWARP_GUARD_TICKS;

	if (prewarp_clear) {
		winner = NULL;
		weaver.stats.pre_warp_clears++;
	}

	for (size_t i = 0; i < ARRAY_SIZE(snap); i++) {
		struct weaver_thread_data *wd = snap[i];

		if (wd == NULL || !wd->in_use || wd->is_warp) {
			continue;
		}

		int8_t target = (wd == winner) ? wd->weft_boost_prio : wd->base_prio;

		if (k_thread_priority_get(wd->thread) != target) {
			k_thread_priority_set(wd->thread, target);
			if (wd == winner) {
				weaver.stats.weft_promotions++;
			}
		}
	}

#ifdef CONFIG_WEAVER_TIMING
	weaver.last_tick_cycles = cyc_now() - cyc_start;
#endif
}

uint32_t weaver_get_last_tick_cycles(void)
{
#ifdef CONFIG_WEAVER_TIMING
	return weaver.last_tick_cycles;
#else
	return 0;
#endif
}

uint32_t weaver_system_pressure(void)
{
	return weaver.system_pressure;
}

bool weaver_should_throttle(void)
{
	/* Backed by the smoothed fuzzy level so single-tick spikes do not
	 * flap throttle on/off near the threshold.
	 */
	return weaver.throttle_level_smooth >= 128U;
}

uint8_t weaver_throttle_level(void)
{
	return weaver.throttle_level_smooth;
}

uint32_t weaver_ticks_to_next_warp(void)
{
	return weaver.ticks_to_next_warp;
}

void weaver_get_stats(struct weaver_stats *out)
{
	if (out == NULL) {
		return;
	}
	k_spinlock_key_t key = k_spin_lock(&weaver.lock);
	*out = weaver.stats;
	k_spin_unlock(&weaver.lock, key);
}

void weaver_set_weights(uint32_t w_urgency_q16, uint32_t w_density_q16,
			uint32_t w_aging_q16)
{
	k_spinlock_key_t key = k_spin_lock(&weaver.lock);
	if (w_urgency_q16 != 0) {
		weaver.w_urgency = w_urgency_q16;
	}
	if (w_density_q16 != 0) {
		weaver.w_density = w_density_q16;
	}
	if (w_aging_q16 != 0) {
		weaver.w_aging = w_aging_q16;
	}
	k_spin_unlock(&weaver.lock, key);
}
