/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Weaver scheduler: single-precision floating-point variant.
 *
 * Algorithmic mirror of weaver_sched.c with Q16.16 fixed-point math
 * replaced by IEEE-754 single-precision floats. See weaver_sched_fp.h
 * for the rationale on when to pick this variant vs. the fixed-point
 * one.
 */

#include <zephyr/kernel.h>
#include <zephyr/kernel/weaver_sched_fp.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <string.h>
#include <float.h>

#ifdef CONFIG_WEAVER_FP_TIMING
#include <cmsis_core.h>  /* DWT->CYCCNT */
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(weaver_sched_fp, CONFIG_WEAVER_FP_LOG_LEVEL);

#ifndef CONFIG_WEAVER_FP_PREWARP_GUARD_TICKS
#define CONFIG_WEAVER_FP_PREWARP_GUARD_TICKS 1
#endif

#ifndef CONFIG_WEAVER_FP_DEFAULT_BOOST_LEVELS
#define CONFIG_WEAVER_FP_DEFAULT_BOOST_LEVELS 1
#endif

/* Throttle threshold default: 4.0 normalized pressure units, matching
 * the 0x40000 Q16.16 default in the fixed-point variant. The wearable
 * preset tightens this to 3.0 (see Kconfig.weaver_fp).
 */
#ifndef CONFIG_WEAVER_FP_THROTTLE_THRESHOLD_X1000
#define CONFIG_WEAVER_FP_THROTTLE_THRESHOLD_X1000 4000
#endif

static struct {
	struct weaver_fp_thread_data *slots[CONFIG_WEAVER_FP_MAX_THREADS];
	float    system_pressure;
	uint32_t ticks_to_next_warp;
	float    w_urgency;
	float    w_density;
	float    w_aging;
	float    throttle_threshold;
	uint8_t  throttle_level_raw;
	uint8_t  throttle_level_smooth;
#ifdef CONFIG_WEAVER_FP_TIMING
	uint32_t last_tick_cycles;
#endif
	struct weaver_fp_stats stats;
	struct k_spinlock lock;
} weaver_fp = {
	.w_urgency = WEAVER_FP_W_URGENCY,
	.w_density = WEAVER_FP_W_DENSITY,
	.w_aging   = WEAVER_FP_W_AGING,
	.throttle_threshold =
		(float)CONFIG_WEAVER_FP_THROTTLE_THRESHOLD_X1000 / 1000.0f,
};

#ifdef CONFIG_WEAVER_FP_TIMING
static inline uint32_t cyc_now(void)
{
	return DWT->CYCCNT;
}
#endif

/*
 * 0-order TSK fuzzy throttle controller, float version.
 * Same three rules (LOW / MID / HIGH) symmetric about T.
 */
static inline uint8_t fuzzy_throttle_level(float pressure)
{
	const float T      = weaver_fp.throttle_threshold;
	const float T_low  = 0.5f * T;
	const float T_high = 1.5f * T;

	if (pressure <= T_low) {
		return 0;
	}
	if (pressure >= T_high) {
		return 255;
	}

	float level = 255.0f * (pressure - T_low) / T;
	if (level < 0.0f)   return 0;
	if (level > 255.0f) return 255;
	return (uint8_t)level;
}

static inline int8_t clamp_boost(int8_t base, uint8_t levels)
{
	int target = (int)base - (int)levels;
	if (target < 0) {
		target = (base > 0) ? 0 : base;
	}
	return (int8_t)target;
}

float weaver_fp_calculate_pressure(const struct weaver_fp_thread_data *t)
{
	if (t->is_warp) {
		return WEAVER_FP_PRESSURE_WARP;
	}

	float p_urgency = t->priority    * weaver_fp.w_urgency;
	float p_density = t->buffer_fill * weaver_fp.w_density;

	/* Aging scales LINEARLY with wait_ticks, clamped at 65535 ticks
	 * (matching WEAVER_WAIT_TICKS_MAX in the Q16.16 variant). This
	 * keeps the FP dispatch decision aligned with the fixed-point
	 * one - aging dominates over time so abandoned threads
	 * eventually win. Do NOT normalize to [0,1] here; that would
	 * change the algorithm's scale and the variants would disagree
	 * on which Weft wins.
	 */
	float wait = (t->wait_ticks > 65535U) ? 65535.0f : (float)t->wait_ticks;
	float p_aging = wait * weaver_fp.w_aging;

	return p_urgency + p_density + p_aging;
}

int weaver_fp_register(struct weaver_fp_thread_data *wd,
		       struct k_thread *thread,
		       float priority, bool is_warp)
{
	if (wd == NULL || thread == NULL) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&weaver_fp.lock);

	for (size_t i = 0; i < ARRAY_SIZE(weaver_fp.slots); i++) {
		if (weaver_fp.slots[i] == NULL) {
			memset(wd, 0, sizeof(*wd));
			wd->thread = thread;
			wd->priority = priority;
			wd->is_warp = is_warp ? 1U : 0U;
			wd->base_prio = (int8_t)k_thread_priority_get(thread);
			wd->boost_levels = CONFIG_WEAVER_FP_DEFAULT_BOOST_LEVELS;
			wd->weft_boost_prio = clamp_boost(wd->base_prio, wd->boost_levels);
			wd->in_use = 1U;
			weaver_fp.slots[i] = wd;
			k_spin_unlock(&weaver_fp.lock, key);
			return 0;
		}
	}

	k_spin_unlock(&weaver_fp.lock, key);
	return -ENOMEM;
}

int weaver_fp_unregister(struct weaver_fp_thread_data *wd)
{
	if (wd == NULL) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&weaver_fp.lock);

	for (size_t i = 0; i < ARRAY_SIZE(weaver_fp.slots); i++) {
		if (weaver_fp.slots[i] == wd) {
			if (wd->thread != NULL) {
				k_thread_priority_set(wd->thread, wd->base_prio);
			}
			weaver_fp.slots[i] = NULL;
			wd->in_use = 0U;
			k_spin_unlock(&weaver_fp.lock, key);
			return 0;
		}
	}

	k_spin_unlock(&weaver_fp.lock, key);
	return -ENOENT;
}

void weaver_fp_set_buffer_fill(struct weaver_fp_thread_data *wd, float fill)
{
	if (wd == NULL) {
		return;
	}
	if (fill < 0.0f) fill = 0.0f;
	if (fill > 1.0f) fill = 1.0f;
	wd->last_fill = wd->buffer_fill;
	wd->buffer_fill = fill;
}

void weaver_fp_set_buffer_fill_predictive(struct weaver_fp_thread_data *wd,
					  float fill)
{
	if (wd == NULL) {
		return;
	}
	if (fill < 0.0f) fill = 0.0f;
	if (fill > 1.0f) fill = 1.0f;

	float prev = wd->buffer_fill;
	wd->last_fill = prev;

	float projected = fill + (fill - prev);
	if (projected < 0.0f) projected = 0.0f;
	if (projected > 1.0f) projected = 1.0f;
	wd->buffer_fill = projected;
}

void weaver_fp_set_warp_deadline(struct weaver_fp_thread_data *wd,
				 uint32_t period_ticks)
{
	if (wd == NULL) {
		return;
	}
	wd->period_ticks = period_ticks;
	wd->next_deadline_ticks = period_ticks;
}

void weaver_fp_set_boost_levels(struct weaver_fp_thread_data *wd, uint8_t levels)
{
	if (wd == NULL || levels == 0) {
		return;
	}
	wd->boost_levels = levels;
	wd->weft_boost_prio = clamp_boost(wd->base_prio, levels);
}

void weaver_fp_set_weights(float w_urgency, float w_density, float w_aging)
{
	k_spinlock_key_t key = k_spin_lock(&weaver_fp.lock);
	if (w_urgency > 0.0f) weaver_fp.w_urgency = w_urgency;
	if (w_density > 0.0f) weaver_fp.w_density = w_density;
	if (w_aging   > 0.0f) weaver_fp.w_aging   = w_aging;
	k_spin_unlock(&weaver_fp.lock, key);
}

void weaver_fp_tick(void)
{
#ifdef CONFIG_WEAVER_FP_TIMING
	uint32_t cyc_start = cyc_now();
#endif
	struct weaver_fp_thread_data *winner = NULL;
	float max_pressure = 0.0f;
	float sys_pressure = 0.0f;
	uint32_t min_warp_remaining = UINT32_MAX;
	bool any_weft_pressure = false;

	struct weaver_fp_thread_data *snap[CONFIG_WEAVER_FP_MAX_THREADS];

	k_spinlock_key_t key = k_spin_lock(&weaver_fp.lock);
	memcpy(snap, weaver_fp.slots, sizeof(snap));
	k_spin_unlock(&weaver_fp.lock, key);

	/* Pass 1: aging + Warp deadline countdown. */
	struct k_thread *current = k_current_get();
	for (size_t i = 0; i < ARRAY_SIZE(snap); i++) {
		struct weaver_fp_thread_data *wd = snap[i];
		if (wd == NULL || !wd->in_use) continue;

		if (wd->thread == current) {
			wd->wait_ticks = 0;
		} else if (wd->wait_ticks < 65535U) {
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

	/* Pass 2: pressure + dispatch decisions. */
	for (size_t i = 0; i < ARRAY_SIZE(snap); i++) {
		struct weaver_fp_thread_data *wd = snap[i];
		if (wd == NULL || !wd->in_use || wd->is_warp) continue;

		float p = weaver_fp_calculate_pressure(wd);
		sys_pressure += p;
		if (p > 0.0f) any_weft_pressure = true;
		if (p > max_pressure) {
			max_pressure = p;
			winner = wd;
		}
	}

	weaver_fp.system_pressure = sys_pressure;
	weaver_fp.ticks_to_next_warp = min_warp_remaining;
	weaver_fp.stats.total_ticks++;

	uint8_t level = fuzzy_throttle_level(sys_pressure);
	weaver_fp.throttle_level_raw = level;
	weaver_fp.throttle_level_smooth =
		(uint8_t)((3U * (uint32_t)weaver_fp.throttle_level_smooth + level) >> 2);

	{
		static uint8_t prev_above;
		uint8_t now_above = (weaver_fp.throttle_level_smooth >= 128U) ? 1U : 0U;
		if (now_above && !prev_above) {
			weaver_fp.stats.throttle_events++;
		}
		prev_above = now_above;
	}

#ifdef CONFIG_WEAVER_FP_POWER_AWARE
	if (!any_weft_pressure) {
		weaver_fp.stats.skipped_idle_ticks++;
#ifdef CONFIG_WEAVER_FP_TIMING
		weaver_fp.last_tick_cycles = cyc_now() - cyc_start;
#endif
		return;
	}
#endif

	bool prewarp_clear =
		min_warp_remaining <= (uint32_t)CONFIG_WEAVER_FP_PREWARP_GUARD_TICKS;

	if (prewarp_clear) {
		winner = NULL;
		weaver_fp.stats.pre_warp_clears++;
	}

	for (size_t i = 0; i < ARRAY_SIZE(snap); i++) {
		struct weaver_fp_thread_data *wd = snap[i];
		if (wd == NULL || !wd->in_use || wd->is_warp) continue;

		int8_t target = (wd == winner) ? wd->weft_boost_prio : wd->base_prio;
		if (k_thread_priority_get(wd->thread) != target) {
			k_thread_priority_set(wd->thread, target);
			if (wd == winner) {
				weaver_fp.stats.weft_promotions++;
			}
		}
	}

#ifdef CONFIG_WEAVER_FP_TIMING
	weaver_fp.last_tick_cycles = cyc_now() - cyc_start;
#endif
}

float weaver_fp_system_pressure(void)
{
	return weaver_fp.system_pressure;
}

uint32_t weaver_fp_ticks_to_next_warp(void)
{
	return weaver_fp.ticks_to_next_warp;
}

bool weaver_fp_should_throttle(void)
{
	return weaver_fp.throttle_level_smooth >= 128U;
}

uint8_t weaver_fp_throttle_level(void)
{
	return weaver_fp.throttle_level_smooth;
}

void weaver_fp_get_stats(struct weaver_fp_stats *out)
{
	if (out == NULL) return;
	k_spinlock_key_t key = k_spin_lock(&weaver_fp.lock);
	*out = weaver_fp.stats;
	k_spin_unlock(&weaver_fp.lock, key);
}

uint32_t weaver_fp_get_last_tick_cycles(void)
{
#ifdef CONFIG_WEAVER_FP_TIMING
	return weaver_fp.last_tick_cycles;
#else
	return 0;
#endif
}
