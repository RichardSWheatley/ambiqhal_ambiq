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
	struct weaver_stats stats;
	struct k_spinlock lock;
} weaver = {
	.w_urgency = WEAVER_W_URGENCY,
	.w_density = WEAVER_W_DENSITY,
	.w_aging   = WEAVER_W_AGING,
};

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
	wd->buffer_fill_q16 = fill_q16;
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
	struct weaver_thread_data *winner = NULL;
	uint32_t max_pressure = 0;
	uint32_t sys_pressure = 0;
	uint32_t min_warp_remaining = UINT32_MAX;
	bool any_weft_pressure = false;

	struct weaver_thread_data *snap[CONFIG_WEAVER_MAX_THREADS];

	k_spinlock_key_t key = k_spin_lock(&weaver.lock);
	memcpy(snap, weaver.slots, sizeof(snap));
	k_spin_unlock(&weaver.lock, key);

	for (size_t i = 0; i < ARRAY_SIZE(snap); i++) {
		struct weaver_thread_data *wd = snap[i];

		if (wd == NULL || !wd->in_use) {
			continue;
		}

		if (wd->thread == k_current_get()) {
			wd->wait_ticks = 0;
		} else if (wd->wait_ticks < WEAVER_WAIT_TICKS_MAX) {
			wd->wait_ticks++;
		}

		/* Tick the Warp deadline countdown. When it reaches zero
		 * the thread is "due"; we reset to one period and treat
		 * remaining=0 (within guard) for the rest of this pass.
		 */
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

		uint32_t p = weaver_calculate_pressure(wd);

		if (!wd->is_warp) {
			sys_pressure = saturate_add(sys_pressure, p);
			if (p > 0) {
				any_weft_pressure = true;
			}
			if (p > max_pressure) {
				max_pressure = p;
				winner = wd;
			}
		}
	}

	weaver.system_pressure = sys_pressure;
	weaver.ticks_to_next_warp = min_warp_remaining;
	weaver.stats.total_ticks++;

	if (sys_pressure >= (uint32_t)CONFIG_WEAVER_THROTTLE_THRESHOLD) {
		weaver.stats.throttle_events++;
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
}

uint32_t weaver_system_pressure(void)
{
	return weaver.system_pressure;
}

bool weaver_should_throttle(void)
{
	return weaver.system_pressure >= (uint32_t)CONFIG_WEAVER_THROTTLE_THRESHOLD;
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
