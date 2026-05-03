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

static struct {
	struct weaver_thread_data *slots[CONFIG_WEAVER_MAX_THREADS];
	uint32_t system_pressure;
	struct k_spinlock lock;
} weaver;

static inline uint32_t saturate_add(uint32_t a, uint32_t b)
{
	uint32_t s = a + b;
	return (s < a) ? UINT32_MAX : s;
}

uint32_t weaver_calculate_pressure(const struct weaver_thread_data *t)
{
	if (t->is_warp) {
		return WEAVER_PRESSURE_WARP;
	}

	/* Q16.16 multiply: (A * B) >> 16, done in 64-bit to avoid overflow. */
	uint32_t p_urgency = WEAVER_Q16_MUL(t->priority_q16, WEAVER_W_URGENCY);
	uint32_t p_density = WEAVER_Q16_MUL(t->buffer_fill_q16, WEAVER_W_DENSITY);

	uint32_t wait = MIN(t->wait_ticks, WEAVER_WAIT_TICKS_MAX);
	uint32_t p_aging = WEAVER_Q16_MUL(WEAVER_TO_Q16(wait), WEAVER_W_AGING);

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
			/* Boost slot for promoted Weft: one level above base,
			 * but never crossing into cooperative space.
			 */
			wd->weft_boost_prio =
				(wd->base_prio > 0) ? (int8_t)(wd->base_prio - 1)
						    : wd->base_prio;
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
			/* Restore base priority before releasing the slot. */
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
	/* Single 32-bit store; safe without locking on all Zephyr-supported archs. */
	wd->buffer_fill_q16 = fill_q16;
}

void weaver_tick(void)
{
	struct weaver_thread_data *winner = NULL;
	uint32_t max_pressure = 0;
	uint32_t sys_pressure = 0;

	/* Snapshot under lock; pressure math runs lock-free. The slot array
	 * is fixed size and registration is rare, so a brief lock is cheap.
	 */
	struct weaver_thread_data *snap[CONFIG_WEAVER_MAX_THREADS];

	k_spinlock_key_t key = k_spin_lock(&weaver.lock);
	memcpy(snap, weaver.slots, sizeof(snap));
	k_spin_unlock(&weaver.lock, key);

	for (size_t i = 0; i < ARRAY_SIZE(snap); i++) {
		struct weaver_thread_data *wd = snap[i];

		if (wd == NULL || !wd->in_use) {
			continue;
		}

		/* Age non-running threads. The current thread (if registered)
		 * already had its turn this tick, so reset its aging to
		 * prevent runaway boosting.
		 */
		if (wd->thread == k_current_get()) {
			wd->wait_ticks = 0;
		} else if (wd->wait_ticks < WEAVER_WAIT_TICKS_MAX) {
			wd->wait_ticks++;
		}

		uint32_t p = weaver_calculate_pressure(wd);

		if (!wd->is_warp) {
			sys_pressure = saturate_add(sys_pressure, p);
			if (p > max_pressure) {
				max_pressure = p;
				winner = wd;
			}
		}
	}

	weaver.system_pressure = sys_pressure;

	/* Promote the highest-pressure Weft thread; restore others to base.
	 * Warp threads are never re-prioritized.
	 */
	for (size_t i = 0; i < ARRAY_SIZE(snap); i++) {
		struct weaver_thread_data *wd = snap[i];

		if (wd == NULL || !wd->in_use || wd->is_warp) {
			continue;
		}

		int8_t target = (wd == winner) ? wd->weft_boost_prio : wd->base_prio;

		if (k_thread_priority_get(wd->thread) != target) {
			k_thread_priority_set(wd->thread, target);
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
