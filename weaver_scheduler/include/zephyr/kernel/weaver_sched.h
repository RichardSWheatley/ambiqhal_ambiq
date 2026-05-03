/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Weaver: a predictive, pressure-aware fixed-point scheduling layer.
 *
 * The Weaver scheduler treats threads as either:
 *   - "Warp" (hard real-time, deterministic, infinite pressure), or
 *   - "Weft" (opportunistic, pressure scales with buffer fill and aging).
 *
 * Each tick, Weaver scans its registered thread set, computes a Q16.16
 * "pressure" score per thread using only integer math (no FPU), and
 * promotes the highest-pressure Weft thread by elevating its Zephyr
 * priority. Warp threads always dominate.
 *
 * All math is integer-only (Q16.16). Designed to run in <15 us at 64 MHz
 * for a bounded thread set (CONFIG_WEAVER_MAX_THREADS).
 */

#ifndef ZEPHYR_INCLUDE_KERNEL_WEAVER_SCHED_H_
#define ZEPHYR_INCLUDE_KERNEL_WEAVER_SCHED_H_

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Q16.16 fixed-point format. */
#define WEAVER_Q16_SHIFT     16
#define WEAVER_Q16_ONE       (1U << WEAVER_Q16_SHIFT)
#define WEAVER_TO_Q16(x)     ((uint32_t)(x) << WEAVER_Q16_SHIFT)
#define WEAVER_Q16_MUL(a, b) (uint32_t)(((uint64_t)(a) * (uint64_t)(b)) >> WEAVER_Q16_SHIFT)

/** Saturating Warp pressure: always wins the dispatch tournament. */
#define WEAVER_PRESSURE_WARP 0xFFFFFFFFU

/**
 * Weights (Q16.16). Must roughly sum to ONE for normalized pressure.
 * Defaults: urgency 0.4, density 0.4, aging 0.2.
 */
#define WEAVER_W_URGENCY 0x00006666U
#define WEAVER_W_DENSITY 0x00006666U
#define WEAVER_W_AGING   0x00003333U

/**
 * @brief Per-thread scheduling state used by the Weaver dispatcher.
 *
 * The owner allocates this struct (typically file-static) and registers
 * it with weaver_register(). The Weaver tick handler updates wait_ticks
 * and reads the buffer_fill_q16 / is_warp fields.
 */
struct weaver_thread_data {
	struct k_thread *thread;       /**< Zephyr thread handle. */
	uint32_t priority_q16;         /**< Static "Warp priority" (Q16.16). */
	uint32_t buffer_fill_q16;      /**< Buffer fill ratio: 0..ONE. */
	uint32_t wait_ticks;           /**< Aging counter (incremented per tick). */
	uint8_t  is_warp;              /**< Non-zero: deterministic Warp thread. */
	int8_t   base_prio;            /**< Captured Zephyr base priority. */
	int8_t   weft_boost_prio;      /**< Priority used when promoted (Weft). */
	uint8_t  in_use;               /**< Slot occupancy flag. */
};

/**
 * @brief Compute Q16.16 pressure for a single thread.
 *
 * Pure function. Warp returns WEAVER_PRESSURE_WARP. Weft computes
 *   p_urgency = priority_q16 * W_URGENCY
 *   p_density = buffer_fill_q16 * W_DENSITY
 *   p_aging   = TO_Q16(wait_ticks) * W_AGING
 *   return    = p_urgency + p_density + p_aging
 *
 * Uses only addition, shifts, and one 32x32->64 multiply per term.
 *
 * @param t Registered thread data.
 * @return Q16.16 pressure (saturating at WEAVER_PRESSURE_WARP).
 */
uint32_t weaver_calculate_pressure(const struct weaver_thread_data *t);

/**
 * @brief Register a thread with the Weaver dispatcher.
 *
 * @param wd  Caller-owned, zero-initialized thread data slot.
 * @param thread Zephyr thread handle (must already be created).
 * @param priority_q16 Static Warp priority (Q16.16). Use WEAVER_TO_Q16(n).
 * @param is_warp Non-zero for hard real-time threads.
 *
 * @retval 0 on success.
 * @retval -ENOMEM if the registry is full.
 * @retval -EINVAL on bad arguments.
 */
int weaver_register(struct weaver_thread_data *wd, struct k_thread *thread,
		    uint32_t priority_q16, bool is_warp);

/** @brief Remove a thread from the Weaver registry. */
int weaver_unregister(struct weaver_thread_data *wd);

/**
 * @brief Update the buffer-fill metric driving informational pressure.
 *
 * Called by I/O subsystems whose backpressure should influence scheduling
 * (e.g. a logging buffer, network TX queue, sensor FIFO).
 *
 * @param wd Registered thread data.
 * @param fill_q16 Fill ratio in Q16.16 (0 = empty, WEAVER_Q16_ONE = full).
 */
void weaver_set_buffer_fill_q16(struct weaver_thread_data *wd, uint32_t fill_q16);

/**
 * @brief One Weaver scheduling pass.
 *
 * Iterates the registry (bounded by CONFIG_WEAVER_MAX_THREADS), computes
 * pressure, and elevates the highest-pressure Weft thread's Zephyr
 * priority. Increments wait_ticks for non-running threads. Safe to call
 * from a system work queue or a periodic timer; should NOT be called
 * from an ISR (uses k_thread_priority_set()).
 */
void weaver_tick(void);

/**
 * @brief Aggregate system pressure from the last weaver_tick().
 *
 * Sum of all Weft pressures, clamped to UINT32_MAX. Used by callers
 * (e.g. sensor polling drivers) to decide whether to throttle themselves.
 */
uint32_t weaver_system_pressure(void);

/**
 * @brief Returns true if system pressure has crossed the throttle threshold.
 *
 * When true, non-essential producers should slow down (e.g. drop sensor
 * polling rate) until the data fabric stabilizes.
 */
bool weaver_should_throttle(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_KERNEL_WEAVER_SCHED_H_ */
