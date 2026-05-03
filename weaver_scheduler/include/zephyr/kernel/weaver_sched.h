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
	uint32_t last_fill_q16;        /**< Previous fill (for predictive delta). */
	uint32_t wait_ticks;           /**< Aging counter (incremented per tick). */
	uint32_t period_ticks;         /**< Warp period (0 if Weft or aperiodic). */
	uint32_t next_deadline_ticks;  /**< Ticks remaining to next Warp run. */
	void    *meta;                 /**< Owner-defined: e.g. sensor FIFO ptr. */
	uint8_t  is_warp;              /**< Non-zero: deterministic Warp thread. */
	int8_t   base_prio;            /**< Captured Zephyr base priority. */
	int8_t   weft_boost_prio;      /**< Priority used when promoted (Weft). */
	uint8_t  boost_levels;         /**< How many priority levels to lift. */
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
 * @brief Configure a periodic Warp deadline for a thread.
 *
 * Used by sensor sampling threads (IMU, PPG) and BLE radio handlers
 * whose work recurs at a known cadence. The dispatcher uses this to
 * detect "Warp gaps" (ticks until the next deadline) and to throttle
 * Weft work that risks overlapping a sub-millisecond deadline.
 *
 * @param wd Registered thread data.
 * @param period_ticks Cadence in Weaver ticks. 0 disables the deadline.
 */
void weaver_set_warp_deadline(struct weaver_thread_data *wd, uint32_t period_ticks);

/**
 * @brief Override the boost magnitude (priority levels) for a Weft thread.
 *
 * When promoted, the thread's Zephyr priority drops by this many levels
 * (lower numeric value = higher priority). Default is 1; wearable
 * presets use 2 to give sensor-fusion threads a stronger advantage
 * over background work. Capped so the boosted priority never crosses
 * into cooperative space.
 */
void weaver_set_boost_levels(struct weaver_thread_data *wd, uint8_t levels);

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
 * @brief Update buffer fill with one-step predictive extrapolation.
 *
 * Equivalent to weaver_set_buffer_fill_q16(wd, fill_q16) but stores
 * (fill_q16 + (fill_q16 - last_fill_q16)) so the scheduler reacts to
 * where the FIFO will be next tick rather than where it is now. This
 * is the "Predictive" half of "Predictive and Pressure-Aware" without
 * any non-deterministic ML in the hot path: 2 subtractions, 1 add,
 * 1 saturation, ~6 cycles on Cortex-M55.
 *
 * Producers (sensor drivers, GATT TX) call this on every FIFO update.
 */
void weaver_set_buffer_fill_predictive_q16(struct weaver_thread_data *wd,
					   uint32_t fill_q16);

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
 *
 * Backed by the smoothed fuzzy throttle level: stable across single-tick
 * pressure spikes near the threshold (no flap).
 */
bool weaver_should_throttle(void);

/**
 * @brief Graded throttle level (0..255) from a TSK fuzzy controller.
 *
 * Three rules, linear membership, EMA-smoothed across ticks:
 *   p <= T/2  -> 0     (no throttle)
 *   p ~  T    -> 128   (entering throttle region)
 *   p >= 2T   -> 255   (full throttle)
 *
 * where T is CONFIG_WEAVER_THROTTLE_THRESHOLD. Producers that can do
 * graceful degradation (drop sensor rate by N%) should consume this
 * value directly instead of the binary weaver_should_throttle().
 *
 * Recomputed once per weaver_tick(); reading is a single byte load.
 */
uint8_t weaver_throttle_level(void);

/**
 * @brief Ticks remaining until the next Warp deadline across the registry.
 *
 * Returns UINT32_MAX if no Warp threads have a deadline configured.
 * Producers can use this to defer non-essential work that would extend
 * past the next radio or sensor window (the "Pattern Slicing" strategy).
 */
uint32_t weaver_ticks_to_next_warp(void);

/**
 * @brief Aggregate Weaver runtime statistics ("Fabric View").
 *
 * Tracks per-system counters useful for tuning. Reset on system start.
 */
struct weaver_stats {
	uint32_t total_ticks;          /**< weaver_tick() invocation count. */
	uint32_t throttle_events;      /**< Times throttle threshold crossed. */
	uint32_t weft_promotions;      /**< Total Weft priority elevations. */
	uint32_t pre_warp_clears;      /**< "Pattern Slicing" clears performed. */
	uint32_t skipped_idle_ticks;   /**< Power-aware ticks bypassed. */
};

/** @brief Snapshot the Weaver stats counters. */
void weaver_get_stats(struct weaver_stats *out);

/**
 * @brief Cycles spent inside the most recent weaver_tick().
 *
 * Available only when CONFIG_WEAVER_TIMING=y (requires CPU_HAS_DWT).
 * Used for on-target WCET validation on Apollo510 and other M55 / M7
 * targets. Returns 0 if the counter has not been sampled yet.
 */
uint32_t weaver_get_last_tick_cycles(void);

/**
 * @brief Override the global pressure weights at runtime.
 *
 * Defaults are WEAVER_W_URGENCY / W_DENSITY / W_AGING. Exposed so an
 * external controller (e.g. a workload-classifier task or a fuzzy
 * inference engine) can adapt weights without recompiling. All three
 * are Q16.16 and SHOULD sum to roughly WEAVER_Q16_ONE for normalized
 * pressure, but the dispatcher does not enforce that.
 *
 * Pass 0 to a weight to leave it unchanged.
 */
void weaver_set_weights(uint32_t w_urgency_q16, uint32_t w_density_q16,
			uint32_t w_aging_q16);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_KERNEL_WEAVER_SCHED_H_ */
