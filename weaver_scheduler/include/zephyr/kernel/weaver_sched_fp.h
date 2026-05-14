/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Weaver scheduler: single-precision floating-point variant.
 *
 * Parallel implementation of the Weaver predictive pressure-aware
 * scheduling layer using IEEE-754 single-precision floats on the
 * Cortex-M55 FPv5-SP FPU. Same dispatch semantics, same Warp/Weft
 * classification, same fuzzy-throttle and predictive-fill helpers as
 * the fixed-point variant - just float math instead of Q16.16.
 *
 * When to pick this variant over the fixed-point one
 * --------------------------------------------------
 *
 *   - Your Weft threads ALREADY use the FPU (sensor fusion with
 *     Mahony/Madgwick, HR FFT, audio frame-by-frame DSP, graphics
 *     alpha-blend). The lazy floating-point context-save tax is
 *     already paid by those threads; adding FPU to the scheduler
 *     costs essentially zero extra at preemption time.
 *
 *   - You want a code path that is easier to audit and modify than
 *     the Q16.16 fixed-point one. No (uint64_t) intermediate casts,
 *     no Q-format reasoning, no saturating-add helpers - just
 *     readable math.
 *
 *   - Apollo510 is in HP mode (192/250 MHz) where the absolute cost
 *     of FP is dwarfed by the available cycle budget.
 *
 * When to stay with the fixed-point variant
 * -----------------------------------------
 *
 *   - Battery-constrained wearable in LP mode (96 MHz) where NO
 *     other thread uses the FPU. The integer path keeps lazy
 *     stacking disarmed, saving ~17 cycles per preemption.
 *
 *   - MCUs without an FPU at all (Cortex-M0/M0+, M3, some M4-NOFP).
 *
 *   - WCET-certified workloads where bit-identical numeric
 *     reproducibility across compilers/versions is required.
 *
 * CONFIG_WEAVER_SCHED_FP is mutually exclusive with
 * CONFIG_WEAVER_SCHED. Pick one per build.
 */

#ifndef ZEPHYR_INCLUDE_KERNEL_WEAVER_SCHED_FP_H_
#define ZEPHYR_INCLUDE_KERNEL_WEAVER_SCHED_FP_H_

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Saturating Warp pressure: always wins the dispatch tournament. */
#define WEAVER_FP_PRESSURE_WARP   3.4028235e38f  /* FLT_MAX */

/**
 * Default weights (fractions). Must roughly sum to 1.0 for normalized
 * pressure. Defaults: urgency 0.4, density 0.4, aging 0.2 - same
 * apportionment as the fixed-point variant for identical dispatch
 * decisions (modulo float rounding).
 */
#define WEAVER_FP_W_URGENCY 0.4f
#define WEAVER_FP_W_DENSITY 0.4f
#define WEAVER_FP_W_AGING   0.2f

/**
 * @brief Per-thread scheduling state used by the FP Weaver dispatcher.
 *
 * Mirrors weaver_thread_data from the fixed-point variant with the
 * Q16.16 fields replaced by `float`. `wait_ticks` stays integer
 * because it's a counter, not a fraction.
 */
struct weaver_fp_thread_data {
	struct k_thread *thread;       /**< Zephyr thread handle. */
	float    priority;             /**< Static "Warp priority" (e.g. 1.0..20.0). */
	float    buffer_fill;          /**< Buffer fill ratio: 0.0..1.0. */
	float    last_fill;            /**< Previous fill (for predictive delta). */
	uint32_t wait_ticks;           /**< Aging counter (incremented per tick). */
	uint32_t period_ticks;         /**< Warp period (0 if Weft or aperiodic). */
	uint32_t next_deadline_ticks;  /**< Ticks remaining to next Warp run. */
	void    *meta;                 /**< Owner-defined: e.g. sensor FIFO ptr. */
	uint8_t  is_warp;              /**< Non-zero: deterministic Warp thread. */
	int8_t   base_prio;            /**< Captured Zephyr base priority. */
	int8_t   weft_boost_prio;      /**< Priority used when promoted. */
	uint8_t  boost_levels;         /**< How many priority levels to lift. */
	uint8_t  in_use;               /**< Slot occupancy flag. */
};

/**
 * @brief Compute pressure for a single thread (float).
 *
 * Warp returns WEAVER_FP_PRESSURE_WARP. Weft computes
 *   pressure = priority * W_URGENCY
 *            + buffer_fill * W_DENSITY
 *            + min(wait_ticks, 1.0) * W_AGING
 *
 * Same algebraic structure as the fixed-point variant, so dispatch
 * decisions are equivalent within float rounding.
 */
float weaver_fp_calculate_pressure(const struct weaver_fp_thread_data *t);

/**
 * @brief Register a thread with the FP Weaver dispatcher.
 *
 * @param wd       Caller-owned zero-initialized thread data slot.
 * @param thread   Zephyr thread handle (must already be created).
 * @param priority Static Warp priority (e.g. 1.0..20.0).
 * @param is_warp  Non-zero for hard real-time threads.
 *
 * @retval 0       on success.
 * @retval -ENOMEM if the registry is full.
 * @retval -EINVAL on bad arguments.
 */
int weaver_fp_register(struct weaver_fp_thread_data *wd,
		       struct k_thread *thread,
		       float priority, bool is_warp);

/** @brief Remove a thread from the FP registry. */
int weaver_fp_unregister(struct weaver_fp_thread_data *wd);

/** @brief Update buffer-fill metric. */
void weaver_fp_set_buffer_fill(struct weaver_fp_thread_data *wd, float fill);

/**
 * @brief Update buffer fill with one-step predictive extrapolation.
 *
 * Stores fill + (fill - last_fill), clamped to [0.0, 1.0]. Identical
 * predictive semantic to the fixed-point variant.
 */
void weaver_fp_set_buffer_fill_predictive(struct weaver_fp_thread_data *wd,
					  float fill);

/** @brief Configure a periodic Warp deadline (ticks). */
void weaver_fp_set_warp_deadline(struct weaver_fp_thread_data *wd,
				 uint32_t period_ticks);

/** @brief Override the boost magnitude (priority levels) for a Weft. */
void weaver_fp_set_boost_levels(struct weaver_fp_thread_data *wd, uint8_t levels);

/** @brief Override the global pressure weights at runtime. */
void weaver_fp_set_weights(float w_urgency, float w_density, float w_aging);

/** @brief One scheduling pass. */
void weaver_fp_tick(void);

/** @brief Sum of all Weft pressures from the last tick. */
float weaver_fp_system_pressure(void);

/** @brief Ticks remaining until the next Warp deadline across the registry. */
uint32_t weaver_fp_ticks_to_next_warp(void);

/** @brief True if smoothed throttle level is at or above mid (graceful). */
bool weaver_fp_should_throttle(void);

/** @brief Graded throttle level (0..255), TSK fuzzy + EMA smoothed. */
uint8_t weaver_fp_throttle_level(void);

struct weaver_fp_stats {
	uint32_t total_ticks;
	uint32_t throttle_events;
	uint32_t weft_promotions;
	uint32_t pre_warp_clears;
	uint32_t skipped_idle_ticks;
};

void weaver_fp_get_stats(struct weaver_fp_stats *out);

/** @brief DWT cycle count of the most recent weaver_fp_tick(). */
uint32_t weaver_fp_get_last_tick_cycles(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_KERNEL_WEAVER_SCHED_FP_H_ */
