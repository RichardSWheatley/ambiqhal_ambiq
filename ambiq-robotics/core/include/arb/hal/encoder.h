/*
 * ARB HAL abstraction - quadrature / incremental encoder
 *
 * The portable core (core/src/hal/encoder.c) turns a raw signed tick count into
 * accumulated angle and angular velocity, differentiating against the broker
 * timebase. A port supplies an @ref arb_encoder_ops_t backed by a hardware
 * counter (e.g. an Ambiq TIMER in event/quadrature mode) or GPIO edge counting.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_HAL_ENCODER_H
#define ARB_HAL_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Backend operations a port must provide for an encoder channel. */
typedef struct {
	/** Read the current raw signed tick count. */
	int (*read_count)(void *ctx, int32_t *count);
	/** Zero the hardware counter (optional, may be NULL). */
	int (*reset)(void *ctx);
} arb_encoder_ops_t;

/** @brief Encoder instance. Populate via arb_encoder_init(). */
typedef struct {
	const arb_encoder_ops_t *ops;
	void     *ctx;
	int32_t   counts_per_rev; /**< Ticks for one full output revolution. */
	bool      inverted;       /**< Flip counting direction.              */

	/* internal differentiation state */
	int32_t   last_count;
	uint64_t  last_ts_us;
	float     position_rad;
	bool      primed;
} arb_encoder_t;

/** @brief One decoded encoder reading. */
typedef struct {
	int32_t count;          /**< raw signed ticks (direction-corrected) */
	float   position_rad;   /**< accumulated angle, rad                 */
	float   velocity_rad_s; /**< angular velocity, rad/s                */
} arb_encoder_reading_t;

/**
 * @brief Bind an encoder instance to a backend.
 * @param counts_per_rev Encoder ticks per output revolution (>0).
 * @return ARB_OK or negative @ref arb_err_t.
 */
int arb_encoder_init(arb_encoder_t *e, const arb_encoder_ops_t *ops, void *ctx,
		     int32_t counts_per_rev);

/**
 * @brief Sample the encoder and compute position + velocity.
 *
 * Velocity is derived from the delta since the previous call, so call at a
 * roughly regular rate. The first call only primes the state (velocity 0).
 *
 * @return ARB_OK or negative @ref arb_err_t.
 */
int arb_encoder_read(arb_encoder_t *e, arb_encoder_reading_t *out);

/**
 * @brief Reset accumulated angle and (if supported) the hardware counter.
 */
int arb_encoder_reset(arb_encoder_t *e);

#ifdef __cplusplus
}
#endif

#endif /* ARB_HAL_ENCODER_H */
