/*
 * ARB HAL abstraction - DC motor / actuator
 *
 * Robotics-oriented interface over a PWM + direction backend. The portable core
 * (core/src/hal/motor.c) handles clamping, inversion, deadband and unit scaling;
 * a port supplies an @ref arb_motor_ops_t that actually pokes the Ambiq HAL
 * (e.g. am_hal_timer_compare1_set() for PWM, am_hal_gpio_output_set() for DIR).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_HAL_MOTOR_H
#define ARB_HAL_MOTOR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Backend operations a port must provide for a motor channel.
 *
 * @c ctx is the per-instance backend context (see @ref arb_motor_t::ctx).
 */
typedef struct {
	/** Apply a signed Q15 duty (-32768..32767). Sign selects direction. */
	int (*set_duty)(void *ctx, int16_t duty_q15);
	/** Enable/disable the driver stage (optional, may be NULL). */
	int (*enable)(void *ctx, bool en);
	/** Short the motor terminals for active braking (optional, may be NULL). */
	int (*brake)(void *ctx);
} arb_motor_ops_t;

/** @brief Motor instance. Populate via arb_motor_init(); fields below are tunable. */
typedef struct {
	const arb_motor_ops_t *ops;
	void                  *ctx;
	int16_t  deadband_q15; /**< |duty| below this is treated as 0.  */
	int16_t  max_q15;      /**< Output magnitude clamp (<= 32767).  */
	bool     inverted;     /**< Flip sign of every command.         */
} arb_motor_t;

/**
 * @brief Bind a motor instance to a backend.
 *
 * Sets sane defaults: deadband 0, max 32767, not inverted.
 * @return ARB_OK or negative @ref arb_err_t.
 */
int arb_motor_init(arb_motor_t *m, const arb_motor_ops_t *ops, void *ctx);

/**
 * @brief Command duty as a normalized float in [-1.0, 1.0].
 */
int arb_motor_set_duty(arb_motor_t *m, float duty);

/**
 * @brief Command duty directly in signed Q15.
 */
int arb_motor_set_duty_q15(arb_motor_t *m, int16_t duty_q15);

/**
 * @brief Coast to stop (zero duty).
 */
int arb_motor_stop(arb_motor_t *m);

/**
 * @brief Active brake (falls back to stop if the backend has no brake op).
 */
int arb_motor_brake(arb_motor_t *m);

#ifdef __cplusplus
}
#endif

#endif /* ARB_HAL_MOTOR_H */
