/*
 * ARB AmbiqSuite port - HAL binding helpers.
 *
 * Concrete constructors that wire the portable ARB HAL device handles to Apollo
 * peripherals via direct AmbiqSuite HAL calls. Include from no-OS samples.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_AMBIQ_HAL_BIND_H
#define ARB_AMBIQ_HAL_BIND_H

#include <stdint.h>
#include <stdbool.h>
#include "arb/hal/motor.h"
#include "arb/hal/encoder.h"
#include "arb/hal/imu.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Per-motor backend context (a TIMER in PWM mode + optional DIR pin). */
typedef struct {
	uint32_t timer;     /**< Ambiq TIMER instance number.            */
	uint32_t out_pad;   /**< GPIO pad driven by the timer PWM output.*/
	uint32_t period;    /**< PWM period in timer counts.             */
	uint32_t dir_pin;   /**< DIR GPIO (valid when has_dir is true).  */
	bool     has_dir;   /**< If false, only forward duty is applied. */
} arb_ambiq_motor_ctx_t;

/** @brief Per-encoder backend context (a TIMER configured as an event counter).*/
typedef struct {
	uint32_t timer;     /**< Ambiq TIMER instance used as counter. */
} arb_ambiq_encoder_ctx_t;

/** @brief IMU backend context over an IOM (I2C) bus. */
typedef struct {
	void    *iom_handle; /**< Handle from am_hal_iom_initialize().    */
	uint16_t i2c_addr;   /**< 7-bit device address.                   */
	float    accel_lsb;  /**< (m/s^2) per LSB.                        */
	float    gyro_lsb;   /**< (rad/s) per LSB.                        */
} arb_ambiq_imu_ctx_t;

/**
 * @brief Configure a TIMER for PWM and bind it as an ARB motor.
 *
 * @param m       Motor handle to initialize.
 * @param ctx     Caller-owned context, filled in by this call.
 * @param timer   TIMER instance to use.
 * @param out_pad GPIO pad for the PWM output.
 * @param dir_pin DIR GPIO, or 0xFFFFFFFF for single-direction drive.
 * @param pwm_hz  Desired PWM frequency.
 * @return ARB_OK or negative @ref arb_err_t.
 */
int arb_ambiq_motor_bind(arb_motor_t *m, arb_ambiq_motor_ctx_t *ctx,
			 uint32_t timer, uint32_t out_pad, uint32_t dir_pin,
			 uint32_t pwm_hz);

/**
 * @brief Configure a TIMER as an event counter and bind it as an ARB encoder.
 */
int arb_ambiq_encoder_bind(arb_encoder_t *e, arb_ambiq_encoder_ctx_t *ctx,
			   uint32_t timer, int32_t counts_per_rev);

/**
 * @brief Bind an I2C IMU (ICM-class) onto an already-initialized IOM handle.
 */
int arb_ambiq_imu_bind(arb_imu_t_inst *imu, arb_ambiq_imu_ctx_t *ctx,
		       void *iom_handle, uint16_t i2c_addr);

#ifdef __cplusplus
}
#endif

#endif /* ARB_AMBIQ_HAL_BIND_H */
