/*
 * ARB Zephyr port - HAL binding helpers.
 *
 * Binds the portable ARB HAL device handles to Zephyr device-model drivers
 * (PWM, GPIO, sensor). This is the idiomatic path; for bit-exact parity with the
 * no-OS build you can instead reuse port/ambiqsuite/hal_bind.c against the same
 * Ambiq HAL (Zephyr does not own those peripherals in that configuration).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_ZEPHYR_HAL_BIND_H
#define ARB_ZEPHYR_HAL_BIND_H

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>

#include "arb/hal/motor.h"
#include "arb/hal/encoder.h"
#include "arb/hal/imu.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Zephyr motor backend context (PWM channel + optional DIR GPIO). */
typedef struct {
	struct pwm_dt_spec  pwm;
	struct gpio_dt_spec dir;
	bool                has_dir;
} arb_zephyr_motor_ctx_t;

/** @brief Zephyr encoder backend context (a qdec sensor device). */
typedef struct {
	const struct device *qdec;
} arb_zephyr_encoder_ctx_t;

/** @brief Zephyr IMU backend context (a 6-axis sensor device). */
typedef struct {
	const struct device *imu;
} arb_zephyr_imu_ctx_t;

/**
 * @brief Bind an ARB motor onto a Zephyr PWM (and optional DIR GPIO).
 * @param dir Pass NULL for single-direction drive.
 */
int arb_zephyr_motor_bind(arb_motor_t *m, arb_zephyr_motor_ctx_t *ctx,
			  const struct pwm_dt_spec *pwm,
			  const struct gpio_dt_spec *dir);

/**
 * @brief Bind an ARB encoder onto a Zephyr qdec sensor device.
 *
 * The qdec driver already accounts for the physical PPR, so this reports
 * rotation in centi-degrees (counts_per_rev fixed at 36000) and the core decode
 * yields correct radians/velocity.
 */
int arb_zephyr_encoder_bind(arb_encoder_t *e, arb_zephyr_encoder_ctx_t *ctx,
			    const struct device *qdec);

/**
 * @brief Bind an ARB IMU onto a Zephyr 6-axis sensor device.
 */
int arb_zephyr_imu_bind(arb_imu_t_inst *imu, arb_zephyr_imu_ctx_t *ctx,
			const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ARB_ZEPHYR_HAL_BIND_H */
