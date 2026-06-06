/*
 * ARB HAL abstraction - inertial measurement unit
 *
 * Bus-agnostic IMU interface. The portable core (core/src/hal/imu.c) applies
 * axis remapping and sign conventions and packs results into an arb_imu_t
 * message. A port supplies an @ref arb_imu_ops_t that talks to a specific device
 * over the Ambiq IOM (I2C/SPI) - e.g. an ICM-class 6-axis sensor.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_HAL_IMU_H
#define ARB_HAL_IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "arb/msg.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Raw, SI-unit sample produced by an IMU backend. */
typedef struct {
	arb_vec3_t accel;       /**< m/s^2 */
	arb_vec3_t gyro;        /**< rad/s */
	arb_vec3_t mag;         /**< uT (zero if device has no magnetometer) */
	float      temperature; /**< deg C */
	bool       has_mag;
} arb_imu_sample_t;

/** @brief Backend operations a port must provide for an IMU. */
typedef struct {
	/** Probe/configure the device. Returns ARB_OK when ready. */
	int (*start)(void *ctx);
	/** Read one fully-scaled sample in SI units. */
	int (*read)(void *ctx, arb_imu_sample_t *out);
} arb_imu_ops_t;

/**
 * @brief Axis remap entry: source axis index (0=x,1=y,2=z) and sign (+1/-1).
 */
typedef struct {
	uint8_t src;  /**< 0..2 */
	int8_t  sign; /**< +1 or -1 */
} arb_axis_map_t;

/** @brief IMU instance. Populate via arb_imu_init(). */
typedef struct {
	const arb_imu_ops_t *ops;
	void  *ctx;
	arb_axis_map_t remap[3]; /**< Maps device axes -> robot body frame. */
} arb_imu_t_inst; /* note: distinct from arb_imu_t message type */

/**
 * @brief Bind an IMU instance to a backend (identity axis mapping).
 * @return ARB_OK or negative @ref arb_err_t.
 */
int arb_imu_init(arb_imu_t_inst *imu, const arb_imu_ops_t *ops, void *ctx);

/**
 * @brief Override the device->body axis remapping.
 * @param remap Array of 3 entries for body x,y,z.
 */
int arb_imu_set_axis_map(arb_imu_t_inst *imu, const arb_axis_map_t remap[3]);

/**
 * @brief Bring the IMU online (calls backend start()).
 */
int arb_imu_start(arb_imu_t_inst *imu);

/**
 * @brief Read one remapped sample.
 */
int arb_imu_sample(arb_imu_t_inst *imu, arb_imu_sample_t *out);

/**
 * @brief Read a sample and pack it into a ready-to-publish @ref arb_imu_t.
 * @param source Source/node id stamped into the header.
 * @param seq    Sequence number stamped into the header.
 */
int arb_imu_sample_msg(arb_imu_t_inst *imu, uint16_t source, uint32_t seq,
		       arb_imu_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* ARB_HAL_IMU_H */
