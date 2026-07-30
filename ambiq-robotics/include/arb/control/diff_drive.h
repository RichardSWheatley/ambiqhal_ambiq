/*
 * ARB control - differential-drive kinematics
 *
 * Pure, allocation-free conversions between a body twist (forward velocity +
 * yaw rate) and per-wheel angular velocities. Shared by the diff-drive samples
 * so the kinematics live in one tested place instead of being copy-pasted.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_CONTROL_DIFF_DRIVE_H
#define ARB_CONTROL_DIFF_DRIVE_H

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Geometry of a two-wheel differential-drive base. */
typedef struct {
	float wheel_base;   /**< track width between wheels, m */
	float wheel_radius; /**< wheel radius, m               */
} arb_diffdrive_t;

/**
 * @brief Body twist -> per-wheel angular velocities.
 *
 * @param dd        Geometry.
 * @param v         Forward velocity, m/s.
 * @param w         Yaw rate, rad/s.
 * @param wl_radps  [out] Left wheel angular velocity, rad/s.
 * @param wr_radps  [out] Right wheel angular velocity, rad/s.
 */
void arb_diffdrive_twist_to_wheels(const arb_diffdrive_t *dd, float v, float w,
				   float *wl_radps, float *wr_radps);

/**
 * @brief Per-wheel angular velocities -> body twist.
 *
 * @param dd        Geometry.
 * @param wl_radps  Left wheel angular velocity, rad/s.
 * @param wr_radps  Right wheel angular velocity, rad/s.
 * @param v         [out] Forward velocity, m/s.
 * @param w         [out] Yaw rate, rad/s.
 */
void arb_diffdrive_wheels_to_twist(const arb_diffdrive_t *dd, float wl_radps,
				   float wr_radps, float *v, float *w);

#ifdef __cplusplus
}
#endif

#endif /* ARB_CONTROL_DIFF_DRIVE_H */
