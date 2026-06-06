/*
 * ARB control - PID controller
 *
 * A small, allocation-free PID with output clamping, integral anti-windup
 * (integrator clamped to the output range) and derivative-on-measurement to
 * avoid setpoint-change kick. Used by the diff-drive samples to close the loop
 * around wheel velocity (setpoint from /cmd_vel, feedback from encoders).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_CONTROL_PID_H
#define ARB_CONTROL_PID_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief PID controller state. Initialize with arb_pid_init(). */
typedef struct {
	float kp, ki, kd;
	float out_min, out_max;

	/* internal state */
	float i_term;    /**< accumulated integral term (already * ki)   */
	float prev_meas; /**< last measurement (derivative-on-measurement)*/
	bool  primed;
} arb_pid_t;

/**
 * @brief Initialize/configure a PID controller and clear its state.
 *
 * @param pid     Controller to set up.
 * @param kp,ki,kd Gains.
 * @param out_min Lower output clamp.
 * @param out_max Upper output clamp (must be >= out_min).
 */
void arb_pid_init(arb_pid_t *pid, float kp, float ki, float kd,
		  float out_min, float out_max);

/**
 * @brief Clear integrator and derivative history (e.g. on re-enable).
 */
void arb_pid_reset(arb_pid_t *pid);

/**
 * @brief Advance the controller one step.
 *
 * @param pid         Controller.
 * @param setpoint    Desired value.
 * @param measurement Measured value.
 * @param dt          Timestep in seconds (>0). Non-positive dt returns the
 *                    proportional response only and does not integrate.
 * @return Clamped control output.
 */
float arb_pid_update(arb_pid_t *pid, float setpoint, float measurement,
		     float dt);

#ifdef __cplusplus
}
#endif

#endif /* ARB_CONTROL_PID_H */
