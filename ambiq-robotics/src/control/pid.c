/*
 * ARB control - PID controller implementation.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include "arb/control/pid.h"

static float clampf(float v, float lo, float hi)
{
	if (v < lo) {
		return lo;
	}
	if (v > hi) {
		return hi;
	}
	return v;
}

void arb_pid_init(arb_pid_t *pid, float kp, float ki, float kd,
		  float out_min, float out_max)
{
	if (!pid) {
		return;
	}
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->out_min = out_min;
	pid->out_max = out_max;
	arb_pid_reset(pid);
}

void arb_pid_reset(arb_pid_t *pid)
{
	if (!pid) {
		return;
	}
	pid->i_term = 0.0f;
	pid->prev_meas = 0.0f;
	pid->primed = false;
}

float arb_pid_update(arb_pid_t *pid, float setpoint, float measurement,
		     float dt)
{
	if (!pid) {
		return 0.0f;
	}

	float error = setpoint - measurement;
	float p = pid->kp * error;

	/* Derivative on measurement (negated) to dodge setpoint kick. */
	float d = 0.0f;
	if (pid->primed && dt > 0.0f) {
		d = -pid->kd * (measurement - pid->prev_meas) / dt;
	}
	pid->prev_meas = measurement;
	pid->primed = true;

	/* Integrate, then clamp the integrator so it can't wind up beyond the
	 * output range (simple, robust anti-windup). */
	if (dt > 0.0f) {
		pid->i_term += pid->ki * error * dt;
		pid->i_term = clampf(pid->i_term, pid->out_min, pid->out_max);
	}

	return clampf(p + pid->i_term + d, pid->out_min, pid->out_max);
}
