/*
 * ARB HAL - portable motor logic (clamp / deadband / inversion / scaling).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include "arb/hal/motor.h"
#include "arb/topic.h" /* arb_err_t */

#define Q15_MAX  32767
#define Q15_MIN (-32768)

int arb_motor_init(arb_motor_t *m, const arb_motor_ops_t *ops, void *ctx)
{
	if (!m || !ops || !ops->set_duty) {
		return ARB_ERR_INVAL;
	}
	m->ops          = ops;
	m->ctx          = ctx;
	m->deadband_q15 = 0;
	m->max_q15      = Q15_MAX;
	m->inverted     = false;
	return ARB_OK;
}

int arb_motor_set_duty_q15(arb_motor_t *m, int16_t duty_q15)
{
	if (!m || !m->ops) {
		return ARB_ERR_INVAL;
	}

	int32_t d = m->inverted ? -(int32_t)duty_q15 : (int32_t)duty_q15;

	/* Deadband around zero. */
	if (d > -m->deadband_q15 && d < m->deadband_q15) {
		d = 0;
	}

	/* Magnitude clamp (symmetric). */
	if (d > m->max_q15) {
		d = m->max_q15;
	} else if (d < -m->max_q15) {
		d = -m->max_q15;
	}

	if (d < Q15_MIN) {
		d = Q15_MIN;
	}

	return m->ops->set_duty(m->ctx, (int16_t)d);
}

int arb_motor_set_duty(arb_motor_t *m, float duty)
{
	if (duty > 1.0f) {
		duty = 1.0f;
	} else if (duty < -1.0f) {
		duty = -1.0f;
	}
	int32_t q = (int32_t)(duty * (float)Q15_MAX);
	if (q > Q15_MAX) {
		q = Q15_MAX;
	} else if (q < Q15_MIN) {
		q = Q15_MIN;
	}
	return arb_motor_set_duty_q15(m, (int16_t)q);
}

int arb_motor_stop(arb_motor_t *m)
{
	if (!m || !m->ops) {
		return ARB_ERR_INVAL;
	}
	if (m->ops->enable) {
		/* leave enabled; just zero the duty for a coast stop */
	}
	return m->ops->set_duty(m->ctx, 0);
}

int arb_motor_brake(arb_motor_t *m)
{
	if (!m || !m->ops) {
		return ARB_ERR_INVAL;
	}
	if (m->ops->brake) {
		return m->ops->brake(m->ctx);
	}
	return arb_motor_stop(m);
}
