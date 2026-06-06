/*
 * ARB Zephyr port - HAL bindings via the Zephyr device model.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include "hal_bind.h"
#include "arb/topic.h" /* arb_err_t */

#include <zephyr/drivers/sensor.h>

/* ---------------------------------------------------------------------- */
/* Motor: Zephyr PWM pulse width = duty; sign -> DIR GPIO.                 */
/* ---------------------------------------------------------------------- */

static int motor_set_duty(void *vctx, int16_t duty_q15)
{
	arb_zephyr_motor_ctx_t *c = (arb_zephyr_motor_ctx_t *)vctx;

	bool reverse = (duty_q15 < 0);
	uint32_t mag = (uint32_t)(reverse ? -(int32_t)duty_q15 : duty_q15);

	uint32_t pulse = (uint32_t)(((uint64_t)mag * c->pwm.period) / 32767u);

	if (c->has_dir) {
		gpio_pin_set_dt(&c->dir, reverse ? 1 : 0);
	}

	int rc = pwm_set_pulse_dt(&c->pwm, pulse);
	return (rc == 0) ? ARB_OK : ARB_ERR_AGAIN;
}

static const arb_motor_ops_t s_motor_ops = {
	.set_duty = motor_set_duty,
	.enable   = NULL,
	.brake    = NULL,
};

int arb_zephyr_motor_bind(arb_motor_t *m, arb_zephyr_motor_ctx_t *ctx,
			  const struct pwm_dt_spec *pwm,
			  const struct gpio_dt_spec *dir)
{
	if (!m || !ctx || !pwm) {
		return ARB_ERR_INVAL;
	}
	if (!pwm_is_ready_dt(pwm)) {
		return ARB_ERR_AGAIN;
	}

	ctx->pwm     = *pwm;
	ctx->has_dir = (dir != NULL);
	if (dir) {
		ctx->dir = *dir;
		if (!gpio_is_ready_dt(dir)) {
			return ARB_ERR_AGAIN;
		}
		gpio_pin_configure_dt(dir, GPIO_OUTPUT_INACTIVE);
	}

	return arb_motor_init(m, &s_motor_ops, ctx);
}

/* ---------------------------------------------------------------------- */
/* Encoder: Zephyr qdec sensor; rotation reported in centi-degrees.       */
/* ---------------------------------------------------------------------- */

static int enc_read_count(void *vctx, int32_t *count)
{
	arb_zephyr_encoder_ctx_t *c = (arb_zephyr_encoder_ctx_t *)vctx;
	struct sensor_value rot;

	if (sensor_sample_fetch(c->qdec) != 0) {
		return ARB_ERR_AGAIN;
	}
	if (sensor_channel_get(c->qdec, SENSOR_CHAN_ROTATION, &rot) != 0) {
		return ARB_ERR_AGAIN;
	}

	/* rot is degrees (val1.val2). Convert to centi-degrees integer. */
	*count = rot.val1 * 100 + rot.val2 / 10000;
	return ARB_OK;
}

static const arb_encoder_ops_t s_enc_ops = {
	.read_count = enc_read_count,
	.reset      = NULL,
};

int arb_zephyr_encoder_bind(arb_encoder_t *e, arb_zephyr_encoder_ctx_t *ctx,
			    const struct device *qdec)
{
	if (!e || !ctx || !qdec) {
		return ARB_ERR_INVAL;
	}
	if (!device_is_ready(qdec)) {
		return ARB_ERR_AGAIN;
	}
	ctx->qdec = qdec;
	/* centi-degrees: 360 deg * 100 = 36000 "counts" per revolution. */
	return arb_encoder_init(e, &s_enc_ops, ctx, 36000);
}

/* ---------------------------------------------------------------------- */
/* IMU: Zephyr 6-axis sensor; channels already in SI units.               */
/* ---------------------------------------------------------------------- */

static void copy_xyz(struct sensor_value v[3], arb_vec3_t *out)
{
	out->x = (float)sensor_value_to_double(&v[0]);
	out->y = (float)sensor_value_to_double(&v[1]);
	out->z = (float)sensor_value_to_double(&v[2]);
}

static int imu_read(void *vctx, arb_imu_sample_t *out)
{
	arb_zephyr_imu_ctx_t *c = (arb_zephyr_imu_ctx_t *)vctx;
	struct sensor_value acc[3], gyr[3];

	if (sensor_sample_fetch(c->imu) != 0) {
		return ARB_ERR_AGAIN;
	}
	if (sensor_channel_get(c->imu, SENSOR_CHAN_ACCEL_XYZ, acc) != 0 ||
	    sensor_channel_get(c->imu, SENSOR_CHAN_GYRO_XYZ, gyr) != 0) {
		return ARB_ERR_AGAIN;
	}

	copy_xyz(acc, &out->accel);
	copy_xyz(gyr, &out->gyro);
	out->mag.x = out->mag.y = out->mag.z = 0.0f;
	out->has_mag = false;

	struct sensor_value die;
	if (sensor_channel_get(c->imu, SENSOR_CHAN_DIE_TEMP, &die) == 0) {
		out->temperature = (float)sensor_value_to_double(&die);
	} else {
		out->temperature = 0.0f;
	}
	return ARB_OK;
}

static const arb_imu_ops_t s_imu_ops = {
	.start = NULL,
	.read  = imu_read,
};

int arb_zephyr_imu_bind(arb_imu_t_inst *imu, arb_zephyr_imu_ctx_t *ctx,
			const struct device *dev)
{
	if (!imu || !ctx || !dev) {
		return ARB_ERR_INVAL;
	}
	if (!device_is_ready(dev)) {
		return ARB_ERR_AGAIN;
	}
	ctx->imu = dev;
	return arb_imu_init(imu, &s_imu_ops, ctx);
}
