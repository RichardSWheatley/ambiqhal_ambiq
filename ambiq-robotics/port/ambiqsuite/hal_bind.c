/*
 * ARB AmbiqSuite port - direct HAL bindings for motor / encoder / IMU.
 *
 * These are reference bindings: they show how each ARB HAL backend maps onto the
 * Apollo510 peripheral HAL. Pin/timer/register choices are examples - adapt them
 * to your board. IMU register layout below follows an ICM-class 6-axis sensor.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include "hal_bind.h"
#include "arb/topic.h" /* arb_err_t */

#include "am_mcu_apollo.h"

/* HFRC/16 ~= 6 MHz timer clock used for PWM period math below. */
#define MOTOR_TIMER_CLK_HZ 6000000u

/* ---------------------------------------------------------------------- */
/* Motor: TIMER in PWM function, duty via compare1, sign via DIR GPIO.    */
/* ---------------------------------------------------------------------- */

static int motor_set_duty(void *vctx, int16_t duty_q15)
{
	arb_ambiq_motor_ctx_t *c = (arb_ambiq_motor_ctx_t *)vctx;

	bool reverse = (duty_q15 < 0);
	uint32_t mag = (uint32_t)(reverse ? -(int32_t)duty_q15 : duty_q15);

	/* Scale Q15 magnitude (0..32767) to 0..period. */
	uint32_t compare = (uint32_t)(((uint64_t)mag * c->period) / 32767u);

	if (c->has_dir) {
		if (reverse) {
			am_hal_gpio_output_set(c->dir_pin);
		} else {
			am_hal_gpio_output_clear(c->dir_pin);
		}
	}

	am_hal_timer_compare1_set(c->timer, compare);
	return ARB_OK;
}

static const arb_motor_ops_t s_motor_ops = {
	.set_duty = motor_set_duty,
	.enable   = NULL,
	.brake    = NULL,
};

int arb_ambiq_motor_bind(arb_motor_t *m, arb_ambiq_motor_ctx_t *ctx,
			 uint32_t timer, uint32_t out_pad, uint32_t dir_pin,
			 uint32_t pwm_hz)
{
	if (!m || !ctx || pwm_hz == 0) {
		return ARB_ERR_INVAL;
	}

	ctx->timer   = timer;
	ctx->out_pad = out_pad;
	ctx->period  = MOTOR_TIMER_CLK_HZ / pwm_hz;
	ctx->dir_pin = dir_pin;
	ctx->has_dir = (dir_pin != 0xFFFFFFFFu);

	am_hal_timer_config_t cfg;
	am_hal_timer_default_config_set(&cfg);
	cfg.eInputClock  = AM_HAL_TIMER_CLOCK_HFRC_DIV16;
	cfg.eFunction    = AM_HAL_TIMER_FN_PWM;
	cfg.ui32Compare0 = ctx->period;        /* period */
	cfg.ui32Compare1 = 0;                  /* duty = 0 at startup */

	am_hal_timer_config(timer, &cfg);
	am_hal_timer_output_config(out_pad,
		AM_HAL_TIMER_OUTPUT_TMR0_OUT1 + (timer * 2));

	am_hal_gpio_pinconfig(out_pad, am_hal_gpio_pincfg_output);
	if (ctx->has_dir) {
		am_hal_gpio_pinconfig(dir_pin, am_hal_gpio_pincfg_output);
		am_hal_gpio_output_clear(dir_pin);
	}

	am_hal_timer_clear(timer);
	am_hal_timer_start(timer);

	int rc = arb_motor_init(m, &s_motor_ops, ctx);
	if (rc == ARB_OK) {
		/* Small deadband to suppress driver buzz near zero. */
		m->deadband_q15 = 200;
	}
	return rc;
}

/* ---------------------------------------------------------------------- */
/* Encoder: TIMER as a free-running event counter; read the count.        */
/* ---------------------------------------------------------------------- */

static int enc_read_count(void *vctx, int32_t *count)
{
	arb_ambiq_encoder_ctx_t *c = (arb_ambiq_encoder_ctx_t *)vctx;
	*count = (int32_t)am_hal_timer_read(c->timer);
	return ARB_OK;
}

static int enc_reset(void *vctx)
{
	arb_ambiq_encoder_ctx_t *c = (arb_ambiq_encoder_ctx_t *)vctx;
	am_hal_timer_clear(c->timer);
	return ARB_OK;
}

static const arb_encoder_ops_t s_enc_ops = {
	.read_count = enc_read_count,
	.reset      = enc_reset,
};

int arb_ambiq_encoder_bind(arb_encoder_t *e, arb_ambiq_encoder_ctx_t *ctx,
			   uint32_t timer, int32_t counts_per_rev)
{
	if (!e || !ctx) {
		return ARB_ERR_INVAL;
	}
	ctx->timer = timer;

	am_hal_timer_config_t cfg;
	am_hal_timer_default_config_set(&cfg);
	/* Count external events on the timer's input (quadrature decode or a
	 * single channel, depending on board wiring). */
	cfg.eFunction    = AM_HAL_TIMER_FN_EDGE;
	cfg.ui32Compare0 = 0xFFFFFFFFu;
	cfg.ui32Compare1 = 0xFFFFFFFFu;

	am_hal_timer_config(timer, &cfg);
	am_hal_timer_clear(timer);
	am_hal_timer_start(timer);

	return arb_encoder_init(e, &s_enc_ops, ctx, counts_per_rev);
}

/* ---------------------------------------------------------------------- */
/* IMU: I2C register read over IOM (ICM-class register map, example).     */
/* ---------------------------------------------------------------------- */

/* Example register map (ICM-426xx style). */
#define IMU_REG_PWR_MGMT0  0x4E
#define IMU_REG_ACCEL_X_H  0x1F /* burst: AX,AY,AZ,GX,GY,GZ, 12 bytes */

static int imu_reg_read(arb_ambiq_imu_ctx_t *c, uint8_t reg, uint8_t *buf,
			uint32_t len)
{
	am_hal_iom_transfer_t tr = {0};
	tr.uPeerInfo.ui32I2CDevAddr = c->i2c_addr;
	tr.ui32InstrLen = 1;
	tr.ui64Instr    = reg;
	tr.eDirection   = AM_HAL_IOM_RX;
	tr.ui32NumBytes = len;
	tr.pui32RxBuffer = (uint32_t *)(void *)buf;

	uint32_t rc = am_hal_iom_blocking_transfer(c->iom_handle, &tr);
	return (rc == AM_HAL_STATUS_SUCCESS) ? ARB_OK : ARB_ERR_AGAIN;
}

static int imu_reg_write(arb_ambiq_imu_ctx_t *c, uint8_t reg, uint8_t val)
{
	uint32_t tx = val;
	am_hal_iom_transfer_t tr = {0};
	tr.uPeerInfo.ui32I2CDevAddr = c->i2c_addr;
	tr.ui32InstrLen = 1;
	tr.ui64Instr    = reg;
	tr.eDirection   = AM_HAL_IOM_TX;
	tr.ui32NumBytes = 1;
	tr.pui32TxBuffer = &tx;

	uint32_t rc = am_hal_iom_blocking_transfer(c->iom_handle, &tr);
	return (rc == AM_HAL_STATUS_SUCCESS) ? ARB_OK : ARB_ERR_AGAIN;
}

static int imu_start(void *vctx)
{
	arb_ambiq_imu_ctx_t *c = (arb_ambiq_imu_ctx_t *)vctx;
	/* Power up accel + gyro in low-noise mode (PWR_MGMT0 = 0x0F). */
	return imu_reg_write(c, IMU_REG_PWR_MGMT0, 0x0F);
}

static int imu_read(void *vctx, arb_imu_sample_t *out)
{
	arb_ambiq_imu_ctx_t *c = (arb_ambiq_imu_ctx_t *)vctx;
	uint8_t b[12];

	int rc = imu_reg_read(c, IMU_REG_ACCEL_X_H, b, sizeof(b));
	if (rc != ARB_OK) {
		return rc;
	}

	int16_t ax = (int16_t)((b[0] << 8) | b[1]);
	int16_t ay = (int16_t)((b[2] << 8) | b[3]);
	int16_t az = (int16_t)((b[4] << 8) | b[5]);
	int16_t gx = (int16_t)((b[6] << 8) | b[7]);
	int16_t gy = (int16_t)((b[8] << 8) | b[9]);
	int16_t gz = (int16_t)((b[10] << 8) | b[11]);

	out->accel.x = ax * c->accel_lsb;
	out->accel.y = ay * c->accel_lsb;
	out->accel.z = az * c->accel_lsb;
	out->gyro.x  = gx * c->gyro_lsb;
	out->gyro.y  = gy * c->gyro_lsb;
	out->gyro.z  = gz * c->gyro_lsb;
	out->mag.x = out->mag.y = out->mag.z = 0.0f;
	out->temperature = 0.0f;
	out->has_mag = false;
	return ARB_OK;
}

static const arb_imu_ops_t s_imu_ops = {
	.start = imu_start,
	.read  = imu_read,
};

int arb_ambiq_imu_bind(arb_imu_t_inst *imu, arb_ambiq_imu_ctx_t *ctx,
		       void *iom_handle, uint16_t i2c_addr)
{
	if (!imu || !ctx || !iom_handle) {
		return ARB_ERR_INVAL;
	}
	ctx->iom_handle = iom_handle;
	ctx->i2c_addr   = i2c_addr;
	/* +/-4g -> 9.81/8192 m/s^2/LSB ; +/-2000dps -> (pi/180)/16.4 rad/s/LSB */
	ctx->accel_lsb  = 9.80665f / 8192.0f;
	ctx->gyro_lsb   = 0.01745329f / 16.4f;

	return arb_imu_init(imu, &s_imu_ops, ctx);
}
