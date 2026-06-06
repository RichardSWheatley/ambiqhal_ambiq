/*
 * ARB sample (bare-metal AmbiqSuite): differential-drive base.
 *
 * Demonstrates the full middleware on a no-OS super-loop:
 *   - subscribe to /cmd_vel (arb_twist_t) and drive two motors,
 *   - read two wheel encoders and publish /odom (arb_odom_t),
 *   - sample an IMU and publish /imu (arb_imu_t).
 *
 * Pin/timer/IOM choices are placeholders - adapt to your board.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <math.h>

#include "am_mcu_apollo.h"

#include "arb/platform.h"
#include "arb/topic.h"
#include "arb/msg.h"
#include "hal_bind.h"

/* ---- topic ids (shared with any bridge) ------------------------------ */
#define ARB_TOPIC_CMD_VEL 1
#define ARB_TOPIC_ODOM    2
#define ARB_TOPIC_IMU     3

/* ---- robot geometry -------------------------------------------------- */
#define WHEEL_BASE_M       0.20f  /* track width                         */
#define WHEEL_RADIUS_M     0.033f /* wheel radius                        */
#define MAX_WHEEL_SPEED_MS 0.6f   /* maps to full duty                   */
#define ENC_COUNTS_PER_REV 1440

/* ---- bound peripherals ----------------------------------------------- */
static arb_motor_t   g_left, g_right;
static arb_encoder_t g_enc_l, g_enc_r;
static arb_imu_t_inst g_imu;

static arb_ambiq_motor_ctx_t   g_lmctx, g_rmctx;
static arb_ambiq_encoder_ctx_t g_lectx, g_rectx;
static arb_ambiq_imu_ctx_t     g_imctx;

static void *g_iom_handle;

/* odometry integration state */
static float g_odom_x, g_odom_y, g_odom_theta;
static uint32_t g_odom_seq, g_imu_seq;

/* ---- /cmd_vel subscriber: convert twist -> wheel duties -------------- */

static void on_cmd_vel(arb_topic_id_t topic, const void *msg, size_t len,
		       void *user)
{
	(void)topic; (void)user;
	if (len != sizeof(arb_twist_t)) {
		return;
	}
	const arb_twist_t *t = (const arb_twist_t *)msg;

	float v  = t->linear.x;   /* m/s   */
	float w  = t->angular.z;  /* rad/s */

	float v_l = v - w * (WHEEL_BASE_M * 0.5f);
	float v_r = v + w * (WHEEL_BASE_M * 0.5f);

	arb_motor_set_duty(&g_left,  v_l / MAX_WHEEL_SPEED_MS);
	arb_motor_set_duty(&g_right, v_r / MAX_WHEEL_SPEED_MS);
}

/* ---- odometry from encoders ------------------------------------------ */

static void publish_odom(void)
{
	arb_encoder_reading_t rl, rr;
	if (arb_encoder_read(&g_enc_l, &rl) != ARB_OK ||
	    arb_encoder_read(&g_enc_r, &rr) != ARB_OK) {
		return;
	}

	/* wheel angular velocity (rad/s) -> ground velocity */
	float vl = rl.velocity_rad_s * WHEEL_RADIUS_M;
	float vr = rr.velocity_rad_s * WHEEL_RADIUS_M;
	float v  = 0.5f * (vl + vr);
	float w  = (vr - vl) / WHEEL_BASE_M;

	/* integrate pose (simple Euler, dt folded into velocity*loop period) */
	static uint64_t last_us;
	uint64_t now = arb_platform_time_us();
	float dt = last_us ? (float)(now - last_us) * 1e-6f : 0.0f;
	last_us = now;

	g_odom_theta += w * dt;
	g_odom_x     += v * cosf(g_odom_theta) * dt;
	g_odom_y     += v * sinf(g_odom_theta) * dt;

	arb_odom_t odom;
	arb_header_init(&odom.header, ARB_MSG_ODOM, 0, now, g_odom_seq++);
	odom.pose.x      = g_odom_x;
	odom.pose.y      = g_odom_y;
	odom.pose.theta  = g_odom_theta;
	odom.linear_vel  = v;
	odom.angular_vel = w;

	arb_topic_publish(ARB_TOPIC_ODOM, &odom, sizeof(odom));
}

/* ---- imu --------------------------------------------------------------*/

static void publish_imu(void)
{
	arb_imu_t msg;
	if (arb_imu_sample_msg(&g_imu, 0, g_imu_seq++, &msg) == ARB_OK) {
		arb_topic_publish(ARB_TOPIC_IMU, &msg, sizeof(msg));
	}
}

/* ---- board / peripheral bring-up ------------------------------------- */

static void board_init(void)
{
	/* IOM0 as I2C for the IMU (example configuration). */
	am_hal_iom_initialize(0, &g_iom_handle);
	am_hal_iom_power_ctrl(g_iom_handle, AM_HAL_SYSCTRL_WAKE, false);

	am_hal_iom_config_t iom_cfg = {0};
	iom_cfg.eInterfaceMode = AM_HAL_IOM_I2C_MODE;
	iom_cfg.ui32ClockFreq  = AM_HAL_IOM_400KHZ;
	am_hal_iom_configure(g_iom_handle, &iom_cfg);
	am_hal_iom_enable(g_iom_handle);
}

int main(void)
{
	arb_platform_init();
	arb_topic_init();

	board_init();

	/* Bind peripherals (timers/pads are examples). */
	arb_ambiq_motor_bind(&g_left,  &g_lmctx, 0, 12, 13, 20000);
	arb_ambiq_motor_bind(&g_right, &g_rmctx, 1, 14, 15, 20000);
	arb_ambiq_encoder_bind(&g_enc_l, &g_lectx, 2, ENC_COUNTS_PER_REV);
	arb_ambiq_encoder_bind(&g_enc_r, &g_rectx, 3, ENC_COUNTS_PER_REV);
	arb_ambiq_imu_bind(&g_imu, &g_imctx, g_iom_handle, 0x68);
	arb_imu_start(&g_imu);

	/* Topics. */
	arb_topic_advertise(ARB_TOPIC_CMD_VEL, sizeof(arb_twist_t));
	arb_topic_advertise(ARB_TOPIC_ODOM,    sizeof(arb_odom_t));
	arb_topic_advertise(ARB_TOPIC_IMU,     sizeof(arb_imu_t));
	arb_topic_subscribe(ARB_TOPIC_CMD_VEL, on_cmd_vel, NULL);

	uint64_t next_50hz = arb_platform_time_us();

	for (;;) {
		uint64_t now = arb_platform_time_us();

		/* 50 Hz control / telemetry tick. */
		if ((int64_t)(now - next_50hz) >= 0) {
			next_50hz += 20000; /* 20 ms */
			publish_odom();
			publish_imu();
		}

		/* Deliver everything that was published this iteration. */
		arb_platform_dispatch();
	}
}
