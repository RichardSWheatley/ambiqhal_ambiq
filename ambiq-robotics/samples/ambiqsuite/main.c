/*
 * ARB sample (bare-metal AmbiqSuite): differential-drive base, closed loop.
 *
 * Demonstrates the full middleware on a no-OS super-loop:
 *   - subscribe to /cmd_vel (arb_twist_t) -> per-wheel velocity setpoints,
 *   - read two wheel encoders, run a per-wheel PID (feedforward + correction)
 *     to track the setpoint, and drive the motors,
 *   - publish /odom (arb_odom_t) from the measured wheel velocities,
 *   - sample an IMU and publish /imu (arb_imu_t).
 *
 * Pin/timer/IOM choices and PID gains are placeholders - adapt to your board.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <math.h>

#include "am_mcu_apollo.h"

#include "arb/platform.h"
#include "arb/topic.h"
#include "arb/msg.h"
#include "arb/control/pid.h"
#include "arb/control/diff_drive.h"
#include "hal_bind.h"

/* ---- topic ids (shared with any bridge) ------------------------------ */
#define ARB_TOPIC_CMD_VEL 1
#define ARB_TOPIC_ODOM    2
#define ARB_TOPIC_IMU     3

/* ---- robot geometry -------------------------------------------------- */
#define WHEEL_BASE_M       0.20f  /* track width                          */
#define WHEEL_RADIUS_M     0.033f /* wheel radius                         */
#define MAX_WHEEL_SPEED_MS 0.6f   /* ground speed at full duty            */
#define ENC_COUNTS_PER_REV 1440
/* wheel angular velocity (rad/s) at full duty - used for feedforward */
#define MAX_WHEEL_ANGVEL   (MAX_WHEEL_SPEED_MS / WHEEL_RADIUS_M)

/* ---- control tuning (per wheel) -------------------------------------- */
#define VEL_KP 0.03f
#define VEL_KI 0.15f
#define VEL_KD 0.0f

/* ---- bound peripherals ----------------------------------------------- */
static arb_motor_t   g_left, g_right;
static arb_encoder_t g_enc_l, g_enc_r;
static arb_imu_t_inst g_imu;

static arb_ambiq_motor_ctx_t   g_lmctx, g_rmctx;
static arb_ambiq_encoder_ctx_t g_lectx, g_rectx;
static arb_ambiq_imu_ctx_t     g_imctx;

static void *g_iom_handle;

/* ---- control state --------------------------------------------------- */
static const arb_diffdrive_t g_dd = {
	.wheel_base   = WHEEL_BASE_M,
	.wheel_radius = WHEEL_RADIUS_M,
};
static arb_pid_t g_pid_l, g_pid_r;
static float g_sp_l, g_sp_r; /* wheel angular-velocity setpoints, rad/s */

/* odometry integration state */
static float g_odom_x, g_odom_y, g_odom_theta;
static uint32_t g_odom_seq, g_imu_seq;

/* ---- /cmd_vel subscriber: twist -> per-wheel velocity setpoints ------ */

static void on_cmd_vel(arb_topic_id_t topic, const void *msg, size_t len,
		       void *user)
{
	(void)topic; (void)user;
	if (len != sizeof(arb_twist_t)) {
		return;
	}
	const arb_twist_t *t = (const arb_twist_t *)msg;

	/* twist -> per-wheel angular-velocity setpoints */
	arb_diffdrive_twist_to_wheels(&g_dd, t->linear.x, t->angular.z,
				      &g_sp_l, &g_sp_r);
}

/* ---- per-wheel closed-loop output: feedforward + PID correction ------ */

static float wheel_output(arb_pid_t *pid, float sp_radps, float meas_radps,
			  float dt)
{
	float ff = sp_radps / MAX_WHEEL_ANGVEL;          /* nominal duty */
	float corr = arb_pid_update(pid, sp_radps, meas_radps, dt);
	return ff + corr;                                /* motor clamps */
}

/* ---- control + odometry tick ----------------------------------------- */

static void control_and_odom(float dt)
{
	arb_encoder_reading_t rl, rr;
	if (arb_encoder_read(&g_enc_l, &rl) != ARB_OK ||
	    arb_encoder_read(&g_enc_r, &rr) != ARB_OK) {
		return;
	}

	/* close the loop on measured wheel angular velocity */
	arb_motor_set_duty(&g_left,
			   wheel_output(&g_pid_l, g_sp_l, rl.velocity_rad_s, dt));
	arb_motor_set_duty(&g_right,
			   wheel_output(&g_pid_r, g_sp_r, rr.velocity_rad_s, dt));

	/* odometry from measured wheel velocities */
	float v, w;
	arb_diffdrive_wheels_to_twist(&g_dd, rl.velocity_rad_s,
				      rr.velocity_rad_s, &v, &w);

	g_odom_theta += w * dt;
	g_odom_x     += v * cosf(g_odom_theta) * dt;
	g_odom_y     += v * sinf(g_odom_theta) * dt;

	arb_odom_t odom;
	arb_header_init(&odom.header, ARB_MSG_ODOM, 0, arb_platform_time_us(),
			g_odom_seq++);
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

	/* Per-wheel velocity controllers (output is normalized duty). */
	arb_pid_init(&g_pid_l, VEL_KP, VEL_KI, VEL_KD, -1.0f, 1.0f);
	arb_pid_init(&g_pid_r, VEL_KP, VEL_KI, VEL_KD, -1.0f, 1.0f);

	/* Topics. */
	arb_topic_advertise(ARB_TOPIC_CMD_VEL, sizeof(arb_twist_t));
	arb_topic_advertise(ARB_TOPIC_ODOM,    sizeof(arb_odom_t));
	arb_topic_advertise(ARB_TOPIC_IMU,     sizeof(arb_imu_t));
	arb_topic_subscribe(ARB_TOPIC_CMD_VEL, on_cmd_vel, NULL);

	uint64_t next_tick = arb_platform_time_us();
	uint64_t last_tick = next_tick;

	for (;;) {
		uint64_t now = arb_platform_time_us();

		/* 50 Hz control / telemetry tick. */
		if ((int64_t)(now - next_tick) >= 0) {
			float dt = (float)(now - last_tick) * 1e-6f;
			last_tick = now;
			next_tick += 20000; /* 20 ms */

			control_and_odom(dt);
			publish_imu();
		}

		/* Deliver everything that was published this iteration. */
		arb_platform_dispatch();
	}
}
