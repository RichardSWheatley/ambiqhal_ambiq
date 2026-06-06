/*
 * ARB sample (Apache NuttX on Apollo510): differential-drive base.
 *
 * The closed-loop diff-drive node on the NuttX broker port. A POSIX thread acts
 * as the (stand-in) command source while the main task runs the 50 Hz control +
 * telemetry loop and drains the broker - demonstrating cross-thread pub/sub on
 * NuttX. Peripheral bindings reuse the AmbiqSuite direct-HAL bindings.
 *
 * Install under apps/ (see Makefile/Kconfig/Make.defs) and enable
 * CONFIG_ARB_DIFFDRIVE.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>

#include "am_mcu_apollo.h"

#include "arb/platform.h"
#include "arb/topic.h"
#include "arb/msg.h"
#include "arb/control/pid.h"
#include "arb/control/diff_drive.h"
#include "hal_bind.h"

#define ARB_TOPIC_CMD_VEL 1
#define ARB_TOPIC_ODOM    2
#define ARB_TOPIC_IMU     3

#define WHEEL_BASE_M       0.20f
#define WHEEL_RADIUS_M     0.033f
#define MAX_WHEEL_SPEED_MS 0.6f
#define MAX_WHEEL_ANGVEL   (MAX_WHEEL_SPEED_MS / WHEEL_RADIUS_M)
#define ENC_COUNTS_PER_REV 1440

#define VEL_KP 0.03f
#define VEL_KI 0.15f
#define VEL_KD 0.0f

#define CONTROL_PERIOD_MS 20

static const arb_diffdrive_t g_dd = {
	.wheel_base = WHEEL_BASE_M, .wheel_radius = WHEEL_RADIUS_M,
};

static arb_motor_t   g_left, g_right;
static arb_encoder_t g_enc_l, g_enc_r;
static arb_imu_t_inst g_imu;
static arb_ambiq_motor_ctx_t   g_lmctx, g_rmctx;
static arb_ambiq_encoder_ctx_t g_lectx, g_rectx;
static arb_ambiq_imu_ctx_t     g_imctx;
static void *g_iom_handle;

static arb_pid_t g_pid_l, g_pid_r;
static volatile float g_sp_l, g_sp_r;
static float g_odom_x, g_odom_y, g_odom_theta;
static uint32_t g_odom_seq, g_imu_seq;
static volatile bool g_run = true;

static void on_cmd_vel(arb_topic_id_t topic, const void *msg, size_t len,
		       void *user)
{
	(void)topic; (void)user;
	if (len != sizeof(arb_twist_t)) {
		return;
	}
	const arb_twist_t *t = (const arb_twist_t *)msg;
	float l, r;
	arb_diffdrive_twist_to_wheels(&g_dd, t->linear.x, t->angular.z, &l, &r);
	g_sp_l = l;
	g_sp_r = r;
}

static float wheel_output(arb_pid_t *pid, float sp, float meas, float dt)
{
	return (sp / MAX_WHEEL_ANGVEL) + arb_pid_update(pid, sp, meas, dt);
}

static void *command_thread(void *arg)
{
	(void)arg;
	while (g_run) {
		arb_twist_t tw = {0};
		arb_header_init(&tw.header, ARB_MSG_TWIST, 0,
				arb_platform_time_us(), 0);
		tw.linear.x = 0.3f;
		arb_topic_publish(ARB_TOPIC_CMD_VEL, &tw, sizeof(tw));
		usleep(2000 * 1000);

		tw.angular.z = 0.8f;
		arb_topic_publish(ARB_TOPIC_CMD_VEL, &tw, sizeof(tw));
		usleep(2000 * 1000);
	}
	return NULL;
}

static void board_init(void)
{
	am_hal_iom_initialize(0, &g_iom_handle);
	am_hal_iom_power_ctrl(g_iom_handle, AM_HAL_SYSCTRL_WAKE, false);
	am_hal_iom_config_t cfg = {0};
	cfg.eInterfaceMode = AM_HAL_IOM_I2C_MODE;
	cfg.ui32ClockFreq  = AM_HAL_IOM_400KHZ;
	am_hal_iom_configure(g_iom_handle, &cfg);
	am_hal_iom_enable(g_iom_handle);
}

int main(int argc, FAR char *argv[])
{
	(void)argc; (void)argv;

	arb_platform_init();
	arb_topic_init();
	board_init();

	arb_ambiq_motor_bind(&g_left,  &g_lmctx, 0, 12, 13, 20000);
	arb_ambiq_motor_bind(&g_right, &g_rmctx, 1, 14, 15, 20000);
	arb_ambiq_encoder_bind(&g_enc_l, &g_lectx, 2, ENC_COUNTS_PER_REV);
	arb_ambiq_encoder_bind(&g_enc_r, &g_rectx, 3, ENC_COUNTS_PER_REV);
	arb_ambiq_imu_bind(&g_imu, &g_imctx, g_iom_handle, 0x68);
	arb_imu_start(&g_imu);

	arb_pid_init(&g_pid_l, VEL_KP, VEL_KI, VEL_KD, -1.0f, 1.0f);
	arb_pid_init(&g_pid_r, VEL_KP, VEL_KI, VEL_KD, -1.0f, 1.0f);

	arb_topic_advertise(ARB_TOPIC_CMD_VEL, sizeof(arb_twist_t));
	arb_topic_advertise(ARB_TOPIC_ODOM,    sizeof(arb_odom_t));
	arb_topic_advertise(ARB_TOPIC_IMU,     sizeof(arb_imu_t));
	arb_topic_subscribe(ARB_TOPIC_CMD_VEL, on_cmd_vel, NULL);

	pthread_t cmd;
	pthread_create(&cmd, NULL, command_thread, NULL);

	printf("ARB NuttX diff-drive sample running\n");

	const float dt = CONTROL_PERIOD_MS * 1e-3f;
	for (;;) {
		arb_encoder_reading_t rl, rr;
		if (arb_encoder_read(&g_enc_l, &rl) == ARB_OK &&
		    arb_encoder_read(&g_enc_r, &rr) == ARB_OK) {
			arb_motor_set_duty(&g_left,
				wheel_output(&g_pid_l, g_sp_l,
					     rl.velocity_rad_s, dt));
			arb_motor_set_duty(&g_right,
				wheel_output(&g_pid_r, g_sp_r,
					     rr.velocity_rad_s, dt));

			float v, w;
			arb_diffdrive_wheels_to_twist(&g_dd, rl.velocity_rad_s,
						      rr.velocity_rad_s, &v, &w);
			g_odom_theta += w * dt;
			g_odom_x += v * cosf(g_odom_theta) * dt;
			g_odom_y += v * sinf(g_odom_theta) * dt;

			arb_odom_t odom;
			arb_header_init(&odom.header, ARB_MSG_ODOM, 0,
					arb_platform_time_us(), g_odom_seq++);
			odom.pose.x = g_odom_x;
			odom.pose.y = g_odom_y;
			odom.pose.theta = g_odom_theta;
			odom.linear_vel = v;
			odom.angular_vel = w;
			arb_topic_publish(ARB_TOPIC_ODOM, &odom, sizeof(odom));
		}

		arb_imu_t imu;
		if (arb_imu_sample_msg(&g_imu, 0, g_imu_seq++, &imu) == ARB_OK) {
			arb_topic_publish(ARB_TOPIC_IMU, &imu, sizeof(imu));
		}

		arb_platform_dispatch();
		usleep(CONTROL_PERIOD_MS * 1000);
	}

	g_run = false;
	pthread_join(cmd, NULL);
	return 0;
}
