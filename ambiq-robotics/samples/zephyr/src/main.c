/*
 * ARB sample (Zephyr): differential-drive base, closed loop.
 *
 * Same closed-loop diff-drive node as the bare-metal sample, but using the
 * Zephyr device model for the HAL bindings and a dedicated dispatcher thread
 * for delivery. Expects these devicetree aliases (provide a board overlay):
 *
 *   pwm-motor-l / pwm-motor-r   : PWM channels
 *   qdec-l / qdec-r             : quadrature decoder sensor nodes
 *   robot-imu                   : 6-axis IMU sensor node
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "arb/platform.h"
#include "arb/topic.h"
#include "arb/msg.h"
#include "arb/control/pid.h"
#include "arb/control/diff_drive.h"
#include "hal_bind.h"

LOG_MODULE_REGISTER(arb_sample, LOG_LEVEL_INF);

#define ARB_TOPIC_CMD_VEL 1
#define ARB_TOPIC_ODOM    2
#define ARB_TOPIC_IMU     3

#define WHEEL_BASE_M       0.20f
#define WHEEL_RADIUS_M     0.033f
#define MAX_WHEEL_SPEED_MS 0.6f
#define MAX_WHEEL_ANGVEL   (MAX_WHEEL_SPEED_MS / WHEEL_RADIUS_M)

#define VEL_KP 0.03f
#define VEL_KI 0.15f
#define VEL_KD 0.0f

#define CONTROL_PERIOD_MS 20

static const struct pwm_dt_spec pwm_l = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor_l));
static const struct pwm_dt_spec pwm_r = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor_r));

static arb_motor_t   g_left, g_right;
static arb_encoder_t g_enc_l, g_enc_r;
static arb_imu_t_inst g_imu;

static arb_zephyr_motor_ctx_t   g_lmctx, g_rmctx;
static arb_zephyr_encoder_ctx_t g_lectx, g_rectx;
static arb_zephyr_imu_ctx_t     g_imctx;

static const arb_diffdrive_t g_dd = {
	.wheel_base   = WHEEL_BASE_M,
	.wheel_radius = WHEEL_RADIUS_M,
};
static arb_pid_t g_pid_l, g_pid_r;
static float g_sp_l, g_sp_r; /* wheel angular-velocity setpoints, rad/s */

static float g_odom_x, g_odom_y, g_odom_theta;
static uint32_t g_odom_seq, g_imu_seq;

static void on_cmd_vel(arb_topic_id_t topic, const void *msg, size_t len,
		       void *user)
{
	ARG_UNUSED(topic);
	ARG_UNUSED(user);
	if (len != sizeof(arb_twist_t)) {
		return;
	}
	const arb_twist_t *t = msg;

	arb_diffdrive_twist_to_wheels(&g_dd, t->linear.x, t->angular.z,
				      &g_sp_l, &g_sp_r);
}

static float wheel_output(arb_pid_t *pid, float sp_radps, float meas_radps,
			  float dt)
{
	float ff = sp_radps / MAX_WHEEL_ANGVEL;
	float corr = arb_pid_update(pid, sp_radps, meas_radps, dt);
	return ff + corr;
}

static void control_and_odom(float dt)
{
	arb_encoder_reading_t rl, rr;
	if (arb_encoder_read(&g_enc_l, &rl) != ARB_OK ||
	    arb_encoder_read(&g_enc_r, &rr) != ARB_OK) {
		return;
	}

	arb_motor_set_duty(&g_left,
			   wheel_output(&g_pid_l, g_sp_l, rl.velocity_rad_s, dt));
	arb_motor_set_duty(&g_right,
			   wheel_output(&g_pid_r, g_sp_r, rr.velocity_rad_s, dt));

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

static void publish_imu(void)
{
	arb_imu_t msg;
	if (arb_imu_sample_msg(&g_imu, 0, g_imu_seq++, &msg) == ARB_OK) {
		arb_topic_publish(ARB_TOPIC_IMU, &msg, sizeof(msg));
	}
}

int main(void)
{
	arb_platform_init();
	arb_topic_init();

	if (arb_zephyr_motor_bind(&g_left,  &g_lmctx, &pwm_l, NULL) != ARB_OK ||
	    arb_zephyr_motor_bind(&g_right, &g_rmctx, &pwm_r, NULL) != ARB_OK) {
		LOG_ERR("motor bind failed");
		return -1;
	}

	arb_zephyr_encoder_bind(&g_enc_l, &g_lectx,
				DEVICE_DT_GET(DT_ALIAS(qdec_l)));
	arb_zephyr_encoder_bind(&g_enc_r, &g_rectx,
				DEVICE_DT_GET(DT_ALIAS(qdec_r)));
	arb_zephyr_imu_bind(&g_imu, &g_imctx, DEVICE_DT_GET(DT_ALIAS(robot_imu)));
	arb_imu_start(&g_imu);

	arb_pid_init(&g_pid_l, VEL_KP, VEL_KI, VEL_KD, -1.0f, 1.0f);
	arb_pid_init(&g_pid_r, VEL_KP, VEL_KI, VEL_KD, -1.0f, 1.0f);

	arb_topic_advertise(ARB_TOPIC_CMD_VEL, sizeof(arb_twist_t));
	arb_topic_advertise(ARB_TOPIC_ODOM,    sizeof(arb_odom_t));
	arb_topic_advertise(ARB_TOPIC_IMU,     sizeof(arb_imu_t));
	arb_topic_subscribe(ARB_TOPIC_CMD_VEL, on_cmd_vel, NULL);

	LOG_INF("ARB diff-drive sample running (closed loop)");

	/* Dispatcher thread (CONFIG_ARB_DISPATCH_THREAD) delivers messages;
	 * this loop runs the control + telemetry at a fixed rate. */
	const float dt = CONTROL_PERIOD_MS * 1e-3f;
	for (;;) {
		control_and_odom(dt);
		publish_imu();
		k_msleep(CONTROL_PERIOD_MS);
	}
	return 0;
}
