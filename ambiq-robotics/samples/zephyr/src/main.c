/*
 * ARB sample (Zephyr): differential-drive base.
 *
 * Mirrors the bare-metal sample but uses the Zephyr device model for the HAL
 * bindings and a dedicated dispatcher thread for delivery. Expects the following
 * devicetree aliases (provide a board overlay):
 *
 *   pwm-motor-l / pwm-motor-r   : PWM channels
 *   gpio dir pins               : via the overlay's &gpioX nodes (optional)
 *   qdec-l / qdec-r             : quadrature decoder sensor nodes
 *   robot-imu                   : 6-axis IMU sensor node
 *
 * If the dispatcher thread is enabled (default), main() only needs to publish;
 * delivery happens on the ARB thread.
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
#include "hal_bind.h"

LOG_MODULE_REGISTER(arb_sample, LOG_LEVEL_INF);

#define ARB_TOPIC_CMD_VEL 1
#define ARB_TOPIC_ODOM    2
#define ARB_TOPIC_IMU     3

#define WHEEL_BASE_M       0.20f
#define WHEEL_RADIUS_M     0.033f
#define MAX_WHEEL_SPEED_MS 0.6f

static const struct pwm_dt_spec pwm_l = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor_l));
static const struct pwm_dt_spec pwm_r = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor_r));

static arb_motor_t   g_left, g_right;
static arb_encoder_t g_enc_l, g_enc_r;
static arb_imu_t_inst g_imu;

static arb_zephyr_motor_ctx_t   g_lmctx, g_rmctx;
static arb_zephyr_encoder_ctx_t g_lectx, g_rectx;
static arb_zephyr_imu_ctx_t     g_imctx;

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

	float v = t->linear.x;
	float w = t->angular.z;
	float v_l = v - w * (WHEEL_BASE_M * 0.5f);
	float v_r = v + w * (WHEEL_BASE_M * 0.5f);

	arb_motor_set_duty(&g_left,  v_l / MAX_WHEEL_SPEED_MS);
	arb_motor_set_duty(&g_right, v_r / MAX_WHEEL_SPEED_MS);
}

static void publish_odom(void)
{
	arb_encoder_reading_t rl, rr;
	if (arb_encoder_read(&g_enc_l, &rl) != ARB_OK ||
	    arb_encoder_read(&g_enc_r, &rr) != ARB_OK) {
		return;
	}

	float vl = rl.velocity_rad_s * WHEEL_RADIUS_M;
	float vr = rr.velocity_rad_s * WHEEL_RADIUS_M;
	float v  = 0.5f * (vl + vr);
	float w  = (vr - vl) / WHEEL_BASE_M;

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

	arb_topic_advertise(ARB_TOPIC_CMD_VEL, sizeof(arb_twist_t));
	arb_topic_advertise(ARB_TOPIC_ODOM,    sizeof(arb_odom_t));
	arb_topic_advertise(ARB_TOPIC_IMU,     sizeof(arb_imu_t));
	arb_topic_subscribe(ARB_TOPIC_CMD_VEL, on_cmd_vel, NULL);

	LOG_INF("ARB diff-drive sample running");

	/* Dispatcher thread (CONFIG_ARB_DISPATCH_THREAD) delivers messages;
	 * this loop just produces telemetry at 50 Hz. */
	for (;;) {
		publish_odom();
		publish_imu();
		k_msleep(20);
	}
	return 0;
}
