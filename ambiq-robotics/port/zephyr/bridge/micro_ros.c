/*
 * ARB Zephyr port - micro-ROS bridge.
 *
 * Mirrors internal ARB topics onto a micro-ROS node so the Apollo510 appears as
 * a ROS2 node on the network:
 *
 *   ROS2  /cmd_vel  (geometry_msgs/Twist)  --> ARB ARB_TOPIC_CMD_VEL
 *   ARB   ARB_TOPIC_ODOM                   --> ROS2 /odom  (nav_msgs/Odometry)
 *   ARB   ARB_TOPIC_IMU                    --> ROS2 /imu   (sensor_msgs/Imu)
 *
 * Inbound ROS messages are injected with arb_topic_publish(); outbound ARB
 * messages are captured by ARB subscribers and republished via rcl. Build with
 * CONFIG_ARB_MICRO_ROS_BRIDGE=y (requires the micro-ROS Zephyr module).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <string.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>

#include "arb/topic.h"
#include "arb/msg.h"
#include "bridge/micro_ros.h"

/* These topic ids are shared with the application (see samples). */
#ifndef ARB_TOPIC_CMD_VEL
#define ARB_TOPIC_CMD_VEL 1
#endif
#ifndef ARB_TOPIC_ODOM
#define ARB_TOPIC_ODOM 2
#endif
#ifndef ARB_TOPIC_IMU
#define ARB_TOPIC_IMU 3
#endif

struct bridge {
	rcl_publisher_t  odom_pub;
	rcl_publisher_t  imu_pub;
	rcl_subscription_t cmd_vel_sub;

	geometry_msgs__msg__Twist cmd_vel_in;
	nav_msgs__msg__Odometry   odom_out;
	sensor_msgs__msg__Imu     imu_out;
};

static struct bridge s_br;

/* ---- ROS2 -> ARB: /cmd_vel subscription callback ---------------------- */

static void cmd_vel_cb(const void *msgin)
{
	const geometry_msgs__msg__Twist *t = msgin;

	arb_twist_t cmd;
	arb_header_init(&cmd.header, ARB_MSG_TWIST, 0, 0, 0);
	cmd.linear.x  = (float)t->linear.x;
	cmd.linear.y  = (float)t->linear.y;
	cmd.linear.z  = (float)t->linear.z;
	cmd.angular.x = (float)t->angular.x;
	cmd.angular.y = (float)t->angular.y;
	cmd.angular.z = (float)t->angular.z;

	arb_topic_publish(ARB_TOPIC_CMD_VEL, &cmd, sizeof(cmd));
}

/* ---- ARB -> ROS2: odom / imu subscribers ------------------------------ */

static void odom_to_ros(arb_topic_id_t topic, const void *msg, size_t len,
			void *user)
{
	ARG_UNUSED(topic);
	ARG_UNUSED(user);
	if (len != sizeof(arb_odom_t)) {
		return;
	}
	const arb_odom_t *o = msg;

	nav_msgs__msg__Odometry *r = &s_br.odom_out;
	r->pose.pose.position.x = o->pose.x;
	r->pose.pose.position.y = o->pose.y;
	/* yaw -> quaternion (planar). */
	r->pose.pose.orientation.z = (double)sinf(o->pose.theta * 0.5f);
	r->pose.pose.orientation.w = (double)cosf(o->pose.theta * 0.5f);
	r->twist.twist.linear.x  = o->linear_vel;
	r->twist.twist.angular.z = o->angular_vel;

	(void)rcl_publish(&s_br.odom_pub, r, NULL);
}

static void imu_to_ros(arb_topic_id_t topic, const void *msg, size_t len,
		       void *user)
{
	ARG_UNUSED(topic);
	ARG_UNUSED(user);
	if (len != sizeof(arb_imu_t)) {
		return;
	}
	const arb_imu_t *s = msg;

	sensor_msgs__msg__Imu *r = &s_br.imu_out;
	r->linear_acceleration.x = s->accel.x;
	r->linear_acceleration.y = s->accel.y;
	r->linear_acceleration.z = s->accel.z;
	r->angular_velocity.x = s->gyro.x;
	r->angular_velocity.y = s->gyro.y;
	r->angular_velocity.z = s->gyro.z;

	(void)rcl_publish(&s_br.imu_pub, r, NULL);
}

/* ---- public init ------------------------------------------------------ */

int arb_micro_ros_bridge_init(rcl_node_t *node, rclc_executor_t *executor)
{
	if (!node || !executor) {
		return ARB_ERR_INVAL;
	}

	rcl_ret_t rc;

	rc = rclc_publisher_init_default(
		&s_br.odom_pub, node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");
	if (rc != RCL_RET_OK) {
		return ARB_ERR_AGAIN;
	}

	rc = rclc_publisher_init_default(
		&s_br.imu_pub, node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu");
	if (rc != RCL_RET_OK) {
		return ARB_ERR_AGAIN;
	}

	rc = rclc_subscription_init_default(
		&s_br.cmd_vel_sub, node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"cmd_vel");
	if (rc != RCL_RET_OK) {
		return ARB_ERR_AGAIN;
	}

	rc = rclc_executor_add_subscription(executor, &s_br.cmd_vel_sub,
					    &s_br.cmd_vel_in, cmd_vel_cb,
					    ON_NEW_DATA);
	if (rc != RCL_RET_OK) {
		return ARB_ERR_AGAIN;
	}

	/* Outbound: forward internal ARB topics to ROS. */
	arb_topic_subscribe(ARB_TOPIC_ODOM, odom_to_ros, NULL);
	arb_topic_subscribe(ARB_TOPIC_IMU, imu_to_ros, NULL);

	return ARB_OK;
}
