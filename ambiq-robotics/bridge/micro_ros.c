/*
 * ARB - micro-ROS bridge.
 *
 * Mirrors the ARB zbus channels into a ROS 2 graph via micro-ROS:
 *   arb_chan_imu  -> sensor_msgs/msg/Imu      on "imu/data_raw"
 *   arb_chan_odom -> nav_msgs-free minimal    on "arb/odom" (geometry only)
 *   "cmd_vel" (geometry_msgs/msg/Twist)       -> arb_chan_cmd_vel
 *
 * Requires the micro_ros_zephyr module in the west workspace and a configured
 * micro-ROS transport (serial or UDP) - see that module's documentation.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/pose2_d.h>
#include <sensor_msgs/msg/imu.h>

#include "arb/topics.h"

LOG_MODULE_DECLARE(arb, CONFIG_ARB_LOG_LEVEL);

static rcl_allocator_t  allocator;
static rclc_support_t   support;
static rcl_node_t       ros_node;
static rclc_executor_t  executor;

static rcl_publisher_t          pub_imu;
static rcl_publisher_t          pub_pose;
static rcl_subscription_t       sub_cmd_vel;
static geometry_msgs__msg__Twist cmd_vel_ros;

static uint32_t bridge_seq;

static uint64_t stamp_us(void)
{
	return k_ticks_to_us_floor64(k_uptime_ticks());
}

/* ---- ROS 2 -> zbus ------------------------------------------------------ */

static void ros_cmd_vel_cb(const void *msgin)
{
	const geometry_msgs__msg__Twist *in = msgin;
	arb_twist_t out = { 0 };

	arb_header_init(&out.header, ARB_MSG_TWIST, 0, stamp_us(),
			bridge_seq++);
	out.linear.x  = (float)in->linear.x;
	out.linear.y  = (float)in->linear.y;
	out.linear.z  = (float)in->linear.z;
	out.angular.x = (float)in->angular.x;
	out.angular.y = (float)in->angular.y;
	out.angular.z = (float)in->angular.z;

	(void)zbus_chan_pub(&arb_chan_cmd_vel, &out, K_MSEC(5));
}

/* ---- zbus -> ROS 2 ------------------------------------------------------ */

static void bridge_out_cb(const struct zbus_channel *chan)
{
	if (chan == &arb_chan_imu) {
		const arb_imu_t *imu = zbus_chan_const_msg(chan);
		sensor_msgs__msg__Imu out;

		memset(&out, 0, sizeof(out));
		out.linear_acceleration.x = imu->accel.x;
		out.linear_acceleration.y = imu->accel.y;
		out.linear_acceleration.z = imu->accel.z;
		out.angular_velocity.x    = imu->gyro.x;
		out.angular_velocity.y    = imu->gyro.y;
		out.angular_velocity.z    = imu->gyro.z;
		out.orientation_covariance[0] = -1.0; /* no orientation */

		(void)rcl_publish(&pub_imu, &out, NULL);
	} else if (chan == &arb_chan_odom) {
		const arb_odom_t *od = zbus_chan_const_msg(chan);
		geometry_msgs__msg__Pose2D out;

		out.x     = od->pose.x;
		out.y     = od->pose.y;
		out.theta = od->pose.theta;

		(void)rcl_publish(&pub_pose, &out, NULL);
	}
}

ZBUS_LISTENER_DEFINE(arb_uros_listener, bridge_out_cb);
ZBUS_CHAN_ADD_OBS(arb_chan_imu, arb_uros_listener, 4);
ZBUS_CHAN_ADD_OBS(arb_chan_odom, arb_uros_listener, 4);

/* ---- executor thread ---------------------------------------------------- */

static void uros_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	allocator = rcl_get_default_allocator();

	if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
		LOG_ERR("micro-ROS support init failed (agent up?)");
		return;
	}
	rclc_node_init_default(&ros_node, "arb_bridge", "", &support);

	rclc_publisher_init_best_effort(
		&pub_imu, &ros_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"imu/data_raw");
	rclc_publisher_init_best_effort(
		&pub_pose, &ros_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D),
		"arb/pose");
	rclc_subscription_init_default(
		&sub_cmd_vel, &ros_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"cmd_vel");

	rclc_executor_init(&executor, &support.context, 1, &allocator);
	rclc_executor_add_subscription(&executor, &sub_cmd_vel, &cmd_vel_ros,
				       ros_cmd_vel_cb, ON_NEW_DATA);

	LOG_INF("micro-ROS bridge up");

	while (1) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		k_msleep(1);
	}
}

K_THREAD_DEFINE(arb_uros_tid, CONFIG_ARB_MICRO_ROS_STACK_SIZE, uros_thread,
		NULL, NULL, NULL, CONFIG_ARB_MICRO_ROS_PRIORITY, 0, 500);
