/*
 * ARB Zephyr port - micro-ROS bridge public interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_BRIDGE_MICRO_ROS_H
#define ARB_BRIDGE_MICRO_ROS_H

#include <rcl/rcl.h>
#include <rclc/executor.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Wire the standard ARB topics to a micro-ROS node.
 *
 * Creates ROS2 publishers (/odom, /imu), a subscriber (/cmd_vel), and ARB
 * subscribers that forward outbound traffic. The caller owns the rclc support,
 * node and executor and is responsible for spinning the executor.
 *
 * @param node     Initialized rcl node.
 * @param executor Initialized rclc executor with room for >=1 handle.
 * @return ARB_OK or a negative @ref arb_err_t.
 */
int arb_micro_ros_bridge_init(rcl_node_t *node, rclc_executor_t *executor);

#ifdef __cplusplus
}
#endif

#endif /* ARB_BRIDGE_MICRO_ROS_H */
