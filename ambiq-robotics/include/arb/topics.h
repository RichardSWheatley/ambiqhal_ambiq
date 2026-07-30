/*
 * ARB - well-known topics as zbus channels
 *
 * Every ARB topic is a statically-defined zbus channel carrying exactly one
 * message struct from arb/msg.h. Publish with zbus_chan_pub(), observe with
 * ZBUS_LISTENER_DEFINE / ZBUS_SUBSCRIBER_DEFINE — there is no ARB-private
 * pub/sub layer.
 *
 * Each channel also has a stable 16-bit wire id used by the serial transport
 * (arb/transport.h) so frames stay compatible with the multi-OS ARB tree.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_TOPICS_H
#define ARB_TOPICS_H

#include <zephyr/zbus/zbus.h>

#include "arb/msg.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Wire ids for the well-known channels.
 *
 * Match the topic numbering used by the multi-OS ARB samples so both trees
 * interoperate over the same serial link. Application channels use ids at
 * @ref ARB_TOPIC_USER_BASE and above (see arb_transport_export()).
 */
enum arb_topic_id {
	ARB_TOPIC_CMD_VEL   = 1,  /**< arb_twist_t       */
	ARB_TOPIC_ODOM      = 2,  /**< arb_odom_t        */
	ARB_TOPIC_IMU       = 3,  /**< arb_imu_t         */
	ARB_TOPIC_MOTOR_CMD = 4,  /**< arb_motor_cmd_t   */
	ARB_TOPIC_ENCODER   = 5,  /**< arb_encoder_msg_t */
	ARB_TOPIC_BATTERY   = 6,  /**< arb_battery_t     */
	ARB_TOPIC_RANGE     = 7,  /**< arb_range_t       */
	ARB_TOPIC_POSE2D    = 8,  /**< arb_pose2d_t      */
	ARB_TOPIC_LOG       = 9,  /**< arb_log_t         */
	ARB_TOPIC_HEARTBEAT = 10, /**< arb_heartbeat_t   */
	ARB_TOPIC_ESTOP     = 11, /**< arb_estop_t       */
	/**
	 * arb_timesync_t. Transport-reserved: has no zbus channel - the
	 * transport core consumes these frames directly (arb/timesync.h), so
	 * sync traffic never round-trips through the channel layer.
	 */
	ARB_TOPIC_TIMESYNC  = 12,

	ARB_TOPIC_USER_BASE = 0x1000,
};

ZBUS_CHAN_DECLARE(arb_chan_cmd_vel);   /* arb_twist_t       */
ZBUS_CHAN_DECLARE(arb_chan_odom);      /* arb_odom_t        */
ZBUS_CHAN_DECLARE(arb_chan_imu);       /* arb_imu_t         */
ZBUS_CHAN_DECLARE(arb_chan_motor_cmd); /* arb_motor_cmd_t   */
ZBUS_CHAN_DECLARE(arb_chan_encoder);   /* arb_encoder_msg_t */
ZBUS_CHAN_DECLARE(arb_chan_battery);   /* arb_battery_t     */
ZBUS_CHAN_DECLARE(arb_chan_range);     /* arb_range_t       */
ZBUS_CHAN_DECLARE(arb_chan_pose2d);    /* arb_pose2d_t      */
ZBUS_CHAN_DECLARE(arb_chan_log);       /* arb_log_t         */
ZBUS_CHAN_DECLARE(arb_chan_heartbeat); /* arb_heartbeat_t   */
ZBUS_CHAN_DECLARE(arb_chan_estop);     /* arb_estop_t       */

/**
 * @brief Look up the zbus channel for a wire id.
 * @return Channel, or NULL if the id is unknown.
 */
const struct zbus_channel *arb_topic_chan(uint16_t id);

/**
 * @brief Look up the wire id for a zbus channel.
 * @return Wire id, or 0 if the channel is not a registered ARB topic.
 */
uint16_t arb_topic_id(const struct zbus_channel *chan);

#ifdef __cplusplus
}
#endif

#endif /* ARB_TOPICS_H */
