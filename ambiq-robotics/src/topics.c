/*
 * ARB - zbus channel definitions for the well-known topics
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <zephyr/zbus/zbus.h>

#include "arb/topics.h"

BUILD_ASSERT(sizeof(arb_twist_t)       <= ARB_MSG_MAX_SIZE);
BUILD_ASSERT(sizeof(arb_odom_t)        <= ARB_MSG_MAX_SIZE);
BUILD_ASSERT(sizeof(arb_imu_t)         <= ARB_MSG_MAX_SIZE);
BUILD_ASSERT(sizeof(arb_motor_cmd_t)   <= ARB_MSG_MAX_SIZE);
BUILD_ASSERT(sizeof(arb_encoder_msg_t) <= ARB_MSG_MAX_SIZE);
BUILD_ASSERT(sizeof(arb_battery_t)     <= ARB_MSG_MAX_SIZE);
BUILD_ASSERT(sizeof(arb_range_t)       <= ARB_MSG_MAX_SIZE);
BUILD_ASSERT(sizeof(arb_pose2d_t)      <= ARB_MSG_MAX_SIZE);
BUILD_ASSERT(sizeof(arb_log_t)         <= ARB_MSG_MAX_SIZE);
BUILD_ASSERT(sizeof(arb_heartbeat_t)   <= ARB_MSG_MAX_SIZE);
BUILD_ASSERT(sizeof(arb_estop_t)       <= ARB_MSG_MAX_SIZE);

#define ARB_CHAN(_name, _type)                                                \
	ZBUS_CHAN_DEFINE(_name, _type, NULL, NULL, ZBUS_OBSERVERS_EMPTY,      \
			 ZBUS_MSG_INIT(0))

ARB_CHAN(arb_chan_cmd_vel,   arb_twist_t);
ARB_CHAN(arb_chan_odom,      arb_odom_t);
ARB_CHAN(arb_chan_imu,       arb_imu_t);
ARB_CHAN(arb_chan_motor_cmd, arb_motor_cmd_t);
ARB_CHAN(arb_chan_encoder,   arb_encoder_msg_t);
ARB_CHAN(arb_chan_battery,   arb_battery_t);
ARB_CHAN(arb_chan_range,     arb_range_t);
ARB_CHAN(arb_chan_pose2d,    arb_pose2d_t);
ARB_CHAN(arb_chan_log,       arb_log_t);
ARB_CHAN(arb_chan_heartbeat, arb_heartbeat_t);
ARB_CHAN(arb_chan_estop,     arb_estop_t);

struct arb_topic_map {
	uint16_t                   id;
	const struct zbus_channel *chan;
};

static const struct arb_topic_map map[] = {
	{ ARB_TOPIC_CMD_VEL,   &arb_chan_cmd_vel   },
	{ ARB_TOPIC_ODOM,      &arb_chan_odom      },
	{ ARB_TOPIC_IMU,       &arb_chan_imu       },
	{ ARB_TOPIC_MOTOR_CMD, &arb_chan_motor_cmd },
	{ ARB_TOPIC_ENCODER,   &arb_chan_encoder   },
	{ ARB_TOPIC_BATTERY,   &arb_chan_battery   },
	{ ARB_TOPIC_RANGE,     &arb_chan_range     },
	{ ARB_TOPIC_POSE2D,    &arb_chan_pose2d    },
	{ ARB_TOPIC_LOG,       &arb_chan_log       },
	{ ARB_TOPIC_HEARTBEAT, &arb_chan_heartbeat },
	{ ARB_TOPIC_ESTOP,     &arb_chan_estop     },
};

const struct zbus_channel *arb_topic_chan(uint16_t id)
{
	for (size_t i = 0; i < ARRAY_SIZE(map); i++) {
		if (map[i].id == id) {
			return map[i].chan;
		}
	}
	return NULL;
}

uint16_t arb_topic_id(const struct zbus_channel *chan)
{
	for (size_t i = 0; i < ARRAY_SIZE(map); i++) {
		if (map[i].chan == chan) {
			return map[i].id;
		}
	}
	return 0;
}
