/*
 * Ambiq Robotics Broker (ARB) - message layer
 *
 * Flat, statically-allocated message structs with integer type IDs. This is the
 * MCU-sized analog of ROS2 .msg files / JAUS message sets: no IDL codegen, no
 * dynamic allocation, every message is a POD struct that maps 1:1 onto a zbus
 * channel and can be framed onto a serial link unchanged.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_MSG_H
#define ARB_MSG_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Well-known message type identifiers.
 *
 * Values 0..0x0FFF are reserved for the broker / common robotics messages.
 * Application-specific message types should start at @ref ARB_MSG_USER_BASE.
 */
typedef enum {
	ARB_MSG_NONE       = 0,
	ARB_MSG_TWIST      = 1,  /**< @ref arb_twist_t   - velocity command (cmd_vel) */
	ARB_MSG_ODOM       = 2,  /**< @ref arb_odom_t    - wheel/fused odometry        */
	ARB_MSG_IMU        = 3,  /**< @ref arb_imu_t     - inertial sample             */
	ARB_MSG_MOTOR_CMD  = 4,  /**< @ref arb_motor_cmd_t - per-motor duty command    */
	ARB_MSG_ENCODER    = 5,  /**< @ref arb_encoder_msg_t - encoder state           */
	ARB_MSG_BATTERY    = 6,  /**< @ref arb_battery_t - battery state               */
	ARB_MSG_RANGE      = 7,  /**< @ref arb_range_t   - range / proximity reading   */
	ARB_MSG_POSE2D     = 8,  /**< @ref arb_pose2d_t  - stamped 2D pose             */
	ARB_MSG_LOG        = 9,  /**< @ref arb_log_t     - text log line               */
	ARB_MSG_HEARTBEAT  = 10, /**< @ref arb_heartbeat_t - node liveness             */
	ARB_MSG_ESTOP      = 11, /**< @ref arb_estop_t   - emergency stop              */
	ARB_MSG_TIMESYNC   = 12, /**< @ref arb_timesync_t - link time sync exchange    */

	ARB_MSG_USER_BASE  = 0x1000,
} arb_msg_type_t;

/** @brief Severity levels used by @ref arb_log_t. */
typedef enum {
	ARB_LOG_DEBUG = 0,
	ARB_LOG_INFO  = 1,
	ARB_LOG_WARN  = 2,
	ARB_LOG_ERROR = 3,
} arb_log_level_t;

/**
 * @brief Common header carried by every stamped message.
 *
 * Kept first in each message struct so generic tooling can read it without
 * knowing the concrete type.
 */
typedef struct {
	uint64_t stamp_us; /**< Source timestamp in us (arb_time_now_us()).    */
	uint32_t seq;      /**< Per-publisher monotonic sequence number.        */
	uint16_t type;     /**< One of @ref arb_msg_type_t.                     */
	uint16_t source;   /**< Node / source identifier (0 = unspecified).     */
} arb_header_t;

/* ---- Plain data types (no header) -------------------------------------- */

typedef struct { float x, y, z; } arb_vec3_t;
typedef struct { float w, x, y, z; } arb_quat_t;
typedef struct { float x, y, theta; } arb_pose2d_data_t;

/* ---- Stamped message types -------------------------------------------- */

/** @brief Body-frame velocity command (subset of geometry_msgs/Twist). */
typedef struct {
	arb_header_t header;
	arb_vec3_t   linear;  /**< m/s   */
	arb_vec3_t   angular; /**< rad/s */
} arb_twist_t;

/** @brief Planar odometry: pose + body-frame velocity. */
typedef struct {
	arb_header_t      header;
	arb_pose2d_data_t pose;        /**< x[m], y[m], theta[rad] */
	float             linear_vel;  /**< forward velocity, m/s  */
	float             angular_vel; /**< yaw rate, rad/s        */
} arb_odom_t;

/** @brief Inertial measurement sample. */
typedef struct {
	arb_header_t header;
	arb_vec3_t   accel;       /**< m/s^2  */
	arb_vec3_t   gyro;        /**< rad/s  */
	arb_vec3_t   mag;         /**< uT (zero if not present) */
	float        temperature; /**< deg C  */
} arb_imu_t;

/** @brief Per-motor open-loop duty command. */
typedef struct {
	arb_header_t header;
	uint8_t      motor_id;
	int16_t      duty_q15; /**< signed Q15: -32768 (full reverse) .. 32767 */
} arb_motor_cmd_t;

/** @brief Encoder state for a single channel. */
typedef struct {
	arb_header_t header;
	uint8_t      encoder_id;
	int32_t      count;          /**< raw signed tick count   */
	float        position_rad;   /**< accumulated angle, rad  */
	float        velocity_rad_s; /**< angular velocity, rad/s */
} arb_encoder_msg_t;

/** @brief Battery / power-rail state. */
typedef struct {
	arb_header_t header;
	float        voltage;    /**< V         */
	float        current;    /**< A (signed; negative = charging) */
	float        percentage; /**< 0.0 .. 1.0 */
} arb_battery_t;

/** @brief Single range / proximity reading. */
typedef struct {
	arb_header_t header;
	uint8_t      sensor_id;
	float        range_m; /**< meters (INFINITY = out of range) */
} arb_range_t;

/** @brief Stamped 2D pose. */
typedef struct {
	arb_header_t      header;
	arb_pose2d_data_t pose;
} arb_pose2d_t;

/**
 * @brief Short text log line, fixed capacity (no heap).
 *
 * text is sized so the whole struct (with alignment padding) fits inside
 * ARB_MSG_MAX_SIZE and can be framed onto the wire.
 */
typedef struct {
	arb_header_t header;
	uint8_t      level;    /**< @ref arb_log_level_t */
	char         text[40];
} arb_log_t;

/** @name Heartbeat flags / time source advertisement
 * @{
 */
#define ARB_HB_FLAG_TIME_SYNCED   0x01u /**< arb_time_synced() was true      */
#define ARB_HB_TIME_SOURCE_MASK   0x06u /**< bits 1-2: time source           */
#define ARB_HB_TIME_SOURCE_UPTIME (0u << 1)
#define ARB_HB_TIME_SOURCE_LINK   (1u << 1)
#define ARB_HB_TIME_SOURCE_PTP    (2u << 1)
/** @} */

/**
 * @brief Node liveness beacon, published on @c arb_chan_heartbeat.
 *
 * @c flags occupies what used to be a padding byte, so the wire layout is
 * unchanged; peers that predate it always see 0 there.
 */
typedef struct {
	arb_header_t header;
	uint16_t     node;      /**< node id                          */
	uint8_t      state;     /**< @ref arb_node_state_t (node.h)   */
	uint8_t      flags;     /**< ARB_HB_FLAG_* | time source bits */
	uint32_t     uptime_ms; /**< node uptime                      */
} arb_heartbeat_t;

/** @brief Emergency stop event, published on @c arb_chan_estop. */
typedef struct {
	arb_header_t header;
	uint16_t     source; /**< node id that raised the stop (0 = external) */
	uint8_t      reason; /**< application-defined                          */
} arb_estop_t;

/** @name Timesync exchange phases (@ref arb_timesync_t) @{ */
#define ARB_TIMESYNC_REQ  0u
#define ARB_TIMESYNC_RESP 1u
/** @} */

/**
 * @brief NTP-style four-timestamp exchange (wire id ARB_TOPIC_TIMESYNC).
 *
 * Client sends REQ with t1 (client clock, captured immediately before
 * transmission); server replies RESP echoing t1, with t2 = its receive
 * stamp and t3 captured immediately before the reply is transmitted (both
 * server clock). The client's receive stamp is t4. Offset to the server's
 * clock: ((t2-t1)+(t3-t4))/2. REQ and RESP frames are the same length, so
 * the serialization delay is symmetric and cancels in that math.
 */
typedef struct {
	arb_header_t header;
	uint64_t     t1_us; /**< client TX time (client clock)      */
	uint64_t     t2_us; /**< server RX time (server clock)      */
	uint64_t     t3_us; /**< server TX time (server clock)      */
	uint8_t      phase; /**< ARB_TIMESYNC_REQ / ARB_TIMESYNC_RESP */
} arb_timesync_t;

/**
 * @brief Largest payload the transport must be able to frame.
 *
 * Keep all message structs at or below it; compile-time guards live in
 * topics.c.
 */
#define ARB_MSG_MAX_SIZE 64u

/**
 * @brief Initialize a message header in place.
 *
 * @param h        Header to fill (must be the first member of the message).
 * @param type     @ref arb_msg_type_t value.
 * @param source   Source/node id (0 if unused).
 * @param stamp_us Timestamp (typically arb_time_now_us(), see arb/time.h).
 * @param seq      Sequence number (caller-managed per publisher).
 */
static inline void arb_header_init(arb_header_t *h, uint16_t type,
				   uint16_t source, uint64_t stamp_us,
				   uint32_t seq)
{
	h->stamp_us = stamp_us;
	h->seq      = seq;
	h->type     = type;
	h->source   = source;
}

#ifdef __cplusplus
}
#endif

#endif /* ARB_MSG_H */
