/*
 * ARB bridge - JAUS (SAE AS-4 / AS5710) interoperability.
 *
 * Makes an ARB node speak a useful subset of JAUS so it can interoperate on a
 * JAUS network (and with OpenJAUS). This is the protocol/codec layer plus the
 * ARB-topic mapping; it is transport-agnostic - actual bytes go out through a
 * caller-supplied send hook (a JUDP socket, a serial link, or the OpenJAUS SDK).
 *
 * Mapping (Mobility + Core service subset):
 *   JAUS SetWrenchEffort        -> ARB /cmd_vel (twist)
 *   ARB  /odom                  -> JAUS ReportVelocityState + ReportLocalPose
 *   JAUS Query{VelocityState,LocalPose} -> the matching Report (last /odom)
 *   JAUS QueryIdentification    -> ReportIdentification
 *   JAUS QueryHeartbeatPulse    -> ReportHeartbeatPulse
 *
 * JAUS messages here are "command code (u16 LE) + payload"; scaled-integer
 * fields use the canonical JAUS mapping (see arb_jaus_scale_*). The transport
 * header (JUDP/AS5669A) is handled by the transport, not by this codec. Field
 * ranges follow the SAE messages but should be verified against the exact stack
 * you interoperate with.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_BRIDGE_JAUS_H
#define ARB_BRIDGE_JAUS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "arb/topic.h"
#include "arb/msg.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief JAUS component address (subsystem / node / component). */
typedef struct {
	uint16_t subsystem;
	uint8_t  node;
	uint8_t  component;
} arb_jaus_addr_t;

/** @brief JAUS message (command) codes - subset we encode/decode. */
enum {
	ARB_JAUS_QUERY_IDENTIFICATION  = 0x2B00,
	ARB_JAUS_REPORT_IDENTIFICATION = 0x4B00,
	ARB_JAUS_QUERY_HEARTBEAT       = 0x2202,
	ARB_JAUS_REPORT_HEARTBEAT      = 0x4202,
	ARB_JAUS_SET_WRENCH_EFFORT     = 0x0405,
	ARB_JAUS_QUERY_VELOCITY_STATE  = 0x2404,
	ARB_JAUS_REPORT_VELOCITY_STATE = 0x4404,
	ARB_JAUS_QUERY_LOCAL_POSE      = 0x2403,
	ARB_JAUS_REPORT_LOCAL_POSE     = 0x4403,
};

/**
 * @brief Transport send hook.
 * @param ctx  Opaque (see arb_jaus_cfg_t::send_ctx).
 * @param dest Destination JAUS address.
 * @param msg  JAUS message bytes (command code + payload).
 * @param len  Length of @p msg.
 * @return 0 on success, negative on failure.
 */
typedef int (*arb_jaus_send_fn)(void *ctx, const arb_jaus_addr_t *dest,
				const uint8_t *msg, size_t len);

/** @brief Bridge configuration. */
typedef struct {
	arb_jaus_addr_t  self;        /**< Our JAUS address.                 */
	arb_jaus_send_fn send;        /**< Transport send hook.              */
	void            *send_ctx;    /**< Passed to @ref send.              */

	arb_topic_id_t cmd_vel_topic; /**< Where decoded commands are published. */
	arb_topic_id_t odom_topic;    /**< Subscribed -> velocity/pose reports.  */

	float max_linear;             /**< m/s mapped to 100% wrench effort. */
	float max_angular;            /**< rad/s mapped to 100% wrench effort.*/

	char  identification[32];     /**< Reported in ReportIdentification. */
} arb_jaus_cfg_t;

/** @brief Bridge instance. */
typedef struct {
	arb_jaus_cfg_t  cfg;
	arb_jaus_addr_t controller;     /**< Last node that queried/commanded us. */
	bool            have_controller;
	arb_odom_t      last_odom;      /**< Cached for query responses.          */
	bool            have_odom;
	uint16_t        seq;            /**< Sequence for emitted reports.        */
} arb_jaus_bridge_t;

/**
 * @brief Initialize the bridge and subscribe it to the odom topic.
 * @return ARB_OK or negative @ref arb_err_t.
 */
int arb_jaus_init(arb_jaus_bridge_t *br, const arb_jaus_cfg_t *cfg);

/**
 * @brief Feed one received JAUS message (already de-framed by the transport).
 * @param src JAUS address the message came from.
 * @param msg Command code (u16 LE) + payload.
 * @param len Length of @p msg.
 * @return ARB_OK if handled, ARB_ERR_NOTFOUND for an unsupported command code,
 *         or another negative @ref arb_err_t.
 */
int arb_jaus_rx(arb_jaus_bridge_t *br, const arb_jaus_addr_t *src,
		const uint8_t *msg, size_t len);

/* ---- scaled-integer helpers (canonical JAUS mapping) ------------------ */

/** @brief Real value -> unsigned scaled integer of @p bits bits. */
uint32_t arb_jaus_scale_to_uint(double v, double lo, double hi, unsigned bits);

/** @brief Unsigned scaled integer of @p bits bits -> real value. */
double   arb_jaus_scale_from_uint(uint32_t i, double lo, double hi,
				  unsigned bits);

/* ---- message builders (return bytes written, or negative) ------------- */

int arb_jaus_build_report_identification(const arb_jaus_bridge_t *br,
					 uint8_t *buf, size_t cap);
int arb_jaus_build_report_velocity_state(const arb_odom_t *odom,
					 uint8_t *buf, size_t cap);
int arb_jaus_build_report_local_pose(const arb_odom_t *odom,
				     uint8_t *buf, size_t cap);
int arb_jaus_build_set_wrench_effort(float linear_pct, float rot_pct,
				     uint8_t *buf, size_t cap);

#ifdef __cplusplus
}
#endif

#endif /* ARB_BRIDGE_JAUS_H */
