/*
 * ARB bridge - JAUS codec + ARB-topic mapping (transport-agnostic).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <string.h>

#include "arb/bridge/jaus.h"
#include "arb/platform.h"

/* ---- little-endian byte helpers --------------------------------------- */

static void put_u16(uint8_t *b, size_t *off, uint16_t v)
{
	b[(*off)++] = (uint8_t)(v & 0xFF);
	b[(*off)++] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *b, size_t *off, uint32_t v)
{
	b[(*off)++] = (uint8_t)(v & 0xFF);
	b[(*off)++] = (uint8_t)((v >> 8) & 0xFF);
	b[(*off)++] = (uint8_t)((v >> 16) & 0xFF);
	b[(*off)++] = (uint8_t)((v >> 24) & 0xFF);
}

static uint16_t get_u16(const uint8_t *b, size_t *off)
{
	uint16_t v = (uint16_t)(b[*off] | (b[*off + 1] << 8));
	*off += 2;
	return v;
}

static uint32_t get_u32(const uint8_t *b, size_t *off)
{
	uint32_t v = (uint32_t)b[*off] | ((uint32_t)b[*off + 1] << 8) |
		     ((uint32_t)b[*off + 2] << 16) |
		     ((uint32_t)b[*off + 3] << 24);
	*off += 4;
	return v;
}

/* ---- scaled integers -------------------------------------------------- */

uint32_t arb_jaus_scale_to_uint(double v, double lo, double hi, unsigned bits)
{
	double span = (double)((bits >= 32) ? 0xFFFFFFFFu
					    : ((1u << bits) - 1u));
	if (v < lo) {
		v = lo;
	}
	if (v > hi) {
		v = hi;
	}
	double scaled = (v - lo) / (hi - lo) * span + 0.5; /* round */
	return (uint32_t)scaled;
}

double arb_jaus_scale_from_uint(uint32_t i, double lo, double hi, unsigned bits)
{
	double span = (double)((bits >= 32) ? 0xFFFFFFFFu
					    : ((1u << bits) - 1u));
	return (double)i / span * (hi - lo) + lo;
}

/* JAUS field ranges (SAE messages). Verify against your interop target. */
#define EFFORT_LO   (-100.0)
#define EFFORT_HI   (100.0)
#define VEL_LO      (-327.68)
#define VEL_HI      (327.67)
#define RATE_LO     (-32.768)
#define RATE_HI     (32.767)
#define POS_LO      (-100000.0)
#define POS_HI      (100000.0)
#define ANGLE_LO    (-3.14159265358979)
#define ANGLE_HI    (3.14159265358979)

/* ---- builders --------------------------------------------------------- */

int arb_jaus_build_set_wrench_effort(float linear_pct, float rot_pct,
				     uint8_t *buf, size_t cap)
{
	if (cap < 7) {
		return ARB_ERR_TOOBIG;
	}
	size_t off = 0;
	put_u16(buf, &off, ARB_JAUS_SET_WRENCH_EFFORT);
	/* presence vector (2 bytes): bit0 PropulsiveLinearEffortX,
	 * bit5 PropulsiveRotationalEffortZ */
	put_u16(buf, &off, (uint16_t)((1u << 0) | (1u << 5)));
	put_u16(buf, &off,
		(uint16_t)arb_jaus_scale_to_uint(linear_pct, EFFORT_LO,
						 EFFORT_HI, 16));
	put_u16(buf, &off,
		(uint16_t)arb_jaus_scale_to_uint(rot_pct, EFFORT_LO,
						 EFFORT_HI, 16));
	return (int)off;
}

int arb_jaus_build_report_velocity_state(const arb_odom_t *odom,
					 uint8_t *buf, size_t cap)
{
	if (cap < 12) {
		return ARB_ERR_TOOBIG;
	}
	size_t off = 0;
	put_u16(buf, &off, ARB_JAUS_REPORT_VELOCITY_STATE);
	/* PV: bit0 VelocityX, bit6 YawRate */
	put_u16(buf, &off, (uint16_t)((1u << 0) | (1u << 6)));
	put_u32(buf, &off,
		arb_jaus_scale_to_uint(odom->linear_vel, VEL_LO, VEL_HI, 32));
	put_u16(buf, &off,
		(uint16_t)arb_jaus_scale_to_uint(odom->angular_vel, RATE_LO,
						 RATE_HI, 16));
	return (int)off;
}

int arb_jaus_build_report_local_pose(const arb_odom_t *odom,
				     uint8_t *buf, size_t cap)
{
	if (cap < 14) {
		return ARB_ERR_TOOBIG;
	}
	size_t off = 0;
	put_u16(buf, &off, ARB_JAUS_REPORT_LOCAL_POSE);
	/* PV: bit0 X, bit1 Y, bit6 Yaw */
	put_u16(buf, &off, (uint16_t)((1u << 0) | (1u << 1) | (1u << 6)));
	put_u32(buf, &off,
		arb_jaus_scale_to_uint(odom->pose.x, POS_LO, POS_HI, 32));
	put_u32(buf, &off,
		arb_jaus_scale_to_uint(odom->pose.y, POS_LO, POS_HI, 32));
	put_u16(buf, &off,
		(uint16_t)arb_jaus_scale_to_uint(odom->pose.theta, ANGLE_LO,
						 ANGLE_HI, 16));
	return (int)off;
}

int arb_jaus_build_report_identification(const arb_jaus_bridge_t *br,
					 uint8_t *buf, size_t cap)
{
	size_t name_len = strnlen(br->cfg.identification,
				  sizeof(br->cfg.identification));
	if (cap < 2 + 1 + 2 + 1 + name_len) {
		return ARB_ERR_TOOBIG;
	}
	size_t off = 0;
	put_u16(buf, &off, ARB_JAUS_REPORT_IDENTIFICATION);
	buf[off++] = 0;                        /* QueryType: system */
	put_u16(buf, &off, 10001);             /* Type: vehicle (example) */
	buf[off++] = (uint8_t)name_len;        /* identification count */
	memcpy(&buf[off], br->cfg.identification, name_len);
	off += name_len;
	return (int)off;
}

/* ---- emit helper ------------------------------------------------------ */

static void send_to(arb_jaus_bridge_t *br, const arb_jaus_addr_t *dest,
		    const uint8_t *msg, int len)
{
	if (len > 0 && br->cfg.send) {
		br->cfg.send(br->cfg.send_ctx, dest, msg, (size_t)len);
	}
}

static void emit_odom_reports(arb_jaus_bridge_t *br, const arb_jaus_addr_t *dest)
{
	uint8_t buf[32];
	int n;

	n = arb_jaus_build_report_velocity_state(&br->last_odom, buf,
						 sizeof(buf));
	send_to(br, dest, buf, n);

	n = arb_jaus_build_report_local_pose(&br->last_odom, buf, sizeof(buf));
	send_to(br, dest, buf, n);
}

/* ---- ARB /odom subscriber: cache + report to controller --------------- */

static void on_odom(arb_topic_id_t topic, const void *msg, size_t len,
		    void *user)
{
	(void)topic;
	arb_jaus_bridge_t *br = (arb_jaus_bridge_t *)user;
	if (len != sizeof(arb_odom_t)) {
		return;
	}
	br->last_odom = *(const arb_odom_t *)msg;
	br->have_odom = true;

	if (br->have_controller) {
		emit_odom_reports(br, &br->controller);
	}
}

/* ---- init ------------------------------------------------------------- */

int arb_jaus_init(arb_jaus_bridge_t *br, const arb_jaus_cfg_t *cfg)
{
	if (!br || !cfg || !cfg->send) {
		return ARB_ERR_INVAL;
	}
	memset(br, 0, sizeof(*br));
	br->cfg = *cfg;
	if (br->cfg.max_linear <= 0.0f) {
		br->cfg.max_linear = 1.0f;
	}
	if (br->cfg.max_angular <= 0.0f) {
		br->cfg.max_angular = 1.0f;
	}
	return arb_topic_subscribe(cfg->odom_topic, on_odom, br);
}

/* ---- rx --------------------------------------------------------------- */

static void decode_set_wrench(arb_jaus_bridge_t *br, const arb_jaus_addr_t *src,
			      const uint8_t *msg, size_t len)
{
	/* The node driving us also becomes the telemetry controller. */
	br->controller = *src;
	br->have_controller = true;

	size_t off = 2; /* skip command code */
	if (len < off + 2) {
		return;
	}
	uint16_t pv = get_u16(msg, &off);

	float lin_pct = 0.0f, rot_pct = 0.0f;
	if (pv & (1u << 0)) {
		if (len < off + 2) {
			return;
		}
		lin_pct = (float)arb_jaus_scale_from_uint(get_u16(msg, &off),
							  EFFORT_LO, EFFORT_HI,
							  16);
	}
	/* skip linear Y, Z if present (bits 1,2) */
	if (pv & (1u << 1)) { off += 2; }
	if (pv & (1u << 2)) { off += 2; }
	/* skip rotational X, Y (bits 3,4) */
	if (pv & (1u << 3)) { off += 2; }
	if (pv & (1u << 4)) { off += 2; }
	if (pv & (1u << 5)) {
		if (len < off + 2) {
			return;
		}
		rot_pct = (float)arb_jaus_scale_from_uint(get_u16(msg, &off),
							  EFFORT_LO, EFFORT_HI,
							  16);
	}

	/* effort percent -> body twist */
	arb_twist_t tw;
	memset(&tw, 0, sizeof(tw));
	arb_header_init(&tw.header, ARB_MSG_TWIST, br->cfg.self.subsystem,
			arb_platform_time_us(), br->seq++);
	tw.linear.x  = (lin_pct / 100.0f) * br->cfg.max_linear;
	tw.angular.z = (rot_pct / 100.0f) * br->cfg.max_angular;
	arb_topic_publish(br->cfg.cmd_vel_topic, &tw, sizeof(tw));
}

int arb_jaus_rx(arb_jaus_bridge_t *br, const arb_jaus_addr_t *src,
		const uint8_t *msg, size_t len)
{
	if (!br || !src || !msg || len < 2) {
		return ARB_ERR_INVAL;
	}

	size_t off = 0;
	uint16_t code = get_u16(msg, &off);
	uint8_t buf[32];
	int n;

	switch (code) {
	case ARB_JAUS_SET_WRENCH_EFFORT:
		decode_set_wrench(br, src, msg, len);
		return ARB_OK;

	case ARB_JAUS_QUERY_VELOCITY_STATE:
	case ARB_JAUS_QUERY_LOCAL_POSE:
		/* Remember the requester and answer with cached odom. */
		br->controller = *src;
		br->have_controller = true;
		if (br->have_odom) {
			if (code == ARB_JAUS_QUERY_VELOCITY_STATE) {
				n = arb_jaus_build_report_velocity_state(
					&br->last_odom, buf, sizeof(buf));
			} else {
				n = arb_jaus_build_report_local_pose(
					&br->last_odom, buf, sizeof(buf));
			}
			send_to(br, src, buf, n);
		}
		return ARB_OK;

	case ARB_JAUS_QUERY_IDENTIFICATION:
		n = arb_jaus_build_report_identification(br, buf, sizeof(buf));
		send_to(br, src, buf, n);
		return ARB_OK;

	case ARB_JAUS_QUERY_HEARTBEAT: {
		size_t o = 0;
		put_u16(buf, &o, ARB_JAUS_REPORT_HEARTBEAT);
		send_to(br, src, buf, (int)o);
		return ARB_OK;
	}

	default:
		return ARB_ERR_NOTFOUND;
	}
}
