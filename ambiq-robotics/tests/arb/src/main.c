/*
 * ARB - unit tests (native_sim / qemu).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <zephyr/ztest.h>
#include <zephyr/zbus/zbus.h>

#include "arb/topics.h"
#include "arb/node.h"
#include "arb/serial.h"
#include "arb/control/pid.h"
#include "arb/control/diff_drive.h"

/* ---- serial codec ------------------------------------------------------- */

static uint8_t  loop_buf[256];
static size_t   loop_len;
static uint16_t got_topic;
static uint8_t  got_payload[ARB_MSG_MAX_SIZE];
static size_t   got_len;

static int loop_write(void *ctx, const uint8_t *buf, size_t len)
{
	ARG_UNUSED(ctx);
	zassert_true(loop_len + len <= sizeof(loop_buf));
	memcpy(&loop_buf[loop_len], buf, len);
	loop_len += len;
	return 0;
}

static void frame_cb(uint16_t topic, const void *payload, size_t len,
		     void *user)
{
	ARG_UNUSED(user);
	got_topic = topic;
	got_len   = len;
	memcpy(got_payload, payload, len);
}

ZTEST(arb, test_serial_roundtrip)
{
	arb_serial_t tx, rx;
	uint8_t payload[24];

	for (size_t i = 0; i < sizeof(payload); i++) {
		payload[i] = (uint8_t)(i * 7 + 1);
	}

	loop_len = 0;
	got_len = 0;
	zassert_ok(arb_serial_init(&tx, loop_write, NULL, NULL, NULL));
	zassert_ok(arb_serial_init(&rx, NULL, NULL, frame_cb, NULL));

	zassert_ok(arb_serial_send(&tx, ARB_TOPIC_IMU, payload,
				   sizeof(payload)));
	/* leading garbage must resync */
	arb_serial_rx_byte(&rx, 0x00);
	arb_serial_rx_byte(&rx, 0xFF);
	arb_serial_rx(&rx, loop_buf, loop_len);

	zassert_equal(rx.rx_frames, 1);
	zassert_equal(rx.rx_crc_errors, 0);
	zassert_equal(got_topic, ARB_TOPIC_IMU);
	zassert_equal(got_len, sizeof(payload));
	zassert_mem_equal(got_payload, payload, sizeof(payload));
}

ZTEST(arb, test_serial_crc_reject)
{
	arb_serial_t tx, rx;
	uint8_t payload[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };

	loop_len = 0;
	zassert_ok(arb_serial_init(&tx, loop_write, NULL, NULL, NULL));
	zassert_ok(arb_serial_init(&rx, NULL, NULL, frame_cb, NULL));

	zassert_ok(arb_serial_send(&tx, ARB_TOPIC_ODOM, payload,
				   sizeof(payload)));
	loop_buf[6] ^= 0x55; /* corrupt payload */
	arb_serial_rx(&rx, loop_buf, loop_len);

	zassert_equal(rx.rx_frames, 0);
	zassert_equal(rx.rx_crc_errors, 1);
}

ZTEST(arb, test_crc16_vector)
{
	/* CRC-16/CCITT-FALSE("123456789") = 0x29B1 */
	uint16_t crc = arb_serial_crc16(0xFFFF, (const uint8_t *)"123456789",
					9);
	zassert_equal(crc, 0x29B1, "crc=0x%04x", crc);
}

/* ---- control ------------------------------------------------------------ */

ZTEST(arb, test_pid_converges)
{
	arb_pid_t pid;
	float plant = 0.0f;

	arb_pid_init(&pid, 0.8f, 4.0f, 0.0f, -10.0f, 10.0f);
	for (int i = 0; i < 400; i++) {
		float u = arb_pid_update(&pid, 5.0f, plant, 0.01f);

		plant += 0.5f * u * 0.01f + (5.0f - plant) * 0.0f;
		plant += 0.0f;
		plant  = plant + 0.5f * (u - plant) * 0.01f; /* lag plant */
	}
	zassert_within(plant, 5.0f, 0.5f, "plant=%f", (double)plant);
}

ZTEST(arb, test_diffdrive_inverse)
{
	const arb_diffdrive_t dd = { .wheel_base = 0.3f, .wheel_radius = 0.05f };
	float wl, wr, v, w;

	arb_diffdrive_twist_to_wheels(&dd, 0.5f, 1.2f, &wl, &wr);
	arb_diffdrive_wheels_to_twist(&dd, wl, wr, &v, &w);

	zassert_within(v, 0.5f, 1e-4f);
	zassert_within(w, 1.2f, 1e-4f);
}

/* ---- node lifecycle ----------------------------------------------------- */

static int calls_cfg, calls_act, calls_shdn;
static int cfg_hook(void *ctx)  { ARG_UNUSED(ctx); calls_cfg++;  return ARB_OK; }
static int act_hook(void *ctx)  { ARG_UNUSED(ctx); calls_act++;  return ARB_OK; }
static int shdn_hook(void *ctx) { ARG_UNUSED(ctx); calls_shdn++; return ARB_OK; }

static const arb_node_ops_t ops = {
	.on_configure = cfg_hook,
	.on_activate  = act_hook,
	.on_shutdown  = shdn_hook,
};

ZTEST(arb, test_node_lifecycle_and_estop)
{
	arb_node_t *n = arb_node_create("t_node", &ops, NULL);

	zassert_not_null(n);
	zassert_equal(n->state, ARB_NODE_INIT);

	/* invalid: activate before configure */
	zassert_equal(arb_node_activate(n), ARB_ERR_STATE);

	zassert_ok(arb_node_configure(n));
	zassert_equal(n->state, ARB_NODE_READY);
	zassert_ok(arb_node_activate(n));
	zassert_equal(n->state, ARB_NODE_ACTIVE);
	zassert_equal(calls_cfg, 1);
	zassert_equal(calls_act, 1);

	zassert_ok(arb_node_estop_all(n->id, 0x42));
	zassert_equal(n->state, ARB_NODE_ESTOP);
	zassert_equal(calls_shdn, 1);

	/* estop message landed on the channel */
	arb_estop_t es;

	zassert_ok(zbus_chan_read(&arb_chan_estop, &es, K_MSEC(10)));
	zassert_equal(es.source, n->id);
	zassert_equal(es.reason, 0x42);
	zassert_equal(es.header.type, ARB_MSG_ESTOP);
}

/* ---- topics ------------------------------------------------------------- */

ZTEST(arb, test_topic_map)
{
	zassert_equal(arb_topic_chan(ARB_TOPIC_CMD_VEL), &arb_chan_cmd_vel);
	zassert_equal(arb_topic_id(&arb_chan_imu), ARB_TOPIC_IMU);
	zassert_is_null(arb_topic_chan(0x7777));
	zassert_equal(arb_topic_id(NULL), 0);
}

ZTEST(arb, test_zbus_pub_read)
{
	arb_twist_t t = { 0 };

	arb_header_init(&t.header, ARB_MSG_TWIST, 0, 123, 9);
	t.linear.x  = 0.25f;
	t.angular.z = -0.5f;
	zassert_ok(zbus_chan_pub(&arb_chan_cmd_vel, &t, K_MSEC(10)));

	arb_twist_t r;

	zassert_ok(zbus_chan_read(&arb_chan_cmd_vel, &r, K_MSEC(10)));
	zassert_within(r.linear.x, 0.25f, 1e-6f);
	zassert_within(r.angular.z, -0.5f, 1e-6f);
	zassert_equal(r.header.seq, 9);
}

/* ---- wire layout --------------------------------------------------------- */

/*
 * The 16-byte header and the framing are shared with peers built from the
 * multi-OS ARB tree; these are the wire-compatibility invariants.
 */
BUILD_ASSERT(sizeof(arb_header_t) == 16);
BUILD_ASSERT(offsetof(arb_header_t, stamp_us) == 0);
BUILD_ASSERT(offsetof(arb_header_t, seq) == 8);
BUILD_ASSERT(offsetof(arb_header_t, type) == 12);
BUILD_ASSERT(offsetof(arb_header_t, source) == 14);
BUILD_ASSERT(sizeof(arb_heartbeat_t) == 24);
BUILD_ASSERT(offsetof(arb_heartbeat_t, flags) == 19); /* the old pad byte */
BUILD_ASSERT(sizeof(arb_timesync_t) == 48);
BUILD_ASSERT(sizeof(arb_twist_t) <= ARB_MSG_MAX_SIZE);
BUILD_ASSERT(sizeof(arb_odom_t) <= ARB_MSG_MAX_SIZE);
BUILD_ASSERT(sizeof(arb_imu_t) <= ARB_MSG_MAX_SIZE);
BUILD_ASSERT(sizeof(arb_motor_cmd_t) <= ARB_MSG_MAX_SIZE);
BUILD_ASSERT(sizeof(arb_encoder_msg_t) <= ARB_MSG_MAX_SIZE);
BUILD_ASSERT(sizeof(arb_battery_t) <= ARB_MSG_MAX_SIZE);
BUILD_ASSERT(sizeof(arb_range_t) <= ARB_MSG_MAX_SIZE);
BUILD_ASSERT(sizeof(arb_pose2d_t) <= ARB_MSG_MAX_SIZE);
BUILD_ASSERT(sizeof(arb_log_t) <= ARB_MSG_MAX_SIZE);
BUILD_ASSERT(sizeof(arb_estop_t) <= ARB_MSG_MAX_SIZE);

ZTEST(arb, test_wire_layout)
{
	/* runtime mirrors of the key compile-time guards, for visibility */
	zassert_equal(sizeof(arb_header_t), 16);
	zassert_equal(offsetof(arb_header_t, seq), 8);
	zassert_equal(offsetof(arb_header_t, type), 12);
	zassert_equal(offsetof(arb_header_t, source), 14);
	zassert_equal(sizeof(arb_heartbeat_t), 24);
	zassert_true(sizeof(arb_log_t) <= ARB_MSG_MAX_SIZE);
}

ZTEST_SUITE(arb, NULL, NULL, NULL, NULL, NULL);
