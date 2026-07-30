/*
 * ARB - UDP transport backend loopback test (native_sim, NSOS).
 *
 * The backend is configured with peer 127.0.0.1:<own port>, so every
 * exported frame arrives right back on the same socket: one publish must
 * produce exactly one received frame (the loop guard keeps the local
 * republication from being exported again, which would echo forever).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <zephyr/ztest.h>
#include <zephyr/zbus/zbus.h>

#include "arb/topics.h"
#include "arb/time.h"
#include "arb/transport.h"

ZTEST(arb_transport_udp, test_udp_loopback)
{
	uint32_t frames = 0, crc = 0;

	zassert_ok(arb_transport_export(&arb_chan_pose2d));

	arb_pose2d_t p = { 0 };

	arb_header_init(&p.header, ARB_MSG_POSE2D, 1, arb_time_now_us(), 42);
	p.pose.x = 1.5f;
	p.pose.y = -0.25f;
	p.pose.theta = 0.5f;
	zassert_ok(zbus_chan_pub(&arb_chan_pose2d, &p, K_MSEC(10)));

	/* wait for the datagram to loop back through the host stack */
	for (int i = 0; i < 100; i++) {
		arb_transport_stats(&frames, &crc);
		if (frames >= 1) {
			break;
		}
		k_sleep(K_MSEC(20));
	}
	zassert_equal(frames, 1, "expected 1 looped frame, got %u", frames);
	zassert_equal(crc, 0, "crc errors on loopback");

	/* the republication must not have been re-exported (echo loop) */
	k_sleep(K_MSEC(300));
	arb_transport_stats(&frames, &crc);
	zassert_equal(frames, 1, "echo loop: frame count grew to %u", frames);

	/* the loopback republished our message onto the channel */
	arb_pose2d_t r;

	zassert_ok(zbus_chan_read(&arb_chan_pose2d, &r, K_MSEC(10)));
	zassert_equal(r.header.seq, 42);
	zassert_within(r.pose.x, 1.5f, 1e-6f);

	zassert_ok(arb_transport_unexport(&arb_chan_pose2d));
}

ZTEST_SUITE(arb_transport_udp, NULL, NULL, NULL, NULL, NULL);
