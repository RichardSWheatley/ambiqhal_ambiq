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

/* counts pose2d publications: local publish + loopback republication */
static volatile uint32_t pose_pubs;

static void pose_count_cb(const struct zbus_channel *chan)
{
	ARG_UNUSED(chan);
	pose_pubs++;
}
ZBUS_LISTENER_DEFINE(pose_count_listener, pose_count_cb);
ZBUS_CHAN_ADD_OBS(arb_chan_pose2d, pose_count_listener, 4);

ZTEST(arb_transport_udp, test_udp_loopback)
{
	uint32_t crc = 0, frames = 0;

	zassert_ok(arb_transport_export(&arb_chan_pose2d));
	pose_pubs = 0;

	arb_pose2d_t p = { 0 };

	arb_header_init(&p.header, ARB_MSG_POSE2D, 1, arb_time_now_us(), 42);
	p.pose.x = 1.5f;
	p.pose.y = -0.25f;
	p.pose.theta = 0.5f;
	zassert_ok(zbus_chan_pub(&arb_chan_pose2d, &p, K_MSEC(10)));

	/*
	 * Expect exactly 2 publications: ours, plus the republication of
	 * the frame after it loops back through the host stack. (Transport
	 * stats can't be used here - timesync traffic also counts frames.)
	 */
	for (int i = 0; i < 100 && pose_pubs < 2; i++) {
		k_sleep(K_MSEC(20));
	}
	zassert_equal(pose_pubs, 2, "expected 2 pose2d pubs, got %u",
		      pose_pubs);

	/* the republication must not have been re-exported (echo loop) */
	k_sleep(K_MSEC(500));
	zassert_equal(pose_pubs, 2, "echo loop: pub count grew to %u",
		      pose_pubs);

	arb_transport_stats(&frames, &crc);
	zassert_equal(crc, 0, "crc errors on loopback");

	/* the loopback republished our message onto the channel */
	arb_pose2d_t r;

	zassert_ok(zbus_chan_read(&arb_chan_pose2d, &r, K_MSEC(10)));
	zassert_equal(r.header.seq, 42);
	zassert_within(r.pose.x, 1.5f, 1e-6f);

	zassert_ok(arb_transport_unexport(&arb_chan_pose2d));
}

#ifdef CONFIG_ARB_TIME_LINK
/*
 * Self-sync: the client's REQ loops back, this board answers as server,
 * the RESP loops back again and feeds the filter. Both ends are the same
 * clock, so the disciplined offset must converge to ~0 and the LINK time
 * source must report synced.
 */
ZTEST(arb_transport_udp, test_udp_self_timesync)
{
	bool synced = false;

	for (int i = 0; i < 100; i++) {
		if (arb_time_synced()) {
			synced = true;
			break;
		}
		k_sleep(K_MSEC(100));
	}
	zassert_true(synced, "timesync never reached synced state");

	int64_t skew = (int64_t)arb_time_now_us() -
		       (int64_t)k_ticks_to_us_floor64(k_uptime_ticks());

	zassert_true(skew > -1000 && skew < 1000,
		     "self-sync offset %lld us, expected ~0", (long long)skew);
}
#endif /* CONFIG_ARB_TIME_LINK */

ZTEST_SUITE(arb_transport_udp, NULL, NULL, NULL, NULL, NULL);
