/*
 * ARB - transport core (backend-agnostic).
 *
 * Bridges exported zbus channels over whatever backend is linked
 * (transport_serial.c, transport_udp.c): outbound publications are framed
 * and written in the publisher's context; inbound bytes arrive via
 * arb_transport_core_rx() and complete frames are re-published onto the
 * matching channel.
 *
 * Note on TX: the write happens synchronously in the publishing context
 * and may block for the frame's serialization time (~683 us for an IMU
 * frame at 921600 baud). This is deliberate - the seam is the deliverable,
 * the e2e_cmd_vel benchmark measures the cost, and a queued-TX backend can
 * be added behind the same write op without touching this file.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

#include "arb/serial.h"
#include "arb/topics.h"
#include "arb/transport.h"
#include "arb/transport_backend.h"

#ifdef CONFIG_ARB_TIMESYNC
#include "arb/timesync.h"
#endif

LOG_MODULE_DECLARE(arb, CONFIG_ARB_LOG_LEVEL);

static arb_serial_t codec;
static K_MUTEX_DEFINE(tx_lock);

/*
 * Timestamp the backend attributed to the bytes of the frame currently
 * being decoded; consumed by the timesync handler (t2/t4).
 */
static uint64_t rx_frame_stamp_us;

/*
 * Thread republishing a received frame: its own publication must not be
 * exported back out (echo loop). Tracked per-thread so concurrent local
 * publishers are unaffected.
 */
static k_tid_t rx_pub_tid;

/* ---- TX ----------------------------------------------------------------- */

static int backend_write(void *ctx, const uint8_t *buf, size_t len)
{
	ARG_UNUSED(ctx);
	return arb_transport_backend.write(buf, len);
}

int arb_transport_send_raw(uint16_t topic, const void *payload, size_t len)
{
	k_mutex_lock(&tx_lock, K_FOREVER);
	int rc = arb_serial_send(&codec, topic, payload, len);

	k_mutex_unlock(&tx_lock);
	return rc;
}

/* zbus listener: frame and transmit every publication on exported channels. */
static void export_cb(const struct zbus_channel *chan)
{
	uint16_t id = arb_topic_id(chan);

	if (id == 0 || k_current_get() == rx_pub_tid) {
		return;
	}

	k_mutex_lock(&tx_lock, K_FOREVER);
	(void)arb_serial_send(&codec, id, zbus_chan_const_msg(chan),
			      zbus_chan_msg_size(chan));
	k_mutex_unlock(&tx_lock);
}

ZBUS_LISTENER_DEFINE(arb_transport_listener, export_cb);

int arb_transport_export(const struct zbus_channel *chan)
{
	if (!chan || arb_topic_id(chan) == 0) {
		return ARB_ERR_INVAL;
	}
	int rc = zbus_chan_add_obs(chan, &arb_transport_listener, K_MSEC(100));

	return (rc == 0 || rc == -EEXIST || rc == -EALREADY) ? ARB_OK :
							       ARB_ERR_AGAIN;
}

int arb_transport_unexport(const struct zbus_channel *chan)
{
	if (!chan) {
		return ARB_ERR_INVAL;
	}
	return zbus_chan_rm_obs(chan, &arb_transport_listener, K_MSEC(100)) ==
			       0 ? ARB_OK : ARB_ERR_NOTFOUND;
}

/* ---- RX ----------------------------------------------------------------- */

/* Complete frame (backend rx context): publish on the mapped channel. */
static void frame_cb(uint16_t topic, const void *payload, size_t len,
		     void *user)
{
	ARG_UNUSED(user);

#ifdef CONFIG_ARB_TIMESYNC
	if (topic == ARB_TOPIC_TIMESYNC) {
		arb_timesync_rx(payload, len, rx_frame_stamp_us);
		return;
	}
#endif

	const struct zbus_channel *chan = arb_topic_chan(topic);

	if (!chan) {
		LOG_WRN("rx frame for unknown topic %u", topic);
		return;
	}
	if (len != zbus_chan_msg_size(chan)) {
		LOG_WRN("rx frame size %u != %u for topic %u", (unsigned)len,
			(unsigned)zbus_chan_msg_size(chan), topic);
		return;
	}

	rx_pub_tid = k_current_get();
	(void)zbus_chan_pub(chan, payload, K_MSEC(5));
	rx_pub_tid = NULL;
}

void arb_transport_core_rx(const uint8_t *buf, size_t len,
			   uint64_t rx_stamp_us)
{
	rx_frame_stamp_us = rx_stamp_us;
	arb_serial_rx(&codec, buf, len);
}

void arb_transport_stats(uint32_t *rx_frames, uint32_t *rx_crc_errors)
{
	if (rx_frames) {
		*rx_frames = codec.rx_frames;
	}
	if (rx_crc_errors) {
		*rx_crc_errors = codec.rx_crc_errors;
	}
}

/* ---- init --------------------------------------------------------------- */

static int arb_transport_init(void)
{
	(void)arb_serial_init(&codec, backend_write, NULL, frame_cb, NULL);

	int rc = arb_transport_backend.init();

	if (rc != 0) {
		LOG_ERR("transport backend init failed (%d)", rc);
		return rc;
	}
	return 0;
}

SYS_INIT(arb_transport_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
