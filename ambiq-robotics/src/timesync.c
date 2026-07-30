/*
 * ARB - timesync transport glue (CONFIG_ARB_TIMESYNC).
 *
 * Server role is always on: every REQ is answered in place with t2 (the
 * backend's receive stamp) and t3 (captured immediately before the reply
 * is written), both on this board's arb_time base - the server is the
 * time authority, so it answers in the time base it advertises.
 *
 * Client role (CONFIG_ARB_TIMESYNC_CLIENT) sends a REQ every
 * CONFIG_ARB_TIMESYNC_INTERVAL_MS with t1 captured on the *raw* local
 * clock immediately before transmission, and matches RESPs by t1. The
 * backend's receive stamp is de-corrected back to the raw clock (the
 * backend stamps with arb_time_now_us(), which under ARB_TIME_LINK
 * already contains the applied offset; the offset math needs raw t1/t4).
 *
 * Sync accuracy is bounded by the backend's receive-stamp attribution
 * error (one FIFO batch on the serial backend) plus scheduling noise on
 * t3; the median window absorbs both.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>

#include "arb/msg.h"
#include "arb/time.h"
#include "arb/timesync.h"
#include "arb/topics.h"
#include "arb/transport_backend.h"

LOG_MODULE_DECLARE(arb, CONFIG_ARB_LOG_LEVEL);

static arb_ts_filter_t filter;
static struct k_spinlock ts_lock;

static inline uint64_t raw_uptime_us(void)
{
	return k_ticks_to_us_floor64(k_uptime_ticks());
}

int64_t arb_timesync_offset_us(uint64_t local_now_us)
{
	k_spinlock_key_t key = k_spin_lock(&ts_lock);
	int64_t off = arb_ts_filter_offset(&filter, local_now_us);

	k_spin_unlock(&ts_lock, key);
	return off;
}

bool arb_timesync_synced(void)
{
	k_spinlock_key_t key = k_spin_lock(&ts_lock);
	bool synced = arb_ts_filter_synced(&filter, raw_uptime_us());

	k_spin_unlock(&ts_lock, key);
	return synced;
}

int32_t arb_timesync_drift_ppb(void)
{
	k_spinlock_key_t key = k_spin_lock(&ts_lock);
	int32_t ppb = arb_ts_filter_drift_ppb(&filter);

	k_spin_unlock(&ts_lock, key);
	return ppb;
}

/* ---- client -------------------------------------------------------------- */

#ifdef CONFIG_ARB_TIMESYNC_CLIENT

static uint64_t outstanding_t1;
static uint32_t sync_seq;

static void sync_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(sync_work, sync_work_fn);

static void sync_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	arb_timesync_t req = { 0 };

	req.phase = ARB_TIMESYNC_REQ;
	arb_header_init(&req.header, ARB_MSG_TIMESYNC, 0, arb_time_now_us(),
			sync_seq++);
	/* t1 as late as possible: right before the frame is serialized */
	req.t1_us = raw_uptime_us();
	outstanding_t1 = req.t1_us;
	(void)arb_transport_send_raw(ARB_TOPIC_TIMESYNC, &req, sizeof(req));

	k_work_schedule(&sync_work, K_MSEC(CONFIG_ARB_TIMESYNC_INTERVAL_MS));
}

static int arb_timesync_start(void)
{
	k_work_schedule(&sync_work, K_MSEC(CONFIG_ARB_TIMESYNC_INTERVAL_MS));
	return 0;
}

/* after the transport (same level, later priority) */
SYS_INIT(arb_timesync_start, APPLICATION, 99);

#endif /* CONFIG_ARB_TIMESYNC_CLIENT */

/* ---- rx ------------------------------------------------------------------ */

void arb_timesync_rx(const void *payload, size_t len, uint64_t rx_stamp_us)
{
	if (len != sizeof(arb_timesync_t)) {
		LOG_WRN("timesync frame size %u", (unsigned)len);
		return;
	}

	const arb_timesync_t *m = payload;

	if (m->phase == ARB_TIMESYNC_REQ) {
		arb_timesync_t resp = { 0 };

		resp.phase = ARB_TIMESYNC_RESP;
		resp.t1_us = m->t1_us;
		resp.t2_us = rx_stamp_us;
		arb_header_init(&resp.header, ARB_MSG_TIMESYNC, 0,
				rx_stamp_us, m->header.seq);
		/* t3 as late as possible: right before serialization */
		resp.t3_us = arb_time_now_us();
		(void)arb_transport_send_raw(ARB_TOPIC_TIMESYNC, &resp,
					     sizeof(resp));
		return;
	}

#ifdef CONFIG_ARB_TIMESYNC_CLIENT
	if (m->phase == ARB_TIMESYNC_RESP) {
		if (m->t1_us == 0 || m->t1_us != outstanding_t1) {
			return; /* stale or unsolicited */
		}
		outstanding_t1 = 0;

		k_spinlock_key_t key = k_spin_lock(&ts_lock);
		/* de-correct the backend stamp back to the raw clock */
		uint64_t t4 = rx_stamp_us -
			      (uint64_t)arb_ts_filter_applied(&filter);
		bool ok = arb_ts_filter_sample(&filter, m->t1_us, m->t2_us,
					       m->t3_us, t4, raw_uptime_us());

		k_spin_unlock(&ts_lock, key);
		if (!ok) {
			LOG_DBG("timesync sample rejected");
		}
	}
#endif
}

static int arb_timesync_init(void)
{
	arb_ts_filter_init(&filter, CONFIG_ARB_TIMESYNC_WINDOW,
			   CONFIG_ARB_TIMESYNC_MAX_SLEW_PPM,
			   (uint64_t)CONFIG_ARB_TIMESYNC_INTERVAL_MS * 5000U);
	return 0;
}

/* before the transport can deliver frames */
SYS_INIT(arb_timesync_init, POST_KERNEL, 99);
