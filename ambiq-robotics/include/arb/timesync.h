/*
 * ARB - serial timesync: offset/drift filter and transport glue.
 *
 * The filter (timesync_filter.c) is pure C with no OS calls so the math is
 * unit-testable on native_sim. The glue (timesync.c, CONFIG_ARB_TIMESYNC)
 * wires it to the transport: the core hands ARB_TOPIC_TIMESYNC frames to
 * arb_timesync_rx(), a client work item sends periodic REQs, and the
 * ARB_TIME_LINK time source reads the disciplined offset.
 *
 * Discipline model: the *offset applied to stamps* is disciplined, never
 * the kernel clock. The applied offset slews toward the filtered target at
 * a bounded rate, so link time never steps backward once sync is acquired
 * (the very first accepted sample may step once, in either direction).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_TIMESYNC_H
#define ARB_TIMESYNC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- pure filter --------------------------------------------------------- */

#define ARB_TS_WINDOW_MAX 15

typedef struct {
	/* configuration */
	uint8_t  window;        /**< samples in the median window (3..15) */
	uint32_t max_slew_ppm;  /**< max applied-offset slew rate         */
	uint64_t fresh_us;      /**< max sample age to report synced      */

	/* sample ring */
	int64_t  offset_hist[ARB_TS_WINDOW_MAX];
	uint64_t delay_hist[ARB_TS_WINDOW_MAX];
	uint8_t  head;
	uint8_t  n;

	/* state */
	int64_t  target_offset_us;  /**< median of the window            */
	int64_t  applied_offset_us; /**< what stamps actually get        */
	bool     have_applied;
	uint64_t last_slew_local_us;
	uint64_t last_sample_local_us;
	int32_t  drift_ppb;         /**< EMA of d(target)/d(local)       */
} arb_ts_filter_t;

void arb_ts_filter_init(arb_ts_filter_t *f, uint8_t window,
			uint32_t max_slew_ppm, uint64_t fresh_us);

/**
 * @brief Feed one completed exchange.
 *
 * All timestamps in microseconds; t1/t4 on the local (raw, undisciplined)
 * clock, t2/t3 on the peer's clock.
 *
 * @return true if the sample was accepted, false if rejected (negative
 *         path delay, or delay > 2x the window's median delay - a spike).
 */
bool arb_ts_filter_sample(arb_ts_filter_t *f, uint64_t t1, uint64_t t2,
			  uint64_t t3, uint64_t t4, uint64_t local_now_us);

/**
 * @brief Offset to add to a raw local time, slewing toward the target.
 *
 * Mutates slew state; serialize calls (the glue holds a spinlock).
 */
int64_t arb_ts_filter_offset(arb_ts_filter_t *f, uint64_t local_now_us);

/** @brief Applied offset without slewing (no state change). */
int64_t arb_ts_filter_applied(const arb_ts_filter_t *f);

bool arb_ts_filter_synced(const arb_ts_filter_t *f, uint64_t local_now_us);
int32_t arb_ts_filter_drift_ppb(const arb_ts_filter_t *f);

/* ---- transport glue (CONFIG_ARB_TIMESYNC) -------------------------------- */

/**
 * @brief Handle a received ARB_TOPIC_TIMESYNC frame.
 *
 * Called by the transport core. REQ frames are answered in place (server
 * role, always on); RESP frames feed the client filter.
 *
 * @param payload     Frame payload (arb_timesync_t).
 * @param len         Payload length.
 * @param rx_stamp_us Backend receive stamp for this frame.
 */
void arb_timesync_rx(const void *payload, size_t len, uint64_t rx_stamp_us);

/** @brief Disciplined offset for a raw local microsecond time. */
int64_t arb_timesync_offset_us(uint64_t local_now_us);

/** @brief Whether the client filter currently holds fresh sync. */
bool arb_timesync_synced(void);

/** @brief Estimated local-vs-peer drift, parts per billion. */
int32_t arb_timesync_drift_ppb(void);

#ifdef __cplusplus
}
#endif

#endif /* ARB_TIMESYNC_H */
