/*
 * ARB - timesync filter unit tests (pure math, no transport).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <zephyr/ztest.h>

#include "arb/timesync.h"

#define WINDOW   8
#define SLEW_PPM 1000
#define FRESH_US 5000000

/*
 * Build one exchange with a given true offset (server = client + offset)
 * and symmetric one-way delay: t1 at local time "at".
 */
static void feed(arb_ts_filter_t *f, uint64_t at, int64_t true_offset,
		 uint64_t one_way_us, bool expect_ok)
{
	uint64_t t1 = at;
	uint64_t t2 = (uint64_t)((int64_t)(t1 + one_way_us) + true_offset);
	uint64_t t3 = t2 + 50; /* server turnaround */
	uint64_t t4 = (uint64_t)((int64_t)t3 - true_offset) + one_way_us;

	bool ok = arb_ts_filter_sample(f, t1, t2, t3, t4, t4);

	zassert_equal(ok, expect_ok, "sample at %llu: ok=%d expected=%d",
		      (unsigned long long)at, ok, expect_ok);
}

ZTEST(arb_timesync, test_offset_exact)
{
	arb_ts_filter_t f;

	arb_ts_filter_init(&f, WINDOW, SLEW_PPM, FRESH_US);

	/* symmetric path: offset must be recovered exactly */
	feed(&f, 1000000, 250000, 300, true);
	zassert_equal(f.target_offset_us, 250000);
	/* first sample steps the applied offset (pre-sync step) */
	zassert_equal(arb_ts_filter_applied(&f), 250000);

	/* negative offsets work too */
	arb_ts_filter_init(&f, WINDOW, SLEW_PPM, FRESH_US);
	feed(&f, 1000000, -73000, 450, true);
	zassert_equal(f.target_offset_us, -73000);
}

ZTEST(arb_timesync, test_delay_spike_rejected)
{
	arb_ts_filter_t f;
	uint64_t at = 1000000;

	arb_ts_filter_init(&f, WINDOW, SLEW_PPM, FRESH_US);

	for (int i = 0; i < 4; i++) {
		feed(&f, at, 1000, 300, true);
		at += 1000000;
	}
	/* delay spike (10x median): must be rejected, target unchanged */
	feed(&f, at, 500000, 3000, false);
	zassert_equal(f.target_offset_us, 1000);

	/* negative path delay is garbage: rejected */
	zassert_false(arb_ts_filter_sample(&f, 100, 200, 300, 50, at));
}

ZTEST(arb_timesync, test_median_robustness)
{
	arb_ts_filter_t f;
	uint64_t at = 1000000;

	arb_ts_filter_init(&f, WINDOW, SLEW_PPM, FRESH_US);

	/* majority at 5000, one asymmetric-delay outlier at 9000 (same
	 * total delay, so the spike filter passes it - the median kills it)
	 */
	for (int i = 0; i < 5; i++) {
		feed(&f, at, 5000, 300, true);
		at += 1000000;
	}
	feed(&f, at, 9000, 300, true);
	zassert_equal(f.target_offset_us, 5000);
}

ZTEST(arb_timesync, test_slew_monotonic)
{
	arb_ts_filter_t f;
	uint64_t at = 1000000;

	arb_ts_filter_init(&f, WINDOW, SLEW_PPM, FRESH_US);

	/* sync at +10000, then the target steps down 5000 us */
	for (int i = 0; i < 3; i++) {
		feed(&f, at, 10000, 300, true);
		at += 1000000;
	}
	for (int i = 0; i < 5; i++) {
		feed(&f, at, 5000, 300, true);
		at += 1000000;
	}
	zassert_equal(f.target_offset_us, 5000);

	/* link time = local + offset must never decrease while slewing */
	uint64_t prev_link = 0;
	int64_t off = arb_ts_filter_applied(&f);

	zassert_true(off > 5000, "applied should still be above target");
	for (int i = 0; i < 20000; i++) {
		at += 1000;
		uint64_t link = at + (uint64_t)arb_ts_filter_offset(&f, at);

		zassert_true(link > prev_link,
			     "link time ran backward at i=%d", i);
		prev_link = link;
	}
	/* and it must have converged to the target by now */
	zassert_equal(arb_ts_filter_applied(&f), 5000);
}

ZTEST(arb_timesync, test_drift_estimate)
{
	arb_ts_filter_t f;
	uint64_t at = 1000000;

	arb_ts_filter_init(&f, 3, SLEW_PPM, FRESH_US);

	/* peer clock gains 100 us per second: +100000 ppb */
	int64_t off = 0;

	for (int i = 0; i < 20; i++) {
		feed(&f, at, off, 300, true);
		at += 1000000;
		off += 100;
	}
	int32_t ppb = arb_ts_filter_drift_ppb(&f);

	zassert_true(ppb > 50000 && ppb < 150000,
		     "drift estimate %d ppb, expected ~100000", ppb);
}

ZTEST(arb_timesync, test_synced_freshness)
{
	arb_ts_filter_t f;
	uint64_t at = 1000000;

	arb_ts_filter_init(&f, 3, SLEW_PPM, FRESH_US);
	zassert_false(arb_ts_filter_synced(&f, at));

	for (int i = 0; i < 3; i++) {
		feed(&f, at, 1000, 300, true);
		at += 1000000;
	}
	zassert_true(arb_ts_filter_synced(&f, at));
	/* stale after fresh_us with no samples */
	zassert_false(arb_ts_filter_synced(&f, at + FRESH_US + 1000000));
}

ZTEST_SUITE(arb_timesync, NULL, NULL, NULL, NULL, NULL);
