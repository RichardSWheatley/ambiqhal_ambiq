/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared scaffolding for the five Weaver isolation scenarios.
 *
 * Each scenario constructs its own thread set, runs for a fixed
 * window, then reports a single-line result. main.c runs them
 * sequentially so one flash + one log capture exercises all five.
 *
 * Each scenario is intentionally MINIMAL - it isolates one Weaver
 * feature (pressure boost, pre-Warp guard, predictive fill, fuzzy
 * throttle, aging) so the metric improvement can be attributed to
 * the right design choice. Compare against samples/.../weaver_benchmark
 * for the integrated wearable workload.
 */

#ifndef WEAVER_SCEN_COMMON_H
#define WEAVER_SCEN_COMMON_H

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#ifdef CONFIG_WEAVER_SCHED
#include <zephyr/kernel/weaver_sched.h>
#endif

#define SCEN_STACK_SIZE   1536

/* Per-scenario result captured by the scenario function and printed
 * by main(). Each scenario fills only the fields it uses; the rest
 * stay zero.
 */
struct scen_result {
	const char *name;
	bool        pass;             /* PASS / FAIL relative to threshold */
	uint32_t    overruns;         /* count of dropped samples */
	uint32_t    latency_p50_us;
	uint32_t    latency_p99_us;
	uint32_t    latency_max_us;
	uint32_t    deadline_misses;
	uint32_t    throttle_peak;    /* peak fuzzy throttle level (0-255) */
	uint32_t    promotions;
	uint32_t    misc;             /* scenario-specific counter */
	const char *misc_label;
};

#if defined(CONFIG_WEAVER_SCHED)
#  define BUILD_LABEL "weaver"
#else
#  define BUILD_LABEL "stock"
#endif

/* Minimal P50/P99/max accumulator: 16 bucket histogram. */
#define HIST_BUCKETS 16
struct lat_hist {
	const uint32_t bucket_us[HIST_BUCKETS];
	atomic_t       buckets[HIST_BUCKETS];
	uint32_t       count;
	uint32_t       max_us;
};

static inline void lat_add(struct lat_hist *h, uint32_t us)
{
	for (int i = 0; i < HIST_BUCKETS; i++) {
		if (us <= h->bucket_us[i]) {
			atomic_inc(&h->buckets[i]);
			break;
		}
	}
	h->count++;
	if (us > h->max_us) h->max_us = us;
}

static inline uint32_t lat_percentile(const struct lat_hist *h, int pct)
{
	uint32_t total = 0;
	for (int i = 0; i < HIST_BUCKETS; i++) total += atomic_get(&h->buckets[i]);
	if (total == 0) return 0;
	uint32_t target = (total * pct) / 100;
	uint32_t cum = 0;
	for (int i = 0; i < HIST_BUCKETS; i++) {
		cum += atomic_get(&h->buckets[i]);
		if (cum >= target) return h->bucket_us[i];
	}
	return h->bucket_us[HIST_BUCKETS - 1];
}

/* Default latency bucket edges (microseconds). */
#define LAT_BUCKETS_DEFAULT { 50, 100, 200, 500, 1000, 2000, 5000, 10000, \
			      20000, 50000, 100000, 200000, 500000,      \
			      1000000, 2000000, UINT32_MAX }

/* Scenario entry points. main.c calls these in order. */
void scen_imu_overrun(struct scen_result *out);
void scen_ble_deadline(struct scen_result *out);
void scen_ppg_latency(struct scen_result *out);
void scen_burst_shed(struct scen_result *out);
void scen_display_defer(struct scen_result *out);

#endif /* WEAVER_SCEN_COMMON_H */
