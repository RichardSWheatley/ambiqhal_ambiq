/*
 * ARB benchmarks - histogram and reporting.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "bench.h"

void bench_hist_reset(struct bench_hist *h)
{
	memset(h, 0, sizeof(*h));
	h->min = UINT64_MAX;
}

static uint32_t bucket_index(uint64_t v)
{
	if (v < 16) {
		return (uint32_t)v;
	}

	uint32_t msb = 63u - (uint32_t)__builtin_clzll(v);
	uint32_t sub = (uint32_t)((v >> (msb - 4)) & 0xF);
	uint32_t idx = ((msb - 3) << 4) | sub;

	return MIN(idx, BENCH_HIST_BUCKETS - 1);
}

static uint64_t bucket_midpoint(uint32_t idx)
{
	if (idx < 16) {
		return idx;
	}

	uint32_t msb = (idx >> 4) + 3;
	uint64_t sub = idx & 0xF;
	uint64_t step = UINT64_C(1) << (msb - 4);
	uint64_t low = (UINT64_C(1) << msb) + sub * step;

	return low + step / 2;
}

void bench_hist_add(struct bench_hist *h, uint64_t val)
{
	h->min = MIN(h->min, val);
	h->max = MAX(h->max, val);
	h->sum += val;
	h->n++;
	h->bucket[bucket_index(val)]++;
}

uint64_t bench_hist_percentile(const struct bench_hist *h, uint32_t pct_x100)
{
	if (h->n == 0) {
		return 0;
	}

	uint64_t rank = ((uint64_t)h->n * pct_x100 + 9999) / 10000;
	uint64_t seen = 0;

	for (uint32_t i = 0; i < BENCH_HIST_BUCKETS; i++) {
		seen += h->bucket[i];
		if (seen >= rank) {
			/* clamp the midpoint estimate into the exact range */
			return CLAMP(bucket_midpoint(i), h->min, h->max);
		}
	}
	return h->max;
}

void bench_report(const char *bench, const char *load,
		  const struct bench_hist *h, const char *extra)
{
	uint64_t avg = h->n ? h->sum / h->n : 0;

	printk("RECORD: {\"bench\":\"%s\",\"board\":\"%s\",\"load\":\"%s\","
	       "\"n\":%u,\"min_ns\":%llu,\"avg_ns\":%llu,\"max_ns\":%llu,"
	       "\"p99_ns\":%llu%s%s}\n",
	       bench, CONFIG_BOARD, load, h->n,
	       (unsigned long long)(h->n ? h->min : 0),
	       (unsigned long long)avg,
	       (unsigned long long)h->max,
	       (unsigned long long)bench_hist_percentile(h, 9900),
	       extra ? "," : "", extra ? extra : "");
}
