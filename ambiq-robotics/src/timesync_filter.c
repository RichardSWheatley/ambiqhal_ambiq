/*
 * ARB - timesync offset/drift filter (pure C, no OS calls).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <string.h>

#include "arb/timesync.h"

void arb_ts_filter_init(arb_ts_filter_t *f, uint8_t window,
			uint32_t max_slew_ppm, uint64_t fresh_us)
{
	memset(f, 0, sizeof(*f));
	f->window = (window < 3) ? 3 :
		    (window > ARB_TS_WINDOW_MAX) ? ARB_TS_WINDOW_MAX : window;
	f->max_slew_ppm = max_slew_ppm;
	f->fresh_us = fresh_us;
}

/* median of the first n entries of a small array (insertion-sorted copy) */
static int64_t median_i64(const int64_t *v, uint8_t n)
{
	int64_t s[ARB_TS_WINDOW_MAX];

	for (uint8_t i = 0; i < n; i++) {
		int64_t x = v[i];
		uint8_t j = i;

		while (j > 0 && s[j - 1] > x) {
			s[j] = s[j - 1];
			j--;
		}
		s[j] = x;
	}
	return s[n / 2];
}

static uint64_t median_u64(const uint64_t *v, uint8_t n)
{
	uint64_t s[ARB_TS_WINDOW_MAX];

	for (uint8_t i = 0; i < n; i++) {
		uint64_t x = v[i];
		uint8_t j = i;

		while (j > 0 && s[j - 1] > x) {
			s[j] = s[j - 1];
			j--;
		}
		s[j] = x;
	}
	return s[n / 2];
}

bool arb_ts_filter_sample(arb_ts_filter_t *f, uint64_t t1, uint64_t t2,
			  uint64_t t3, uint64_t t4, uint64_t local_now_us)
{
	/* offset = ((t2-t1)+(t3-t4))/2, delay = (t4-t1)-(t3-t2) */
	int64_t off = (((int64_t)(t2 - t1)) + ((int64_t)(t3 - t4))) / 2;
	int64_t delay = (int64_t)(t4 - t1) - (int64_t)(t3 - t2);

	if (delay < 0) {
		return false;
	}

	/* spike filter once the window has substance */
	if (f->n >= 3 && (uint64_t)delay > 2 * median_u64(f->delay_hist, f->n)) {
		return false;
	}

	f->offset_hist[f->head] = off;
	f->delay_hist[f->head] = (uint64_t)delay;
	f->head = (uint8_t)((f->head + 1) % f->window);
	if (f->n < f->window) {
		f->n++;
	}

	int64_t prev_target = f->target_offset_us;
	uint64_t prev_sample_at = f->last_sample_local_us;

	f->target_offset_us = median_i64(f->offset_hist, f->n);
	f->last_sample_local_us = local_now_us;

	/* drift: EMA over target movement per local elapsed time */
	if (prev_sample_at != 0 && local_now_us > prev_sample_at) {
		int64_t d_off = f->target_offset_us - prev_target;
		uint64_t d_loc = local_now_us - prev_sample_at;
		int64_t obs_ppb = (d_off * 1000000000LL) / (int64_t)d_loc;

		f->drift_ppb = (int32_t)((3 * (int64_t)f->drift_ppb + obs_ppb) / 4);
	}

	/* first accepted sample: one-time step (pre-sync, either direction) */
	if (!f->have_applied) {
		f->applied_offset_us = f->target_offset_us;
		f->last_slew_local_us = local_now_us;
		f->have_applied = true;
	}
	return true;
}

int64_t arb_ts_filter_offset(arb_ts_filter_t *f, uint64_t local_now_us)
{
	if (!f->have_applied) {
		return 0;
	}

	if (local_now_us > f->last_slew_local_us) {
		uint64_t elapsed = local_now_us - f->last_slew_local_us;
		int64_t max_step =
			(int64_t)((elapsed * f->max_slew_ppm) / 1000000U);
		int64_t diff = f->target_offset_us - f->applied_offset_us;

		if (diff > max_step) {
			diff = max_step;
		} else if (diff < -max_step) {
			diff = -max_step;
		}
		f->applied_offset_us += diff;
		f->last_slew_local_us = local_now_us;
	}
	return f->applied_offset_us;
}

int64_t arb_ts_filter_applied(const arb_ts_filter_t *f)
{
	return f->have_applied ? f->applied_offset_us : 0;
}

bool arb_ts_filter_synced(const arb_ts_filter_t *f, uint64_t local_now_us)
{
	if (f->n < 3) {
		return false;
	}
	return (local_now_us - f->last_sample_local_us) <= f->fresh_us;
}

int32_t arb_ts_filter_drift_ppb(const arb_ts_filter_t *f)
{
	return f->drift_ppb;
}
