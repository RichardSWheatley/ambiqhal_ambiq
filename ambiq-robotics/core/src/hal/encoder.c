/*
 * ARB HAL - portable encoder decode (angle + velocity from raw tick count).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include "arb/hal/encoder.h"
#include "arb/platform.h"
#include "arb/topic.h" /* arb_err_t */

#ifndef ARB_TWO_PI
#define ARB_TWO_PI 6.28318530717958647692f
#endif

int arb_encoder_init(arb_encoder_t *e, const arb_encoder_ops_t *ops, void *ctx,
		     int32_t counts_per_rev)
{
	if (!e || !ops || !ops->read_count || counts_per_rev <= 0) {
		return ARB_ERR_INVAL;
	}
	e->ops            = ops;
	e->ctx            = ctx;
	e->counts_per_rev = counts_per_rev;
	e->inverted       = false;
	e->last_count     = 0;
	e->last_ts_us     = 0;
	e->position_rad   = 0.0f;
	e->primed         = false;
	return ARB_OK;
}

int arb_encoder_read(arb_encoder_t *e, arb_encoder_reading_t *out)
{
	if (!e || !e->ops || !out) {
		return ARB_ERR_INVAL;
	}

	int32_t raw = 0;
	int rc = e->ops->read_count(e->ctx, &raw);
	if (rc != ARB_OK) {
		return rc;
	}
	if (e->inverted) {
		raw = -raw;
	}

	uint64_t now = arb_platform_time_us();
	const float rad_per_count = ARB_TWO_PI / (float)e->counts_per_rev;

	if (!e->primed) {
		/* First sample: establish a baseline, report zero velocity. */
		e->last_count   = raw;
		e->last_ts_us   = now;
		e->position_rad = 0.0f;
		e->primed       = true;
		out->count          = raw;
		out->position_rad   = 0.0f;
		out->velocity_rad_s = 0.0f;
		return ARB_OK;
	}

	int32_t dcount = raw - e->last_count;
	uint64_t dt_us = now - e->last_ts_us;

	float dpos = (float)dcount * rad_per_count;
	e->position_rad += dpos;

	float vel = 0.0f;
	if (dt_us > 0) {
		vel = dpos / ((float)dt_us * 1e-6f);
	}

	e->last_count = raw;
	e->last_ts_us = now;

	out->count          = raw;
	out->position_rad   = e->position_rad;
	out->velocity_rad_s = vel;
	return ARB_OK;
}

int arb_encoder_reset(arb_encoder_t *e)
{
	if (!e || !e->ops) {
		return ARB_ERR_INVAL;
	}
	e->position_rad = 0.0f;
	e->primed       = false;
	if (e->ops->reset) {
		return e->ops->reset(e->ctx);
	}
	return ARB_OK;
}
