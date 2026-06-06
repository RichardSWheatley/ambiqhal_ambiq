/*
 * ARB control - differential-drive kinematics implementation.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include "arb/control/diff_drive.h"

void arb_diffdrive_twist_to_wheels(const arb_diffdrive_t *dd, float v, float w,
				   float *wl_radps, float *wr_radps)
{
	if (!dd || dd->wheel_radius <= 0.0f) {
		return;
	}
	/* wheel ground speeds, then convert to angular velocity */
	float vl = v - w * (dd->wheel_base * 0.5f);
	float vr = v + w * (dd->wheel_base * 0.5f);
	if (wl_radps) {
		*wl_radps = vl / dd->wheel_radius;
	}
	if (wr_radps) {
		*wr_radps = vr / dd->wheel_radius;
	}
}

void arb_diffdrive_wheels_to_twist(const arb_diffdrive_t *dd, float wl_radps,
				   float wr_radps, float *v, float *w)
{
	if (!dd || dd->wheel_base <= 0.0f) {
		return;
	}
	float vl = wl_radps * dd->wheel_radius;
	float vr = wr_radps * dd->wheel_radius;
	if (v) {
		*v = 0.5f * (vl + vr);
	}
	if (w) {
		*w = (vr - vl) / dd->wheel_base;
	}
}
