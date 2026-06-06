/*
 * ARB HAL - portable IMU logic (axis remap + message packing).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include "arb/hal/imu.h"
#include "arb/platform.h"
#include "arb/topic.h" /* arb_err_t */

static float axis_pick(const arb_vec3_t *v, arb_axis_map_t m)
{
	const float *a = &v->x; /* x,y,z are contiguous */
	float val = a[m.src & 0x3];
	return (m.sign < 0) ? -val : val;
}

static void remap_vec(const arb_axis_map_t map[3], const arb_vec3_t *in,
		      arb_vec3_t *out)
{
	out->x = axis_pick(in, map[0]);
	out->y = axis_pick(in, map[1]);
	out->z = axis_pick(in, map[2]);
}

int arb_imu_init(arb_imu_t_inst *imu, const arb_imu_ops_t *ops, void *ctx)
{
	if (!imu || !ops || !ops->read) {
		return ARB_ERR_INVAL;
	}
	imu->ops = ops;
	imu->ctx = ctx;
	for (int i = 0; i < 3; i++) {
		imu->remap[i].src  = (uint8_t)i;
		imu->remap[i].sign = 1;
	}
	return ARB_OK;
}

int arb_imu_set_axis_map(arb_imu_t_inst *imu, const arb_axis_map_t remap[3])
{
	if (!imu || !remap) {
		return ARB_ERR_INVAL;
	}
	for (int i = 0; i < 3; i++) {
		imu->remap[i] = remap[i];
	}
	return ARB_OK;
}

int arb_imu_start(arb_imu_t_inst *imu)
{
	if (!imu || !imu->ops) {
		return ARB_ERR_INVAL;
	}
	if (imu->ops->start) {
		return imu->ops->start(imu->ctx);
	}
	return ARB_OK;
}

int arb_imu_sample(arb_imu_t_inst *imu, arb_imu_sample_t *out)
{
	if (!imu || !imu->ops || !out) {
		return ARB_ERR_INVAL;
	}

	arb_imu_sample_t raw;
	int rc = imu->ops->read(imu->ctx, &raw);
	if (rc != ARB_OK) {
		return rc;
	}

	remap_vec(imu->remap, &raw.accel, &out->accel);
	remap_vec(imu->remap, &raw.gyro,  &out->gyro);
	if (raw.has_mag) {
		remap_vec(imu->remap, &raw.mag, &out->mag);
	} else {
		out->mag.x = out->mag.y = out->mag.z = 0.0f;
	}
	out->temperature = raw.temperature;
	out->has_mag     = raw.has_mag;
	return ARB_OK;
}

int arb_imu_sample_msg(arb_imu_t_inst *imu, uint16_t source, uint32_t seq,
		       arb_imu_t *msg)
{
	if (!msg) {
		return ARB_ERR_INVAL;
	}

	arb_imu_sample_t s;
	int rc = arb_imu_sample(imu, &s);
	if (rc != ARB_OK) {
		return rc;
	}

	arb_header_init(&msg->header, ARB_MSG_IMU, source,
			arb_platform_time_us(), seq);
	msg->accel       = s.accel;
	msg->gyro        = s.gyro;
	msg->mag         = s.mag;
	msg->temperature = s.temperature;
	return ARB_OK;
}
