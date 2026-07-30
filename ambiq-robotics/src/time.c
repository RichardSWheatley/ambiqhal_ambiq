/*
 * ARB - time base backends.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <zephyr/kernel.h>

#include "arb/time.h"

static inline uint64_t uptime_us(void)
{
	return k_ticks_to_us_floor64(k_uptime_ticks());
}

#if defined(CONFIG_ARB_TIME_UPTIME)

uint64_t arb_time_now_us(void)
{
	return uptime_us();
}

bool arb_time_synced(void)
{
	return false;
}

#elif defined(CONFIG_ARB_TIME_LINK)

#include "arb/timesync.h"

uint64_t arb_time_now_us(void)
{
	uint64_t up = uptime_us();

	return up + (uint64_t)arb_timesync_offset_us(up);
}

bool arb_time_synced(void)
{
	return arb_timesync_synced();
}

#elif defined(CONFIG_ARB_TIME_PTP)

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/ptp_clock.h>

/*
 * Stub backend: reads the chosen ptp_clock device. This is the backend an
 * Ethernet-capable board runs under gPTP; it exists so applications written
 * against arb_time_now_us() port to such boards without change. Falls back
 * to local uptime (and reports unsynced) while the device is not ready.
 */
#if !DT_HAS_CHOSEN(arb_ptp_clock)
#error "CONFIG_ARB_TIME_PTP=y requires a devicetree 'chosen { arb,ptp-clock = &...; }' node"
#endif

static const struct device *const ptp_dev =
	DEVICE_DT_GET(DT_CHOSEN(arb_ptp_clock));

uint64_t arb_time_now_us(void)
{
	struct net_ptp_time tm;

	if (!device_is_ready(ptp_dev) || ptp_clock_get(ptp_dev, &tm) != 0) {
		return uptime_us();
	}
	return tm.second * USEC_PER_SEC + tm.nanosecond / NSEC_PER_USEC;
}

bool arb_time_synced(void)
{
	return device_is_ready(ptp_dev);
}

#endif
