/*
 * ARB - time base.
 *
 * Every stamp in ARB goes through this seam so the time source is a build
 * decision, not something scattered across call sites:
 *
 *   CONFIG_ARB_TIME_UPTIME  local monotonic kernel uptime (default)
 *   CONFIG_ARB_TIME_LINK    uptime + a disciplined offset from the serial
 *                           timesync exchange (arb/timesync.h) - stamps are
 *                           meaningful across the link
 *   CONFIG_ARB_TIME_PTP     a ptp_clock device (chosen "arb,ptp-clock"),
 *                           for boards disciplined by gPTP
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_TIME_H
#define ARB_TIME_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Current time on the configured time base, microseconds.
 *
 * ISR-safe in all backends. With ARB_TIME_LINK the value is uptime plus a
 * slewed offset and is strictly monotonic once sync is acquired.
 */
uint64_t arb_time_now_us(void);

/**
 * @brief Whether stamps are currently meaningful beyond this board.
 *
 * UPTIME: always false. LINK: true while the timesync filter holds a fresh
 * window. PTP: true while the clock device is ready.
 */
bool arb_time_synced(void);

#ifdef __cplusplus
}
#endif

#endif /* ARB_TIME_H */
