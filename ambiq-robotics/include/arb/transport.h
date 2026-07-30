/*
 * ARB - serial transport: bridge zbus channels over a UART.
 *
 * Outbound: arb_transport_export(chan) attaches a zbus listener to the channel
 * so every publication is framed (arb/serial.h) and written to the UART named
 * by the devicetree chosen node "arb,uart".
 *
 * Inbound: received CRC-valid frames are looked up by wire id (arb/topics.h)
 * and published on the matching zbus channel from the system workqueue, so
 * remote boards and local code observe identical traffic.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_TRANSPORT_H
#define ARB_TRANSPORT_H

#include <zephyr/zbus/zbus.h>

#include "arb/err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start forwarding a channel's publications out over the UART.
 *
 * The channel must be one of the ARB topics (arb/topics.h) so peers can map
 * the wire id back to a channel.
 */
int arb_transport_export(const struct zbus_channel *chan);

/** @brief Stop forwarding a channel. */
int arb_transport_unexport(const struct zbus_channel *chan);

/** @brief Frames received / dropped-on-CRC counters. */
void arb_transport_stats(uint32_t *rx_frames, uint32_t *rx_crc_errors);

#ifdef __cplusplus
}
#endif

#endif /* ARB_TRANSPORT_H */
