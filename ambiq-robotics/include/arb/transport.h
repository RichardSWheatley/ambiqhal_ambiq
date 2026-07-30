/*
 * ARB - transport: bridge zbus channels to a peer.
 *
 * Outbound: arb_transport_export(chan) attaches a zbus listener to the channel
 * so every publication is framed (arb/serial.h) and handed to the configured
 * backend. Note the write happens synchronously in the publisher's context
 * and may block for the frame's serialization time; see transport_core.c for
 * why that trade-off is kept and how a queued backend would replace it.
 *
 * Inbound: received CRC-valid frames are looked up by wire id (arb/topics.h)
 * and published on the matching zbus channel from the backend's rx context,
 * so remote boards and local code observe identical traffic. Frames a peer
 * echoes back are re-published locally but never re-exported (loop guard).
 *
 * Backends (choice ARB_TRANSPORT_BACKEND, see arb/transport_backend.h):
 *   ARB_TRANSPORT_SERIAL  UART named by chosen "arb,uart" (default)
 *   ARB_TRANSPORT_UDP     UDP socket; runs on native_sim via NSOS
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
 * @brief Start forwarding a channel's publications to the peer.
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
