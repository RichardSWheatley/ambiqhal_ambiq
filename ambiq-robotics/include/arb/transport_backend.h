/*
 * ARB - transport backend interface.
 *
 * The transport core (zbus export listener, framing codec, topic dispatch)
 * is byte-stream agnostic; a backend supplies exactly two things and is
 * typically ~60 lines:
 *
 *   - write: push already-framed bytes toward the peer (may block)
 *   - rx:    call arb_transport_core_rx() with received bytes and the
 *            earliest timestamp it can attribute to them
 *
 * Exactly one backend is linked, selected by the ARB_TRANSPORT_BACKEND
 * Kconfig choice. This is the seam a TSN/Ethernet or Zenoh-style backend
 * drops into without touching the core.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_TRANSPORT_BACKEND_H
#define ARB_TRANSPORT_BACKEND_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct arb_transport_backend {
	/** Bring the link up; called once from SYS_INIT. */
	int (*init)(void);
	/** Transmit framed bytes; called under the core's TX lock. */
	int (*write)(const uint8_t *buf, size_t len);
};

/** The single linked backend (defined by transport_serial.c / _udp.c). */
extern const struct arb_transport_backend arb_transport_backend;

/**
 * @brief Feed received bytes into the core.
 *
 * @param buf         Received bytes (any framing alignment).
 * @param len         Byte count.
 * @param rx_stamp_us arb_time_now_us() captured as close to physical
 *                    reception as the backend allows (UART ISR entry,
 *                    recvfrom return). Used as t2/t4 by timesync, so its
 *                    error bounds the achievable sync accuracy - document
 *                    the bound in the backend.
 */
void arb_transport_core_rx(const uint8_t *buf, size_t len,
			   uint64_t rx_stamp_us);

/**
 * @brief Frame and transmit a raw payload on a topic (internal).
 *
 * For transport-level protocol messages (e.g. timesync) that must not
 * round-trip through zbus channels. Takes the core TX lock.
 */
int arb_transport_send_raw(uint16_t topic, const void *payload, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* ARB_TRANSPORT_BACKEND_H */
