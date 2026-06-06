/*
 * ARB bridge - framed serial transport (UART / SPI link).
 *
 * A tiny, OS-agnostic framer that carries ARB topic messages over any byte
 * stream so a remote host can exchange topics with the node (telemetry out,
 * commands in). The codec lives here; the actual byte I/O is supplied by the
 * caller (e.g. the AmbiqSuite UART port - see port/ambiqsuite/transport_uart.c).
 *
 * Frame (little-endian):
 *   0x7E  | topic(2) | len(2) | payload[len] | crc16(2)
 *   where crc16 (CCITT, init 0xFFFF) covers topic + len + payload.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_BRIDGE_SERIAL_H
#define ARB_BRIDGE_SERIAL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "arb/topic.h"
#include "arb/msg.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Byte-stream write hook (returns 0 on success). */
typedef int (*arb_serial_write_fn)(void *ctx, const uint8_t *buf, size_t len);

/** @brief Receive-side parser states. */
typedef enum {
	ARB_SER_SYNC = 0,
	ARB_SER_T0, ARB_SER_T1,
	ARB_SER_L0, ARB_SER_L1,
	ARB_SER_PAYLOAD,
	ARB_SER_C0, ARB_SER_C1,
} arb_serial_state_t;

/** @brief Serial transport instance. */
typedef struct {
	arb_serial_write_fn write;
	void               *write_ctx;
	bool                publish_on_rx; /**< decoded frames -> arb_topic_publish */

	/* rx state */
	arb_serial_state_t state;
	uint16_t topic;
	uint16_t len;
	uint16_t idx;
	uint16_t crc_rx;
	uint8_t  buf[ARB_MSG_MAX_SIZE];

	/* stats */
	uint32_t rx_frames;
	uint32_t rx_crc_errors;
} arb_serial_t;

/**
 * @brief Initialize a serial transport.
 * @param write Byte-stream output hook (may be NULL for rx-only).
 * @param ctx   Passed to @p write.
 */
int arb_serial_init(arb_serial_t *s, arb_serial_write_fn write, void *ctx);

/**
 * @brief Frame and transmit a topic message over the link.
 */
int arb_serial_send(arb_serial_t *s, arb_topic_id_t topic, const void *msg,
		    size_t len);

/**
 * @brief Subscriber callback that streams a topic out over the link.
 *
 * Subscribe it with @p user set to the @ref arb_serial_t to forward a topic.
 */
void arb_serial_stream_cb(arb_topic_id_t topic, const void *msg, size_t len,
			  void *user);

/**
 * @brief Feed one received byte into the parser.
 *
 * On a complete, CRC-valid frame: if @c publish_on_rx is set, the message is
 * published to its topic via arb_topic_publish().
 */
void arb_serial_rx_byte(arb_serial_t *s, uint8_t b);

/**
 * @brief Feed a buffer of received bytes.
 */
void arb_serial_rx(arb_serial_t *s, const uint8_t *buf, size_t len);

/** @brief CRC16-CCITT (init 0xFFFF) over a buffer - exposed for tests. */
uint16_t arb_serial_crc16(uint16_t crc, const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* ARB_BRIDGE_SERIAL_H */
