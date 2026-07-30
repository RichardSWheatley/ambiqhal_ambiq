/*
 * ARB - framed serial codec (UART / SPI link).
 *
 * A tiny framer that carries ARB messages over any byte stream. The codec is
 * pure (no OS calls); byte I/O and frame delivery are supplied by the caller
 * (see src/transport.c for the Zephyr UART glue).
 *
 * Frame (little-endian), wire-compatible with the multi-OS ARB tree:
 *   0x7E  | topic(2) | len(2) | payload[len] | crc16(2)
 *   where crc16 (CCITT, init 0xFFFF) covers topic + len + payload.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_SERIAL_H
#define ARB_SERIAL_H

#include <stdint.h>
#include <stddef.h>

#include "arb/err.h"
#include "arb/msg.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Byte-stream write hook (returns 0 on success). */
typedef int (*arb_serial_write_fn)(void *ctx, const uint8_t *buf, size_t len);

/**
 * @brief Complete-frame callback.
 *
 * Invoked from arb_serial_rx_byte() context once a CRC-valid frame has been
 * assembled.
 */
typedef void (*arb_serial_frame_fn)(uint16_t topic, const void *payload,
				    size_t len, void *user);

/** @brief Receive-side parser states. */
typedef enum {
	ARB_SER_SYNC = 0,
	ARB_SER_T0, ARB_SER_T1,
	ARB_SER_L0, ARB_SER_L1,
	ARB_SER_PAYLOAD,
	ARB_SER_C0, ARB_SER_C1,
} arb_serial_state_t;

/** @brief Serial codec instance. */
typedef struct {
	arb_serial_write_fn write;
	void               *write_ctx;
	arb_serial_frame_fn on_frame;
	void               *frame_user;

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
 * @brief Initialize a serial codec.
 *
 * @param s        Instance.
 * @param write    Byte-stream output hook (may be NULL for rx-only).
 * @param ctx      Passed to @p write.
 * @param on_frame Called for every CRC-valid received frame (may be NULL).
 * @param user     Passed to @p on_frame.
 */
int arb_serial_init(arb_serial_t *s, arb_serial_write_fn write, void *ctx,
		    arb_serial_frame_fn on_frame, void *user);

/** @brief Frame and transmit a message over the link. */
int arb_serial_send(arb_serial_t *s, uint16_t topic, const void *msg,
		    size_t len);

/** @brief Feed one received byte into the parser. */
void arb_serial_rx_byte(arb_serial_t *s, uint8_t b);

/** @brief Feed a received buffer into the parser. */
void arb_serial_rx(arb_serial_t *s, const uint8_t *buf, size_t len);

/** @brief CRC-16/CCITT (poly 0x1021), seedable for incremental use. */
uint16_t arb_serial_crc16(uint16_t crc, const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* ARB_SERIAL_H */
