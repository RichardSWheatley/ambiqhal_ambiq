/*
 * ARB bridge - framed serial transport (codec).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <string.h>

#include "arb/bridge/serial.h"

#define ARB_SER_SYNC_BYTE 0x7E

uint16_t arb_serial_crc16(uint16_t crc, const uint8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		crc ^= (uint16_t)buf[i] << 8;
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
					     : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

int arb_serial_init(arb_serial_t *s, arb_serial_write_fn write, void *ctx)
{
	if (!s) {
		return ARB_ERR_INVAL;
	}
	memset(s, 0, sizeof(*s));
	s->write = write;
	s->write_ctx = ctx;
	s->state = ARB_SER_SYNC;
	return ARB_OK;
}

int arb_serial_send(arb_serial_t *s, arb_topic_id_t topic, const void *msg,
		    size_t len)
{
	if (!s || !s->write || !msg) {
		return ARB_ERR_INVAL;
	}
	if (len > ARB_MSG_MAX_SIZE) {
		return ARB_ERR_TOOBIG;
	}

	uint8_t hdr[5];
	hdr[0] = ARB_SER_SYNC_BYTE;
	hdr[1] = (uint8_t)(topic & 0xFF);
	hdr[2] = (uint8_t)(topic >> 8);
	hdr[3] = (uint8_t)(len & 0xFF);
	hdr[4] = (uint8_t)(len >> 8);

	uint16_t crc = 0xFFFF;
	crc = arb_serial_crc16(crc, &hdr[1], 4);
	crc = arb_serial_crc16(crc, (const uint8_t *)msg, len);
	uint8_t crcb[2] = { (uint8_t)(crc & 0xFF), (uint8_t)(crc >> 8) };

	if (s->write(s->write_ctx, hdr, sizeof(hdr)) != 0) {
		return ARB_ERR_AGAIN;
	}
	if (len && s->write(s->write_ctx, (const uint8_t *)msg, len) != 0) {
		return ARB_ERR_AGAIN;
	}
	if (s->write(s->write_ctx, crcb, sizeof(crcb)) != 0) {
		return ARB_ERR_AGAIN;
	}
	return ARB_OK;
}

void arb_serial_stream_cb(arb_topic_id_t topic, const void *msg, size_t len,
			  void *user)
{
	arb_serial_t *s = (arb_serial_t *)user;
	if (s) {
		(void)arb_serial_send(s, topic, msg, len);
	}
}

static void rx_complete(arb_serial_t *s)
{
	uint16_t crc = 0xFFFF;
	uint8_t hdr[4] = {
		(uint8_t)(s->topic & 0xFF), (uint8_t)(s->topic >> 8),
		(uint8_t)(s->len & 0xFF),   (uint8_t)(s->len >> 8),
	};
	crc = arb_serial_crc16(crc, hdr, 4);
	crc = arb_serial_crc16(crc, s->buf, s->len);

	if (crc == s->crc_rx) {
		s->rx_frames++;
		if (s->publish_on_rx) {
			(void)arb_topic_publish(s->topic, s->buf, s->len);
		}
	} else {
		s->rx_crc_errors++;
	}
}

void arb_serial_rx_byte(arb_serial_t *s, uint8_t b)
{
	if (!s) {
		return;
	}

	switch (s->state) {
	case ARB_SER_SYNC:
		if (b == ARB_SER_SYNC_BYTE) {
			s->state = ARB_SER_T0;
		}
		break;
	case ARB_SER_T0:
		s->topic = b;
		s->state = ARB_SER_T1;
		break;
	case ARB_SER_T1:
		s->topic |= (uint16_t)b << 8;
		s->state = ARB_SER_L0;
		break;
	case ARB_SER_L0:
		s->len = b;
		s->state = ARB_SER_L1;
		break;
	case ARB_SER_L1:
		s->len |= (uint16_t)b << 8;
		s->idx = 0;
		if (s->len > ARB_MSG_MAX_SIZE) {
			s->state = ARB_SER_SYNC; /* bogus length, resync */
		} else if (s->len == 0) {
			s->state = ARB_SER_C0;
		} else {
			s->state = ARB_SER_PAYLOAD;
		}
		break;
	case ARB_SER_PAYLOAD:
		s->buf[s->idx++] = b;
		if (s->idx >= s->len) {
			s->state = ARB_SER_C0;
		}
		break;
	case ARB_SER_C0:
		s->crc_rx = b;
		s->state = ARB_SER_C1;
		break;
	case ARB_SER_C1:
		s->crc_rx |= (uint16_t)b << 8;
		rx_complete(s);
		s->state = ARB_SER_SYNC;
		break;
	default:
		s->state = ARB_SER_SYNC;
		break;
	}
}

void arb_serial_rx(arb_serial_t *s, const uint8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		arb_serial_rx_byte(s, buf[i]);
	}
}
