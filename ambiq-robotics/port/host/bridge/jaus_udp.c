/*
 * ARB JAUS transport - UDP (host / POSIX).
 *
 * Framing (little-endian), one JAUS message per datagram:
 *
 *   offset 0 : u8   version (= 2)
 *   offset 1 : u16  src.subsystem
 *   offset 3 : u8   src.node
 *   offset 4 : u8   src.component
 *   offset 5 : u16  dst.subsystem
 *   offset 7 : u8   dst.node
 *   offset 8 : u8   dst.component
 *   offset 9 : u16  payload length
 *   offset 11: ...  JAUS message bytes (command code + payload)
 *
 * This is a deliberately simple stand-in for AS5669A JUDP so two ARB nodes can
 * talk without an external stack; it is NOT wire-compatible with OpenJAUS.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "jaus_udp.h"

#define ARB_JUDP_VERSION 2
#define ARB_JUDP_HDR     11
#define ARB_JUDP_MAX     512

static void put16(uint8_t *b, size_t *o, uint16_t v)
{
	b[(*o)++] = (uint8_t)(v & 0xFF);
	b[(*o)++] = (uint8_t)(v >> 8);
}

static uint16_t get16(const uint8_t *b, size_t *o)
{
	uint16_t v = (uint16_t)(b[*o] | (b[*o + 1] << 8));
	*o += 2;
	return v;
}

int arb_jaus_udp_open(arb_jaus_udp_t *t, uint16_t bind_port,
		      const char *peer_ip, uint16_t peer_port)
{
	if (!t) {
		return -EINVAL;
	}
	t->fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (t->fd < 0) {
		return -errno;
	}

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(bind_port);
	if (bind(t->fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		int e = -errno;
		close(t->fd);
		t->fd = -1;
		return e;
	}

	int fl = fcntl(t->fd, F_GETFL, 0);
	fcntl(t->fd, F_SETFL, fl | O_NONBLOCK);

	t->peer_ip = peer_ip ? inet_addr(peer_ip) : htonl(INADDR_BROADCAST);
	t->peer_port = htons(peer_port);
	return 0;
}

int arb_jaus_udp_send(void *ctx, const arb_jaus_addr_t *dest,
		      const uint8_t *msg, size_t len)
{
	arb_jaus_udp_t *t = (arb_jaus_udp_t *)ctx;
	if (!t || t->fd < 0 || len + ARB_JUDP_HDR > ARB_JUDP_MAX) {
		return -EINVAL;
	}

	uint8_t pkt[ARB_JUDP_MAX];
	size_t o = 0;
	pkt[o++] = ARB_JUDP_VERSION;
	put16(pkt, &o, 0); /* src.subsystem - filled by caller's bridge id if desired */
	pkt[o++] = 0;
	pkt[o++] = 0;
	put16(pkt, &o, dest->subsystem);
	pkt[o++] = dest->node;
	pkt[o++] = dest->component;
	put16(pkt, &o, (uint16_t)len);
	memcpy(&pkt[o], msg, len);
	o += len;

	struct sockaddr_in to;
	memset(&to, 0, sizeof(to));
	to.sin_family = AF_INET;
	to.sin_addr.s_addr = t->peer_ip;
	to.sin_port = t->peer_port;

	ssize_t s = sendto(t->fd, pkt, o, 0, (struct sockaddr *)&to,
			   sizeof(to));
	return (s == (ssize_t)o) ? 0 : -errno;
}

int arb_jaus_udp_poll(arb_jaus_udp_t *t, arb_jaus_bridge_t *br)
{
	if (!t || t->fd < 0 || !br) {
		return -EINVAL;
	}

	int processed = 0;
	for (;;) {
		uint8_t pkt[ARB_JUDP_MAX];
		ssize_t n = recv(t->fd, pkt, sizeof(pkt), 0);
		if (n < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				break;
			}
			return -errno;
		}
		if (n < ARB_JUDP_HDR || pkt[0] != ARB_JUDP_VERSION) {
			continue;
		}

		size_t o = 1;
		arb_jaus_addr_t src;
		src.subsystem = get16(pkt, &o);
		src.node = pkt[o++];
		src.component = pkt[o++];
		o += 4; /* skip dst (we are the destination) */
		uint16_t plen = get16(pkt, &o);
		if (o + plen > (size_t)n) {
			continue;
		}

		arb_jaus_rx(br, &src, &pkt[o], plen);
		processed++;
	}
	return processed;
}

void arb_jaus_udp_close(arb_jaus_udp_t *t)
{
	if (t && t->fd >= 0) {
		close(t->fd);
		t->fd = -1;
	}
}
