/*
 * ARB JAUS transport - UDP (Zephyr sockets).
 *
 * Framing matches port/host/bridge/jaus_udp.c (version byte + src/dst JAUS
 * addresses + length + JAUS message). Requires CONFIG_NET_SOCKETS and a working
 * network interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <string.h>
#include <errno.h>

#include <zephyr/net/socket.h>

#include "bridge/jaus_udp.h"

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

int arb_zjaus_udp_open(arb_zjaus_udp_t *t, uint16_t bind_port,
		       const char *peer_ip, uint16_t peer_port)
{
	if (!t) {
		return -EINVAL;
	}

	t->sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (t->sock < 0) {
		return -errno;
	}

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(bind_port);
	if (zsock_bind(t->sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		int e = -errno;
		zsock_close(t->sock);
		t->sock = -1;
		return e;
	}

	zsock_fcntl(t->sock, F_SETFL, O_NONBLOCK);

	struct in_addr pa;
	if (peer_ip && zsock_inet_pton(AF_INET, peer_ip, &pa) == 1) {
		t->peer_addr = pa.s_addr;
	} else {
		t->peer_addr = htonl(INADDR_BROADCAST);
	}
	t->peer_port = htons(peer_port);
	return 0;
}

int arb_zjaus_udp_send(void *ctx, const arb_jaus_addr_t *dest,
		       const uint8_t *msg, size_t len)
{
	arb_zjaus_udp_t *t = (arb_zjaus_udp_t *)ctx;
	if (!t || t->sock < 0 || len + ARB_JUDP_HDR > ARB_JUDP_MAX) {
		return -EINVAL;
	}

	uint8_t pkt[ARB_JUDP_MAX];
	size_t o = 0;
	pkt[o++] = ARB_JUDP_VERSION;
	put16(pkt, &o, 0);
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
	to.sin_addr.s_addr = t->peer_addr;
	to.sin_port = t->peer_port;

	ssize_t s = zsock_sendto(t->sock, pkt, o, 0, (struct sockaddr *)&to,
				 sizeof(to));
	return (s == (ssize_t)o) ? 0 : -errno;
}

int arb_zjaus_udp_poll(arb_zjaus_udp_t *t, arb_jaus_bridge_t *br)
{
	if (!t || t->sock < 0 || !br) {
		return -EINVAL;
	}

	int processed = 0;
	for (;;) {
		uint8_t pkt[ARB_JUDP_MAX];
		ssize_t n = zsock_recv(t->sock, pkt, sizeof(pkt), 0);
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
		o += 4; /* skip dst */
		uint16_t plen = get16(pkt, &o);
		if (o + plen > (size_t)n) {
			continue;
		}

		arb_jaus_rx(br, &src, &pkt[o], plen);
		processed++;
	}
	return processed;
}

void arb_zjaus_udp_close(arb_zjaus_udp_t *t)
{
	if (t && t->sock >= 0) {
		zsock_close(t->sock);
		t->sock = -1;
	}
}
