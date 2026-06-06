/*
 * ARB JAUS transport - UDP (host / POSIX).
 *
 * A small datagram transport that carries ARB-JAUS messages between nodes. It
 * uses a simplified, self-describing framing (see jaus_udp.c) - enough for two
 * ARB nodes to exchange JAUS messages over a network or to bench-test the JAUS
 * bridge. For interoperating with a full JAUS/AS5669A stack, use the OpenJAUS
 * adapter (open_jaus.c) instead, which delegates transport to the SDK.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_JAUS_UDP_H
#define ARB_JAUS_UDP_H

#include <stdint.h>
#include "arb/bridge/jaus.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief UDP transport state. */
typedef struct {
	int      fd;
	uint32_t peer_ip;   /**< network byte order */
	uint16_t peer_port; /**< network byte order */
} arb_jaus_udp_t;

/**
 * @brief Open a UDP socket bound to @p bind_port and aimed at a peer.
 * @return 0 on success, negative errno-style on failure.
 */
int arb_jaus_udp_open(arb_jaus_udp_t *t, uint16_t bind_port,
		      const char *peer_ip, uint16_t peer_port);

/** @brief Transport send hook (use as arb_jaus_cfg_t::send, ctx = the transport). */
int arb_jaus_udp_send(void *ctx, const arb_jaus_addr_t *dest,
		      const uint8_t *msg, size_t len);

/**
 * @brief Drain any pending datagrams, feeding decoded messages to the bridge.
 * @return number of messages processed, or negative on error.
 */
int arb_jaus_udp_poll(arb_jaus_udp_t *t, arb_jaus_bridge_t *br);

/** @brief Close the socket. */
void arb_jaus_udp_close(arb_jaus_udp_t *t);

#ifdef __cplusplus
}
#endif

#endif /* ARB_JAUS_UDP_H */
