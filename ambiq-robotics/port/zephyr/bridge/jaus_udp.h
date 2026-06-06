/*
 * ARB JAUS transport - UDP (Zephyr sockets).
 *
 * Zephyr-native counterpart of port/host/bridge/jaus_udp.c: carries ARB-JAUS
 * messages over UDP using the Zephyr socket API. Same simplified framing as the
 * host transport (see that file); for a full AS5669A stack use OpenJAUS.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_ZEPHYR_JAUS_UDP_H
#define ARB_ZEPHYR_JAUS_UDP_H

#include <stdint.h>
#include "arb/bridge/jaus.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Zephyr UDP transport state. */
typedef struct {
	int      sock;
	uint32_t peer_addr;  /**< network byte order */
	uint16_t peer_port;  /**< network byte order */
} arb_zjaus_udp_t;

/** @brief Open + bind a UDP socket aimed at a peer. Returns 0 or negative. */
int arb_zjaus_udp_open(arb_zjaus_udp_t *t, uint16_t bind_port,
		       const char *peer_ip, uint16_t peer_port);

/** @brief Transport send hook (arb_jaus_cfg_t::send; ctx = the transport). */
int arb_zjaus_udp_send(void *ctx, const arb_jaus_addr_t *dest,
		       const uint8_t *msg, size_t len);

/** @brief Drain pending datagrams into the bridge. Returns count or negative. */
int arb_zjaus_udp_poll(arb_zjaus_udp_t *t, arb_jaus_bridge_t *br);

/** @brief Close the socket. */
void arb_zjaus_udp_close(arb_zjaus_udp_t *t);

#ifdef __cplusplus
}
#endif

#endif /* ARB_ZEPHYR_JAUS_UDP_H */
