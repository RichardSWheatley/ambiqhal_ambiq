/*
 * ARB - UDP transport backend.
 *
 * Frames are kept inside the datagrams unchanged (one or more whole frames
 * per datagram), so the codec, host tooling and wire captures are
 * transport-agnostic. Runs on native_sim with the NSOS offloaded-socket
 * driver with zero host setup, and on any board with a network stack.
 *
 * RX timestamping: arb_time_now_us() immediately after zsock_recvfrom()
 * returns - scheduler-noise bounded, which the timesync filter absorbs.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>

#include "arb/time.h"
#include "arb/transport_backend.h"

LOG_MODULE_DECLARE(arb, CONFIG_ARB_LOG_LEVEL);

static int sock = -1;
static struct sockaddr_in peer;

/*
 * The rx thread auto-starts and blocks on udp_ready: static threads are
 * set up after APPLICATION-level SYS_INIT runs, so calling k_thread_start
 * from the init hook would be silently undone.
 */
static K_SEM_DEFINE(udp_ready, 0, 1);

static void rx_thread(void *a, void *b, void *c);
K_THREAD_DEFINE(arb_udp_rx, 2048, rx_thread, NULL, NULL, NULL, 7, 0, 0);

static int udp_write(const uint8_t *buf, size_t len)
{
	if (sock < 0) {
		return -ENOTCONN;
	}
	ssize_t n = zsock_sendto(sock, buf, len, 0, (struct sockaddr *)&peer,
				 sizeof(peer));

	return (n == (ssize_t)len) ? 0 : -EIO;
}

static void rx_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	static uint8_t buf[256];

	k_sem_take(&udp_ready, K_FOREVER);

	while (1) {
		ssize_t n = zsock_recvfrom(sock, buf, sizeof(buf), 0, NULL,
					   NULL);

		if (n <= 0) {
			k_sleep(K_MSEC(10));
			continue;
		}
		arb_transport_core_rx(buf, (size_t)n, arb_time_now_us());
	}
}

static int udp_init(void)
{
	struct sockaddr_in local = {
		.sin_family = AF_INET,
		.sin_port = htons(CONFIG_ARB_TRANSPORT_UDP_LOCAL_PORT),
		.sin_addr = { .s_addr = htonl(INADDR_ANY) },
	};

	sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		LOG_ERR("udp socket failed (%d)", -errno);
		return -errno;
	}
	if (zsock_bind(sock, (struct sockaddr *)&local, sizeof(local)) < 0) {
		LOG_ERR("udp bind :%u failed (%d)",
			CONFIG_ARB_TRANSPORT_UDP_LOCAL_PORT, -errno);
		return -errno;
	}

	peer.sin_family = AF_INET;
	peer.sin_port = htons(CONFIG_ARB_TRANSPORT_UDP_PEER_PORT);
	if (zsock_inet_pton(AF_INET, CONFIG_ARB_TRANSPORT_UDP_PEER_ADDR,
			    &peer.sin_addr) != 1) {
		LOG_ERR("bad peer address '%s'",
			CONFIG_ARB_TRANSPORT_UDP_PEER_ADDR);
		return -EINVAL;
	}

	k_sem_give(&udp_ready);
	LOG_INF("udp transport up, peer %s:%u",
		CONFIG_ARB_TRANSPORT_UDP_PEER_ADDR,
		CONFIG_ARB_TRANSPORT_UDP_PEER_PORT);
	return 0;
}

const struct arb_transport_backend arb_transport_backend = {
	.init = udp_init,
	.write = udp_write,
};
