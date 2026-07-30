/*
 * ARB benchmarks - background load for the "loaded" variants.
 *
 * Mixed-criticality traffic per the Robotics WG benchmarking intent: the
 * numbers that matter are the ones taken while best-effort work (message
 * traffic, logging, memory churn) shares the CPU with the measured path.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

#include "bench.h"

LOG_MODULE_REGISTER(bench_load, LOG_LEVEL_INF);

struct load_msg {
	uint64_t stamp;
	uint32_t seq;
	uint8_t payload[52];
};

ZBUS_CHAN_DEFINE(bench_load_chan, struct load_msg, NULL, NULL,
		 ZBUS_OBSERVERS(bench_load_listener), ZBUS_MSG_INIT(0));

static volatile uint32_t rx_count;

static void load_cb(const struct zbus_channel *chan)
{
	const struct load_msg *m = zbus_chan_const_msg(chan);

	rx_count = m->seq;
}
ZBUS_LISTENER_DEFINE(bench_load_listener, load_cb);

#define LOAD_PUB_PRIO   10
#define LOAD_CHURN_PRIO 12

static void pub_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	struct load_msg m = { 0 };

	while (1) {
		m.stamp = k_uptime_ticks();
		m.seq++;
		memset(m.payload, (uint8_t)m.seq, sizeof(m.payload));
		(void)zbus_chan_pub(&bench_load_chan, &m, K_NO_WAIT);
		if ((m.seq % 10) == 0) {
			LOG_INF("load seq %u", m.seq);
		}
		k_sleep(K_MSEC(1));
	}
}

static void churn_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	static uint8_t src[1024], dst[1024];

	while (1) {
		memcpy(dst, src, sizeof(dst));
		src[0] = dst[sizeof(dst) - 1] + 1;
		if (IS_ENABLED(CONFIG_ARCH_POSIX)) {
			/*
			 * On the POSIX arch simulated time only advances when
			 * every thread blocks; a pure yield loop would stall
			 * the simulation. Real load numbers come from
			 * hardware anyway.
			 */
			k_sleep(K_TICKS(1));
		} else {
			k_yield();
		}
	}
}

K_THREAD_DEFINE(bench_load_pub, 1024, pub_thread, NULL, NULL, NULL,
		LOAD_PUB_PRIO, 0, -1);
K_THREAD_DEFINE(bench_load_churn, 1024, churn_thread, NULL, NULL, NULL,
		LOAD_CHURN_PRIO, 0, -1);

void bench_load_start(void)
{
	k_thread_start(bench_load_pub);
	k_thread_start(bench_load_churn);
}
