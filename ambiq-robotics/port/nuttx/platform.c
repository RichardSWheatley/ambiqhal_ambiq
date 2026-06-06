/*
 * ARB platform port - Apache NuttX.
 *
 * NuttX is POSIX-flavored. We use a small static ring guarded by NuttX critical
 * sections (enter_critical_section() is valid from task and ISR context and
 * returns the saved IRQ state), and CLOCK_MONOTONIC for the timebase.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <string.h>
#include <time.h>

#include <nuttx/config.h>
#include <nuttx/irq.h>

#include "arb/platform.h"
#include "arb/topic.h"
#include "arb/msg.h"

#ifndef ARB_QUEUE_DEPTH
#define ARB_QUEUE_DEPTH 16
#endif

struct qslot {
	arb_topic_id_t topic;
	uint16_t       len;
	uint8_t        data[ARB_MSG_MAX_SIZE];
};

static struct qslot       s_queue[ARB_QUEUE_DEPTH];
static volatile unsigned  s_head, s_tail;

void arb_platform_init(void)
{
	s_head = s_tail = 0;
}

uint64_t arb_platform_time_us(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

arb_lock_t arb_platform_lock(void)
{
	return (arb_lock_t)enter_critical_section();
}

void arb_platform_unlock(arb_lock_t key)
{
	leave_critical_section((irqstate_t)key);
}

int arb_platform_post(arb_topic_id_t topic, const void *msg, size_t len)
{
	if (len > ARB_MSG_MAX_SIZE) {
		return ARB_ERR_TOOBIG;
	}

	arb_lock_t k = arb_platform_lock();
	unsigned next = (s_head + 1u) % ARB_QUEUE_DEPTH;
	if (next == s_tail) {
		arb_platform_unlock(k);
		return ARB_ERR_FULL;
	}
	s_queue[s_head].topic = topic;
	s_queue[s_head].len   = (uint16_t)len;
	memcpy(s_queue[s_head].data, msg, len);
	s_head = next;
	arb_platform_unlock(k);
	return ARB_OK;
}

void arb_platform_dispatch(void)
{
	for (;;) {
		arb_lock_t k = arb_platform_lock();
		if (s_tail == s_head) {
			arb_platform_unlock(k);
			return;
		}
		static struct qslot local; /* single dispatcher */
		local = s_queue[s_tail];
		s_tail = (s_tail + 1u) % ARB_QUEUE_DEPTH;
		arb_platform_unlock(k);

		arb_topic_deliver(local.topic, local.data, local.len);
	}
}
