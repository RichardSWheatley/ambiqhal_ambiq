/*
 * ARB platform port - RIOT OS.
 *
 * Static ring guarded by RIOT's irq_disable()/irq_restore() (ISR-safe, returns
 * saved state), timebase from xtimer's 64-bit microsecond clock.
 *
 * If your RIOT build uses ztimer instead of xtimer, swap xtimer_now_usec64()
 * for ztimer64_now(ZTIMER64_USEC) and depend on the ztimer64_usec module.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <string.h>

#include "irq.h"
#include "xtimer.h"

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

static struct qslot      s_queue[ARB_QUEUE_DEPTH];
static volatile unsigned s_head, s_tail;

void arb_platform_init(void)
{
	s_head = s_tail = 0;
}

uint64_t arb_platform_time_us(void)
{
	return xtimer_now_usec64();
}

arb_lock_t arb_platform_lock(void)
{
	return (arb_lock_t)irq_disable();
}

void arb_platform_unlock(arb_lock_t key)
{
	irq_restore((unsigned)key);
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
