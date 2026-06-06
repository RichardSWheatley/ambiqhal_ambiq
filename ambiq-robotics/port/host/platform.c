/*
 * ARB platform port - host (POSIX) simulation / unit testing.
 *
 * Lets the OS-agnostic core run on a development machine: useful for unit tests,
 * algorithm bring-up, and replaying logs. Timebase is CLOCK_MONOTONIC; the queue
 * is a plain array drained by arb_platform_dispatch(); the "lock" is a no-op
 * (single-threaded host use).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#define _POSIX_C_SOURCE 200112L

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <time.h>

#include "arb/platform.h"
#include "arb/topic.h"
#include "arb/msg.h"

#ifndef ARB_QUEUE_DEPTH
#define ARB_QUEUE_DEPTH 32
#endif

struct qslot {
	arb_topic_id_t topic;
	uint16_t       len;
	uint8_t        data[ARB_MSG_MAX_SIZE];
	int            used;
};

static struct qslot s_queue[ARB_QUEUE_DEPTH];
static unsigned     s_head, s_tail;

void arb_platform_init(void)
{
	s_head = s_tail = 0;
	memset(s_queue, 0, sizeof(s_queue));
}

uint64_t arb_platform_time_us(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

arb_lock_t arb_platform_lock(void)
{
	return 0; /* single-threaded host: nothing to mask */
}

void arb_platform_unlock(arb_lock_t key)
{
	(void)key;
}

int arb_platform_post(arb_topic_id_t topic, const void *msg, size_t len)
{
	if (len > ARB_MSG_MAX_SIZE) {
		return ARB_ERR_TOOBIG;
	}
	unsigned next = (s_head + 1u) % ARB_QUEUE_DEPTH;
	if (next == s_tail) {
		return ARB_ERR_FULL;
	}
	s_queue[s_head].topic = topic;
	s_queue[s_head].len = (uint16_t)len;
	memcpy(s_queue[s_head].data, msg, len);
	s_head = next;
	return ARB_OK;
}

void arb_platform_dispatch(void)
{
	while (s_tail != s_head) {
		struct qslot *s = &s_queue[s_tail];
		arb_topic_deliver(s->topic, s->data, s->len);
		s_tail = (s_tail + 1u) % ARB_QUEUE_DEPTH;
	}
}
