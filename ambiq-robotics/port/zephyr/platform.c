/*
 * ARB platform port - Zephyr RTOS.
 *
 * Timebase  : kernel uptime (k_ticks -> microseconds).
 * Post/queue: k_msgq (ISR-safe producer).
 * Dispatch  : drained either manually via arb_platform_dispatch() or by a
 *             dedicated dispatcher thread (CONFIG_ARB_DISPATCH_THREAD).
 * Lock      : irq_lock()/irq_unlock().
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <zephyr/kernel.h>

#include "arb/platform.h"
#include "arb/topic.h"

struct qslot {
	arb_topic_id_t topic;
	uint16_t       len;
	uint8_t        data[ARB_MSG_MAX_SIZE];
};

#ifndef CONFIG_ARB_QUEUE_DEPTH
#define CONFIG_ARB_QUEUE_DEPTH 16
#endif

K_MSGQ_DEFINE(s_arb_msgq, sizeof(struct qslot), CONFIG_ARB_QUEUE_DEPTH, 4);

void arb_platform_init(void)
{
	/* k_msgq is statically defined; kernel timebase is always running. */
	k_msgq_purge(&s_arb_msgq);
}

uint64_t arb_platform_time_us(void)
{
	return k_ticks_to_us_floor64(k_uptime_ticks());
}

arb_lock_t arb_platform_lock(void)
{
	return (arb_lock_t)irq_lock();
}

void arb_platform_unlock(arb_lock_t key)
{
	irq_unlock((unsigned int)key);
}

int arb_platform_post(arb_topic_id_t topic, const void *msg, size_t len)
{
	if (len > ARB_MSG_MAX_SIZE) {
		return ARB_ERR_TOOBIG;
	}

	struct qslot slot;
	slot.topic = topic;
	slot.len   = (uint16_t)len;
	memcpy(slot.data, msg, len);

	/* K_NO_WAIT keeps this usable from ISR context. */
	if (k_msgq_put(&s_arb_msgq, &slot, K_NO_WAIT) != 0) {
		return ARB_ERR_FULL;
	}
	return ARB_OK;
}

void arb_platform_dispatch(void)
{
	struct qslot slot;

	while (k_msgq_get(&s_arb_msgq, &slot, K_NO_WAIT) == 0) {
		arb_topic_deliver(slot.topic, slot.data, slot.len);
	}
}

#ifdef CONFIG_ARB_DISPATCH_THREAD

static void arb_dispatch_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct qslot slot;
	for (;;) {
		/* Block until a message is available, then deliver it. */
		if (k_msgq_get(&s_arb_msgq, &slot, K_FOREVER) == 0) {
			arb_topic_deliver(slot.topic, slot.data, slot.len);
		}
	}
}

K_THREAD_DEFINE(arb_dispatcher, CONFIG_ARB_DISPATCH_THREAD_STACK_SIZE,
		arb_dispatch_thread, NULL, NULL, NULL,
		CONFIG_ARB_DISPATCH_THREAD_PRIORITY, 0, 0);

#endif /* CONFIG_ARB_DISPATCH_THREAD */
