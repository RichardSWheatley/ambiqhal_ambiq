/*
 * ARB platform port - Eclipse ThreadX (formerly Azure RTOS ThreadX).
 *
 * ThreadX queue messages are capped at 16 x ULONG (64 bytes), which is smaller
 * than a whole ARB slot, so we pass slot *pointers* through the queue and back
 * them with a TX_BLOCK_POOL.
 *
 * Lock : TX_DISABLE/TX_RESTORE (valid from task and ISR context).
 * Time : tick-resolution by default; override arb_platform_time_us() (weak)
 *        with a hardware timer for true microseconds.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <string.h>
#include <stdint.h>

#include "tx_api.h"

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

/* One pointer (1 ULONG) per queued message. */
static TX_QUEUE      s_q;
static ULONG         s_q_mem[ARB_QUEUE_DEPTH];

/* Block pool sized for ARB_QUEUE_DEPTH slots (+per-block overhead). */
static TX_BLOCK_POOL s_pool;
static uint8_t       s_pool_mem[(sizeof(struct qslot) + sizeof(void *)) *
				 ARB_QUEUE_DEPTH];

void arb_platform_init(void)
{
	tx_block_pool_create(&s_pool, (CHAR *)"arb_pool",
			     sizeof(struct qslot),
			     s_pool_mem, sizeof(s_pool_mem));
	tx_queue_create(&s_q, (CHAR *)"arb_q", TX_1_ULONG,
			s_q_mem, sizeof(s_q_mem));
}

__attribute__((weak)) uint64_t arb_platform_time_us(void)
{
	return (uint64_t)tx_time_get() *
	       (1000000ull / TX_TIMER_TICKS_PER_SECOND);
}

arb_lock_t arb_platform_lock(void)
{
	TX_INTERRUPT_SAVE_AREA
	TX_DISABLE
	return (arb_lock_t)interrupt_save;
}

void arb_platform_unlock(arb_lock_t key)
{
	TX_INTERRUPT_SAVE_AREA
	interrupt_save = (UINT)key;
	TX_RESTORE
}

int arb_platform_post(arb_topic_id_t topic, const void *msg, size_t len)
{
	if (len > ARB_MSG_MAX_SIZE) {
		return ARB_ERR_TOOBIG;
	}

	struct qslot *slot;
	if (tx_block_allocate(&s_pool, (void **)&slot, TX_NO_WAIT)
	    != TX_SUCCESS) {
		return ARB_ERR_FULL;
	}
	slot->topic = topic;
	slot->len   = (uint16_t)len;
	memcpy(slot->data, msg, len);

	ULONG ptr = (ULONG)(uintptr_t)slot;
	if (tx_queue_send(&s_q, &ptr, TX_NO_WAIT) != TX_SUCCESS) {
		tx_block_release(slot);
		return ARB_ERR_FULL;
	}
	return ARB_OK;
}

void arb_platform_dispatch(void)
{
	ULONG ptr;
	while (tx_queue_receive(&s_q, &ptr, TX_NO_WAIT) == TX_SUCCESS) {
		struct qslot *slot = (struct qslot *)(uintptr_t)ptr;
		arb_topic_deliver(slot->topic, slot->data, slot->len);
		tx_block_release(slot);
	}
}
