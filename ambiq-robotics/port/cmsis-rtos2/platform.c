/*
 * ARB platform port - CMSIS-RTOS2 (e.g. Keil RTX5, or any CMSIS-RTOS2 RTOS).
 *
 * Queue : osMessageQueue carries whole messages; osMessageQueuePut with a zero
 *         timeout is ISR-safe.
 * Lock  : PRIMASK save/restore via CMSIS intrinsics (valid in task and ISR).
 * Time  : kernel tick resolution by default; override arb_platform_time_us()
 *         (weak) with a hardware timer for true microseconds.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <string.h>

#include "cmsis_os2.h"
#include "cmsis_compiler.h" /* __get_PRIMASK / __disable_irq / __enable_irq */

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

static osMessageQueueId_t s_q;

void arb_platform_init(void)
{
	s_q = osMessageQueueNew(ARB_QUEUE_DEPTH, sizeof(struct qslot), NULL);
}

__attribute__((weak)) uint64_t arb_platform_time_us(void)
{
	uint32_t freq = osKernelGetTickFreq();
	return (uint64_t)osKernelGetTickCount() * (1000000ull / freq);
}

arb_lock_t arb_platform_lock(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	return (arb_lock_t)primask;
}

void arb_platform_unlock(arb_lock_t key)
{
	if (!key) {
		__enable_irq();
	}
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

	/* timeout 0 -> safe to call from ISR */
	return (osMessageQueuePut(s_q, &slot, 0, 0) == osOK) ? ARB_OK
							     : ARB_ERR_FULL;
}

void arb_platform_dispatch(void)
{
	struct qslot slot;
	while (osMessageQueueGet(s_q, &slot, NULL, 0) == osOK) {
		arb_topic_deliver(slot.topic, slot.data, slot.len);
	}
}
