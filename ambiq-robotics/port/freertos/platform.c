/*
 * ARB platform port - FreeRTOS.
 *
 * Queue : a static FreeRTOS queue carries whole messages between post() and
 *         dispatch(). post() is ISR-safe (xQueueSendFromISR).
 * Lock  : BASEPRI mask save/restore via portSET/CLEAR_INTERRUPT_MASK_FROM_ISR,
 *         which are valid from both task and ISR context on Cortex-M.
 * Time  : tick-resolution by default; override arb_platform_time_us() (it is
 *         weak) with a hardware timer for true microseconds - on Apollo510 use
 *         the STIMER exactly as the bare-metal port does.
 *
 * Delivery: call arb_platform_dispatch() from your application, or spawn a task
 * that loops { arb_platform_dispatch(); vTaskDelay(1); } (or blocks on the
 * queue) to deliver automatically.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

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

static StaticQueue_t s_qcb;
static uint8_t       s_qstore[ARB_QUEUE_DEPTH * sizeof(struct qslot)];
static QueueHandle_t s_q;

void arb_platform_init(void)
{
	s_q = xQueueCreateStatic(ARB_QUEUE_DEPTH, sizeof(struct qslot),
				 s_qstore, &s_qcb);
}

__attribute__((weak)) uint64_t arb_platform_time_us(void)
{
	return (uint64_t)xTaskGetTickCount() * (1000000ull / configTICK_RATE_HZ);
}

arb_lock_t arb_platform_lock(void)
{
	return (arb_lock_t)portSET_INTERRUPT_MASK_FROM_ISR();
}

void arb_platform_unlock(arb_lock_t key)
{
	portCLEAR_INTERRUPT_MASK_FROM_ISR((UBaseType_t)key);
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

	BaseType_t ok;
	if (xPortIsInsideInterrupt()) {
		BaseType_t hpw = pdFALSE;
		ok = xQueueSendFromISR(s_q, &slot, &hpw);
		portYIELD_FROM_ISR(hpw);
	} else {
		ok = xQueueSend(s_q, &slot, 0);
	}
	return (ok == pdTRUE) ? ARB_OK : ARB_ERR_FULL;
}

void arb_platform_dispatch(void)
{
	struct qslot slot;
	while (xQueueReceive(s_q, &slot, 0) == pdTRUE) {
		arb_topic_deliver(slot.topic, slot.data, slot.len);
	}
}
