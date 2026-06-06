/*
 * ARB platform port - bare-metal AmbiqSuite (no OS).
 *
 * Timebase  : STIMER free-running at 6 MHz (HFRC_6MHZ), extended to 64-bit us.
 * Post/queue: lock-free-enough static ring drained from the super-loop.
 * Lock      : PRIMASK save/restore (interrupt masking).
 *
 * Drive it from main() like:
 *
 *     arb_platform_init();
 *     arb_topic_init();
 *     ... advertise / subscribe ...
 *     for (;;) {
 *         app_step();                 // publishes
 *         arb_platform_dispatch();    // delivers to subscribers
 *     }
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include "arb/platform.h"
#include "arb/topic.h"

#include "am_mcu_apollo.h"

/* ---- timebase --------------------------------------------------------- */

#define ARB_STIMER_HZ 6000000u /* AM_HAL_STIMER_HFRC_6MHZ */

static volatile uint32_t s_last_stimer;
static volatile uint64_t s_stimer_hi; /* accumulated counts above 32-bit wrap */

void arb_platform_init(void)
{
	/* Free-running STIMER as the microsecond timebase. */
	am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR);
	am_hal_stimer_config(AM_HAL_STIMER_HFRC_6MHZ | AM_HAL_STIMER_CFG_RUN);

	s_last_stimer = am_hal_stimer_counter_get();
	s_stimer_hi   = 0;
}

uint64_t arb_platform_time_us(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	uint32_t now = am_hal_stimer_counter_get();
	if (now < s_last_stimer) {
		/* 32-bit counter wrapped since last read. */
		s_stimer_hi += (1ull << 32);
	}
	s_last_stimer = now;
	uint64_t counts = s_stimer_hi + now;

	if (!primask) {
		__enable_irq();
	}

	return counts / (ARB_STIMER_HZ / 1000000u);
}

/* ---- lock ------------------------------------------------------------- */

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

/* ---- post / dispatch queue -------------------------------------------- */

#ifndef ARB_QUEUE_DEPTH
#define ARB_QUEUE_DEPTH 16
#endif

struct qslot {
	arb_topic_id_t topic;
	uint16_t       len;
	uint8_t        data[ARB_MSG_MAX_SIZE];
};

static struct qslot     s_queue[ARB_QUEUE_DEPTH];
static volatile uint8_t s_head; /* producer writes */
static volatile uint8_t s_tail; /* consumer reads  */

static inline uint8_t q_next(uint8_t i)
{
	return (uint8_t)((i + 1u) % ARB_QUEUE_DEPTH);
}

int arb_platform_post(arb_topic_id_t topic, const void *msg, size_t len)
{
	if (len > ARB_MSG_MAX_SIZE) {
		return ARB_ERR_TOOBIG;
	}

	arb_lock_t k = arb_platform_lock();

	uint8_t next = q_next(s_head);
	if (next == s_tail) {
		arb_platform_unlock(k);
		return ARB_ERR_FULL; /* queue full, drop */
	}

	struct qslot *slot = &s_queue[s_head];
	slot->topic = topic;
	slot->len   = (uint16_t)len;
	for (size_t i = 0; i < len; i++) {
		slot->data[i] = ((const uint8_t *)msg)[i];
	}
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
			return; /* empty */
		}

		/* Copy out under lock, deliver with lock released. */
		static struct qslot local; /* not reentrant: single dispatcher */
		local = s_queue[s_tail];
		s_tail = q_next(s_tail);
		arb_platform_unlock(k);

		arb_topic_deliver(local.topic, local.data, local.len);
	}
}
