/*
 * Ambiq Robotics Broker (ARB) - port layer interface
 *
 * This header is THE seam between the OS-agnostic core and a concrete platform
 * (bare-metal AmbiqSuite, Zephyr, host test, ...). A port must implement every
 * function declared here. The core never calls an OS primitive directly; it only
 * calls through this contract.
 *
 * Threading model
 * ---------------
 *  - arb_platform_post() is the producer side and MUST be ISR-safe.
 *  - arb_platform_dispatch() runs in a normal (non-ISR) context and drains
 *    whatever post() queued, calling arb_topic_deliver() for each message.
 *  - arb_platform_lock()/unlock() guard the broker's small static registries.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_PLATFORM_H
#define ARB_PLATFORM_H

#include <stdint.h>
#include <stddef.h>
#include "arb/topic.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Opaque lock token returned by arb_platform_lock(). */
typedef uint32_t arb_lock_t;

/**
 * @brief One-time platform bring-up.
 *
 * Initializes the timebase and the post/dispatch queue. Must be called before
 * any other arb_* API. Safe to call once at startup.
 */
void arb_platform_init(void);

/**
 * @brief Monotonic microsecond timestamp.
 *
 * Used for message stamps and for encoder velocity differentiation. Must be
 * monotonic and safe to read from any context (including ISRs).
 */
uint64_t arb_platform_time_us(void);

/**
 * @brief Enqueue a message for later dispatch (producer side, ISR-safe).
 *
 * Copies @p len bytes of @p msg into the platform queue. The data is delivered
 * to subscribers later, from arb_platform_dispatch(), via arb_topic_deliver().
 *
 * @return 0 on success, negative @ref arb_err_t on failure (e.g. queue full).
 */
int arb_platform_post(arb_topic_id_t topic, const void *msg, size_t len);

/**
 * @brief Drain the platform queue, delivering messages to subscribers.
 *
 * Call from the main loop (no-OS) or a dispatcher thread/work item (Zephyr).
 * Invokes arb_topic_deliver() for each queued message in FIFO order.
 */
void arb_platform_dispatch(void);

/**
 * @brief Enter a short critical section protecting broker registries.
 * @return Token to pass back to arb_platform_unlock().
 */
arb_lock_t arb_platform_lock(void);

/**
 * @brief Leave the critical section opened by arb_platform_lock().
 */
void arb_platform_unlock(arb_lock_t key);

#ifdef __cplusplus
}
#endif

#endif /* ARB_PLATFORM_H */
