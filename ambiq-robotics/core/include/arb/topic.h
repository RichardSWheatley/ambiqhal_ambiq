/*
 * Ambiq Robotics Broker (ARB) - publish/subscribe API
 *
 * A tiny topic registry with static storage. Publishers hand a message to the
 * broker; the broker copies it into the platform queue (decoupling ISR/producer
 * context from delivery), and subscribers registered for that topic are invoked
 * from arb_platform_dispatch() context.
 *
 * No malloc, no RTTI: every limit is a compile-time constant (see below) and can
 * be overridden by the build system with -DARB_MAX_TOPICS=... etc.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_TOPIC_H
#define ARB_TOPIC_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Topic identifier. Applications assign their own stable IDs. */
typedef uint16_t arb_topic_id_t;

/** @brief Common negative return codes shared across the broker APIs. */
typedef enum {
	ARB_OK            =  0,
	ARB_ERR_INVAL     = -1, /**< Bad argument.                       */
	ARB_ERR_NOMEM     = -2, /**< Static pool exhausted.              */
	ARB_ERR_NOTFOUND  = -3, /**< Topic/service not registered.       */
	ARB_ERR_TOOBIG    = -4, /**< Payload exceeds advertised size.    */
	ARB_ERR_FULL      = -5, /**< Queue full, message dropped.        */
	ARB_ERR_AGAIN     = -6, /**< Temporary failure, retry.           */
} arb_err_t;

/**
 * @brief Subscriber callback.
 *
 * Invoked from arb_platform_dispatch() context. @p msg points at a copy owned
 * by the broker and is only valid for the duration of the call.
 *
 * @param topic Topic the message was published on.
 * @param msg   Pointer to the message payload.
 * @param len   Payload length in bytes.
 * @param user  Opaque pointer supplied at subscribe time.
 */
typedef void (*arb_subscriber_fn)(arb_topic_id_t topic, const void *msg,
				  size_t len, void *user);

#ifndef ARB_MAX_TOPICS
#define ARB_MAX_TOPICS 16
#endif

#ifndef ARB_MAX_SUBSCRIBERS
#define ARB_MAX_SUBSCRIBERS 32
#endif

/**
 * @brief Reset the broker to an empty state.
 *
 * Clears all topic and subscriber registrations. Call once after
 * arb_platform_init().
 */
void arb_topic_init(void);

/**
 * @brief Advertise (register) a topic and its fixed message size.
 *
 * @param topic    Application-chosen topic id.
 * @param msg_size Size in bytes of every message published on this topic.
 * @return ARB_OK, or a negative @ref arb_err_t.
 *
 * Re-advertising an existing topic with the same size is a no-op success.
 */
int arb_topic_advertise(arb_topic_id_t topic, size_t msg_size);

/**
 * @brief Subscribe a callback to a topic.
 *
 * The topic need not be advertised yet. Multiple subscribers per topic are
 * supported up to @ref ARB_MAX_SUBSCRIBERS total.
 *
 * @return ARB_OK, or a negative @ref arb_err_t.
 */
int arb_topic_subscribe(arb_topic_id_t topic, arb_subscriber_fn cb, void *user);

/**
 * @brief Publish a message (ISR-safe).
 *
 * Copies the message into the platform queue via arb_platform_post(). Delivery
 * to subscribers happens later from arb_platform_dispatch().
 *
 * @param topic Advertised topic id.
 * @param msg   Message payload.
 * @param len   Payload length; must equal the advertised size.
 * @return ARB_OK, or a negative @ref arb_err_t.
 */
int arb_topic_publish(arb_topic_id_t topic, const void *msg, size_t len);

/**
 * @brief Deliver a message synchronously to all subscribers of @p topic.
 *
 * Called by the platform layer from arb_platform_dispatch(). Not normally called
 * by applications. Exposed here because it is the core<->port delivery seam.
 */
void arb_topic_deliver(arb_topic_id_t topic, const void *msg, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* ARB_TOPIC_H */
