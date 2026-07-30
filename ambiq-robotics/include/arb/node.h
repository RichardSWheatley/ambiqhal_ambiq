/*
 * ARB - node lifecycle
 *
 * A node is a named software component with a JAUS / ROS2-managed-node style
 * state machine. Nodes exist for actuator safety: a single call transitions
 * every active node to ESTOP, invokes their shutdown hooks, and broadcasts the
 * stop on arb_chan_estop so peers (local listeners and remote boards over the
 * transport) can react.
 *
 * Messaging is plain zbus - nodes do not own publishers or subscribers.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_NODE_H
#define ARB_NODE_H

#include <stdint.h>

#include "arb/err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Node lifecycle states. */
typedef enum {
	ARB_NODE_INIT     = 0,
	ARB_NODE_READY    = 1,
	ARB_NODE_ACTIVE   = 2,
	ARB_NODE_ERROR    = 3,
	ARB_NODE_ESTOP    = 4,
	ARB_NODE_SHUTDOWN = 5,
} arb_node_state_t;

/**
 * @brief Lifecycle hooks. All optional; return ARB_OK to accept a transition.
 *
 * A non-ARB_OK return from on_configure/on_activate moves the node to ERROR.
 * on_shutdown is also invoked on ESTOP and must leave actuators safe.
 */
typedef struct {
	int (*on_configure)(void *ctx);  /* INIT   -> READY  */
	int (*on_activate)(void *ctx);   /* READY  -> ACTIVE */
	int (*on_deactivate)(void *ctx); /* ACTIVE -> READY  */
	int (*on_shutdown)(void *ctx);   /* any -> SHUTDOWN / ESTOP */
	int (*on_error)(void *ctx);      /* any -> ERROR     */
} arb_node_ops_t;

/** @brief Node instance. Treat as opaque outside node.c. */
typedef struct {
	uint16_t              id;
	char                  name[16];
	arb_node_state_t      state;
	const arb_node_ops_t *ops;
	void                 *ctx;
	uint32_t              heartbeat_ms; /* 0 = disabled */
	int64_t               next_hb_ms;   /* internal     */
	int64_t               start_ms;     /* internal     */
} arb_node_t;

/**
 * @brief Create a node in INIT state.
 * @return Node handle, or NULL if CONFIG_ARB_MAX_NODES is exhausted.
 */
arb_node_t *arb_node_create(const char *name, const arb_node_ops_t *ops,
			    void *ctx);

/** @brief Find a node by name. */
arb_node_t *arb_node_find(const char *name);

int arb_node_configure(arb_node_t *n);  /* INIT   -> READY  */
int arb_node_activate(arb_node_t *n);   /* READY  -> ACTIVE */
int arb_node_deactivate(arb_node_t *n); /* ACTIVE -> READY  */
int arb_node_shutdown(arb_node_t *n);   /* any -> SHUTDOWN  */
int arb_node_error(arb_node_t *n);      /* any -> ERROR     */

/**
 * @brief Emergency stop.
 *
 * Transitions every READY/ACTIVE node to ESTOP (invoking on_shutdown) and
 * publishes an arb_estop_t on arb_chan_estop.
 *
 * @param source Node id raising the stop (0 = external).
 * @param reason Application-defined reason code.
 */
int arb_node_estop_all(uint16_t source, uint8_t reason);

/**
 * @brief Enable periodic heartbeat publication on arb_chan_heartbeat.
 *
 * Heartbeats are published from a system-workqueue delayable work item with
 * CONFIG_ARB_HEARTBEAT_TICK_MS resolution.
 *
 * @param period_ms Heartbeat period; 0 disables.
 */
void arb_node_set_heartbeat(arb_node_t *n, uint32_t period_ms);

#ifdef __cplusplus
}
#endif

#endif /* ARB_NODE_H */
