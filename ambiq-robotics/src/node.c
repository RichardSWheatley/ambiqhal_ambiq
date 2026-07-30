/*
 * ARB - node lifecycle.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

#include "arb/node.h"
#include "arb/topics.h"

LOG_MODULE_REGISTER(arb, CONFIG_ARB_LOG_LEVEL);

static arb_node_t nodes[CONFIG_ARB_MAX_NODES];
static uint16_t   node_count;
static struct k_spinlock lock;

static uint64_t stamp_us(void)
{
	return k_ticks_to_us_floor64(k_uptime_ticks());
}

arb_node_t *arb_node_create(const char *name, const arb_node_ops_t *ops,
			    void *ctx)
{
	if (!name) {
		return NULL;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);
	if (node_count >= CONFIG_ARB_MAX_NODES) {
		k_spin_unlock(&lock, key);
		return NULL;
	}
	arb_node_t *n = &nodes[node_count];
	n->id = ++node_count; /* ids start at 1; 0 = unspecified */
	k_spin_unlock(&lock, key);

	strncpy(n->name, name, sizeof(n->name) - 1);
	n->name[sizeof(n->name) - 1] = '\0';
	n->ops      = ops;
	n->ctx      = ctx;
	n->state    = ARB_NODE_INIT;
	n->start_ms = k_uptime_get();

	LOG_INF("node '%s' created (id=%u)", n->name, n->id);
	return n;
}

arb_node_t *arb_node_find(const char *name)
{
	for (uint16_t i = 0; i < node_count; i++) {
		if (strncmp(nodes[i].name, name, sizeof(nodes[i].name)) == 0) {
			return &nodes[i];
		}
	}
	return NULL;
}

static int run_hook(int (*hook)(void *), void *ctx)
{
	return hook ? hook(ctx) : ARB_OK;
}

int arb_node_configure(arb_node_t *n)
{
	if (!n || n->state != ARB_NODE_INIT) {
		return ARB_ERR_STATE;
	}
	int rc = run_hook(n->ops ? n->ops->on_configure : NULL, n->ctx);
	if (rc != ARB_OK) {
		n->state = ARB_NODE_ERROR;
		return rc;
	}
	n->state = ARB_NODE_READY;
	LOG_INF("node '%s' -> READY", n->name);
	return ARB_OK;
}

int arb_node_activate(arb_node_t *n)
{
	if (!n || n->state != ARB_NODE_READY) {
		return ARB_ERR_STATE;
	}
	int rc = run_hook(n->ops ? n->ops->on_activate : NULL, n->ctx);
	if (rc != ARB_OK) {
		n->state = ARB_NODE_ERROR;
		return rc;
	}
	n->state = ARB_NODE_ACTIVE;
	LOG_INF("node '%s' -> ACTIVE", n->name);
	return ARB_OK;
}

int arb_node_deactivate(arb_node_t *n)
{
	if (!n || n->state != ARB_NODE_ACTIVE) {
		return ARB_ERR_STATE;
	}
	(void)run_hook(n->ops ? n->ops->on_deactivate : NULL, n->ctx);
	n->state = ARB_NODE_READY;
	LOG_INF("node '%s' -> READY", n->name);
	return ARB_OK;
}

int arb_node_shutdown(arb_node_t *n)
{
	if (!n) {
		return ARB_ERR_INVAL;
	}
	(void)run_hook(n->ops ? n->ops->on_shutdown : NULL, n->ctx);
	n->state = ARB_NODE_SHUTDOWN;
	LOG_INF("node '%s' -> SHUTDOWN", n->name);
	return ARB_OK;
}

int arb_node_error(arb_node_t *n)
{
	if (!n) {
		return ARB_ERR_INVAL;
	}
	(void)run_hook(n->ops ? n->ops->on_error : NULL, n->ctx);
	n->state = ARB_NODE_ERROR;
	LOG_WRN("node '%s' -> ERROR", n->name);
	return ARB_OK;
}

int arb_node_estop_all(uint16_t source, uint8_t reason)
{
	LOG_ERR("*** EMERGENCY STOP (source=%u reason=%u) ***", source, reason);

	for (uint16_t i = 0; i < node_count; i++) {
		arb_node_t *n = &nodes[i];

		if (n->state == ARB_NODE_READY || n->state == ARB_NODE_ACTIVE) {
			(void)run_hook(n->ops ? n->ops->on_shutdown : NULL,
				       n->ctx);
			n->state = ARB_NODE_ESTOP;
		}
	}

	arb_estop_t msg = { 0 };
	arb_header_init(&msg.header, ARB_MSG_ESTOP, source, stamp_us(), 0);
	msg.source = source;
	msg.reason = reason;

	return zbus_chan_pub(&arb_chan_estop, &msg, K_MSEC(10)) == 0 ?
		       ARB_OK : ARB_ERR_AGAIN;
}

void arb_node_set_heartbeat(arb_node_t *n, uint32_t period_ms)
{
	if (n) {
		n->heartbeat_ms = period_ms;
		n->next_hb_ms   = k_uptime_get();
	}
}

/* ---- heartbeat tick ---------------------------------------------------- */

static struct k_work_delayable hb_work;
static uint32_t hb_seq;

static void hb_tick(struct k_work *work)
{
	ARG_UNUSED(work);
	int64_t now = k_uptime_get();

	for (uint16_t i = 0; i < node_count; i++) {
		arb_node_t *n = &nodes[i];

		if (n->heartbeat_ms == 0 || now < n->next_hb_ms) {
			continue;
		}
		n->next_hb_ms = now + n->heartbeat_ms;

		arb_heartbeat_t hb = { 0 };
		arb_header_init(&hb.header, ARB_MSG_HEARTBEAT, n->id,
				stamp_us(), hb_seq++);
		hb.node      = n->id;
		hb.state     = (uint8_t)n->state;
		hb.uptime_ms = (uint32_t)(now - n->start_ms);

		(void)zbus_chan_pub(&arb_chan_heartbeat, &hb, K_NO_WAIT);
	}

	k_work_schedule(&hb_work, K_MSEC(CONFIG_ARB_HEARTBEAT_TICK_MS));
}

static int arb_node_sys_init(void)
{
	k_work_init_delayable(&hb_work, hb_tick);
	k_work_schedule(&hb_work, K_MSEC(CONFIG_ARB_HEARTBEAT_TICK_MS));
	return 0;
}

SYS_INIT(arb_node_sys_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
