/*
 * ARB pub/sub broker - static topic & subscriber registry.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <stdbool.h>

#include "arb/topic.h"
#include "arb/platform.h"
#include "arb/msg.h"

/* The platform queue copies whole messages, so no message may exceed this. */
typedef char arb_msg_size_guard[(ARB_MSG_MAX_SIZE >= sizeof(arb_imu_t)) ? 1 : -1];

struct topic_entry {
	arb_topic_id_t id;
	uint16_t       msg_size;
	bool           used;
};

struct sub_entry {
	arb_topic_id_t    topic;
	arb_subscriber_fn cb;
	void             *user;
	bool              used;
};

static struct topic_entry s_topics[ARB_MAX_TOPICS];
static struct sub_entry   s_subs[ARB_MAX_SUBSCRIBERS];

static struct topic_entry *find_topic(arb_topic_id_t id)
{
	for (int i = 0; i < ARB_MAX_TOPICS; i++) {
		if (s_topics[i].used && s_topics[i].id == id) {
			return &s_topics[i];
		}
	}
	return NULL;
}

void arb_topic_init(void)
{
	arb_lock_t k = arb_platform_lock();
	for (int i = 0; i < ARB_MAX_TOPICS; i++) {
		s_topics[i].used = false;
	}
	for (int i = 0; i < ARB_MAX_SUBSCRIBERS; i++) {
		s_subs[i].used = false;
	}
	arb_platform_unlock(k);
}

int arb_topic_advertise(arb_topic_id_t topic, size_t msg_size)
{
	if (msg_size == 0 || msg_size > ARB_MSG_MAX_SIZE) {
		return ARB_ERR_TOOBIG;
	}

	arb_lock_t k = arb_platform_lock();

	struct topic_entry *t = find_topic(topic);
	if (t) {
		int rc = (t->msg_size == msg_size) ? ARB_OK : ARB_ERR_INVAL;
		arb_platform_unlock(k);
		return rc;
	}

	for (int i = 0; i < ARB_MAX_TOPICS; i++) {
		if (!s_topics[i].used) {
			s_topics[i].used     = true;
			s_topics[i].id       = topic;
			s_topics[i].msg_size = (uint16_t)msg_size;
			arb_platform_unlock(k);
			return ARB_OK;
		}
	}

	arb_platform_unlock(k);
	return ARB_ERR_NOMEM;
}

int arb_topic_subscribe(arb_topic_id_t topic, arb_subscriber_fn cb, void *user)
{
	if (!cb) {
		return ARB_ERR_INVAL;
	}

	arb_lock_t k = arb_platform_lock();
	for (int i = 0; i < ARB_MAX_SUBSCRIBERS; i++) {
		if (!s_subs[i].used) {
			s_subs[i].used  = true;
			s_subs[i].topic = topic;
			s_subs[i].cb    = cb;
			s_subs[i].user  = user;
			arb_platform_unlock(k);
			return ARB_OK;
		}
	}
	arb_platform_unlock(k);
	return ARB_ERR_NOMEM;
}

int arb_topic_publish(arb_topic_id_t topic, const void *msg, size_t len)
{
	if (!msg || len == 0) {
		return ARB_ERR_INVAL;
	}

	arb_lock_t k = arb_platform_lock();
	struct topic_entry *t = find_topic(topic);
	size_t expect = t ? t->msg_size : 0;
	arb_platform_unlock(k);

	if (!t) {
		return ARB_ERR_NOTFOUND;
	}
	if (len != expect) {
		return ARB_ERR_TOOBIG;
	}

	/* Hand off to the port queue; delivery happens in dispatch context. */
	return arb_platform_post(topic, msg, len);
}

void arb_topic_deliver(arb_topic_id_t topic, const void *msg, size_t len)
{
	/*
	 * Snapshot matching subscribers under the lock, then invoke callbacks
	 * with the lock released so a subscriber may (re)publish or (un)register
	 * without deadlocking.
	 */
	for (int i = 0; i < ARB_MAX_SUBSCRIBERS; i++) {
		arb_subscriber_fn cb = NULL;
		void *user = NULL;

		arb_lock_t k = arb_platform_lock();
		if (s_subs[i].used && s_subs[i].topic == topic) {
			cb   = s_subs[i].cb;
			user = s_subs[i].user;
		}
		arb_platform_unlock(k);

		if (cb) {
			cb(topic, msg, len, user);
		}
	}
}
