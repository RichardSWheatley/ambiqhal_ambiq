/*
 * ARB request/response services - static registry, synchronous dispatch.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <stdbool.h>

#include "arb/service.h"
#include "arb/topic.h"   /* arb_err_t */
#include "arb/platform.h"

struct svc_entry {
	arb_service_id_t id;
	arb_service_fn   handler;
	void            *user;
	bool             used;
};

static struct svc_entry s_svcs[ARB_MAX_SERVICES];
static uint32_t         s_seq; /* monotonic request sequence */

static struct svc_entry *find_svc(arb_service_id_t id)
{
	for (int i = 0; i < ARB_MAX_SERVICES; i++) {
		if (s_svcs[i].used && s_svcs[i].id == id) {
			return &s_svcs[i];
		}
	}
	return NULL;
}

void arb_service_init(void)
{
	arb_lock_t k = arb_platform_lock();
	for (int i = 0; i < ARB_MAX_SERVICES; i++) {
		s_svcs[i].used = false;
	}
	s_seq = 0;
	arb_platform_unlock(k);
}

int arb_service_register(arb_service_id_t id, arb_service_fn handler, void *user)
{
	if (!handler) {
		return ARB_ERR_INVAL;
	}

	arb_lock_t k = arb_platform_lock();

	struct svc_entry *s = find_svc(id);
	if (!s) {
		for (int i = 0; i < ARB_MAX_SERVICES; i++) {
			if (!s_svcs[i].used) {
				s = &s_svcs[i];
				s->used = true;
				s->id   = id;
				break;
			}
		}
	}
	if (!s) {
		arb_platform_unlock(k);
		return ARB_ERR_NOMEM;
	}

	s->handler = handler;
	s->user    = user;
	arb_platform_unlock(k);
	return ARB_OK;
}

int arb_service_call(arb_service_id_t id, const void *req, size_t req_len,
		     void *resp, size_t resp_cap, size_t *resp_len)
{
	arb_lock_t k = arb_platform_lock();
	struct svc_entry *s = find_svc(id);
	arb_service_fn handler = s ? s->handler : NULL;
	void *user = s ? s->user : NULL;
	uint32_t seq = ++s_seq;
	arb_platform_unlock(k);

	if (!handler) {
		return ARB_ERR_NOTFOUND;
	}

	size_t written = 0;
	int rc = handler(seq, req, req_len, resp, resp_cap, &written, user);
	if (resp_len) {
		*resp_len = written;
	}
	return rc;
}
