/*
 * Ambiq Robotics Broker (ARB) - request/response (service) API
 *
 * Maps to ROS2 services / JAUS query-command pairs, sized for an MCU. A service
 * is a single registered handler addressed by id; a call invokes it and fills a
 * caller-provided response buffer. Each call carries a sequence id for tracing.
 *
 * The core implementation is synchronous (the handler runs in the caller's
 * context). This keeps it usable from a bare-metal super-loop with zero
 * scheduling machinery; a port may layer async dispatch on top if desired.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_SERVICE_H
#define ARB_SERVICE_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Service identifier. Applications assign their own stable IDs. */
typedef uint16_t arb_service_id_t;

/**
 * @brief Service handler.
 *
 * @param seq      Sequence id of this request (for logging/tracing).
 * @param req      Request payload (read-only).
 * @param req_len  Request length in bytes.
 * @param resp     Response buffer to fill.
 * @param resp_cap Capacity of @p resp in bytes.
 * @param resp_len [out] Number of bytes written to @p resp.
 * @param user     Opaque pointer supplied at register time.
 * @return 0 on success, negative on application error (returned to caller).
 */
typedef int (*arb_service_fn)(uint32_t seq, const void *req, size_t req_len,
			      void *resp, size_t resp_cap, size_t *resp_len,
			      void *user);

#ifndef ARB_MAX_SERVICES
#define ARB_MAX_SERVICES 16
#endif

/**
 * @brief Reset the service registry to empty.
 */
void arb_service_init(void);

/**
 * @brief Register (or replace) the handler for a service id.
 * @return ARB_OK, or a negative @ref arb_err_t.
 */
int arb_service_register(arb_service_id_t id, arb_service_fn handler,
			 void *user);

/**
 * @brief Call a service synchronously.
 *
 * @param id       Service id to invoke.
 * @param req      Request payload.
 * @param req_len  Request length.
 * @param resp     Response buffer.
 * @param resp_cap Response buffer capacity.
 * @param resp_len [out] Bytes written by the handler (may be NULL).
 * @return The handler's return value (>=0), or a negative @ref arb_err_t if the
 *         service is not registered.
 */
int arb_service_call(arb_service_id_t id, const void *req, size_t req_len,
		     void *resp, size_t resp_cap, size_t *resp_len);

#ifdef __cplusplus
}
#endif

#endif /* ARB_SERVICE_H */
