/*
 * ARB - error codes
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_ERR_H
#define ARB_ERR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	ARB_OK           =  0,
	ARB_ERR_INVAL    = -1, /**< Bad argument.                    */
	ARB_ERR_NOMEM    = -2, /**< Static pool exhausted.           */
	ARB_ERR_NOTFOUND = -3, /**< Topic/node not registered.       */
	ARB_ERR_TOOBIG   = -4, /**< Payload exceeds channel size.    */
	ARB_ERR_FULL     = -5, /**< Queue full, message dropped.     */
	ARB_ERR_AGAIN    = -6, /**< Temporary failure, retry.        */
	ARB_ERR_STATE    = -7, /**< Invalid lifecycle transition.    */
} arb_err_t;

#ifdef __cplusplus
}
#endif

#endif /* ARB_ERR_H */
