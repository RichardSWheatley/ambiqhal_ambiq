/*
 * ARB AmbiqSuite (no-OS) - UART transport for the framed serial bridge.
 *
 * Wires arb/bridge/serial onto an Apollo UART so a no-OS node can stream topics
 * to / from a remote host (telemetry out, commands in) over a serial link.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_AMBIQ_TRANSPORT_UART_H
#define ARB_AMBIQ_TRANSPORT_UART_H

#include <stdint.h>
#include "arb/bridge/serial.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief UART transport context (one per link). */
typedef struct {
	void        *uart;   /**< AmbiqSuite UART handle.        */
	arb_serial_t serial; /**< Framed-serial codec instance.  */
} arb_uart_transport_t;

/**
 * @brief Initialize a UART and bind it to a serial-bridge instance.
 *
 * @param t         Transport context (caller-owned).
 * @param uart_inst UART module number.
 * @param baud      Baud rate (e.g. 115200).
 * @param publish_on_rx If true, frames received over UART are published to the
 *                      local broker (the node acts on remote commands).
 * @return ARB_OK or negative @ref arb_err_t.
 */
int arb_uart_transport_init(arb_uart_transport_t *t, uint32_t uart_inst,
			    uint32_t baud, bool publish_on_rx);

/**
 * @brief Stream a topic out over the UART link.
 *
 * Subscribe this with @c user set to the transport, e.g.:
 *   arb_topic_subscribe(ARB_TOPIC_ODOM, arb_uart_transport_stream, &t);
 */
void arb_uart_transport_stream(arb_topic_id_t topic, const void *msg,
			       size_t len, void *user);

/**
 * @brief Pump any received UART bytes through the frame parser.
 *
 * Call periodically from the super-loop (or from the UART RX ISR).
 */
void arb_uart_transport_poll(arb_uart_transport_t *t);

#ifdef __cplusplus
}
#endif

#endif /* ARB_AMBIQ_TRANSPORT_UART_H */
