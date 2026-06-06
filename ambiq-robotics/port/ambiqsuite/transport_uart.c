/*
 * ARB AmbiqSuite (no-OS) - UART transport for the framed serial bridge.
 *
 * Blocking UART I/O is fine for a super-loop node; for higher throughput, switch
 * to the buffered/non-blocking UART API and feed the RX ring into the parser
 * from the UART ISR. Pin (GPIO) configuration for the UART is board-specific and
 * is left to the application.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include "transport_uart.h"
#include "arb/topic.h" /* arb_err_t */

#include "am_mcu_apollo.h"

static int uart_write(void *ctx, const uint8_t *buf, size_t len)
{
	uint32_t sent = 0;
	am_hal_uart_transfer_t tr = {0};
	tr.eType = AM_HAL_UART_BLOCKING_WRITE;
	tr.pui8Data = (uint8_t *)buf;
	tr.ui32NumBytes = (uint32_t)len;
	tr.pui32BytesTransferred = &sent;
	tr.ui32TimeoutMs = 100;

	return (am_hal_uart_transfer(ctx, &tr) == AM_HAL_STATUS_SUCCESS) ? 0 : -1;
}

int arb_uart_transport_init(arb_uart_transport_t *t, uint32_t uart_inst,
			    uint32_t baud, bool publish_on_rx)
{
	if (!t) {
		return ARB_ERR_INVAL;
	}

	if (am_hal_uart_initialize(uart_inst, &t->uart) != AM_HAL_STATUS_SUCCESS) {
		return ARB_ERR_AGAIN;
	}
	am_hal_uart_power_control(t->uart, AM_HAL_SYSCTRL_WAKE, false);

	am_hal_uart_config_t cfg = {0};
	cfg.ui32BaudRate  = baud;
	cfg.eDataBits     = AM_HAL_UART_DATA_BITS_8;
	cfg.eParity       = AM_HAL_UART_PARITY_NONE;
	cfg.eStopBits     = AM_HAL_UART_ONE_STOP_BIT;
	cfg.eFlowControl  = AM_HAL_UART_FLOW_CTRL_NONE;
	am_hal_uart_configure(t->uart, &cfg);
	/* NOTE: configure the UART TX/RX GPIO pins here for your board. */

	arb_serial_init(&t->serial, uart_write, t->uart);
	t->serial.publish_on_rx = publish_on_rx;
	return ARB_OK;
}

void arb_uart_transport_stream(arb_topic_id_t topic, const void *msg,
			       size_t len, void *user)
{
	arb_uart_transport_t *t = (arb_uart_transport_t *)user;
	if (t) {
		(void)arb_serial_send(&t->serial, topic, msg, len);
	}
}

void arb_uart_transport_poll(arb_uart_transport_t *t)
{
	if (!t) {
		return;
	}

	uint8_t tmp[64];
	uint32_t got = 0;
	am_hal_uart_transfer_t tr = {0};
	tr.eType = AM_HAL_UART_BLOCKING_READ;
	tr.pui8Data = tmp;
	tr.ui32NumBytes = sizeof(tmp);
	tr.pui32BytesTransferred = &got;
	tr.ui32TimeoutMs = 0; /* return immediately with whatever is available */

	if (am_hal_uart_transfer(t->uart, &tr) == AM_HAL_STATUS_SUCCESS && got) {
		arb_serial_rx(&t->serial, tmp, got);
	}
}
