/*
 * ARB - serial transport backend (Zephyr UART glue).
 *
 * TX: per-byte polled writes in the caller's context (see the note in
 * transport_core.c). RX: interrupt-driven FIFO drain into a ring buffer,
 * decoded on the system workqueue.
 *
 * RX timestamping: the ISR snapshots arb_time_now_us() at entry, before
 * draining the FIFO; the decode pass attributes that stamp to the bytes it
 * processes. The attribution error is bounded by one FIFO batch (32 bytes,
 * ~350 us at 921600 baud) - the timesync median filter absorbs it, and the
 * bound is part of the documented sync accuracy.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

#include "arb/time.h"
#include "arb/transport_backend.h"

LOG_MODULE_DECLARE(arb, CONFIG_ARB_LOG_LEVEL);

#if !DT_HAS_CHOSEN(arb_uart)
#error "CONFIG_ARB_TRANSPORT_SERIAL=y requires a devicetree 'chosen { arb,uart = &uartX; }' node"
#endif

static const struct device *const uart_dev =
	DEVICE_DT_GET(DT_CHOSEN(arb_uart));

RING_BUF_DECLARE(rx_ring, 512);
static struct k_work rx_work;

/* stamp of the most recent RX interrupt (see file header for the bound) */
static volatile uint64_t rx_isr_stamp_us;

static int serial_write(const uint8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
	return 0;
}

static void rx_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	uint8_t buf[64];
	uint32_t n;

	while ((n = ring_buf_get(&rx_ring, buf, sizeof(buf))) > 0) {
		arb_transport_core_rx(buf, n, rx_isr_stamp_us);
	}
}

static void uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	uint8_t buf[32];

	if (!uart_irq_update(dev)) {
		return;
	}
	rx_isr_stamp_us = arb_time_now_us();
	while (uart_irq_rx_ready(dev)) {
		int n = uart_fifo_read(dev, buf, sizeof(buf));

		if (n <= 0) {
			break;
		}
		(void)ring_buf_put(&rx_ring, buf, n);
	}
	k_work_submit(&rx_work);
}

static int serial_init(void)
{
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("arb,uart device not ready");
		return -ENODEV;
	}

	k_work_init(&rx_work, rx_work_fn);
	uart_irq_callback_user_data_set(uart_dev, uart_isr, NULL);
	uart_irq_rx_enable(uart_dev);

	LOG_INF("serial transport up on %s", uart_dev->name);
	return 0;
}

const struct arb_transport_backend arb_transport_backend = {
	.init = serial_init,
	.write = serial_write,
};
