/*
 * ARB - serial transport (Zephyr UART glue).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

#include "arb/serial.h"
#include "arb/topics.h"
#include "arb/transport.h"

LOG_MODULE_DECLARE(arb, CONFIG_ARB_LOG_LEVEL);

#if !DT_HAS_CHOSEN(arb_uart)
#error "CONFIG_ARB_TRANSPORT=y requires a devicetree 'chosen { arb,uart = &uartX; }' node"
#endif

static const struct device *const uart_dev =
	DEVICE_DT_GET(DT_CHOSEN(arb_uart));

static arb_serial_t codec;
RING_BUF_DECLARE(rx_ring, 512);
static struct k_work rx_work;

/* ---- TX ----------------------------------------------------------------- */

static int uart_write(void *ctx, const uint8_t *buf, size_t len)
{
	ARG_UNUSED(ctx);
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
	return 0;
}

static K_MUTEX_DEFINE(tx_lock);

/* zbus listener: frame and transmit every publication on exported channels. */
static void export_cb(const struct zbus_channel *chan)
{
	uint16_t id = arb_topic_id(chan);

	if (id == 0) {
		return;
	}

	k_mutex_lock(&tx_lock, K_FOREVER);
	(void)arb_serial_send(&codec, id, zbus_chan_const_msg(chan),
			      zbus_chan_msg_size(chan));
	k_mutex_unlock(&tx_lock);
}

ZBUS_LISTENER_DEFINE(arb_transport_listener, export_cb);

int arb_transport_export(const struct zbus_channel *chan)
{
	if (!chan || arb_topic_id(chan) == 0) {
		return ARB_ERR_INVAL;
	}
	int rc = zbus_chan_add_obs(chan, &arb_transport_listener, K_MSEC(100));

	return (rc == 0 || rc == -EEXIST) ? ARB_OK : ARB_ERR_AGAIN;
}

int arb_transport_unexport(const struct zbus_channel *chan)
{
	if (!chan) {
		return ARB_ERR_INVAL;
	}
	return zbus_chan_rm_obs(chan, &arb_transport_listener, K_MSEC(100)) ==
			       0 ? ARB_OK : ARB_ERR_NOTFOUND;
}

/* ---- RX ----------------------------------------------------------------- */

/* Complete frame (system workqueue context): publish on the mapped channel. */
static void frame_cb(uint16_t topic, const void *payload, size_t len,
		     void *user)
{
	ARG_UNUSED(user);
	const struct zbus_channel *chan = arb_topic_chan(topic);

	if (!chan) {
		LOG_WRN("rx frame for unknown topic %u", topic);
		return;
	}
	if (len != zbus_chan_msg_size(chan)) {
		LOG_WRN("rx frame size %u != %u for topic %u", (unsigned)len,
			(unsigned)zbus_chan_msg_size(chan), topic);
		return;
	}
	(void)zbus_chan_pub(chan, payload, K_MSEC(5));
}

static void rx_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	uint8_t buf[64];
	uint32_t n;

	while ((n = ring_buf_get(&rx_ring, buf, sizeof(buf))) > 0) {
		arb_serial_rx(&codec, buf, n);
	}
}

static void uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	uint8_t buf[32];

	if (!uart_irq_update(dev)) {
		return;
	}
	while (uart_irq_rx_ready(dev)) {
		int n = uart_fifo_read(dev, buf, sizeof(buf));

		if (n <= 0) {
			break;
		}
		(void)ring_buf_put(&rx_ring, buf, n);
	}
	k_work_submit(&rx_work);
}

void arb_transport_stats(uint32_t *rx_frames, uint32_t *rx_crc_errors)
{
	if (rx_frames) {
		*rx_frames = codec.rx_frames;
	}
	if (rx_crc_errors) {
		*rx_crc_errors = codec.rx_crc_errors;
	}
}

/* ---- init --------------------------------------------------------------- */

static int arb_transport_init(void)
{
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("arb,uart device not ready");
		return -ENODEV;
	}

	(void)arb_serial_init(&codec, uart_write, NULL, frame_cb, NULL);
	k_work_init(&rx_work, rx_work_fn);

	uart_irq_callback_user_data_set(uart_dev, uart_isr, NULL);
	uart_irq_rx_enable(uart_dev);

	LOG_INF("transport up on %s", uart_dev->name);
	return 0;
}

SYS_INIT(arb_transport_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
