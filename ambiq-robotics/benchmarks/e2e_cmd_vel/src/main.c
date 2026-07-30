/*
 * End-to-end cmd_vel latency benchmark (Robotics WG D3). ARB-dependent by
 * design - this one measures the ARB path itself.
 *
 * Mode A (default): on-board closed-loop command path,
 *
 *   zbus_chan_pub(arb_chan_cmd_vel) -> listener -> twist_to_wheels ->
 *   pwm_set_pulse_dt()
 *
 * zbus listeners run synchronously in the publisher's context, so the
 * cycle delta needs no handshake: c1 is captured right after the PWM
 * write, c0 right before the publish. When zephyr,user has bench-gpios,
 * the publish and the actuation each toggle a pin so the same interval
 * can be cross-checked with a scope.
 *
 * Mode B (CONFIG_ARB_BENCH_E2E_ECHO): host round trip over the frozen
 * framing. Drives the chosen arb,uart directly with the OS-free codec -
 * not through the transport, whose export path would re-publish the
 * byte-true echo and self-loop. Run benchmarks/scripts/e2e_echo.py on the
 * host; reported metric is RTT (halve for one-way until link timesync is
 * validated on hardware).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/zbus/zbus.h>

#include "arb/control/diff_drive.h"
#include "arb/msg.h"
#include "arb/time.h"
#include "arb/topics.h"

#include "bench.h"

#define ZUSER DT_PATH(zephyr_user)

/* ---- actuation path (Mode A) --------------------------------------------- */

#if DT_NODE_HAS_PROP(ZUSER, pwms)
#include <zephyr/drivers/pwm.h>
#define HAVE_PWM 1
static const struct pwm_dt_spec motor_pwm[2] = {
	PWM_DT_SPEC_GET_BY_IDX(ZUSER, 0),
	PWM_DT_SPEC_GET_BY_IDX(ZUSER, 1),
};
#else
/* keep the data path honest on boards without the motor wiring */
static volatile uint32_t pwm_sink;
#endif

#if DT_NODE_HAS_PROP(ZUSER, bench_gpios)
#include <zephyr/drivers/gpio.h>
#define HAVE_BENCH_GPIO 1
static const struct gpio_dt_spec bench_gpio[2] = {
	GPIO_DT_SPEC_GET_BY_IDX(ZUSER, bench_gpios, 0),
	GPIO_DT_SPEC_GET_BY_IDX(ZUSER, bench_gpios, 1),
};
#endif

static const arb_diffdrive_t geom = {
	.wheel_base = 0.30f,
	.wheel_radius = 0.04f,
};

static volatile uint64_t c_actuated;

static void cmd_vel_cb(const struct zbus_channel *chan)
{
	const arb_twist_t *t = zbus_chan_const_msg(chan);
	float wl, wr;

	arb_diffdrive_twist_to_wheels(&geom, t->linear.x, t->angular.z,
				      &wl, &wr);
#ifdef HAVE_PWM
	uint32_t pulse =
		(uint32_t)(MIN(1.0f, (wl < 0 ? -wl : wl) / 25.0f) *
			   (float)motor_pwm[0].period);
	(void)pwm_set_pulse_dt(&motor_pwm[0], pulse);
#else
	pwm_sink = (uint32_t)(wl * 1000.0f) + (uint32_t)(wr * 1000.0f);
#endif
#ifdef HAVE_BENCH_GPIO
	gpio_pin_toggle_dt(&bench_gpio[1]);
#endif
	c_actuated = bench_clock_now();
}
ZBUS_LISTENER_DEFINE(bench_cmd_listener, cmd_vel_cb);
ZBUS_CHAN_ADD_OBS(arb_chan_cmd_vel, bench_cmd_listener, 3);

static struct bench_hist hist;

static int run_onboard(void)
{
	arb_twist_t t = { 0 };
	uint32_t seq = 0;

#ifdef HAVE_PWM
	if (!pwm_is_ready_dt(&motor_pwm[0]) || !pwm_is_ready_dt(&motor_pwm[1])) {
		printk("BENCH ERROR: pwm not ready\n");
		return -ENODEV;
	}
#endif
#ifdef HAVE_BENCH_GPIO
	for (int i = 0; i < 2; i++) {
		if (gpio_is_ready_dt(&bench_gpio[i])) {
			gpio_pin_configure_dt(&bench_gpio[i],
					      GPIO_OUTPUT_INACTIVE);
		}
	}
#endif

	bench_hist_reset(&hist);

	for (uint32_t i = 0; i < CONFIG_BENCH_SAMPLES; i++) {
		t.linear.x = ((i & 1) != 0) ? 0.25f : -0.25f;
		t.angular.z = 0.0f;
		arb_header_init(&t.header, ARB_MSG_TWIST, 0,
				arb_time_now_us(), seq++);

#ifdef HAVE_BENCH_GPIO
		gpio_pin_toggle_dt(&bench_gpio[0]);
#endif
		uint64_t c0 = bench_clock_now();

		if (zbus_chan_pub(&arb_chan_cmd_vel, &t, K_MSEC(10)) != 0) {
			continue;
		}
		bench_hist_add(&hist, bench_clock_ns(c_actuated - c0));

		if ((i % 100) == 99) {
			k_sleep(K_MSEC(1)); /* let background load breathe */
		}
	}

	bench_report("e2e_cmd_vel", BENCH_LOAD_NAME, &hist,
		     "\"metric\":\"onboard_cmd_to_pwm_ns\"");
	return 0;
}

/* ---- Mode B: host echo RTT ----------------------------------------------- */

#ifdef CONFIG_ARB_BENCH_E2E_ECHO

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

#include "arb/serial.h"

static const struct device *const echo_uart =
	DEVICE_DT_GET(DT_CHOSEN(arb_uart));

static arb_serial_t tx_codec, rx_codec;
RING_BUF_DECLARE(echo_ring, 256);
static K_SEM_DEFINE(echo_rx, 0, 1);
static volatile uint64_t c_echo;
static volatile uint32_t echo_seq;

static int echo_write(void *ctx, const uint8_t *buf, size_t len)
{
	ARG_UNUSED(ctx);
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(echo_uart, buf[i]);
	}
	return 0;
}

static void echo_frame_cb(uint16_t topic, const void *payload, size_t len,
			  void *user)
{
	ARG_UNUSED(user);
	if (topic != ARB_TOPIC_CMD_VEL || len != sizeof(arb_twist_t)) {
		return;
	}
	const arb_twist_t *t = payload;

	c_echo = bench_clock_now();
	echo_seq = t->header.seq;
	k_sem_give(&echo_rx);
}

static void echo_isr(const struct device *dev, void *user_data)
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
		(void)ring_buf_put(&echo_ring, buf, n);
	}
	k_sem_give(&echo_rx);
}

static int run_echo(void)
{
	arb_twist_t t = { 0 };
	uint8_t buf[64];

	if (!device_is_ready(echo_uart)) {
		printk("BENCH ERROR: arb,uart not ready\n");
		return -ENODEV;
	}
	(void)arb_serial_init(&tx_codec, echo_write, NULL, NULL, NULL);
	(void)arb_serial_init(&rx_codec, NULL, NULL, echo_frame_cb, NULL);
	uart_irq_callback_user_data_set(echo_uart, echo_isr, NULL);
	uart_irq_rx_enable(echo_uart);

	bench_hist_reset(&hist);

	for (uint32_t i = 0; i < CONFIG_BENCH_SAMPLES; i++) {
		t.linear.x = 0.25f;
		arb_header_init(&t.header, ARB_MSG_TWIST, 0,
				arb_time_now_us(), i);

		uint64_t c0 = bench_clock_now();

		(void)arb_serial_send(&tx_codec, ARB_TOPIC_CMD_VEL, &t,
				      sizeof(t));

		/* drain + decode until our seq echoes back (100 ms budget) */
		int64_t deadline = k_uptime_get() + 100;
		bool got = false;

		while (k_uptime_get() < deadline) {
			uint32_t n = ring_buf_get(&echo_ring, buf, sizeof(buf));

			if (n > 0) {
				arb_serial_rx(&rx_codec, buf, n);
				if (echo_seq == i && c_echo >= c0) {
					got = true;
					break;
				}
				continue;
			}
			(void)k_sem_take(&echo_rx, K_MSEC(5));
		}
		if (got) {
			bench_hist_add(&hist, bench_clock_ns(c_echo - c0));
		}
	}

	bench_report("e2e_cmd_vel", "idle", &hist,
		     "\"metric\":\"host_rtt_ns\"");
	return 0;
}

#endif /* CONFIG_ARB_BENCH_E2E_ECHO */

int main(void)
{
	bench_clock_init();
	bench_load_start();

#ifdef CONFIG_ARB_BENCH_E2E_ECHO
	if (run_echo() != 0) {
		return -EIO;
	}
#else
	if (run_onboard() != 0) {
		return -EIO;
	}
#endif

	bench_finish();
	return 0;
}
