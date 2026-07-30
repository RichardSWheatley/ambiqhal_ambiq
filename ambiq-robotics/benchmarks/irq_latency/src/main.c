/*
 * IRQ latency benchmark (Robotics WG D3).
 *
 * Measures hardware-timer compare event -> alarm callback entry, i.e.
 * hardware interrupt latency plus the counter driver's ISR prologue - an
 * honest upper bound that is consistent across runs and boards.
 *
 * Method: snapshot (counter value, cycle counter) back to back, arm an
 * absolute alarm a randomized 300 us..1 ms ahead, and in the callback read
 * the cycle counter again. The expected fire time in cycles comes from a
 * measured counter-tick -> cycle ratio (Q32), so the reported latency is
 *
 *   t_isr - (t_armed + delay_ticks * cycles_per_tick)
 *
 * quantized by one counter tick (reported as counter_ns_per_tick).
 *
 * Kernel-only on purpose: movable to zephyr/tests/benchmarks/ unchanged.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/sys/printk.h>

#include "bench.h"

static const struct device *const counter_dev =
	DEVICE_DT_GET(DT_ALIAS(bench_counter));

static K_SEM_DEFINE(alarm_done, 0, 1);
static volatile uint64_t c_isr;

static struct bench_hist hist;

/* Q32 fixed-point: high-res clock cycles per counter tick */
static uint64_t cyc_per_tick_q32;

static void alarm_cb(const struct device *dev, uint8_t chan_id,
		     uint32_t ticks, void *user_data)
{
	ARG_UNUSED(dev); ARG_UNUSED(chan_id); ARG_UNUSED(ticks);
	ARG_UNUSED(user_data);
	c_isr = bench_clock_now();
	k_sem_give(&alarm_done);
}

static int calibrate(void)
{
	uint32_t t0, t1;
	uint64_t y0, y1;

	y0 = bench_clock_now();
	if (counter_get_value(counter_dev, &t0) != 0) {
		return -EIO;
	}
	k_busy_wait(100000); /* 100 ms */
	y1 = bench_clock_now();
	if (counter_get_value(counter_dev, &t1) != 0) {
		return -EIO;
	}

	uint32_t dt = t1 - t0;

	if (dt == 0) {
		return -EINVAL;
	}
	cyc_per_tick_q32 = ((y1 - y0) << 32) / dt;
	return 0;
}

/* deterministic LCG so runs are reproducible */
static uint32_t lcg(void)
{
	static uint32_t s = 0x12345678u;

	s = s * 1664525u + 1013904223u;
	return s;
}

static int run(const char *load)
{
	uint32_t freq = counter_get_frequency(counter_dev);
	uint32_t top = counter_get_top_value(counter_dev);
	uint32_t d_min = (uint32_t)(((uint64_t)freq * 300) / 1000000); /* 300 us */
	uint32_t d_max = (uint32_t)((uint64_t)freq / 1000);            /* 1 ms   */

	if (d_min == 0) {
		d_min = 1;
	}
	if (d_max <= d_min) {
		d_max = d_min + 1;
	}

	bench_hist_reset(&hist);

	for (uint32_t i = 0; i < CONFIG_BENCH_SAMPLES; i++) {
		uint32_t delay = d_min + (lcg() % (d_max - d_min));
		uint32_t cnt;
		uint64_t y0, y1;

		y0 = bench_clock_now();
		if (counter_get_value(counter_dev, &cnt) != 0) {
			return -EIO;
		}
		y1 = bench_clock_now();

		/* stay clear of counter wrap; skip the rare tail window */
		if (top - cnt < delay + d_max) {
			k_sleep(K_MSEC(2));
			continue;
		}

		struct counter_alarm_cfg cfg = {
			.callback = alarm_cb,
			.ticks = cnt + delay,
			.flags = COUNTER_ALARM_CFG_ABSOLUTE,
		};

		uint64_t c_arm = y0 + (y1 - y0) / 2;

		if (counter_set_channel_alarm(counter_dev, 0, &cfg) != 0) {
			return -EIO;
		}
		k_sem_take(&alarm_done, K_FOREVER);

		uint64_t expect = c_arm +
			((delay * cyc_per_tick_q32) >> 32);
		uint64_t delta = (c_isr > expect) ? c_isr - expect : 0;

		bench_hist_add(&hist, bench_clock_ns(delta));
	}

	char extra[48];

	snprintk(extra, sizeof(extra), "\"counter_ns_per_tick\":%u",
		 (uint32_t)(UINT64_C(1000000000) / freq));
	bench_report("irq_latency", load, &hist, extra);
	return 0;
}

int main(void)
{
	if (!device_is_ready(counter_dev)) {
		printk("BENCH ERROR: counter not ready\n");
		return -ENODEV;
	}

	bench_clock_init();
	(void)counter_start(counter_dev);

	if (calibrate() != 0) {
		printk("BENCH ERROR: calibration failed\n");
		return -EIO;
	}

	bench_load_start();

	if (run(BENCH_LOAD_NAME) != 0) {
		printk("BENCH ERROR: run failed\n");
		return -EIO;
	}

	bench_finish();
	return 0;
}
