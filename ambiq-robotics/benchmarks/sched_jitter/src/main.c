/*
 * Scheduling jitter benchmark (Robotics WG D3).
 *
 * cyclictest-shaped: a periodic thread at (nominally) 1 kHz using
 * absolute-deadline sleeps, k_sleep(K_TIMEOUT_ABS_TICKS(next)).
 *
 * Two metrics per run, two RECORD lines, distinguished by "metric":
 *
 *  - wake_err_ns: wakeup lateness vs the absolute deadline, measured with
 *    the kernel cycle counter. Deadline and measurement share the system
 *    timer clock, so there is no cross-oscillator drift; resolution is one
 *    hw cycle of that timer (coarse on SoCs that tick at 32.768 kHz -
 *    reported as tickclock_ns_per_cycle).
 *
 *  - period_jitter_ns: |wake-to-wake delta - nominal period| via the
 *    high-resolution clock (DWT on Cortex-M). Fine-grained, and immune to
 *    cross-clock drift because each delta spans only one period.
 *
 * Kernel-only on purpose: movable to zephyr/tests/benchmarks/ unchanged.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "bench.h"

static struct bench_hist wake_hist;
static struct bench_hist period_hist;

/* 64-bit kernel-timer-domain cycles (single caller, no locking needed) */
static uint64_t tickclock_now(void)
{
#ifdef CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER
	return k_cycle_get_64();
#else
	static uint32_t last;
	static uint64_t high;
	uint32_t now = k_cycle_get_32();

	if (now < last) {
		high += UINT64_C(1) << 32;
	}
	last = now;
	return high | now;
#endif
}

int main(void)
{
	uint32_t rate_hz = CONFIG_BENCH_RATE_HZ;
	int64_t period_ticks = MAX(1, CONFIG_SYS_CLOCK_TICKS_PER_SEC / rate_hz);
	uint64_t hw_freq = (uint64_t)sys_clock_hw_cycles_per_sec();
	uint64_t period_ns = k_ticks_to_ns_floor64(period_ticks);

	bench_clock_init();
	bench_hist_reset(&wake_hist);
	bench_hist_reset(&period_hist);

	bench_load_start();

	/* align to a tick boundary before measuring */
	k_sleep(K_TICKS(1));

	int64_t next = k_uptime_ticks() + period_ticks;
	uint64_t prev_hi = 0;

	for (uint32_t i = 0; i < CONFIG_BENCH_SAMPLES; i++, next += period_ticks) {
		k_sleep(K_TIMEOUT_ABS_TICKS(next));

		uint64_t now_hi = bench_clock_now();
		uint64_t now_tc = tickclock_now();

		/* deadline, converted to kernel-timer cycles (same clock) */
		uint64_t deadline_tc =
			((uint64_t)next * hw_freq) / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
		uint64_t late_cyc = (now_tc > deadline_tc) ?
			now_tc - deadline_tc : 0;

		bench_hist_add(&wake_hist, k_cyc_to_ns_floor64(late_cyc));

		if (i > 0) {
			uint64_t delta_ns = bench_clock_ns(now_hi - prev_hi);
			uint64_t jit = (delta_ns > period_ns) ?
				delta_ns - period_ns : period_ns - delta_ns;

			bench_hist_add(&period_hist, jit);
		}
		prev_hi = now_hi;
	}

	char extra[96];

	snprintk(extra, sizeof(extra),
		 "\"metric\":\"wake_err_ns\",\"period_ns\":%llu,"
		 "\"tickclock_ns_per_cycle\":%u",
		 (unsigned long long)period_ns,
		 (uint32_t)(UINT64_C(1000000000) / hw_freq));
	bench_report("sched_jitter", BENCH_LOAD_NAME, &wake_hist, extra);

	snprintk(extra, sizeof(extra),
		 "\"metric\":\"period_jitter_ns\",\"period_ns\":%llu",
		 (unsigned long long)period_ns);
	bench_report("sched_jitter", BENCH_LOAD_NAME, &period_hist, extra);

	bench_finish();
	return 0;
}
