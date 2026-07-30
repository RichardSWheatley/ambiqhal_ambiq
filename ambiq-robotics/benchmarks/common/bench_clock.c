/*
 * ARB benchmarks - high-resolution clock shim.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <zephyr/kernel.h>

#include "bench.h"

#ifdef CONFIG_TIMING_FUNCTIONS

#include <zephyr/timing/timing.h>

void bench_clock_init(void)
{
	timing_init();
	timing_start();
}

uint64_t bench_clock_now(void)
{
	return (uint64_t)timing_counter_get();
}

uint64_t bench_clock_ns(uint64_t cycles)
{
	return timing_cycles_to_ns(cycles);
}

uint64_t bench_clock_freq_hz(void)
{
	return timing_freq_get();
}

#else /* !CONFIG_TIMING_FUNCTIONS */

void bench_clock_init(void)
{
}

uint64_t bench_clock_now(void)
{
#ifdef CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER
	return k_cycle_get_64();
#else
	/*
	 * 32 -> 64 bit extension; assumes calls happen more often than the
	 * 32-bit counter wraps, which every benchmark loop here guarantees.
	 */
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

uint64_t bench_clock_ns(uint64_t cycles)
{
	return k_cyc_to_ns_floor64(cycles);
}

uint64_t bench_clock_freq_hz(void)
{
	return (uint64_t)sys_clock_hw_cycles_per_sec();
}

#endif /* CONFIG_TIMING_FUNCTIONS */
