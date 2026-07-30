/*
 * ARB benchmarks - shared clock, histogram and reporting helpers.
 *
 * Deliberately ARB-free: this directory depends only on kernel APIs so the
 * kernel-only benchmarks can move to zephyr/tests/benchmarks/ unchanged.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifndef ARB_BENCH_H
#define ARB_BENCH_H

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- high-resolution clock ---------------------------------------------- */

/*
 * On Cortex-M with CONFIG_TIMING_FUNCTIONS the clock is the DWT cycle
 * counter (CPU-clock resolution); elsewhere it falls back to the kernel
 * cycle counter. Values are raw cycles of whichever source is active.
 */
void bench_clock_init(void);
uint64_t bench_clock_now(void);
uint64_t bench_clock_ns(uint64_t cycles);
uint64_t bench_clock_freq_hz(void);

/* ---- histogram ----------------------------------------------------------- */

/*
 * HDR-lite histogram: 16 linear buckets under 16, then log2 major buckets
 * with 16 linear sub-buckets each (4 mantissa bits). 1024 x u32 = 4 KiB,
 * relative quantization error <= 1/32 (~3.1%) at the percentiles using
 * bucket midpoints; min/max/sum/count are tracked exactly.
 *
 * Chosen over storing raw samples (100k x u32 = 400 KiB does not fit small
 * targets) and over reservoir sampling (which distorts exactly the tails a
 * p99 is meant to capture).
 */
#define BENCH_HIST_BUCKETS 1024

struct bench_hist {
	uint64_t min;
	uint64_t max;
	uint64_t sum;
	uint32_t n;
	uint32_t bucket[BENCH_HIST_BUCKETS];
};

void bench_hist_reset(struct bench_hist *h);
void bench_hist_add(struct bench_hist *h, uint64_t val);
/* pct_x100: 9900 = p99, 5000 = median */
uint64_t bench_hist_percentile(const struct bench_hist *h, uint32_t pct_x100);

/* ---- reporting ----------------------------------------------------------- */

/*
 * Emits one machine-readable line per run (histogram values must be ns):
 *
 *   RECORD: {"bench":"<bench>","board":"<CONFIG_BOARD>","load":"<load>",
 *            "n":...,"min_ns":...,"avg_ns":...,"max_ns":...,"p99_ns":...}
 *
 * The "RECORD: " prefix matches the twister harness_config record
 * convention (as_json), keeping results diffable across boards and runs.
 * Extra context keys may be appended by passing a non-NULL extra string
 * (must be valid JSON members, e.g. "\"period_ns\":1000000").
 */
void bench_report(const char *bench, const char *load,
		  const struct bench_hist *h, const char *extra);

/*
 * Print the console-harness end marker and, on the POSIX arch, terminate
 * the simulator (a returning main() would leave native_sim idling forever).
 */
static inline void bench_finish(void)
{
	printk("BENCH DONE\n");
#ifdef CONFIG_ARCH_POSIX
	extern void posix_exit(int exit_code);
	posix_exit(0);
#endif
}

/* ---- background load (CONFIG_BENCH_LOAD) --------------------------------- */

/*
 * Mixed-criticality load: a 1 kHz zbus publisher with a listener, periodic
 * logging, and a low-priority memory-churn thread. Started explicitly so
 * the benchmark can calibrate on an idle system first.
 */
#ifdef CONFIG_BENCH_LOAD
void bench_load_start(void);
#define BENCH_LOAD_NAME "zbus+log"
#else
static inline void bench_load_start(void) {}
#define BENCH_LOAD_NAME "idle"
#endif

#ifdef __cplusplus
}
#endif

#endif /* ARB_BENCH_H */
