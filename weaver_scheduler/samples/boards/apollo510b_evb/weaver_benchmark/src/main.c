/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Canonical Weaver benchmark workload (RFC §7.1).
 *
 * Models a wearable sensor pipeline:
 *   3 Warp threads:  BLE LL (50 ms), IMU (50 Hz), PPG (25 Hz)
 *   6 Weft threads:  fusion, HR, GATT TX, classifier, display, NVM
 *
 * Two burst events at t=10 s and t=20 s (50 ms IMU @ 200 Hz + 30
 * GATT notifications) stress the dispatcher.
 *
 * Runs for 60 s, then prints a single-line metrics summary that can
 * be diffed across builds:
 *
 *   stock / scalar / hybrid / full-MVE
 *
 * Build any of these via EXTRA_CONF_FILE=stock.conf|hybrid.conf|mve.conf
 * Default prj.conf selects the SCALAR Weaver variant.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_WEAVER_SCHED
#include <zephyr/kernel/weaver_sched.h>
#endif

LOG_MODULE_REGISTER(weaver_bench, LOG_LEVEL_INF);

#define DEMO_DURATION_MS    60000
#define BURST_AT_MS_1       10000
#define BURST_AT_MS_2       20000
#define BURST_DURATION_MS   50
#define WARMUP_MS           2000

#define IMU_PERIOD_MS       20
#define IMU_BURST_PERIOD_MS 5
#define PPG_PERIOD_MS       40
#define BLE_PERIOD_MS       50
#define DISP_PERIOD_MS      33
#define NVM_PERIOD_MS       1000
#define CLASS_PERIOD_MS     500

/* Mock workload: each thread uses k_busy_wait to simulate compute. */
#define BLE_WORK_US         3000
#define IMU_WORK_US         200
#define PPG_WORK_US         600
#define FUSION_WORK_US      1500
#define HR_WORK_US          8000
#define GATT_WORK_US        300
#define DISP_WORK_US        4000
#define NVM_WORK_US         12000
#define CLASS_WORK_US       2000

/* Priorities. Same values whether Weaver is on or off. */
#define BLE_PRIO        2
#define IMU_PRIO        3
#define PPG_PRIO        4
#define FUSION_PRIO     8
#define HR_PRIO         8
#define GATT_PRIO       9
#define CLASS_PRIO      10
#define DISP_PRIO       11
#define NVM_PRIO        12

#define STACK_SIZE 2048
#define FIFO_DEPTH 32

/* ---- Synthetic FIFO models (replace hardware sensor FIFOs) ---- */

struct fifo {
	atomic_t depth;
	atomic_t produced;
	atomic_t consumed;
	atomic_t overruns;
};

static struct fifo imu_fifo, ppg_fifo, gatt_fifo;

static inline void fifo_produce(struct fifo *f, int n)
{
	int d = atomic_add(&f->depth, n) + n;
	atomic_add(&f->produced, n);
	if (d > FIFO_DEPTH) {
		atomic_add(&f->overruns, d - FIFO_DEPTH);
		atomic_set(&f->depth, FIFO_DEPTH);
	}
}

static inline int fifo_consume(struct fifo *f, int n)
{
	int d = atomic_get(&f->depth);
	int c = (d < n) ? d : n;
	if (c > 0) {
		atomic_sub(&f->depth, c);
		atomic_add(&f->consumed, c);
	}
	return c;
}

#ifdef CONFIG_WEAVER_SCHED
static inline uint32_t depth_to_q16(int depth)
{
	if (depth <= 0) return 0;
	if (depth >= FIFO_DEPTH) return WEAVER_Q16_ONE;
	return ((uint32_t)depth * WEAVER_Q16_ONE) / FIFO_DEPTH;
}
#endif

/* ---- Per-thread latency histograms (sensor-to-process M2) ---- */

#define HIST_BUCKETS 16
struct latency_hist {
	uint32_t count;
	uint32_t bucket_us[HIST_BUCKETS];  /* upper edges in us, last is +inf */
	atomic_t buckets[HIST_BUCKETS];
	uint32_t sum_us;
	uint32_t max_us;
};

static struct latency_hist fusion_lat = {
	.bucket_us = { 100, 200, 500, 1000, 2000, 5000, 10000,
		       20000, 50000, 100000, 200000, 500000,
		       1000000, 2000000, 5000000, UINT32_MAX },
};

static struct latency_hist hr_lat = {
	.bucket_us = { 100, 200, 500, 1000, 2000, 5000, 10000,
		       20000, 50000, 100000, 200000, 500000,
		       1000000, 2000000, 5000000, UINT32_MAX },
};

static void hist_add(struct latency_hist *h, uint32_t lat_us)
{
	for (int i = 0; i < HIST_BUCKETS; i++) {
		if (lat_us <= h->bucket_us[i]) {
			atomic_inc(&h->buckets[i]);
			break;
		}
	}
	h->count++;
	h->sum_us += lat_us;
	if (lat_us > h->max_us) h->max_us = lat_us;
}

/* Return percentile (0..100) latency from histogram. Pessimistic:
 * returns the bucket upper edge. Good enough for relative comparison.
 */
static uint32_t hist_percentile(const struct latency_hist *h, int pct)
{
	uint32_t total = 0;
	for (int i = 0; i < HIST_BUCKETS; i++) total += atomic_get(&h->buckets[i]);
	if (total == 0) return 0;
	uint32_t target = (total * pct) / 100;
	uint32_t cum = 0;
	for (int i = 0; i < HIST_BUCKETS; i++) {
		cum += atomic_get(&h->buckets[i]);
		if (cum >= target) return h->bucket_us[i];
	}
	return h->bucket_us[HIST_BUCKETS - 1];
}

/* Producer-side timestamp queue so consumer can compute latency. */
struct ts_queue {
	uint32_t ts[FIFO_DEPTH];
	atomic_t head;
	atomic_t tail;
};

static struct ts_queue imu_ts, ppg_ts;

static inline void ts_push(struct ts_queue *q, uint32_t ts_us)
{
	int h = atomic_inc(&q->head);
	q->ts[h % FIFO_DEPTH] = ts_us;
}

static inline uint32_t ts_pop(struct ts_queue *q)
{
	int t = atomic_get(&q->tail);
	int h = atomic_get(&q->head);
	if (t >= h) return 0;
	uint32_t v = q->ts[t % FIFO_DEPTH];
	atomic_inc(&q->tail);
	return v;
}

/* ---- Burst injection state ---- */

static atomic_t in_burst = ATOMIC_INIT(0);

/* ---- Warp dummy / Weft dummy ---- */

#ifdef CONFIG_WEAVER_SCHED
static struct weaver_thread_data wd_ble, wd_imu, wd_ppg, wd_fusion, wd_hr,
				 wd_gatt, wd_class, wd_disp, wd_nvm;
#endif

static void ble_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		k_busy_wait(BLE_WORK_US);
		k_sleep(K_MSEC(BLE_PERIOD_MS));
	}
}

static void imu_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		uint32_t period = atomic_get(&in_burst) ?
			IMU_BURST_PERIOD_MS : IMU_PERIOD_MS;

		k_busy_wait(IMU_WORK_US);

		/* One sample per loop normally; 4 per loop in burst to
		 * model the 200 Hz burst as 4× the data without
		 * changing thread cadence. */
		int n = atomic_get(&in_burst) ? 4 : 1;
		uint32_t now_us = k_cyc_to_us_floor32(k_cycle_get_32());
		for (int i = 0; i < n; i++) ts_push(&imu_ts, now_us);
		fifo_produce(&imu_fifo, n);

#ifdef CONFIG_WEAVER_SCHED
		weaver_set_buffer_fill_predictive_q16(
			&wd_fusion, depth_to_q16(atomic_get(&imu_fifo.depth)));
#endif
		k_sleep(K_MSEC(period));
	}
}

static void ppg_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		k_busy_wait(PPG_WORK_US);
		uint32_t now_us = k_cyc_to_us_floor32(k_cycle_get_32());
		ts_push(&ppg_ts, now_us);
		fifo_produce(&ppg_fifo, 1);
#ifdef CONFIG_WEAVER_SCHED
		weaver_set_buffer_fill_predictive_q16(
			&wd_hr, depth_to_q16(atomic_get(&ppg_fifo.depth)));
#endif
		k_sleep(K_MSEC(PPG_PERIOD_MS));
	}
}

static void fusion_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		int got = fifo_consume(&imu_fifo, 8);
		if (got > 0) {
			uint32_t now_us = k_cyc_to_us_floor32(k_cycle_get_32());
			for (int i = 0; i < got; i++) {
				uint32_t prod_us = ts_pop(&imu_ts);
				if (prod_us != 0) {
					hist_add(&fusion_lat, now_us - prod_us);
				}
			}
			k_busy_wait(FUSION_WORK_US);
#ifdef CONFIG_WEAVER_SCHED
			weaver_set_buffer_fill_predictive_q16(
				&wd_fusion,
				depth_to_q16(atomic_get(&imu_fifo.depth)));
#endif
		}
		k_sleep(K_MSEC(5));
	}
}

static void hr_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		int got = fifo_consume(&ppg_fifo, 4);
		if (got > 0) {
			uint32_t now_us = k_cyc_to_us_floor32(k_cycle_get_32());
			for (int i = 0; i < got; i++) {
				uint32_t prod_us = ts_pop(&ppg_ts);
				if (prod_us != 0) {
					hist_add(&hr_lat, now_us - prod_us);
				}
			}
			k_busy_wait(HR_WORK_US);
#ifdef CONFIG_WEAVER_SCHED
			weaver_set_buffer_fill_predictive_q16(
				&wd_hr,
				depth_to_q16(atomic_get(&ppg_fifo.depth)));
#endif
		}
		k_sleep(K_MSEC(10));
	}
}

static void gatt_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		int n = atomic_get(&in_burst) ? 6 : 1;
		fifo_produce(&gatt_fifo, n);
#ifdef CONFIG_WEAVER_SCHED
		weaver_set_buffer_fill_predictive_q16(
			&wd_gatt, depth_to_q16(atomic_get(&gatt_fifo.depth)));
#endif
		k_busy_wait(GATT_WORK_US);
		fifo_consume(&gatt_fifo, 3);
#ifdef CONFIG_WEAVER_SCHED
		weaver_set_buffer_fill_predictive_q16(
			&wd_gatt, depth_to_q16(atomic_get(&gatt_fifo.depth)));
#endif
		k_sleep(K_MSEC(15));
	}
}

static void classifier_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		k_busy_wait(CLASS_WORK_US);
		k_sleep(K_MSEC(CLASS_PERIOD_MS));
	}
}

static void disp_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		k_busy_wait(DISP_WORK_US);
		k_sleep(K_MSEC(DISP_PERIOD_MS));
	}
}

static void nvm_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		k_busy_wait(NVM_WORK_US);
		k_sleep(K_MSEC(NVM_PERIOD_MS));
	}
}

/* ---- Stacks + thread structs ---- */

static K_THREAD_STACK_DEFINE(s_ble,    STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_imu,    STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_ppg,    STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_fusion, STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_hr,     STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_gatt,   STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_class,  STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_disp,   STACK_SIZE);
static K_THREAD_STACK_DEFINE(s_nvm,    STACK_SIZE);

static struct k_thread t_ble, t_imu, t_ppg, t_fusion, t_hr, t_gatt,
		       t_class, t_disp, t_nvm;

#ifdef CONFIG_WEAVER_SCHED
static void weaver_tick_fn(struct k_timer *t) { ARG_UNUSED(t); weaver_tick(); }
static K_TIMER_DEFINE(weaver_timer, weaver_tick_fn, NULL);
#endif

/* ---- Build label for the metrics line ---- */

#if !defined(CONFIG_WEAVER_SCHED)
#define BUILD_LABEL "stock"
#elif defined(CONFIG_WEAVER_BATCH_MVE)
#define BUILD_LABEL "weaver-mve"
#elif defined(CONFIG_WEAVER_BATCH_HYBRID)
#define BUILD_LABEL "weaver-hybrid"
#elif defined(CONFIG_WEAVER_BATCH_SCALAR)
#define BUILD_LABEL "weaver-scalar"
#else
#define BUILD_LABEL "weaver-?"
#endif

int main(void)
{
	printk("\n=== Weaver Benchmark (%s) on Apollo510B EVB LP @ 96 MHz ===\n",
	       BUILD_LABEL);
	printk("Workload: 3 Warp + 6 Weft, 60 s, bursts at 10s and 20s\n\n");

	k_thread_create(&t_ble,    s_ble,    STACK_SIZE, ble_fn,        NULL,NULL,NULL, BLE_PRIO,    0, K_NO_WAIT);
	k_thread_create(&t_imu,    s_imu,    STACK_SIZE, imu_fn,        NULL,NULL,NULL, IMU_PRIO,    0, K_NO_WAIT);
	k_thread_create(&t_ppg,    s_ppg,    STACK_SIZE, ppg_fn,        NULL,NULL,NULL, PPG_PRIO,    0, K_NO_WAIT);
	k_thread_create(&t_fusion, s_fusion, STACK_SIZE, fusion_fn,     NULL,NULL,NULL, FUSION_PRIO, 0, K_NO_WAIT);
	k_thread_create(&t_hr,     s_hr,     STACK_SIZE, hr_fn,         NULL,NULL,NULL, HR_PRIO,     0, K_NO_WAIT);
	k_thread_create(&t_gatt,   s_gatt,   STACK_SIZE, gatt_fn,       NULL,NULL,NULL, GATT_PRIO,   0, K_NO_WAIT);
	k_thread_create(&t_class,  s_class,  STACK_SIZE, classifier_fn, NULL,NULL,NULL, CLASS_PRIO,  0, K_NO_WAIT);
	k_thread_create(&t_disp,   s_disp,   STACK_SIZE, disp_fn,       NULL,NULL,NULL, DISP_PRIO,   0, K_NO_WAIT);
	k_thread_create(&t_nvm,    s_nvm,    STACK_SIZE, nvm_fn,        NULL,NULL,NULL, NVM_PRIO,    0, K_NO_WAIT);

#ifdef CONFIG_WEAVER_SCHED
	weaver_register(&wd_ble,    &t_ble,    WEAVER_TO_Q16(20), true);
	weaver_set_warp_deadline(&wd_ble, BLE_PERIOD_MS);

	weaver_register(&wd_imu,    &t_imu,    WEAVER_TO_Q16(15), true);
	weaver_set_warp_deadline(&wd_imu, IMU_PERIOD_MS);

	weaver_register(&wd_ppg,    &t_ppg,    WEAVER_TO_Q16(15), true);
	weaver_set_warp_deadline(&wd_ppg, PPG_PERIOD_MS);

	weaver_register(&wd_fusion, &t_fusion, WEAVER_TO_Q16(8), false);
	weaver_register(&wd_hr,     &t_hr,     WEAVER_TO_Q16(8), false);
	weaver_register(&wd_gatt,   &t_gatt,   WEAVER_TO_Q16(7), false);
	weaver_register(&wd_class,  &t_class,  WEAVER_TO_Q16(5), false);
	weaver_register(&wd_disp,   &t_disp,   WEAVER_TO_Q16(4), false);
	weaver_register(&wd_nvm,    &t_nvm,    WEAVER_TO_Q16(2), false);

	k_timer_start(&weaver_timer, K_MSEC(1), K_MSEC(1));
#endif

	uint32_t t0 = k_uptime_get_32();
	while (1) {
		uint32_t t = k_uptime_get_32() - t0;
		if (t >= DEMO_DURATION_MS) break;

		bool burst = ((t >= BURST_AT_MS_1 && t < BURST_AT_MS_1 + BURST_DURATION_MS) ||
			      (t >= BURST_AT_MS_2 && t < BURST_AT_MS_2 + BURST_DURATION_MS));
		atomic_set(&in_burst, burst ? 1 : 0);

		k_sleep(K_MSEC(50));
	}

	/* ---- Final report ---- */

	uint32_t imu_overruns  = atomic_get(&imu_fifo.overruns);
	uint32_t ppg_overruns  = atomic_get(&ppg_fifo.overruns);
	uint32_t gatt_overruns = atomic_get(&gatt_fifo.overruns);

	uint32_t fusion_p50 = hist_percentile(&fusion_lat, 50);
	uint32_t fusion_p99 = hist_percentile(&fusion_lat, 99);
	uint32_t hr_p50     = hist_percentile(&hr_lat, 50);
	uint32_t hr_p99     = hist_percentile(&hr_lat, 99);
	uint32_t fusion_max = fusion_lat.max_us;
	uint32_t hr_max     = hr_lat.max_us;

#ifdef CONFIG_WEAVER_SCHED
	struct weaver_stats s;
	weaver_get_stats(&s);
	uint32_t tick_cyc   = weaver_get_last_tick_cycles();
	uint32_t throttle   = s.throttle_events;
	uint32_t promotions = s.weft_promotions;
	uint32_t prewarp    = s.pre_warp_clears;
	uint32_t idle_skip  = s.skipped_idle_ticks;
	uint32_t total_ticks = s.total_ticks;
#else
	uint32_t tick_cyc = 0, throttle = 0, promotions = 0;
	uint32_t prewarp = 0, idle_skip = 0, total_ticks = 0;
#endif

	printk("\n--- METRICS [%s] ---\n", BUILD_LABEL);
	printk("M1 imu_overruns=%u ppg_overruns=%u gatt_overruns=%u\n",
	       imu_overruns, ppg_overruns, gatt_overruns);
	printk("M2 fusion_lat_us p50=%u p99=%u max=%u (n=%u)\n",
	       fusion_p50, fusion_p99, fusion_max, fusion_lat.count);
	printk("M2 hr_lat_us     p50=%u p99=%u max=%u (n=%u)\n",
	       hr_p50, hr_p99, hr_max, hr_lat.count);
	printk("M4 last_tick_cyc=%u\n", tick_cyc);
	printk("WV ticks=%u promo=%u prewarp=%u throttle=%u idle_skip=%u\n",
	       total_ticks, promotions, prewarp, throttle, idle_skip);
	printk("CSV %s,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
	       BUILD_LABEL, imu_overruns, ppg_overruns, gatt_overruns,
	       fusion_p50, fusion_p99, hr_p50, hr_p99,
	       tick_cyc, promotions, prewarp);
	printk("BENCHMARK COMPLETE\n");
	return 0;
}
