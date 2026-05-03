/*
 * weaver_paths_equiv.c
 *
 * Host-side test that proves the scalar, hybrid, and full-MVE
 * pressure-batch implementations produce bit-identical results.
 *
 * MVE intrinsics aren't available on x86, so the hybrid and full-MVE
 * paths are emulated here using scalar code that mirrors the lane
 * structure of the real intrinsics. This catches lane-mapping errors
 * (e.g. swapped even/odd halves, wrong shift amount, missing Warp
 * fixup) that would only surface on real silicon otherwise.
 *
 * Build: gcc -std=c11 -O2 -Wall weaver_paths_equiv.c -o weaver_paths_equiv
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdbool.h>

#define WEAVER_Q16_SHIFT     16
#define WEAVER_Q16_ONE       (1U << WEAVER_Q16_SHIFT)
#define WEAVER_TO_Q16(x)     ((uint32_t)(x) << WEAVER_Q16_SHIFT)
#define WEAVER_Q16_MUL(a, b) (uint32_t)(((uint64_t)(a) * (uint64_t)(b)) >> WEAVER_Q16_SHIFT)
#define WEAVER_PRESSURE_WARP 0xFFFFFFFFU
#define WEAVER_W_URGENCY     0x00006666U
#define WEAVER_W_DENSITY     0x00006666U
#define WEAVER_W_AGING       0x00003333U
#define WEAVER_WAIT_TICKS_MAX (WEAVER_Q16_ONE - 1U)
#define MIN(a, b) ((a) < (b) ? (a) : (b))

struct weaver_thread_data {
	uint32_t priority_q16;
	uint32_t buffer_fill_q16;
	uint32_t wait_ticks;
	uint8_t  is_warp;
	uint8_t  in_use;
};

static uint32_t saturate_add(uint32_t a, uint32_t b)
{
	uint32_t s = a + b;
	return (s < a) ? UINT32_MAX : s;
}

static void gather_inputs(const struct weaver_thread_data *wd,
			  uint32_t *prio, uint32_t *fill, uint32_t *aged)
{
	if (wd == NULL || !wd->in_use || wd->is_warp) {
		*prio = 0; *fill = 0; *aged = 0;
		return;
	}
	*prio = wd->priority_q16;
	*fill = wd->buffer_fill_q16;
	uint32_t w = MIN(wd->wait_ticks, WEAVER_WAIT_TICKS_MAX);
	*aged = WEAVER_TO_Q16(w);
}

static void fixup(const struct weaver_thread_data * const snap[],
		  uint32_t pressures[], size_t base, size_t count)
{
	for (size_t j = 0; j < count; j++) {
		const struct weaver_thread_data *wd = snap[base + j];
		if (wd == NULL || !wd->in_use) {
			pressures[base + j] = 0;
		} else if (wd->is_warp) {
			pressures[base + j] = WEAVER_PRESSURE_WARP;
		}
	}
}

/* --- 1. SCALAR --- */
static uint32_t scalar_calc(const struct weaver_thread_data *t)
{
	if (t->is_warp) return WEAVER_PRESSURE_WARP;
	uint32_t pu = WEAVER_Q16_MUL(t->priority_q16, WEAVER_W_URGENCY);
	uint32_t pd = WEAVER_Q16_MUL(t->buffer_fill_q16, WEAVER_W_DENSITY);
	uint32_t w = MIN(t->wait_ticks, WEAVER_WAIT_TICKS_MAX);
	uint32_t pa = WEAVER_Q16_MUL(WEAVER_TO_Q16(w), WEAVER_W_AGING);
	return saturate_add(saturate_add(pu, pd), pa);
}

static void scalar_batch(const struct weaver_thread_data * const snap[],
			 uint32_t pressures[], size_t n)
{
	for (size_t i = 0; i < n; i++) {
		const struct weaver_thread_data *wd = snap[i];
		if (wd == NULL || !wd->in_use) {
			pressures[i] = 0;
		} else if (wd->is_warp) {
			pressures[i] = WEAVER_PRESSURE_WARP;
		} else {
			pressures[i] = scalar_calc(wd);
		}
	}
}

/* --- 2. HYBRID emulation --- */
/* Same lane structure as the on-target hybrid path: gather 4 inputs,
 * scalar Q16 mul per lane, then a (would-be vector) saturating add.
 */
static void hybrid_batch(const struct weaver_thread_data * const snap[],
			 uint32_t pressures[], size_t n)
{
	size_t i = 0;
	for (; i + 4 <= n; i += 4) {
		uint32_t prio[4], fill[4], aged[4];
		uint32_t pu[4], pd[4], pa[4];

		for (size_t j = 0; j < 4; j++) {
			gather_inputs(snap[i + j], &prio[j], &fill[j], &aged[j]);
		}
		for (size_t j = 0; j < 4; j++) {
			pu[j] = WEAVER_Q16_MUL(prio[j], WEAVER_W_URGENCY);
			pd[j] = WEAVER_Q16_MUL(fill[j], WEAVER_W_DENSITY);
			pa[j] = WEAVER_Q16_MUL(aged[j], WEAVER_W_AGING);
		}
		for (size_t j = 0; j < 4; j++) {
			pressures[i + j] = saturate_add(saturate_add(pu[j], pd[j]), pa[j]);
		}
		fixup(snap, pressures, i, 4);
	}
	if (i < n) scalar_batch(&snap[i], &pressures[i], n - i);
}

/* --- 3. FULL MVE emulation --- */
/* Mirrors vmullbq_int_u32 / vmulltq_int_u32 / vshrnbq / vshrntq. */
static void mve_q16_mul_emu(const uint32_t a[4], const uint32_t b[4],
			    uint32_t out[4])
{
	uint64_t lo[2], hi[2];
	lo[0] = (uint64_t)a[0] * (uint64_t)b[0];
	lo[1] = (uint64_t)a[2] * (uint64_t)b[2];
	hi[0] = (uint64_t)a[1] * (uint64_t)b[1];
	hi[1] = (uint64_t)a[3] * (uint64_t)b[3];

	out[0] = (uint32_t)(lo[0] >> WEAVER_Q16_SHIFT);
	out[2] = (uint32_t)(lo[1] >> WEAVER_Q16_SHIFT);
	out[1] = (uint32_t)(hi[0] >> WEAVER_Q16_SHIFT);
	out[3] = (uint32_t)(hi[1] >> WEAVER_Q16_SHIFT);
}

static void mve_batch(const struct weaver_thread_data * const snap[],
		      uint32_t pressures[], size_t n)
{
	const uint32_t w_u4[4] = { WEAVER_W_URGENCY, WEAVER_W_URGENCY,
				   WEAVER_W_URGENCY, WEAVER_W_URGENCY };
	const uint32_t w_d4[4] = { WEAVER_W_DENSITY, WEAVER_W_DENSITY,
				   WEAVER_W_DENSITY, WEAVER_W_DENSITY };
	const uint32_t w_a4[4] = { WEAVER_W_AGING, WEAVER_W_AGING,
				   WEAVER_W_AGING, WEAVER_W_AGING };

	size_t i = 0;
	for (; i + 4 <= n; i += 4) {
		uint32_t prio[4], fill[4], aged[4];
		for (size_t j = 0; j < 4; j++) {
			gather_inputs(snap[i + j], &prio[j], &fill[j], &aged[j]);
		}

		uint32_t v_pu[4], v_pd[4], v_pa[4];
		mve_q16_mul_emu(prio, w_u4, v_pu);
		mve_q16_mul_emu(fill, w_d4, v_pd);
		mve_q16_mul_emu(aged, w_a4, v_pa);

		for (size_t j = 0; j < 4; j++) {
			pressures[i + j] = saturate_add(saturate_add(v_pu[j], v_pd[j]),
							v_pa[j]);
		}
		fixup(snap, pressures, i, 4);
	}
	if (i < n) scalar_batch(&snap[i], &pressures[i], n - i);
}

/* --- Test driver --- */
static struct weaver_thread_data make(uint32_t prio, uint32_t fill,
				      uint32_t wait, bool warp)
{
	return (struct weaver_thread_data){
		.priority_q16 = prio,
		.buffer_fill_q16 = fill,
		.wait_ticks = wait,
		.is_warp = warp ? 1 : 0,
		.in_use = 1,
	};
}

#define N 12

int main(void)
{
	struct weaver_thread_data wds[N] = {
		make(WEAVER_TO_Q16(15), 0, 0, true),                /* Warp */
		make(WEAVER_TO_Q16(15), 0, 0, true),                /* Warp */
		make(WEAVER_TO_Q16(8), WEAVER_Q16_ONE / 2, 5, false),
		make(WEAVER_TO_Q16(8), WEAVER_Q16_ONE / 4, 12, false),
		make(WEAVER_TO_Q16(7), WEAVER_Q16_ONE - 1, 0, false),
		make(WEAVER_TO_Q16(5), 0, 50, false),
		make(WEAVER_TO_Q16(4), WEAVER_Q16_ONE / 8, 1, false),
		make(WEAVER_TO_Q16(2), 0, 0, false),
		make(WEAVER_TO_Q16(1), WEAVER_Q16_ONE / 16, 100, false),
		make(WEAVER_TO_Q16(15), 0, 0, true),                /* Warp */
		make(WEAVER_TO_Q16(8), WEAVER_Q16_ONE / 3, 20, false),
		make(WEAVER_TO_Q16(8), WEAVER_Q16_ONE * 3 / 4, 8, false),
	};

	const struct weaver_thread_data *snap[N];
	for (int i = 0; i < N; i++) snap[i] = &wds[i];

	uint32_t p_scalar[N], p_hybrid[N], p_mve[N];
	scalar_batch(snap, p_scalar, N);
	hybrid_batch(snap, p_hybrid, N);
	mve_batch(snap, p_mve, N);

	printf("idx  scalar     hybrid     full-MVE   match\n");
	int mismatches = 0;
	for (int i = 0; i < N; i++) {
		bool ok = (p_scalar[i] == p_hybrid[i]) &&
			  (p_scalar[i] == p_mve[i]);
		if (!ok) mismatches++;
		printf("%2d  0x%08x 0x%08x 0x%08x  %s\n",
		       i, p_scalar[i], p_hybrid[i], p_mve[i],
		       ok ? "ok" : "MISMATCH");
	}

	/* Random fuzz: 10000 trials of 12 threads with random inputs. */
	srand(0x5eed);
	int fuzz_mismatches = 0;
	for (int trial = 0; trial < 10000; trial++) {
		struct weaver_thread_data fuzz[N];
		const struct weaver_thread_data *fsnap[N];
		for (int i = 0; i < N; i++) {
			fuzz[i].priority_q16 = (uint32_t)rand() & 0x000FFFFFU;
			fuzz[i].buffer_fill_q16 = (uint32_t)rand() & (WEAVER_Q16_ONE);
			fuzz[i].wait_ticks = (uint32_t)rand() & 0xFFFFU;
			fuzz[i].is_warp = ((uint32_t)rand() & 7U) == 0U;
			fuzz[i].in_use  = ((uint32_t)rand() & 31U) != 0U;
			fsnap[i] = &fuzz[i];
		}
		uint32_t s[N], h[N], m[N];
		scalar_batch(fsnap, s, N);
		hybrid_batch(fsnap, h, N);
		mve_batch(fsnap, m, N);
		for (int i = 0; i < N; i++) {
			if (s[i] != h[i] || s[i] != m[i]) {
				fuzz_mismatches++;
				if (fuzz_mismatches < 5) {
					printf("FUZZ trial %d slot %d: "
					       "scalar=0x%x hybrid=0x%x mve=0x%x\n",
					       trial, i, s[i], h[i], m[i]);
				}
			}
		}
	}
	printf("Fuzz: 10000 trials x %d threads, %d mismatches\n",
	       N, fuzz_mismatches);

	if (mismatches == 0 && fuzz_mismatches == 0) {
		puts("ALL THREE PATHS EQUIVALENT");
		return 0;
	}
	return 1;
}
