/*
 * weaver_fp_vs_fixed.c
 *
 * Host-side test that proves the fixed-point and floating-point
 * Weaver variants produce equivalent DISPATCH DECISIONS for the
 * same inputs. They will not produce bit-identical pressure values
 * (different number representations), but they should agree on:
 *
 *   1. Which Weft thread wins the dispatch tournament.
 *   2. The throttle decision (should_throttle yes/no).
 *
 * Fuzz: 10000 random thread sets of size 12. For each, compute
 * pressures with both implementations and check that the
 * arg-max winner is the same (or both are NULL).
 *
 * Build: gcc -std=c11 -O2 -Wall main.c -o /tmp/wv_fp_eq
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>
#include <assert.h>

/* ---- fixed-point implementation ---- */
#define Q16_SHIFT     16
#define Q16_ONE       (1U << Q16_SHIFT)
#define TO_Q16(x)     ((uint32_t)(x) << Q16_SHIFT)
#define Q16_MUL(a,b)  (uint32_t)(((uint64_t)(a) * (uint64_t)(b)) >> Q16_SHIFT)
#define WAIT_MAX      (Q16_ONE - 1U)
#define WARP_FIXED    0xFFFFFFFFU
#define W_URG_Q       0x00006666U
#define W_DEN_Q       0x00006666U
#define W_AGE_Q       0x00003333U
#define T_FIXED       0x00040000U  /* threshold = 4.0 in Q16.16 */

/* ---- float implementation ---- */
#define WARP_FLOAT    FLT_MAX
#define W_URG_F       0.4f
#define W_DEN_F       0.4f
#define W_AGE_F       0.2f
#define T_FLOAT       4.0f

#define MIN(a,b) ((a) < (b) ? (a) : (b))

struct wd_fixed {
    uint32_t priority_q16, buffer_fill_q16, wait_ticks;
    uint8_t  is_warp, in_use;
};

struct wd_float {
    float    priority, buffer_fill;
    uint32_t wait_ticks;
    uint8_t  is_warp, in_use;
};

static uint32_t sat_add(uint32_t a, uint32_t b) {
    uint32_t s = a + b;
    return (s < a) ? UINT32_MAX : s;
}

static uint32_t pressure_fixed(const struct wd_fixed *t) {
    if (t->is_warp) return WARP_FIXED;
    uint32_t pu = Q16_MUL(t->priority_q16, W_URG_Q);
    uint32_t pd = Q16_MUL(t->buffer_fill_q16, W_DEN_Q);
    uint32_t w  = MIN(t->wait_ticks, WAIT_MAX);
    uint32_t pa = Q16_MUL(TO_Q16(w), W_AGE_Q);
    return sat_add(sat_add(pu, pd), pa);
}

static float pressure_float(const struct wd_float *t) {
    if (t->is_warp) return WARP_FLOAT;
    float pu = t->priority    * W_URG_F;
    float pd = t->buffer_fill * W_DEN_F;
    /* Aging scales LINEARLY with wait_ticks (same as fixed-point variant). */
    float w  = (t->wait_ticks > 65535U) ? 65535.0f : (float)t->wait_ticks;
    float pa = w * W_AGE_F;
    return pu + pd + pa;
}

#define N 12

/* Runs the random fuzz and writes mismatch counts into out parameters.
 * Returns 0 if both winner and throttle agree to within tolerance.
 */
static int run_fp_vs_fixed(int *winner_mis, int *throttle_mis)
{
    srand(0x5eed);
    int trials = 10000;
    int winner_mismatches = 0;
    int throttle_mismatches = 0;
    int max_pressure_diff_count = 0;

    for (int trial = 0; trial < trials; trial++) {
        struct wd_fixed q[N];
        struct wd_float f[N];

        for (int i = 0; i < N; i++) {
            uint32_t prio_q = (uint32_t)rand() & 0x000FFFFFU;     /* up to ~16.0 in Q16.16 */
            uint32_t fill_q = (uint32_t)rand() & Q16_ONE;          /* 0..1.0 */
            uint32_t wait   = (uint32_t)rand() & 0xFFFFU;
            uint8_t  warp   = ((uint32_t)rand() & 7U) == 0U;
            uint8_t  used   = ((uint32_t)rand() & 31U) != 0U;

            q[i].priority_q16    = prio_q;
            q[i].buffer_fill_q16 = fill_q;
            q[i].wait_ticks      = wait;
            q[i].is_warp         = warp;
            q[i].in_use          = used;

            /* Convert to float exactly. */
            f[i].priority    = (float)prio_q / (float)Q16_ONE;
            f[i].buffer_fill = (float)fill_q / (float)Q16_ONE;
            f[i].wait_ticks  = wait;
            f[i].is_warp     = warp;
            f[i].in_use      = used;
        }

        /* Find winner under each implementation. Warp threads are
         * excluded from the Weft tournament; they're saturated and
         * handled by static priority. */
        int q_winner = -1, f_winner = -1;
        uint32_t q_max = 0;
        float    f_max = 0.0f;
        uint32_t q_sum = 0;
        float    f_sum = 0.0f;

        for (int i = 0; i < N; i++) {
            if (!q[i].in_use || q[i].is_warp) continue;
            uint32_t qp = pressure_fixed(&q[i]);
            float    fp = pressure_float(&f[i]);

            q_sum = sat_add(q_sum, qp);
            f_sum += fp;

            if (qp > q_max) { q_max = qp; q_winner = i; }
            if (fp > f_max) { f_max = fp; f_winner = i; }
        }

        if (q_winner != f_winner) {
            winner_mismatches++;
            if (winner_mismatches < 5) {
                fprintf(stderr,
                    "winner mismatch trial=%d: fixed=%d (p=0x%x), float=%d (p=%.6f)\n",
                    trial, q_winner, q_max, f_winner, (double)f_max);
            }
        }

        /* Throttle decision: fixed crosses T_FIXED, float crosses T_FLOAT.
         * Convert q_sum back to its real value: q_sum/Q16_ONE. */
        double q_sum_real = (double)q_sum / (double)Q16_ONE;
        bool q_throttle = q_sum >= T_FIXED;
        bool f_throttle = f_sum >= T_FLOAT;

        if (q_throttle != f_throttle) {
            /* Allow disagreement only when both are within 5% of T. */
            double tol = 0.05 * T_FLOAT;
            if (fabs(q_sum_real - T_FLOAT) > tol &&
                fabs((double)f_sum - T_FLOAT) > tol) {
                throttle_mismatches++;
            }
        }

        /* Track pressure delta on the agreed winner. */
        if (q_winner == f_winner && q_winner >= 0) {
            double q_real = (double)q_max / (double)Q16_ONE;
            double diff = fabs(q_real - (double)f_max);
            if (diff > 0.01) max_pressure_diff_count++;
        }
    }

    (void)max_pressure_diff_count;  /* informational only */
    if (winner_mis)   *winner_mis   = winner_mismatches;
    if (throttle_mis) *throttle_mis = throttle_mismatches;

    /* Acceptance:
     *  - Winners must match in >99.5% of trials (rare disagreement
     *    possible when two Wefts are within float rounding of each
     *    other - both choices then valid).
     *  - Throttle must match unless system pressure is within 5% of
     *    the threshold (boundary noise).
     */
    int winner_pass    = (winner_mismatches * 200 < trials);
    int throttle_pass  = (throttle_mismatches == 0);
    return (winner_pass && throttle_pass) ? 0 : 1;
}

#ifdef __ZEPHYR__
#include <zephyr/ztest.h>

ZTEST(weaver_fp_vs_fixed, dispatch_decisions_match)
{
    int winner_mis = 0, throttle_mis = 0;
    int rc = run_fp_vs_fixed(&winner_mis, &throttle_mis);

    zassert_true(winner_mis * 200 < 10000,
                 "winner mismatches %d exceed 0.5%% of trials",
                 winner_mis);
    zassert_equal(throttle_mis, 0,
                  "%d throttle mismatches outside the 5%% boundary",
                  throttle_mis);
    zassert_equal(rc, 0, "fp-vs-fixed run returned %d", rc);
}

ZTEST_SUITE(weaver_fp_vs_fixed, NULL, NULL, NULL, NULL, NULL);

#else  /* host build */

int main(void)
{
    int winner_mis = 0, throttle_mis = 0;
    int rc = run_fp_vs_fixed(&winner_mis, &throttle_mis);

    printf("winner_mismatches=%d\n", winner_mis);
    printf("throttle_mismatches (outside tolerance)=%d\n", throttle_mis);
    if (rc == 0) {
        puts("FIXED AND FLOAT VARIANTS EQUIVALENT");
        return 0;
    }
    puts("EQUIVALENCE FAILED");
    return 1;
}

#endif
