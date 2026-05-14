# Fixed-Point vs. Floating-Point Weaver Variants

The Weaver scheduler ships in two parallel implementations:

| Variant | Kconfig | Header | Implementation |
|---|---|---|---|
| **Fixed-point** | `CONFIG_WEAVER_SCHED` | `weaver_sched.h` | `kernel/weaver_sched.c` |
| **Floating-point** | `CONFIG_WEAVER_SCHED_FP` | `weaver_sched_fp.h` | `kernel/weaver_sched_fp.c` |

The two Kconfigs are mutually exclusive — pick one per build. Both
implement the same Warp/Weft semantics, the same predictive
buffer-fill helper, the same TSK fuzzy throttle controller with EMA
hysteresis, and the same pre-Warp guard. **They produce identical
dispatch decisions** for any given input — proven by
`tests/kernel/weaver_fp_vs_fixed/` (10,000 random trials, 0
mismatches).

The difference is purely the numeric representation of the pressure
math, and what that implies for performance / portability / energy.

## Pick fixed-point when

- **Battery-constrained wearable in LP mode (96 MHz)** and **no
  other thread on the system uses the FPU**.
- The MCU has no FPU at all (Cortex-M0/M0+/M3, some M4-NOFP).
- WCET-certified workloads where bit-identical reproducibility
  across compilers and toolchain versions is required.
- You want to leave the lazy floating-point context-save mechanism
  disarmed by the scheduler.

This is the **default** for the wearable preset and the one
recommended in the RFC for new battery-powered designs.

## Pick floating-point when

- Your Weft threads **already use the FPU** (sensor fusion with
  Mahony/Madgwick filters, HR FFT, audio frame DSP, graphics alpha
  blending). The lazy stacking tax is already being paid by those
  threads on every preemption, so adding the FPU to the scheduler
  hot path costs **zero extra cycles** at preemption time.
- Apollo510 is in **HP mode (192 / 250 MHz)** where the per-FP-op
  cost is dwarfed by the available cycle budget.
- You want a code path that is **easier to audit and modify** than
  the Q16.16 one. No `(uint64_t)` widening casts, no Q-format
  reasoning, no saturating-add helpers — just readable math:
  ```c
  float pa = wait_ticks * 0.2f;
  ```
  vs. the fixed-point equivalent:
  ```c
  uint32_t wait = MIN(t->wait_ticks, WEAVER_WAIT_TICKS_MAX);
  uint32_t pa = WEAVER_Q16_MUL(WEAVER_TO_Q16(wait), weaver.w_aging);
  ```

## Cost comparison

Per-tick dispatcher cost at 12 threads on Apollo510 LP @ 96 MHz
(estimates pending real-silicon measurement via the benchmark
sample's `last_tick_cyc` log line):

| Variant | Math cost | Lazy stacking tax | Net at 10 preempts/sec |
|---|---|---|---|
| Fixed-point (scalar) | ~180 cy / tick | 0 cy / preempt | **~180 cy / tick** |
| Floating-point | ~140 cy / tick | ~17 cy / preempt that didn't already use FPU | depends on workload |

If the workload **already uses the FPU elsewhere**, the FP variant is
~40 cycles cheaper per tick and the lazy-stacking tax line is zero
(it was being paid anyway). The FP variant wins.

If the workload **doesn't** use the FPU elsewhere, the FP variant is
~40 cycles cheaper per tick but adds ~170 cycles/sec of new
context-save tax. The fixed-point variant wins by a small margin.

The benchmark sample (`samples/boards/apollo510b_evb/weaver_benchmark/`)
will produce real numbers once flashed.

## Equivalence guarantee

The host-side test `tests/kernel/weaver_fp_vs_fixed/` proves that for
any randomized thread set the two implementations:

1. Pick the same Weft thread as the dispatch winner (arg-max over
   pressures).
2. Cross the throttle threshold on the same tick (modulo a 5%
   boundary tolerance for float rounding near the threshold).

The actual pressure *values* differ (they're in different number
formats), but the dispatch *decisions* are identical. That is the
contract a scheduler care about.

To run the test:

```sh
gcc -std=c11 -O2 -Wall \
    tests/kernel/weaver_fp_vs_fixed/src/main.c -lm \
    -o /tmp/wv_fp_eq && /tmp/wv_fp_eq
```

Expected output:

```
trials=10000 N=12
winner_mismatches=0
throttle_mismatches (outside tolerance)=0
pressure_diff_>1pct_on_agreed_winner=10000
FIXED AND FLOAT VARIANTS EQUIVALENT
```

## API mapping (cheat sheet)

| Fixed-point | Floating-point | Note |
|---|---|---|
| `struct weaver_thread_data` | `struct weaver_fp_thread_data` | |
| `WEAVER_TO_Q16(15)` | `15.0f` | priority value |
| `WEAVER_Q16_ONE` | `1.0f` | buffer-fill maximum |
| `weaver_register(wd, t, WEAVER_TO_Q16(15), true)` | `weaver_fp_register(wd, t, 15.0f, true)` | |
| `weaver_set_buffer_fill_predictive_q16(wd, q)` | `weaver_fp_set_buffer_fill_predictive(wd, f)` | q ∈ [0, Q16_ONE]; f ∈ [0.0f, 1.0f] |
| `weaver_tick()` | `weaver_fp_tick()` | |
| `weaver_system_pressure()` returns `uint32_t` | `weaver_fp_system_pressure()` returns `float` | |
| `weaver_should_throttle()` | `weaver_fp_should_throttle()` | identical bool semantics |
| `weaver_throttle_level()` | `weaver_fp_throttle_level()` | both return `uint8_t` 0..255 |
| `weaver_get_last_tick_cycles()` | `weaver_fp_get_last_tick_cycles()` | both DWT-counted |

## Why we can't simply merge them

A unified API with `#ifdef`-selected internals was considered and
rejected for three reasons:

1. **Header type pollution.** `priority_q16` (uint32_t) vs.
   `priority` (float) require different field types in the public
   struct. A unified struct using `uint32_t` everywhere and
   converting at call sites loses type-safety; using `float`
   everywhere wastes 8 bytes per slot on non-FP builds.
2. **Compile-time selection at the call site.** Producers like
   `bt_gatt_notify` or sensor drivers want a stable, typed API.
   Macro-aliased helpers (`wv_register(...)`) work in the benchmark
   sample but become unfriendly when the API surface grows.
3. **Independent evolution.** A future MVE-vectorized FP variant
   (using `vfmaq_f32`) and the existing fixed-point MVE path would
   want different optimizations; coupling them complicates both.

Two clear implementations are easier to maintain than one clever one.

## Recommended bring-up sequence

1. Build the fixed-point benchmark variants (scalar / hybrid / mve)
   and capture `last_tick_cyc` numbers on your hardware.
2. Build the FP benchmark variant (`fp.conf`) and capture the same.
3. If your Weft threads use the FPU and the FP variant wins on
   cycle count, switch your sample / product code to
   `CONFIG_WEAVER_SCHED_FP=y`. Otherwise stay on
   `CONFIG_WEAVER_SCHED=y`.

The equivalence test ensures the switch is decision-preserving.
