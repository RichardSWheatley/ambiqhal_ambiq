# Three Pressure-Batch Implementations

The dispatcher offers three implementations of the per-tick pressure
batch, selectable at build time. **All three produce bit-identical
numeric results** — verified with 10,000 random fuzz trials across
12 threads in `tests/kernel/weaver_paths_equiv/`. The choice is
purely a performance / FP-context-save tradeoff.

## The three paths

```
                                ┌─ Q16 multiply ─┐ ┌─ saturating add ─┐
SCALAR        scalar   load     │  scalar mul    │ │  scalar add+chk  │   never
                                └────────────────┘ └──────────────────┘  touches
                                                                          MVE

HYBRID        scalar   load     │  scalar mul    │ │  vqaddq_u32      │   touches
                                └────────────────┘ └──────────────────┘    MVE

FULL MVE      vld1q_u32         │  vmullbq+vshrnb│ │  vqaddq_u32      │   uses
                                └────────────────┘ └──────────────────┘  vector
                                                                       multiplies
```

| Path | Build flag | Q16 multiply | Sat. add | MVE state? |
|---|---|---|---|---|
| Scalar | `CONFIG_WEAVER_BATCH_SCALAR` (default) | scalar `(uint64_t)(a*b) >> 16` | scalar | No |
| Hybrid | `CONFIG_WEAVER_BATCH_HYBRID` | scalar (4× per batch) | `vqaddq_u32` | Yes |
| Full MVE | `CONFIG_WEAVER_BATCH_MVE` | `vmullbq_int_u32` + `vmulltq_int_u32` + `vshrnbq_n_u64` + `vshrntq_n_u64` | `vqaddq_u32` | Yes |

## Why offer all three?

To **measure** on real Apollo510 silicon what's actually expensive.
Subtracting cycle counts isolates each cost contribution:

- **Hybrid − Scalar** = cost of touching the MVE register file
  (vector loads, stores, saturating add) and arming the lazy FP
  context save mechanism.
- **Full MVE − Hybrid** = net benefit (or cost) of replacing four
  scalar Q16 multiplies with four MVE wide-multiply lanes.
- **Full MVE − Scalar** = total swing from going integer-only to
  full vectorization.

These numbers are workload-dependent (preemption rate, registered
thread count, cache state) and the answer on paper is at best
approximate. Build all three, log `weaver_get_last_tick_cycles()`
under your real workload, and pick the one that wins.

## Expected break-evens on Apollo510 LP @ 96 MHz

Rough back-of-envelope; **measure on hardware before trusting**:

| Threads | Scalar | Hybrid | Full MVE |
|---|---|---|---|
| 4 | ~50 cy | ~70 cy + FP save tax | ~90 cy + FP save tax |
| 12 (wearable) | ~180 cy | ~140 cy + FP save tax | ~110 cy + FP save tax |
| 32 | ~430 cy | ~250 cy + FP save tax | ~150 cy + FP save tax |
| 64 | ~830 cy | ~440 cy + FP save tax | ~250 cy + FP save tax |

"FP save tax" = ~17 cycles (FPU only) or ~50 cycles (MVE state) per
preemption that would not otherwise have happened. At 10 preemptions/sec
in a typical wearable workload that's ~170–500 cycles/sec extra.

Reading: scalar likely wins for the typical 12-thread wearable. Full
MVE likely wins clearly at 64 threads. Hybrid is a measurement tool
(and a fallback if the MVE intrinsics break for some compiler version).

## Verification

`tests/kernel/weaver_paths_equiv/` is a host-buildable test that
emulates the lane structure of all three paths in plain C and proves
they produce identical results. To run on host directly:

```sh
gcc -std=c11 -O2 -Wall \
    tests/kernel/weaver_paths_equiv/src/main.c \
    -o /tmp/equiv && /tmp/equiv
```

Expected output ends with `ALL THREE PATHS EQUIVALENT` after a 10000-
trial random fuzz.

The test would not have caught my earlier asymmetric-fuzzy bug
(different code path), but it does catch:

- swapped even/odd lane indices in `vmullbq_int_u32` vs `vmulltq_int_u32`
- wrong shift amount in `vshrnbq_n_u64` (e.g. 17 vs 16)
- missing Warp-fixup pass after batch
- empty-slot lanes leaking nonzero pressure
- saturation wrap on the addition (would diverge between paths if
  one used `vqaddq_u32` correctly and another did wrap-around `+`)

## How to switch paths

Edit your `prj.conf`:

```
# Scalar (default): integer ALU only
CONFIG_WEAVER_BATCH_SCALAR=y

# Hybrid: vector load/store/add, scalar Q16 multiply
CONFIG_WEAVER_BATCH_HYBRID=y

# Full MVE: 4-wide everything
CONFIG_WEAVER_BATCH_MVE=y
```

The chooser is mutually exclusive (Kconfig `choice` block). The
selected variant is fixed at compile time so there's zero runtime
dispatch overhead.

## A note on the FP-save tax

This deserves spelling out, since it's the reason the hybrid path is
not strictly worse than scalar despite doing strictly more work:

On Cortex-M55 with FPU/MVE, the first FP/MVE instruction in any
preemptible region triggers **lazy stacking** of the FP/MVE register
context. This is per-region (per CPU), not per-instruction. Once
armed, every subsequent preemption pays ~17 cycles (FPU only) or
~50 cycles (MVE state) to push/pop the FP context — even if the
preempted thread itself never used FP/MVE.

The **scheduler runs in a context that gets preempted constantly**
(every interrupt, every context switch). So adding even one MVE
instruction inside `weaver_tick()` arms the lazy stacking forever
(until the next FP context-clear). The cost is paid by every
preemption that follows.

If your *threads* already use FPU/MVE (common for sensor fusion, FFT,
graphics), the tax is already being paid and adding MVE to the
scheduler costs zero extra. In that case full MVE is essentially
free relative to scalar, and the cycle savings drop straight to the
bottom line.

Rule of thumb:

- **Wearable with no other FP/MVE in the system:** scalar.
- **Wearable with sensor fusion, FFT, or graphics on the same CPU:**
  hybrid or full MVE — measure to confirm.
- **>32 registered threads, any system:** full MVE.
- **Bring-up / debug / unsure:** scalar, then run the test to
  confirm equivalence before switching.
