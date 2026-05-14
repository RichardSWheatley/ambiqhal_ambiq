# Handoff to a Fresh Claude Session

This document brings a new Claude Code session up to speed on the
Weaver scheduler project. **Read this file first.** It links to
every other doc you need.

## Origin

- **Idea:** Richard S. Wheatley. The "Weaver" concept — Warp/Weft
  predictive pressure-aware scheduling — originated as a human task-
  scheduling metaphor and was adapted to RTOS thread scheduling on
  the Ambiq Apollo510.
- **Prior implementation work:** Claude (Anthropic) over a single
  multi-turn session.
- **Originating session:** https://claude.ai/code/session_01DPJEf1ccrLMANdqLEtQSaS

## TL;DR

We built a Zephyr scheduling **layer** (not a fork of
`kernel/sched.c`) for Apollo510 Blue EVB at 96 MHz LP mode running a
wearable sensor pipeline. It classifies threads as Warp (hard
real-time) or Weft (opportunistic), computes a Q16.16 pressure
score per Weft each tick, and boosts the highest-pressure Weft via
`k_thread_priority_set()`. Three pressure-batch implementations
(scalar / hybrid-MVE / full-MVE) and a parallel floating-point
variant ship together. A peer-review RFC with seven measurable
acceptance criteria, a canonical benchmark, and host-side
equivalence tests are all in the tree.

## Where the code lives

- **Repo:** https://github.com/RichardSWheatley/ambiqhal_ambiq
- **Branch:** `claude/fixed-point-scheduler-zephyr-RMdbe`
- **Top-level directory:** `weaver_scheduler/`
- **Patch for the actual Zephyr tree:**
  `weaver_scheduler/0001-weaver-scheduler.patch`
  Applies cleanly to `https://github.com/RichardSWheatley/ambiqzephyr`
  at `ambiq-stable`.

## Constraint that drove most layout decisions

The harness signing service was authorized for `ambiqhal_ambiq` but
**not** for `ambiqzephyr`. Direct commits to `ambiqzephyr` were
rejected with 502. Workaround: stage everything inside
`ambiqhal_ambiq/weaver_scheduler/` as a tree-of-files plus a patch.
A future session with elevated access can apply the patch directly
to `ambiqzephyr`.

If your session has access to `ambiqzephyr`, do not regenerate the
patch from scratch — apply it, commit, push.

## Commit ledger (10 commits, latest first)

| Commit  | Subject                                                       |
|---------|---------------------------------------------------------------|
| cb7585c | add Claude Code session URL to README and RFC                 |
| 941b787 | add floating-point variant parallel to fixed-point            |
| ec80747 | add RFC peer-review doc + canonical benchmark sample          |
| f1ffb6a | split pressure batch into three selectable paths              |
| b21ab74 | correct FPU rationale + add MVE and DWT timing hooks          |
| 5a2b901 | add TSK fuzzy throttle controller with EMA hysteresis         |
| a01bb0c | target apollo510b_evb with real Click-board sensors           |
| 84eef75 | tune for Apollo510 LP @ 96 MHz wearable workload              |
| 795ae03 | initial Weaver pressure-aware scheduler                       |

## File map (inside `weaver_scheduler/`)

```
RFC.md                   peer-review design doc with M1-M7 measurables
DECISIONS.md             every wearable assumption with rationale
MVE_PATHS.md             scalar / hybrid / full-MVE comparison
FP_VS_FIXED.md           fixed-point vs float variant decision guide
FUZZY_THROTTLE.md        TSK fuzzy + EMA throttle controller design
ML_AND_FUZZY_LOGIC.md    why no ML in the dispatcher hot path
README.md                file index, attribution, apply instructions

include/zephyr/kernel/
    weaver_sched.h       fixed-point public API (Q16.16)
    weaver_sched_fp.h    floating-point public API (single-precision)

kernel/
    weaver_sched.c       fixed-point implementation
    weaver_sched_fp.c    floating-point implementation
    Kconfig.weaver       CONFIG_WEAVER_SCHED + batch choice + presets
    Kconfig.weaver_fp    CONFIG_WEAVER_SCHED_FP (mutually exclusive)

samples/
    kernel/weaver_sched/                            generic demo
    boards/apollo510_evb/weaver_wearable/           synthetic FIFOs
    boards/apollo510b_evb/weaver_wearable/          real BMI270+MAX30101
    boards/apollo510b_evb/weaver_wearable_fp/       FP variant of above
    boards/apollo510b_evb/weaver_benchmark/         canonical RFC §7.1
        stock.conf       baseline (no Weaver)
        hybrid.conf      hybrid MVE
        mve.conf         full MVE
        fp.conf          floating-point variant
        scripts/bench.sh build + flash + tabulate all 5 variants

tests/kernel/
    weaver_paths_equiv/   scalar = hybrid = full-MVE (bit-identical)
    weaver_fp_vs_fixed/   fixed = float (dispatch decisions identical)

0001-weaver-scheduler.patch   apply to ambiqzephyr at ambiq-stable
```

## Design at a glance

```
                                                Stock Zephyr
                                                priority queue
                                                       │
weaver_register(wd, thread, prio, is_warp)             │
weaver_set_buffer_fill_predictive_q16(wd, fill)        │
                                                       ▼
                                                ┌─────────────┐
                       weaver_tick()  ────────► │  next_up()  │
                       every 1 ms              └─────────────┘
                                                       │
                                                       ▼
                                              chosen thread runs
```

Weaver itself never patches `kernel/sched.c`. It maintains a
bounded registry (default 12 slots), computes Q16.16 pressure each
tick, and influences the existing scheduler via
`k_thread_priority_set()`. `weaver_unregister()` restores the base
priority — completely reversible.

## What's been validated

- Host-side unit tests for Q16.16 math (boundary, monotonicity,
  saturation) — passing.
- Host-side fuzzy throttle test (center value, quarter / three-
  quarter points, EMA settle time, step response) — passing.
- Host-side three-path equivalence test:
  10,000 random fuzz trials × 12 threads = 120,000 lane checks,
  scalar / hybrid / full-MVE all produce **bit-identical** results.
- Host-side fixed-vs-float equivalence test:
  10,000 random fuzz trials, **same dispatch decisions** modulo
  float rounding near the throttle threshold.

To re-run all four:

```sh
gcc -std=c11 -O2 -Wall tests/kernel/weaver_paths_equiv/src/main.c -o /tmp/equiv && /tmp/equiv | tail -3
gcc -std=c11 -O2 -Wall tests/kernel/weaver_fp_vs_fixed/src/main.c -lm -o /tmp/wv_fp_eq && /tmp/wv_fp_eq | tail -5
# Inline math tests live in the conversation history; the C in
# weaver_sched.c and weaver_sched_fp.c is the source of truth.
```

## What has NOT been validated (the gap)

- **No on-target measurement.** Everything is host-side. The
  `tick_cyc` numbers in the docs (~180 cy for scalar, ~50 for MVE)
  are back-of-envelope from M55 instruction counts, not from real
  silicon. The benchmark sample is ready to capture them; it just
  needs to be flashed onto an apollo510b_evb with the BMI270 and
  MAX30101 Click boards.
- **No BLE host integration.** GATT TX in the wearable sample uses
  a synthetic FIFO. Wire `bt_gatt_notify()` to
  `weaver_set_buffer_fill_predictive_q16()` when ready.
- **No real display rendering.** The display Weft thread sleeps
  instead of submitting LVGL frames. The `ap510_disp` shield works
  fine; just hadn't wired it up yet.
- **MVE intrinsic paths not exercised on hardware.** The host
  equivalence test proves the algorithmic structure; a real
  toolchain emitting MVE asm for an M55 still needs to be
  spot-checked.

## Priority next steps (ranked)

1. **Run the benchmark on real hardware.** Five variants, one CSV
   line per run, comparison table from `scripts/bench.sh`. Fills in
   the empty cells under RFC §7 (M1, M2, M4, M5, M7).
2. **Wire real BLE GATT TX** into the fabric so the burst events in
   the benchmark match what real wearable BLE looks like.
3. **Apply the patch to ambiqzephyr** and push. If the new session
   has signing-server authorization for `ambiqzephyr`, this is one
   command:
   ```sh
   cd ambiqzephyr
   git checkout -b weaver-integration ambiq-stable
   git apply ../ambiqhal_ambiq/weaver_scheduler/0001-weaver-scheduler.patch
   git add -A && git commit -m "kernel: add Weaver scheduler"
   git push -u origin weaver-integration
   ```
4. **Open questions Q1–Q7 in RFC.md** are listed for review. Each
   one is a tuning decision (boost magnitude, guard window depth,
   throttle threshold, EMA depth, max-thread size, MVE codegen
   quality, baseline test inclusion).

## Don't redo these (already decided)

- Q16.16 fixed-point in the dispatcher hot path is intentional —
  not because there's no FPU, but to keep lazy floating-point
  context save **disarmed** by the scheduler. See `DECISIONS.md`
  §1 and `MVE_PATHS.md` for the full lazy-stacking analysis.
- The TSK fuzzy throttle controller uses a **symmetric** MID region
  (T/2 to 3T/2) so `pressure == T` maps to `level == 127`. An
  earlier asymmetric version (T/2 to 2T) gave `level == 85` at T;
  fixed before commit. See `FUZZY_THROTTLE.md`.
- Float-aging scales **linearly** with `wait_ticks` (clamped at
  65535), not normalized to [0, 1]. Without this the fixed and FP
  variants disagree on dispatch. See the test history if tempted to
  "fix" it.
- The MVE batch uses `vmullbq_int_u32` + `vmulltq_int_u32` +
  `vshrnbq_n_u64` + `vshrntq_n_u64` (real 4-wide multiply), not the
  earlier scalar-mul-with-vector-add hybrid. The hybrid is kept
  separately as a measurement tool, not a fallback.

## Where to read for full context

If you only have time for one doc: **`weaver_scheduler/RFC.md`** —
problem statement, design, alternatives considered, M1–M7
measurables.

If you're picking up implementation: **`weaver_scheduler/DECISIONS.md`**
plus **`weaver_scheduler/MVE_PATHS.md`** plus **`weaver_scheduler/FP_VS_FIXED.md`**.

If you're tuning fuzzy / throttle behavior:
**`weaver_scheduler/FUZZY_THROTTLE.md`**.

If you're considering adding ML:
**`weaver_scheduler/ML_AND_FUZZY_LOGIC.md`** — and don't add it to
the hot path. Use `weaver_set_weights()` from a 1 Hz Weft thread.
