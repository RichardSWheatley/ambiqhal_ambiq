# Weaver Phase 3 — NPU "Loom" Layer (Design Sketch)

**Status:** DRAFT. Header-only signpost (`include/zephyr/kernel/weaver_sched_npu.h`).
No implementation file exists yet. This document explains the intent so
reviewers — and a future implementer — see the trajectory without having
to reverse-engineer it from the header.

## Why now

Apollo510 SoC variants pair Cortex-M55 with the Ethos-U55 NPU. The
fixed-point Weaver scheduler we built in Phases 1–2 covers CPU threads,
but it has no concept of NPU offload. Real wearable workloads with
keyword spotting, activity classification, AFib detection, and fall
detection will be NPU-driven within a generation. The architecture
needs to extend before customers ask "does Weaver schedule the NPU?"

The good news: the existing Q16.16 dispatcher math, the snapshot
pattern, the predictive helper, and the fuzzy throttle controller all
extend cleanly. **The hot-path code doesn't need to change**; we add
a parallel arbiter that runs once per `weaver_tick()` after the Weft
tournament finishes.

## The Loom class

Today: Warp (hard real-time CPU) + Weft (opportunistic CPU).
Phase 3 adds: **Loom** (asynchronous NPU coprocessor).

Why "Loom"? The loom is the machine that does the actual weaving;
threads just feed it. Naming the NPU layer this way keeps the
metaphor consistent — and gives Ambiq's marketing a coherent story.

| Class | What it is | Scheduling primitive |
|---|---|---|
| Warp | CPU thread, deadline-bound | priority preemption |
| Weft | CPU thread, opportunistic | priority elevation each tick |
| Loom | NPU graph, run-to-completion | dispatch-queue ordering |

A wearable typically has 3–6 Looms (one per ML graph) sharing one NPU.

## Pressure formula for Looms

Same three-term Q16.16 structure as Weft, different inputs:

```
loom_pressure = input_fill_q16 * 0.35       (35%)
              + output_backlog_q16 * 0.25   (25%)
              + TO_Q16(wait_ticks) * 0.40   (40%)
```

Aging gets a larger weight than for threads because a 1-second-old
NPU result is functionally useless — the audio frame or sensor window
it inferred over has moved on. There's no "urgency" term because the
Loom's static rank is encoded in a separate `priority` field used to
break ties.

## What changes in `weaver_tick()`

Add a third pass after the existing two:

```
Pass 1 (existing): aging + Warp deadline countdown
Pass 2 (existing): Weft pressure batch + winner selection
Pass 3 (existing): Weft priority elevation
Pass 4 (NEW):      Loom arbitration via weaver_loom_arbitrate()
```

`weaver_loom_arbitrate()` is the new function. It:

1. Returns early if the NPU is busy (one inference at a time).
2. Computes pressure for each registered Loom.
3. Picks the max-pressure Loom with non-empty input.
4. Checks the pre-Warp guard: if any Warp deadline lands within
   `avg_inference_us` of now, defers (so a 25 ms inference doesn't
   stall a 2 ms audio deadline).
5. Checks the energy budget. If insufficient, defers and counts a
   `deferrals_in_epoch`.
6. Calls the platform NPU driver's `dispatch()` hook.

Total added cost per tick: ~50 cycles at typical N=4 Looms. About
the same overhead profile as the Weft batch.

## Throttle vector

Today: one scalar `weaver_throttle_level()` (0–255). Phase 3 splits
this into three independent fuzzy controllers:

| Throttle | Producers should… | Driven by |
|---|---|---|
| `weaver_throttle_level()` (existing) | slow down CPU producers | sum of Weft pressures |
| `weaver_loom_throttle_level()` (new) | reduce NPU dispatch rate | sum of Loom input+backlog |
| `weaver_io_throttle_level()` (future) | pause DMA when NPU busy | NPU-vs-DMA bus contention |

Each is its own TSK fuzzy with EMA hysteresis. Same near-zero cost.
Producers consuming the right one for their domain don't fight each
other.

## Energy budget

The NPU is the dominant power consumer on a wearable when active.
Phase 3 adds an explicit per-epoch budget:

```c
struct weaver_loom_budget {
    uint32_t epoch_ms;            /* refresh window */
    int32_t  remaining_uj;        /* energy left this epoch */
    uint32_t inferences_in_epoch;
    uint32_t deferrals_in_epoch;
};
```

A 1 Hz background thread (or the PM layer) calls
`weaver_loom_refresh_budget(epoch_uj)` at the start of each epoch.
The arbiter withdraws `expected_energy_uj` from the budget on each
dispatch. When `remaining_uj` drops below what the next graph needs,
the arbiter defers — producers see this via `weaver_loom_throttle_level()`
rising and can downgrade gracefully (e.g., KWS-Lite instead of KWS-Full).

This is a feature stock Zephyr cannot replicate without per-product
glue code.

## Predictive component

Same one-step linear extrapolation as the Weft side. Producers call
`weaver_loom_push_input(ld, current_fill)`. Internally the layer
projects `current + (current - last)` to anticipate fill one tick
ahead. Six cycles of integer math.

For Looms there's a second predictive opportunity: `avg_inference_us`
is an EMA of measured latencies. If the EMA drifts upward, the
arbiter knows the next inference will likely take longer and can
adjust the pre-Warp guard check accordingly. Free latency variance
detection.

## Driver portability

`weaver_loom_set_driver_ops()` lets the platform supply three function
pointers (`dispatch`, `abort`, `get_remaining_us`). The Loom layer is
NPU-agnostic. Today: Ethos-U55. Tomorrow: Cadence Tensilica HiFi, or
a custom Ambiq accelerator. The dispatcher math doesn't care.

## Memory footprint estimate

Per Loom: ~48 bytes. Default `CONFIG_WEAVER_MAX_LOOMS = 6` → 288 bytes
of slot data + ~40 bytes static state. Total Phase 3 RAM cost: ~330
bytes, compared to ~210 bytes for the Phase 1/2 Weft layer.

Code: estimated ~1.2 KB of new `.c` once implemented. Roughly the
size of the existing scheduler. Total Weaver binary footprint after
Phase 3: ~3 KB.

## Compatibility

- `CONFIG_WEAVER_NPU=n` (default once it exists) builds a no-op layer.
  Existing Weft scheduling is unaffected.
- `CONFIG_WEAVER_NPU=y` enables the Loom arbiter and adds ~50 cycles
  per `weaver_tick()`.
- The same wearable preset that today sets `WEAVER_MAX_THREADS=12`
  would also default `WEAVER_MAX_LOOMS=6` if NPU is enabled.

## What I deliberately did NOT include in the header

- A multi-NPU registry. Apollo510 has one Ethos-U55. If Apollo520
  ships with two, the Loom queue becomes per-NPU and load balancing
  is needed. Out of scope until the silicon exists.
- A graph quantization estimator. Some inputs cause longer inference
  than others (e.g., noisy audio in KWS). Tracking per-graph latency
  variance is interesting but adds state per Loom. Deferred.
- Cache-warmth tracking. NPU often runs from its own tightly-coupled
  SRAM; the question of "should we pre-fetch the next graph's weights
  during the current inference?" is a memory-system optimization, not
  a scheduling decision. Belongs in the platform NPU driver, not in
  Weaver.

## What this is good for *today*

This file plus the header don't ship code. What they ship is **a
coherent story for the Ambiq pitch**: yes, Weaver was designed to
extend to NPU silicon; here's exactly how the API will look; here's
why the hot-path math doesn't change; here's the energy-budget hook
that no off-the-shelf RTOS provides.

A Phase 3 implementer can start from this header and the prose above
without re-litigating the design.
