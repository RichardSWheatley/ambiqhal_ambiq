# Weaver Phase 3 — Asymmetric Cognitive Offload (Design Sketch)

**Status:** DRAFT. Header-only signpost (`include/zephyr/kernel/weaver_sched_npu.h`).
No implementation file exists yet. This document explains the intent so
reviewers — and a future implementer — see the trajectory without having
to reverse-engineer it from the header.

## The framing

Phase 3 introduces an **asymmetric operating-system architecture** in
which a primary general-purpose processor (Cortex-M55) offloads three
distinct classes of policy work to an independent on-die neural
coprocessor (e.g. Ethos-U55). The CPU's dispatch hot path stays
deterministic and integer-only; the NPU handles the slow, learnable
work that would otherwise cost the CPU precious nanoseconds.

The three roles are separated so an examiner, a reviewer, or a future
implementer can reason about each one independently:

| Role | What runs | Where | Cadence |
|---|---|---|---|
| **Policy Adjustor** | Reshapes Weft weights based on observed workload | NPU | 1–10 Hz |
| **Arbiter Feeder** | Orders queued NPU jobs (Looms) | CPU integer path | Per tick |
| **Anomaly Detector** | Predicts starvation / budget exhaustion | NPU | 10–50 Hz |

## Why now

Apollo510 SoC variants pair Cortex-M55 with the Ethos-U55 NPU. The
fixed-point Weaver scheduler we built in Phases 1–2 covers CPU threads,
but it has no concept of NPU offload. Real wearable workloads with
keyword spotting, activity classification, AFib detection, and fall
detection will be NPU-driven within a generation. The architecture
needs to extend before customers ask "does Weaver schedule the NPU?"

The good news: the existing Q16.16 dispatcher math, the snapshot
pattern, the predictive helper, and the fuzzy throttle controller all
extend cleanly. **The hot-path code doesn't need to change**; Phase 3
inserts three asynchronous NPU-hosted policy loops around the same
CPU dispatcher.

## The dispatch/policy split (why some work stays on the CPU)

The per-tick dispatch decision itself STAYS ON THE CPU integer ALU.
NPU inference latency is 5–50 ms; a 1 ms scheduler tick cannot wait
for it. The rule:

- **NPU handles POLICY** (slow, learnable): what weights should the
  dispatcher use? what jobs are about to starve? is the energy
  budget healthy?
- **CPU handles DISPATCH** (fast, deterministic): given the current
  weights and pressures, which thread runs next tick?

This split is what makes the architecture asymmetric: two silicon
blocks handling two classes of decision on different timescales,
coupled through a shared-memory mailbox rather than a synchronous
call path. That coupling — kernel-level telemetry ring, asynchronous
NPU consumption, weight write-back into the running dispatcher — is
the concrete hardware-software co-design invention.

## Role 1: Policy Adjustor

The NPU periodically runs a quantized policy model — most likely a
linear bandit or a very small reinforcement-learning kernel — over
recent telemetry, then writes three new Q16.16 weights into the
dispatcher via `weaver_set_weights()`. Cadence 100 ms–1 s. Not in
the CPU dispatch critical path.

The CPU keeps using the same pressure formula:

```
pressure = w_urgency * priority + w_density * fill + w_aging * wait
```

Only `w_*` values change. There is no new math on the CPU side; the
dispatcher can't tell whether the weights came from a static config
or from an NPU inference two ticks ago.

Model shape (draft): 8-input linear bandit with per-workload arms;
output is a Q16.16 triple; quantization INT8 with dequant at output.
Memory footprint on Ethos-U55 target: under 4 KB.

## Role 2: Arbiter Feeder

The classical NPU scheduling role: when the NPU goes idle, pick the
highest-pressure Loom (queued NPU inference job) and dispatch. Runs
**on the CPU** integer path, ~50 cycles per pass, called from inside
`weaver_tick()` as a fourth pass after the Weft tournament. Same
Q16.16 pressure math structure as Weft, three-term weighted sum
(input_fill × 0.35 + output_backlog × 0.25 + aging × 0.40).

Pre-Warp guard applies: if a Warp deadline is inside the NPU's
`avg_inference_us`, defer this dispatch. Otherwise start it.

## Role 3: Anomaly Detector

The NPU consumes the last ~64 telemetry frames, infers over cache
miss surrogate + context-switch delta + IOM stall bits + Weft
promotion rate, and emits a graded 0..255 starvation warning:

- 0 = no predicted starvation in the next window
- 128 = starvation likely within ~50 ms
- 255 = starvation predicted imminent (< 10 ms)

Producers consume `weaver_anomaly_level()` alongside the existing
CPU-side `weaver_throttle_level()`. Two signals, complementary: the
CPU throttle is reactive (pressure has already crossed a threshold);
the anomaly detector is predictive (pressure will cross in ~50 ms
based on the trajectory).

## Telemetry pipeline

Kernel-side hooks populate a shared no-cache SRAM ring at the end of
every `weaver_tick()`. The ring is exclusively producer-CPU /
consumer-NPU; no locking, no cache coherency work, just a monotonic
head/tail pattern read from the NPU side after each of its
inferences completes.

```
    struct weaver_telemetry_frame {
        uint64_t timestamp_cycles;
        uint32_t dispatcher_cycles;
        uint32_t context_switch_delta;
        uint32_t cache_miss_surrogate;   // DWT LSU misses
        uint32_t iom_stall_bits;
        uint32_t weft_promotions;
        uint32_t throttle_level;
        uint32_t loom_dispatched_this_epoch;
        uint32_t reserved[8];
    };
```

64-byte cache-line aligned. Ring depth 32-128 frames. On Cortex-M55,
capturing one frame is ~15 cycles (DWT reads + 8 stores). Zero
allocation, zero locking, zero cache maintenance.

## Kernel-space exclusive NPU ownership

Standard vendor NPU stacks (Arm Ethos-U driver, AMD XDNA, Intel NPU)
expose the accelerator UPWARD to user-space AI runtimes. That is the
wrong direction here — Weaver needs the NPU to be a **kernel-owned
resource** so:

- Policy updates land before the next dispatch tick observes them
- User-space AI apps cannot preempt or crowd out the scheduler
- The NPU's command queue and shared SRAM ring are single-writer
  from the kernel side; no arbitration overhead

Phase 3 requires a **kernel-space NPU driver** that claims exclusive
ownership of at least one spatial partition (or the entire single
instance) of the NPU. The Weaver Loom layer talks to the NPU through
the `weaver_loom_driver_ops` indirection so the driver can be
platform-specific (Ethos-U on Apollo510, something else on future
silicon) while the Weaver policy interface stays constant.

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
