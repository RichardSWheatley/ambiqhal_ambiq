# RFC: Weaver Predictive Pressure-Aware Scheduling Layer for Zephyr

| Field | Value |
|---|---|
| Status | Draft |
| Author | Weaver Scheduler Working Group |
| Target | Zephyr RTOS, Apollo510 LP @ 96 MHz wearable workloads |
| Date | 2026-05 |
| Branch | `claude/fixed-point-scheduler-zephyr-RMdbe` |

---

## 1. Abstract

This RFC proposes **Weaver**, an additive scheduling layer for Zephyr
that augments — but does not replace — the existing strict-priority
dispatcher with a predictive, pressure-aware, fixed-point pre-pass.
Weaver classifies threads as **Warp** (hard real-time, deterministic)
or **Weft** (opportunistic), computes a Q16.16 "informational
pressure" score for each Weft based on associated FIFO depth and
wait-time aging, and elevates the highest-pressure Weft thread's
Zephyr priority before the standard scheduler runs. The intended
result is reduced FIFO overrun rate, lower sensor-to-process tail
latency, and graceful degradation under burst load — all while
preserving Zephyr's existing scheduling guarantees for high-priority
threads.

The implementation is **non-invasive**: it does not patch
`kernel/sched.c`. Instead it maintains a bounded thread registry
(default 12 slots, deterministic per-tick cost) and influences the
existing scheduler via `k_thread_priority_set()`. The full layer is
~1500 lines including three optional ALU/MVE variants, fuzzy
throttle controller, and DWT cycle-counter instrumentation.

We commit to specific, falsifiable acceptance criteria in §7 and
provide a host-buildable equivalence test (`tests/kernel/weaver_paths_equiv/`)
that proves the three multiply-batch paths produce bit-identical
results across 120,000 random inputs.

---

## 2. Problem Statement

### 2.1 Wearable workload characteristics

A typical battery-powered wearable on Apollo510 runs:

- **Sub-millisecond hard deadlines**: BLE link-layer connection
  events (every 7.5–500 ms with sub-ms tolerance), accelerometer/
  gyroscope sampling (50–200 Hz), PPG sampling (25–100 Hz).
- **Variable-rate background work**: sensor fusion (consumes IMU
  FIFO; runtime depends on activity), HR algorithm (FFT on PPG ring),
  GATT TX queue drain, display refresh, NVM flush.
- **Burst behavior**: a movement event may simultaneously fill the
  IMU FIFO, trigger gesture classification, queue BLE notifications,
  and update the watch face — all within tens of milliseconds.

### 2.2 Where stock Zephyr leaves value on the table

Zephyr's scheduler is correct, well-tested, and small. It is not the
target of this RFC. However, for the wearable workload above, three
shortfalls emerge in practice:

1. **No buffer-fill awareness.** A consumer thread waiting on a
   filling IMU FIFO is no more urgent in the scheduler's view than
   one waiting on an empty one. Priority is static; the scheduler
   reacts to the FIFO only after the consumer wakes (typically via
   a semaphore or message queue), at which point it is already up
   to 1–2 sample periods behind.

2. **No deadline anticipation.** A 50 Hz IMU sampling thread that
   is *about* to run gets the same scheduling treatment as one that
   ran 10 ms ago. There is no native concept of "the next deadline
   is in 1 ms; quiet the system."

3. **No graceful degradation signal.** Producers cannot ask "is the
   system overloaded?" Their only choices are "run my full work" or
   "skip this iteration entirely." For wearables this often means
   either dropped sensor samples or delayed BLE transmissions.

### 2.3 Why a layer, not a fork

Zephyr is updated frequently. Wearable products on Apollo510 LP
cannot afford the maintenance cost of a forked scheduler. Any
solution must:

- Live above the existing scheduler, not inside it.
- Be opt-in via Kconfig.
- Restore stock behavior on `weaver_unregister()`.
- Survive Zephyr release rebases without touching `kernel/sched.c`.

---

## 3. Goals and Non-Goals

### Goals

- **G1.** Reduce FIFO overrun rate under burst load relative to
  stock Zephyr by at least **50%** in the canonical wearable
  benchmark (defined in §7.1).
- **G2.** Bound dispatcher overhead at <1% of CPU on Apollo510 LP
  @ 96 MHz with 12 registered threads at a 1 ms tick.
- **G3.** Preserve Warp thread deadline behavior — zero new
  deadline misses introduced by Weaver vs. stock.
- **G4.** Provide producer-side throttle signaling so non-essential
  work can self-throttle before overrun.
- **G5.** Integer-only dispatch path (no FPU/MVE context-save tax)
  by default, with optional MVE variants for systems where the FP
  context save is already amortized.
- **G6.** Bit-identical numeric results across all three
  pressure-batch implementations (proven by the host equivalence
  test).

### Non-Goals

- **N1.** Not a replacement for `kernel/sched.c`. Zephyr's
  priority-queue dispatcher remains the source of truth for
  thread selection.
- **N2.** Not SMP-aware in v1. Apollo510 is single-core M55.
- **N3.** Not a real-time analysis framework. Weaver makes no
  formal WCET claims beyond bounded snapshot-based dispatch.
- **N4.** Not a replacement for proper Bluetooth host integration.
  GATT TX backpressure is exposed via the same setter as any other
  buffer.
- **N5.** Not a substitute for sensor driver tuning. If your IMU
  is configured to interrupt at 1 kHz when you only need 50 Hz,
  Weaver will not save you.

---

## 4. Proposal

### 4.1 Two thread classes

- **Warp** threads: hard real-time, deterministic. Examples: BLE LL
  event handler, IMU sampling thread, PPG sampling thread. Pressure
  is saturated (`0xFFFFFFFF`); they always win against any Weft.
  Their Zephyr priority is never modified by Weaver.
- **Weft** threads: opportunistic. Examples: sensor fusion, HR
  algorithm, GATT TX queue, display, NVM. Pressure is computed each
  tick from a weighted Q16.16 sum of (urgency, buffer fill, wait-
  ticks aging). The highest-pressure Weft is promoted by N priority
  levels (default 2 for the wearable preset).

### 4.2 Pressure formula (Q16.16 fixed point)

```
pressure(weft) = saturate_add(
                   p_urgency,    // priority_q16  * w_urgency
                   p_density,    // buffer_fill   * w_density
                   p_aging )     // TO_Q16(wait)  * w_aging
```

Default weights: 0.4 / 0.4 / 0.2 (sum to 1.0 in Q16.16). Weights
are tunable at runtime via `weaver_set_weights()` so an outer-loop
controller can adapt them based on observed outcomes.

### 4.3 Predictive update

Producers call `weaver_set_buffer_fill_predictive_q16(wd, fill)`
which stores `fill + (fill - last_fill)`, saturated. The dispatcher
acts on the buffer's projected next-tick state, not its current
state. ~6 cycles per producer call. No ML in the hot path.

### 4.4 Pre-Warp guard ("Pattern Slicing")

When any Warp thread's deadline is within `CONFIG_WEAVER_PREWARP_GUARD_TICKS`
(default 1 tick), Weft promotion is suppressed for the current pass.
The deadline-bound thread sees a quiet system and warm cache.

### 4.5 Power-aware idle

When `CONFIG_WEAVER_POWER_AWARE=y` (default for wearable preset)
and no Weft has positive pressure, `weaver_tick()` returns without
priority churn. This avoids waking the scheduler purely to do
nothing — which would otherwise block deepsleep entry.

### 4.6 Graded fuzzy throttle

A 0-order TSK fuzzy controller produces a 0–255 throttle level
from system pressure with three rules and 4-tap EMA smoothing.
Producers can either consume the graded value
(`weaver_throttle_level()`) for graceful degradation or use the
binary `weaver_should_throttle()` for backwards-compatible
bang-bang behavior. The smoothing eliminates flap at the threshold
boundary.

---

## 5. Detailed Design

### 5.1 Public API

```c
struct weaver_thread_data {
    struct k_thread *thread;
    uint32_t priority_q16;
    uint32_t buffer_fill_q16;
    uint32_t last_fill_q16;
    uint32_t wait_ticks;
    uint32_t period_ticks;
    uint32_t next_deadline_ticks;
    void    *meta;
    uint8_t  is_warp;
    int8_t   base_prio;
    int8_t   weft_boost_prio;
    uint8_t  boost_levels;
    uint8_t  in_use;
};

int      weaver_register(struct weaver_thread_data *wd,
                         struct k_thread *thread,
                         uint32_t priority_q16, bool is_warp);
int      weaver_unregister(struct weaver_thread_data *wd);
void     weaver_set_buffer_fill_q16(struct weaver_thread_data *wd, uint32_t);
void     weaver_set_buffer_fill_predictive_q16(struct weaver_thread_data *wd, uint32_t);
void     weaver_set_warp_deadline(struct weaver_thread_data *wd, uint32_t period);
void     weaver_set_boost_levels(struct weaver_thread_data *wd, uint8_t levels);
void     weaver_set_weights(uint32_t w_u, uint32_t w_d, uint32_t w_a);
void     weaver_tick(void);

uint32_t weaver_calculate_pressure(const struct weaver_thread_data *wd);
uint32_t weaver_system_pressure(void);
uint32_t weaver_ticks_to_next_warp(void);
bool     weaver_should_throttle(void);
uint8_t  weaver_throttle_level(void);
uint32_t weaver_get_last_tick_cycles(void);
void     weaver_get_stats(struct weaver_stats *out);
```

### 5.2 Tick loop structure

`weaver_tick()` runs in three passes:

1. **Aging + deadline pass** (scalar). Increments `wait_ticks` for
   non-running threads, decrements `next_deadline_ticks` for Warp
   threads, finds the minimum upcoming Warp deadline.
2. **Pressure batch pass.** Computes per-thread pressure into a
   stack array using the configured implementation
   (`scalar` / `hybrid` / `full MVE`).
3. **Dispatch pass** (scalar). Aggregates `system_pressure`, picks
   the winning Weft, applies pre-Warp clear if needed, calls
   `k_thread_priority_set()` only when the priority actually changes.

### 5.3 Three pressure-batch implementations

See `MVE_PATHS.md` for the full design. Summary:

| Variant | Multiply | Touches MVE? | Best at |
|---|---|---|---|
| Scalar | scalar 64-bit | No | Default; <16 threads |
| Hybrid | scalar (4× per batch) | Yes | Measurement / fallback |
| Full MVE | `vmullbq_int_u32`+`vmulltq_int_u32`+`vshrnbq` | Yes | >32 threads, or when system already uses MVE |

The chooser is a Kconfig `choice` block. Selection is fixed at
compile time; zero runtime dispatch overhead.

### 5.4 Memory footprint

| Item | Size |
|---|---|
| `struct weaver_thread_data` (per thread) | 36 bytes |
| Static `weaver` state | ~80 bytes + (8 × N) for slot pointers |
| Code (scalar only) | ~1.2 KB |
| Code (all three batch paths) | ~2.0 KB |
| Stack at dispatch peak | `4 × CONFIG_WEAVER_MAX_THREADS` bytes (snapshot) + same for pressures |

For a 12-thread wearable: 432 bytes + 176 bytes static + 1.2 KB code
+ 96 bytes peak stack = **~1.9 KB total**.

### 5.5 Reentrancy and SMP

Single-CPU only in v1. The slot array is protected by a spinlock
held only during snapshot-copy (a few cycles). All math runs
lock-free on the snapshot. No re-entrant calls to `weaver_tick()`
are permitted; the timer driver is responsible for serializing.

---

## 6. Alternatives Considered

### 6.1 Patch `kernel/sched.c::next_up()` directly

**Rejected.** Highest-fidelity solution but creates a permanent
maintenance burden. Every Zephyr release would require a rebase.
Risk of breaking SMP, MetaIRQ, timeslicing, and CPU-mask
interactions. The non-invasive layer captures 99% of the value at
~5% of the integration risk.

### 6.2 Use Zephyr's existing meta-IRQ mechanism

**Rejected.** MetaIRQs are designed for ISR-context dispatch of
hard real-time work, not for adaptive priority adjustment of
preemptible threads. They also require a dedicated priority band,
limiting their use in our 16-priority space.

### 6.3 Custom run-queue replacement (sched_simple/sched_multiq)

**Rejected.** Touches the same core scheduler files as 6.1. Same
maintenance burden.

### 6.4 ML-based predictor (LSTM, transformer, decision tree)

**Rejected for the dispatcher hot path.** See `ML_AND_FUZZY_LOGIC.md`.
Determinism, memory, energy, and certifiability all favor the
linear pressure model. Hooks (`weaver_set_weights()`) are exposed
for an outer-loop controller that *could* be ML, running at 1 Hz
in a Weft thread.

### 6.5 Floating-point pressure math

**Rejected for the dispatcher hot path.** See `DECISIONS.md` §1.
The Apollo510 has both FPU and MVE. The cost is not "no FPU
available"; it is the per-preemption lazy floating-point
context-save tax (~17 cycles FPU, ~50 cycles MVE) imposed on every
preemption that follows the first FP instruction in the scheduler
context. At low thread counts, integer wins.

### 6.6 Userspace-only scheduling hint

**Rejected.** Zephyr supports cooperative scheduling and yielding,
but the producer-driven "pressure" concept needs a kernel-side
component to react each tick. A pure userspace solution would have
to poll, which defeats the power-saving goal.

---

## 7. Measurables and Acceptance Criteria

This is the section that distinguishes a proposal from an opinion.
Each metric has a defined measurement procedure, an acceptance
threshold, and a measurement context. **All measurements are run
on the canonical benchmark workload defined in §7.1.**

### 7.1 Canonical benchmark workload

Defined as `samples/boards/apollo510b_evb/weaver_benchmark/` (see
companion patch). Configures:

- BLE LL connection event every 50 ms (3 ms work window)
- IMU sampling at 50 Hz (20 ms period, 200 µs SPI burst)
- PPG sampling at 25 Hz (40 ms period, 600 µs I2C burst)
- Sensor fusion: 8-sample batch consume, 1.5 ms work
- HR algorithm: 4-sample batch consume, 8 ms work (FFT)
- GATT TX queue: 1 notification produced every 15 ms
- Display: 30 Hz refresh, 4 ms work
- NVM flush: every 1 s, 12 ms work

Burst event injected at t=10 s and again at t=20 s: 50 ms of
elevated IMU rate (200 Hz) plus a 30-notification GATT TX burst.
Run for 60 s. All measurements taken in the steady-state windows
(t=2..10 s, t=12..20 s, t=22..60 s) and the burst windows
(t=10..12 s, t=20..22 s) separately.

### 7.2 Metric M1: IMU FIFO overrun rate

**Definition:** Number of dropped IMU samples per minute during the
burst windows.
**Measurement:** Hardware FIFO overflow counter on the BMI270
(register `0x1B`, bit `FIFO_FRAME_COUNTER` overflow flag).
**Acceptance:** Weaver must reduce overrun rate by ≥**50%** vs.
stock Zephyr at the same thread set and priorities. Baseline
expectation in stock Zephyr for the burst workload is 4–8 dropped
frames per burst.
**Why it matters:** Lost IMU samples directly degrade step counter
accuracy and gesture detection latency. Customer-visible.

### 7.3 Metric M2: Sensor-to-process latency

**Definition:** Time from `weaver_set_buffer_fill_predictive_q16()`
call by the IMU producer to start of fusion thread execution.
**Measurement:** `k_uptime_get_32()` timestamp pair around
`k_msgq_put` (producer) and `k_msgq_get` (consumer), histogram
collected via Zephyr's runtime stats.
**Acceptance:**
  - **P50:** ≤ stock Zephyr P50 (no regression)
  - **P99:** ≤ 0.7 × stock Zephyr P99 (30% improvement)
  - **P99.9:** ≤ 0.5 × stock Zephyr P99.9 (50% improvement)
**Why it matters:** Tail latency drives perceived responsiveness.
A wearable that reads HR consistently in 8 ms is better than one
that averages 5 ms but occasionally takes 50.

### 7.4 Metric M3: Warp deadline miss rate

**Definition:** Fraction of IMU/PPG/BLE sampling windows that miss
their nominal deadline by >1 ms.
**Measurement:** Each Warp thread records `k_uptime_get_32()` at
loop entry; compare delta to expected period. Logged via Zephyr
trace.
**Acceptance:** Weaver introduces **zero** new deadline misses vs.
stock Zephyr. This is a regression check, not an improvement
target. If Weaver causes any Warp thread to miss a deadline that
stock would have met, the feature is rejected.
**Why it matters:** The whole point of the Warp/Weft distinction is
to protect Warp behavior. If we degrade it, we have failed.

### 7.5 Metric M4: Dispatcher overhead

**Definition:** Mean and 99th percentile cycles spent inside
`weaver_tick()`, measured via DWT cycle counter
(`weaver_get_last_tick_cycles()`).
**Measurement:** `CONFIG_WEAVER_TIMING=y`, log every 100 ticks for
60 s, compute mean and P99.
**Acceptance:**
  - **Scalar variant:** mean ≤ 200 cycles, P99 ≤ 250 cycles at 12
    threads → ≤ 0.21% CPU at 1 ms tick @ 96 MHz.
  - **Full MVE variant:** mean ≤ 130 cycles, P99 ≤ 180 cycles at
    12 threads (target; may not be achievable due to MVE
    context-save tax — see M5).
**Why it matters:** The whole pitch is "predictive scheduling
without the cost." If scalar cost exceeds 1% of CPU, the cost
argument collapses.

### 7.6 Metric M5: MVE context-save amortization

**Definition:** Increase in mean preemption cost (cycles per
context switch) when switching from `WEAVER_BATCH_SCALAR` to
`WEAVER_BATCH_MVE` on a system where no Weft thread uses FPU/MVE.
**Measurement:** ITM trace timestamps at PendSV entry/exit,
averaged over 60 s.
**Acceptance:** Document the actual measured tax. If it exceeds 60
cycles per preemption, the MVE variant is recommended only for
systems with >32 threads or with FPU/MVE already in use elsewhere.
**Why it matters:** This is the empirical answer to the question
"is MVE worth it on Apollo510?" My back-of-envelope estimate is
~50 cycles. Real hardware will say.

### 7.7 Metric M6: Throttle response time

**Definition:** Time from `weaver_system_pressure` crossing the
threshold to a producer thread observing
`weaver_should_throttle() == true`.
**Measurement:** Inject a controlled pressure spike, log producer
observation timestamp.
**Acceptance:** P50 ≤ 4 ticks (4 ms at 1 ms tick), P99 ≤ 12 ticks
— bounded by the 4-tap EMA settle time.
**Why it matters:** Documents the cost of the EMA hysteresis. If
producers need faster response, they should consume
`weaver_throttle_level()` directly (no EMA) — but they pay flap
risk for it.

### 7.8 Metric M7: Battery life (qualitative)

**Definition:** Average current draw over a 1-hour run of the
canonical workload, measured at the EVB power input.
**Measurement:** Apollo510 `am_hal_pwrctrl` rail telemetry;
external Joulescope or Otii Arc on the Vbat rail.
**Acceptance:** ≤ stock Zephyr current draw + 5% (Weaver overhead
budget). Goal is parity or better thanks to power-aware tick skip.
**Why it matters:** Wearable battery life is the gating customer
metric. A scheduler that adds 20% to active current is a non-starter.

### 7.9 Combined gate

A release of Weaver requires:
- M1, M2, M4 all meet acceptance criteria
- M3 shows zero regression (hard gate)
- M5, M6 documented with real numbers
- M7 within 5% of stock

Failure on any hard gate (M3, or M7 worse than +5%) blocks merge.
M1/M2/M4 missing acceptance triggers redesign, not rejection.

---

## 8. Risk Analysis

### 8.1 Priority inversion

**Risk:** Boosting a Weft above a normally higher Weft could lock
out the displaced thread.
**Mitigation:** Boost is bounded by `boost_levels` (default 2),
clamped so the boosted priority never crosses into cooperative
space, and re-evaluated each tick. A Weft cannot be permanently
locked out for more than a few ticks under any pressure pattern.
**Residual risk:** Low. Mitigated by the `wait_ticks` aging term
which strictly increases pressure for any waiting thread.

### 8.2 Pressure runaway via aging

**Risk:** A long-blocked thread could see `wait_ticks` overflow and
wrap to zero, suddenly losing all aging pressure.
**Mitigation:** `wait_ticks` is clamped to `WEAVER_WAIT_TICKS_MAX`
(`WEAVER_Q16_ONE - 1`). Aging saturates rather than wraps.
**Residual risk:** None.

### 8.3 Lock contention with Zephyr scheduler

**Risk:** `k_thread_priority_set()` from `weaver_tick()` takes
Zephyr's scheduler lock; collision could increase scheduler latency.
**Mitigation:** Weaver's own spinlock is held only during the
snapshot copy (a few cycles). The `k_thread_priority_set()` calls
happen outside Weaver's lock and serialize naturally with the
Zephyr scheduler.
**Residual risk:** Low. Will be confirmed by M3 measurement.

### 8.4 Producer misuse of `weaver_set_buffer_fill_q16`

**Risk:** A bug in a producer (e.g., always writing 1.0) could
permanently elevate one thread.
**Mitigation:** Other Weft threads accumulate `wait_ticks` aging
which eventually overrides the bad signal. Pre-Warp guard further
limits the damage.
**Residual risk:** Medium. Producer correctness is the user's
responsibility. Recommendation: add a Kconfig'd debug assertion
that flags persistent fill==1.0 across N ticks.

### 8.5 Compiler / MVE intrinsic divergence

**Risk:** Different toolchain versions could emit different code
for the MVE intrinsics, breaking the equivalence guarantee.
**Mitigation:** The host equivalence test runs in CI on every
commit. If a future toolchain miscompiles MVE, host CI catches
the divergence in the structural test. (It cannot catch a runtime
miscompile of `vmullbq_int_u32` itself; that requires on-target
test, listed as a follow-up in §10.)

### 8.6 Power-aware skip masking real overload

**Risk:** `WEAVER_POWER_AWARE` skips dispatch when no Weft has
pressure. If a producer is broken and never reports pressure,
Weaver becomes silent.
**Mitigation:** Stats counter `skipped_idle_ticks` is exposed. A
production system should alarm if the ratio of skipped to total
ticks exceeds an SLO.
**Residual risk:** Low. Observable via stats.

---

## 9. Backwards Compatibility

- **Default behavior unchanged.** With `CONFIG_WEAVER_SCHED=n`
  (default), Weaver is not compiled in. Zero binary size impact.
- **Opt-in per thread.** A thread that is not registered is not
  affected by Weaver in any way. Mixed registries are supported.
- **Restorable.** `weaver_unregister(wd)` restores the captured
  base priority. The thread continues exactly as it would have
  without Weaver.
- **API stability.** Public APIs use opaque `struct weaver_thread_data`
  (caller-owned). Internal layout changes do not break ABI for
  out-of-tree users.
- **No changes to `kernel/sched.c`.** Reduces upstream Zephyr
  rebase risk to zero for the dispatcher itself; only the new
  files in `kernel/` and the small `Kconfig` / `CMakeLists.txt`
  adds need rebasing.

---

## 10. Implementation Plan

### Phase 1 (this RFC, complete)

- [x] Core dispatcher with scalar pressure batch
- [x] Q16.16 fixed-point math, integer-only
- [x] Warp/Weft classification
- [x] Pre-Warp guard ("Pattern Slicing")
- [x] Power-aware tick skip
- [x] Predictive buffer-fill helper
- [x] TSK fuzzy throttle controller with EMA hysteresis
- [x] Three pressure-batch variants (scalar/hybrid/full MVE)
- [x] Host-buildable equivalence test
- [x] DWT cycle-counter timing hook
- [x] Apollo510 LP @ 96 MHz wearable preset
- [x] Apollo510B EVB sample with real BMI270 + MAX30101 Click boards

### Phase 2 (next, follow-up patches)

- [ ] Benchmark sample (`samples/.../weaver_benchmark/`) with the
  canonical workload defined in §7.1
- [ ] Stock-Zephyr baseline measurement runs
- [ ] On-target measurement of M1–M7
- [ ] Weaver shell command for live `fabric stats` inspection
- [ ] Real BLE host integration (`bt_gatt_notify` → `weaver_set_buffer_fill_predictive_q16`)
- [ ] Real LVGL display thread on `ap510_disp` shield

### Phase 3 (after data lands)

- [ ] Tune defaults based on M1–M7 measurements
- [ ] 1 Hz outer-loop weight controller (multi-armed bandit or
  workload-state classifier) — only if M1/M2 data suggests
  static weights leave value on the table
- [ ] SMP support if/when a multi-core Apollo target lands
- [ ] `next_up()` integration as an opt-in "deep" mode for
  workloads that need pressure to override priority order

### Phase 4 (potentially upstream)

- [ ] Generalize beyond Apollo510 (test on nRF52840, ESP32-S3)
- [ ] RFC to Zephyr mailing list as a kernel module
- [ ] Twister test suite covering all three batch paths

---

## 11. Open Questions

These are intentionally surfaced for review:

**Q1.** Default `boost_levels` for the wearable preset is **2**
based on intuition (FIFO overrun is data loss, deserves a bigger
swing). Should this be **1** to match the more conservative default?
Data from M1/M2 should answer.

**Q2.** Pre-Warp guard window is **1 tick**. At 1 ms tick on
96 MHz, that's 96k cycles of guard. Is that the right tradeoff
between Warp protection and Weft throughput? Configurable via
`CONFIG_WEAVER_PREWARP_GUARD_TICKS`; recommend tuning per-product.

**Q3.** Throttle threshold default is **0x30000** (Q16.16 = 3.0)
under the wearable preset. Three Weft threads at full pressure trip
it. Too tight? Too loose? M6 will tell.

**Q4.** EMA smoothing is a 4-tap window
(`smoothed = (3·smoothed + level) / 4`). This trades response time
for stability. Is 4 the right depth? Power-of-2 keeps the
arithmetic cheap; the alternatives are 2 (faster, more flap) or 8
(slower, more stable).

**Q5.** `WEAVER_MAX_THREADS` defaults to **12** for the wearable
preset. Is that the right size for typical wearables, or should
the default be smaller (lower stack peak) or larger (room for
growth)?

**Q6.** The MVE batch variant includes a `vshrnbq_n_u64`/`vshrntq_n_u64`
combination. Some MVE toolchains/compilers emit suboptimal code for
this pattern; an inline-asm fallback may be warranted. We deferred
this until on-target measurements show whether the intrinsic path
is good enough.

**Q7.** Should we ship a default **stock Zephyr baseline test** in
the same repo so reviewers can re-run M1/M2 themselves and verify
our claimed improvements? Recommendation: yes, included in Phase 2.

---

## 12. References

- Zephyr scheduler source: `kernel/sched.c`, `kernel/priority_queues.c`
- Apollo510 datasheet: power modes, FPU/MVE specs
- Cortex-M55 Technical Reference Manual: lazy stacking, MVE
  context-save behavior
- ARM Helium intrinsics reference: `vmullbq_int_u32`,
  `vmulltq_int_u32`, `vshrnbq_n_u64`, `vshrntq_n_u64`, `vqaddq_u32`
- Bosch BMI270 datasheet: FIFO overflow flag at register `0x1B`
- Maxim MAX30101 datasheet: PPG sample format
- MikroElektronika 6DOF IMU 14 Click, Heart Rate 4 Click product pages
- Project repo: `RichardSWheatley/ambiqhal_ambiq` branch
  `claude/fixed-point-scheduler-zephyr-RMdbe`
- Companion docs: `DECISIONS.md`, `FUZZY_THROTTLE.md`,
  `ML_AND_FUZZY_LOGIC.md`, `MVE_PATHS.md`

---

## 13. Reviewers' Quick-Start

To exercise the proposal:

```sh
# 1. Apply the patch to ambiqzephyr
cd ambiqzephyr
git checkout -b weaver-review ambiq-stable
git apply /path/to/0001-weaver-scheduler.patch

# 2. Run the host equivalence test
gcc -std=c11 -O2 -Wall \
    tests/kernel/weaver_paths_equiv/src/main.c \
    -o /tmp/equiv && /tmp/equiv | tail -5

# 3. Build the wearable sample with each batch variant
for variant in SCALAR HYBRID MVE; do
    west build -p -b apollo510b_evb \
        samples/boards/apollo510b_evb/weaver_wearable \
        -- -DCONFIG_WEAVER_BATCH_${variant}=y \
           -DCONFIG_WEAVER_TIMING=y
    west flash
    # Read the tick_cyc column from the log
done

# 4. Compare to stock-Zephyr baseline (Phase 2 deliverable)
```

---

## 14. Reviewer Ballot

Please respond with one of:

- **+1**: Ready to merge after Phase 2 measurements confirm M1–M7.
- **+0**: Acceptable with caveats; please address marked Open
  Questions.
- **-0**: Concerned but not blocking. Specifics in review.
- **-1**: Blocks merge. State the specific gate failure.

For -1 votes, please specify which Goal (G1–G6) or Acceptance
Criterion (M1–M7) is at risk.
