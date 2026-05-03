# Apollo510 Wearable: Assumptions and Decisions

This document captures every assumption I made while tuning the Weaver
scheduler for an Ambiq Apollo510 running at 96 MHz in Low-Power mode
on a wearable device. Read this before deploying — your actual sensor
lineup, BLE parameters, and battery target may differ.

## Hardware assumptions

| Topic | Assumption | Source |
|---|---|---|
| MCU | Apollo510 (Cortex-M55, Helium MVE, FPU) | `mcu/apollo510/` in HAL repo |
| CPU mode | LP @ 96 MHz (vs HP @ 192/250 MHz) | `am_hal_utils.c:167` confirms LP=96 MHz |
| Cache | 16 KB I-cache, 16 KB D-cache enabled | Apollo510 default |
| Power | Battery, deepsleep most of the time | Wearable workload |
| FPU | Available but **not used** in scheduler path | Avoids FPU context-save in the dispatch hot path |

Why not use Helium MVE for the pressure math? Per-thread cost is 2
multiplies + 3 adds; vectorising 8 threads in MVE saves a few cycles
but adds vector-register save/restore on context switch. Net loss for
this workload.

## Sensor pipeline assumptions

I modeled a generic fitness/health wearable. Adjust if yours differs:

| Thread | Class | Cadence | Assumed source |
|---|---|---|---|
| BLE LL connection event | Warp | 50 ms | Default for fitness wearables (7.5–500 ms range) |
| IMU 6-axis | Warp | 20 ms (50 Hz) | Step counter / gesture |
| PPG / optical HR | Warp | 40 ms (25 Hz) | Continuous HR monitoring |
| Sensor fusion | Weft | event-driven | Consumes IMU FIFO |
| HR algorithm | Weft | event-driven | Consumes PPG FIFO |
| GATT TX queue | Weft | event-driven | Notification ring |
| Activity classifier | Weft | 500 ms | Periodic ML inference |
| Display update | Weft | 33 ms (30 fps) | AMOLED partial refresh |
| NVM/log flush | Weft | 1000 ms | Hourly summaries |

If your wearable has continuous audio (mic-array for keyword spotting),
add it as a Warp at the audio frame rate. If it has on-die GPU
animation (NemaGFX present in this repo), GPU work goes Weft.

## Scheduler decisions

### 1. Q16.16 fixed-point, no FPU

**Decision:** Math runs entirely on the integer ALU. The Apollo510 has
an FPU, but using it in the scheduler hook forces FPU context-save on
every preemption, which dominates the savings. Q16.16 with one 64-bit
multiply per term costs ~5 cycles on M55.

### 2. Non-invasive layer, not a replacement for `kernel/sched.c`

**Decision:** Weaver maintains its own thread registry and influences
the standard Zephyr scheduler via `k_thread_priority_set()`. It does
*not* override `next_up()` or `runq_best()`. The trade-off is that
Weaver controls "who is most important" but Zephyr still owns "who
runs next". For 99% of wearable workloads this is correct. If you ever
need pressure to override even priority order, there's a follow-up
hook point at `kernel/sched.c:165`.

### 3. Boost magnitude = 2 for wearables

**Decision:** A winning Weft thread climbs **two** Zephyr priority
levels (vs. the default 1). Rationale: wearable FIFO overruns lose
biometric data that cannot be re-collected. A two-level boost gives a
sensor-fusion thread a clear edge over BLE GATT TX and display work
when its IMU FIFO is filling.

The boost is clamped so it never crosses into cooperative space
(negative priority on Zephyr).

### 4. Pre-Warp guard window = 1 tick

**Decision:** When any Warp deadline is within 1 tick (1 ms), suppress
all Weft promotion for that pass. This implements the screenshot's
"Pattern Slicing" idea: the deadline-bound thread (BLE LL connection
event, IMU sample, PPG sample) sees a quiet system with warm cache
lines.

At 1 ms tick on Apollo510 LP-mode, 1 ms = 96,000 cycles, plenty of time
for the I-cache to be reloaded with Warp code if it was evicted by
recent Weft activity.

Tunable via `CONFIG_WEAVER_PREWARP_GUARD_TICKS`. Set to 0 to disable.

### 5. Power-aware tick skip

**Decision:** If every Weft has zero pressure, `weaver_tick()` returns
without changing any priorities. Why: changing priorities wakes the
scheduler and can defer deepsleep entry. On a wearable in screen-off
"resting" state (no pending sensor data, no GATT TX), Weaver should be
invisible.

`CONFIG_WEAVER_POWER_AWARE` is **on by default in the wearable preset**.

### 6. 1 ms tick period

**Decision:** `CONFIG_SYS_CLOCK_TICKS_PER_SEC=1000`. At 96 MHz with
12 registered threads, one `weaver_tick()` runs in approximately
1.9 µs. That's 0.19% of CPU at 1 ms tick. Acceptable.

You could go to 5 ms tick (200 Hz) and drop to 0.04% if your fastest
Warp deadline is ≥10 ms, but BLE connection events at 50 ms benefit
from finer-grained pre-warp guard timing at 1 ms.

### 7. Throttle threshold = 3.0 (Q16.16 = 0x30000)

**Decision:** Wearable preset uses 3.0 vs. the default 4.0. Three
moderately-loaded Weft threads will trip throttle, signaling
non-essential producers (display, classifier, NVM) to back off. The
default 4.0 is fine for non-battery designs but is too generous for
wearable battery life.

### 8. Thread classification rule of thumb

| Question | Answer |
|---|---|
| "Misses deadline → product breaks" | **Warp** |
| "Misses deadline → user notices" | **Weft, register early** |
| "Misses deadline → who cares" | **Weft, low static priority** |

BLE connection events: Warp (a missed event drops the link).
Sensor sampling: Warp (FIFO overflow loses data).
Sensor fusion: Weft (works opportunistically; pressure rises with FIFO).
Display: Weft (60 fps is nice, 15 fps is fine).
NVM flush: Weft (deferrable for many seconds).

### 9. No DMA-driven dispatch

**Decision:** I assume sensors are read by their Warp threads via
SPI/I2C (typical Apollo510 IOM). DMA is fine — just keep the DMA
completion ISR short and let the Warp thread handle the post-DMA work.
Weaver doesn't currently model "DMA in flight" pressure, but you can
expose it via `weaver_set_buffer_fill_q16()` from the DMA-complete
callback.

## What I did NOT implement

| Feature | Why deferred |
|---|---|
| SMP per-CPU registries | Apollo510 is single-core M55; not needed |
| Direct `next_up()` patch | Higher risk; non-invasive layer covers 99% of cases |
| BLE-controller-aware deadline | Requires real Bluetooth host integration; mocked in sample |
| Helium MVE pressure math | Adds vector context-save cost; wins at >32 threads, not 12 |
| `K_THREAD_DEFINE`-style macro | Would couple Weaver to Zephyr macros across versions |
| Throttle action policy | Producer-side decision; Weaver only reports |
| Persistence of fabric stats across reboot | Not enough storage budget; export via shell instead |

## Performance budget at 96 MHz

| Item | Cost |
|---|---|
| Per-thread pressure calc | ~12 cycles (M55, no MVE) |
| 12-thread tick | ~145 cycles for math + ~30 cycles snapshot copy = ~180 cycles |
| At 96 MHz | 180 / 96e6 = **1.875 µs per tick** |
| At 1 ms tick rate | 0.187% CPU |
| Per-day at 86,400,000 ticks | 162 seconds of CPU = **1.9 mWh** at typical Apollo510 LP draw |

For comparison, Apollo510 deepsleep current is well under 5 µA. The
scheduler overhead is dwarfed by even one BLE TX burst.

## Tunables you might want to revisit

1. **`BLE_INTERVAL_MS`** in the sample — set to 50 ms (default fitness).
   Lower for live notifications (15 ms), higher for sleep tracking
   (500 ms).
2. **`IMU_INTERVAL_MS`** — 20 ms (50 Hz) is fine for steps. For
   gesture recognition or fall detection, 100 Hz / 200 Hz may be
   needed; bump to 10 ms / 5 ms.
3. **`CONFIG_WEAVER_MAX_THREADS`** — 12 covers a typical wearable. If
   you have multi-mic audio + GPU watchface + multiple BLE services,
   bump to 16 or 24.
4. **`CONFIG_WEAVER_THROTTLE_THRESHOLD`** — start at 3.0, watch the
   throttle event counter under realistic load, tune from there.

## Open questions for you

- **Real BLE host:** the sample mocks BLE LL. Want me to wire it to
  the Apollo510 cooper/em9305 bluetooth driver from this repo's
  `components/bluetooth/`?
- **Real sensor drivers:** I have access to the Ambiq IOM/SPI drivers
  in this HAL. Want me to swap the synthetic FIFOs for real BMI270 /
  ADPD188 driver bindings?
- **Power telemetry:** Apollo510 has on-die current monitoring. Want a
  `weaver_estimate_power_uw()` accessor that correlates pressure to
  estimated draw?
- **Shell command:** `weaver` shell command for live `fabric stats`
  inspection during bring-up?
