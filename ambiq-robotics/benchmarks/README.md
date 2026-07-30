# ARB benchmarks — latency & jitter (Robotics WG D3)

Reproducible measurements of the timing properties the Zephyr Robotics WG's
benchmarking deliverable asks for: IRQ latency, scheduling jitter, and
end-to-end command latency, each on an idle system and under mixed-criticality
background load (zbus traffic + logging + memory churn).

## Layout

```
common/        clock shim, HDR-lite histogram, JSON reporting, load generator
irq_latency/   timer-compare -> alarm-ISR delta            (kernel-only)
sched_jitter/  1 kHz absolute-deadline wakeup jitter       (kernel-only)
e2e_cmd_vel/   cmd_vel publish -> PWM write / host RTT     (ARB-dependent)
```

`irq_latency` and `sched_jitter` deliberately use nothing but kernel APIs so
they can move to `zephyr/tests/benchmarks/` unchanged if the WG wants them
upstream. Only `e2e_cmd_vel` depends on ARB.

## Output format

One machine-readable line per run, so results across boards and runs diff
cleanly, prefixed to match twister's `record`/`as_json` harness convention:

```
RECORD: {"bench":"irq_latency","board":"apollo510_evb","load":"idle","n":100000,"min_ns":312,"avg_ns":540,"max_ns":2210,"p99_ns":890,"counter_ns_per_tick":666}
```

- `load` is `idle` or `zbus+log` (the `.loaded` twister scenarios).
- `sched_jitter` emits two lines, `"metric":"wake_err_ns"` (lateness vs the
  absolute deadline, measured in the kernel timer's own clock domain — no
  cross-oscillator drift, resolution `tickclock_ns_per_cycle`) and
  `"metric":"period_jitter_ns"` (wake-to-wake delta error via the DWT cycle
  counter — fine-grained, one period per delta so drift cannot accumulate).
- Percentiles come from a 4 KiB log2/linear histogram (≤ 3.1 % relative
  quantization at p99); min/avg/max are exact.

## Running

```
west twister -T ambiq-robotics/benchmarks -p apollo510_evb \
    --device-testing --device-serial /dev/ttyXXX \
    -x=ZEPHYR_EXTRA_MODULES=$PWD/ambiq-robotics
```

`native_sim` runs are **CI plumbing only**: on the POSIX arch simulated time
does not advance while code executes, so latencies collapse to ~0. Numbers
only count from hardware. `CONFIG_BENCH_SAMPLES` (default 100000) scales run
time.

## What exactly is measured

- **irq_latency**: hardware timer compare event → counter-driver alarm
  callback entry, via an absolute counter alarm armed a randomized
  300 µs–1 ms ahead and a calibrated counter-tick→DWT-cycle ratio. This
  includes the driver's ISR prologue: an honest upper bound, consistent
  across boards. Quantization: one counter tick (667 ns on Apollo510's
  HFRC/64 timers).
- **sched_jitter**: `k_sleep(K_TIMEOUT_ABS_TICKS(next))` at 1 kHz;
  `wake_err_ns` is how late the thread actually ran vs `next`. On Apollo510
  the kernel tick is pinned to the 32.768 kHz stimer (30.5 µs resolution)
  which also bounds the deadline granularity itself; `period_jitter_ns`
  supplies the sub-tick view.
- **e2e_cmd_vel**: see `e2e_cmd_vel/` (on-board zbus→PWM path and host RTT
  over the serial framing).
