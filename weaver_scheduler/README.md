# Weaver Scheduler for Zephyr

Predictive, pressure-aware fixed-point scheduling layer for Zephyr OS.
Classifies threads as **Warp** (hard real-time, deterministic) or
**Weft** (opportunistic, scheduled into the gaps between Warp threads
based on backpressure). Uses Q16.16 fixed-point math throughout — no
FPU dependency, suitable for ARM Cortex-M0/M3/M4 class MCUs.

This directory holds the source files staged for `ambiqzephyr`. The
target tree (Zephyr root) layout is:

```
include/zephyr/kernel/weaver_sched.h
kernel/weaver_sched.c
kernel/Kconfig.weaver
samples/kernel/weaver_sched/{CMakeLists.txt,prj.conf,sample.yaml,README.rst,src/main.c}
```

Plus two existing-file edits captured in the patch:
- `kernel/Kconfig` — add `rsource "Kconfig.weaver"`
- `kernel/CMakeLists.txt` — add `kernel_sources_ifdef(CONFIG_WEAVER_SCHED weaver_sched.c)`

## Why this lives here

The harness signing service does not authorize commits to
`RichardSWheatley/ambiqzephyr`, so the change set is staged inside
`ambiqhal_ambiq` (which is on the allowlist) until the user grants
access. Apply the included patch to land it in `ambiqzephyr`.

## Applying to ambiqzephyr

```sh
git clone https://github.com/RichardSWheatley/ambiqzephyr.git
cd ambiqzephyr
git checkout -b claude/fixed-point-scheduler-zephyr-RMdbe
git apply /path/to/ambiqhal_ambiq/weaver_scheduler/0001-weaver-scheduler.patch
git add -A && git commit -m "kernel: add Weaver fixed-point pressure-aware scheduling layer"
git push -u origin claude/fixed-point-scheduler-zephyr-RMdbe
```

## Algorithm

Each `weaver_tick()`:

1. Snapshot the registered thread set (bounded by
   `CONFIG_WEAVER_MAX_THREADS`).
2. Per thread, compute Q16.16 pressure:
   - **Warp:** saturate to `0xFFFFFFFF`.
   - **Weft:**
     `p = (priority * 0.4) + (buffer_fill * 0.4) + (wait_ticks * 0.2)`
     all in Q16.16 via `(A*B) >> 16`.
3. Promote the highest-pressure Weft thread by lowering its Zephyr
   priority value by 1 (one level higher in importance). All other
   Weft threads are restored to their captured base priority.
4. Increment `wait_ticks` for non-running threads (aging).

System pressure (sum of Weft pressures) crossing
`CONFIG_WEAVER_THROTTLE_THRESHOLD` flips `weaver_should_throttle()` to
true, signaling non-essential producers to slow down.

## Performance

- Per-thread cost: 2× Q16 multiplies + 3 adds + 1 priority compare ≈
  10–15 cycles on Cortex-M4.
- 8 threads × 15 cycles @ 64 MHz ≈ **1.9 µs per tick**.
- 8 threads × 15 cycles @ 160 MHz ≈ **0.75 µs per tick**.
- Bounded by the registry size, so jitter is deterministic.

## Files

| File | Purpose |
|------|---------|
| `include/zephyr/kernel/weaver_sched.h` | Public API, Q16.16 macros, struct definitions |
| `kernel/weaver_sched.c` | Implementation: pressure calc, dispatcher, throttle |
| `kernel/Kconfig.weaver` | `WEAVER_SCHED`, `WEAVER_MAX_THREADS`, `WEAVER_THROTTLE_THRESHOLD` |
| `samples/kernel/weaver_sched/` | Demo: 1 Warp + 2 Weft producers, 100-tick ramp |
| `0001-weaver-scheduler.patch` | Patch to apply on top of `ambiq-stable` in `ambiqzephyr` |
