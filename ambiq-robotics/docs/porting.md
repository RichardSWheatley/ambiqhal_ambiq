# ARB Porting Guide

Bringing ARB up on a new OS means implementing one file: a `platform.c` that
satisfies `arb/platform.h`. Optionally, add HAL bindings and a sample. Budget
~50-100 lines for the platform.

## 1. Implement the platform contract

```c
void       arb_platform_init(void);                          /* timebase + queue */
uint64_t   arb_platform_time_us(void);                       /* monotonic, ISR-safe */
int        arb_platform_post(arb_topic_id_t, const void*, size_t); /* ISR-safe */
void       arb_platform_dispatch(void);                      /* drain -> deliver */
arb_lock_t arb_platform_lock(void);                          /* short critical section */
void       arb_platform_unlock(arb_lock_t);
```

Requirements:

- **`post`** must be safe to call from an ISR and must copy the message (the
  caller's buffer is not retained). On overflow return `ARB_ERR_FULL`.
- **`dispatch`** drains the queue and calls `arb_topic_deliver(topic, msg, len)`
  for each message, in FIFO order, from a normal (non-ISR) context.
- **`lock`/`unlock`** guard the broker's small registries. They must work from
  both task and ISR context (mask interrupts or use an ISR-safe RTOS primitive).
  Keep the critical section short.
- **`time_us`** must be monotonic. If the OS only offers tick resolution, return
  ticks scaled to microseconds and mark the function `weak` so an application can
  override it with a hardware timer (e.g. the Apollo510 STIMER).

Two queue patterns are used by the existing ports:

  item is the message slot.
- **Static ring + critical section** (bare-metal, NuttX, RIOT, host): a small
  array guarded by the lock.
- **Pointer queue + block pool** (ThreadX): when the RTOS queue item is smaller
  than a message, enqueue slot pointers from a pool.

Look at `port/host/platform.c` for the simplest reference, and
`port/freertos/platform.c` for an RTOS one.

## 2. (Optional) HAL bindings

Provide an `arb_*_ops_t` for each device class you need (`motor`, `encoder`,
`imu`) and a `*_bind()` helper that configures the peripheral and calls the
matching `arb_*_init()`. You can:

- call your chip vendor's HAL directly (see `port/ambiqsuite/hal_bind.c`), or

On Apollo510 you can simply reuse `port/ambiqsuite/hal_bind.c` from any RTOS that
runs there (FreeRTOS, ThreadX, NuttX, RIOT, CMSIS-RTOS2 all do this).

## 3. Wire the build

Compile the core sources + your `platform.c` (+ HAL bindings). Reuse an existing
fragment as a template:

- Make-based (AmbiqSuite SDK): `port/<os>/arb.mk` (see `freertos`, `threadx`).
- NuttX apps: `samples/nuttx/{Makefile,Kconfig,Make.defs}`.

The core source list is:

```
core/src/topic.c  core/src/service.c
core/src/hal/{motor,encoder,imu}.c
core/src/control/{pid,diff_drive}.c
core/src/bridge/{jaus,serial}.c
```

## 4. Verify

Run the host test suite to confirm the core behaves, then bring up your port:

```bash
cmake -S ambiq-robotics -B build -DARB_BUILD_TESTS=ON
cmake --build build && ctest --test-dir build --output-on-failure
```

A good first hardware smoke test: advertise a topic, subscribe a callback,
publish from a timer ISR, and confirm the callback runs after
`arb_platform_dispatch()` - that exercises `post`, the queue, the lock, and
`dispatch` end to end.

## Checklist

- [ ] `post` is ISR-safe and copies the payload
- [ ] `dispatch` delivers FIFO from task context
- [ ] `lock`/`unlock` valid in task **and** ISR context
- [ ] `time_us` monotonic (override weak symbol for sub-ms if tick-based)
- [ ] core + platform compiled into the image
- [ ] host tests pass; ISR publish smoke test passes on hardware
