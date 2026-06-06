# ARB ThreadX sample — differential-drive base

The closed-loop diff-drive node on the **Eclipse ThreadX** broker port, split
across three threads:

- **control** (50 Hz) — encoder feedback → per-wheel PID → motors; publishes
  `/odom` and `/imu`,
- **dispatcher** — drains the broker queue and delivers to subscribers,
- **command** — stand-in `/cmd_vel` source (replace with UART teleop, the JAUS
  bridge, or micro-ROS).

Threads are created in `tx_application_define()`; peripheral bindings reuse the
AmbiqSuite direct-HAL bindings (`port/ambiqsuite/hal_bind.c`).

## Build

Inside an AmbiqSuite + ThreadX example Makefile:

```make
ARB_ROOT := ../../../ambiq-robotics
include $(ARB_ROOT)/port/threadx/arb.mk

INCLUDES += $(ARB_INCLUDES)
SRC      += $(ARB_SRC)
SRC      += main.c
```

> ThreadX queue messages are capped at 64 bytes, so the ThreadX port passes slot
> pointers through the queue backed by a `TX_BLOCK_POOL` (see
> `port/threadx/platform.c`). Override the `weak` `arb_platform_time_us()` with a
> hardware timer (Apollo510 STIMER) for sub-millisecond stamps.
