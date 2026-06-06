# ARB FreeRTOS sample — differential-drive base

The closed-loop diff-drive node running on the **FreeRTOS** broker port, split
across three tasks to show cross-task pub/sub:

- **control** (50 Hz) — encoder feedback → per-wheel PID → motors; publishes
  `/odom` and `/imu`,
- **dispatcher** — drains the broker queue and delivers to subscribers,
- **command** — a stand-in `/cmd_vel` source (replace with UART teleop, the JAUS
  bridge, or micro-ROS).

Peripheral bindings reuse the AmbiqSuite direct-HAL bindings
(`port/ambiqsuite/hal_bind.c`) — AmbiqSuite ships FreeRTOS, so this is the
common no-Zephyr RTOS setup on Apollo510.

## Build

Inside an AmbiqSuite + FreeRTOS example Makefile:

```make
ARB_ROOT := ../../../ambiq-robotics
include $(ARB_ROOT)/port/freertos/arb.mk

INCLUDES += $(ARB_INCLUDES)
SRC      += $(ARB_SRC)
SRC      += main.c
```

`arb.mk` pulls in the OS-agnostic core, the FreeRTOS `platform.c`, and the
AmbiqSuite HAL bindings. Timer/pad/IOM choices and PID gains are placeholders —
adapt them and your robot geometry (top of `main.c`) to your board.

> The FreeRTOS port's `arb_platform_time_us()` is `weak` and defaults to tick
> resolution. For sub-millisecond stamps, override it with the Apollo510 STIMER
> (see `port/ambiqsuite/platform.c` for the pattern).
