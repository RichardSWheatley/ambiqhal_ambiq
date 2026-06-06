# ARB bare-metal sample — differential-drive base

A complete no-OS robotics node on the Apollo510:

- subscribes to `/cmd_vel` (`arb_twist_t`) and drives two PWM motors,
- reads two wheel encoders and publishes `/odom` (`arb_odom_t`),
- samples an I2C IMU and publishes `/imu` (`arb_imu_t`),
- runs everything from a single super-loop with `arb_platform_dispatch()`.

## Wiring it into an AmbiqSuite project

This sample is a drop-in `main.c`. Build it inside an AmbiqSuite SDK example
(which provides the startup code, linker script, and `am_mcu_apollo.h`):

```make
ARB_ROOT := ../../../ambiq-robotics
include $(ARB_ROOT)/port/ambiqsuite/arb.mk

INCLUDES += $(ARB_INCLUDES)
SRC      += $(ARB_SRC)
SRC      += main.c
```

## Adapt before flashing

The timer numbers, GPIO pads, IOM module, and I2C address in `main.c` and in
`port/ambiqsuite/hal_bind.c` are **placeholders**. Set them to match your board:

- motor PWM timers + output pads + DIR pins,
- encoder counter timers and counts-per-revolution,
- IMU IOM module, I2C address, and full-scale → SI scaling.

The robot geometry (`WHEEL_BASE_M`, `WHEEL_RADIUS_M`, `MAX_WHEEL_SPEED_MS`) also
lives at the top of `main.c`.
