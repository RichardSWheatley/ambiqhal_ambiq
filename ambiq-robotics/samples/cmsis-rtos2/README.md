# ARB CMSIS-RTOS2 sample — differential-drive base

The closed-loop diff-drive node on the **CMSIS-RTOS2** broker port (Keil RTX5 or
any CMSIS-RTOS2 RTOS on the Apollo510), with control / dispatcher / command
threads created via `osThreadNew()`. Peripheral bindings reuse the AmbiqSuite
direct-HAL bindings.

## Build

Add the makefile fragment to your project and the sample source:

```make
ARB_ROOT := ../../../ambiq-robotics
include $(ARB_ROOT)/port/cmsis-rtos2/arb.mk

INCLUDES += $(ARB_INCLUDES)
SRC      += $(ARB_SRC)
SRC      += main.c
```

(For Keil µVision/CMSIS-Pack projects, add the same files to the project and put
`$(ARB_ROOT)/core/include` and `$(ARB_ROOT)/port/ambiqsuite` on the include
path.)

> The CMSIS-RTOS2 port locks with PRIMASK and derives `arb_platform_time_us()`
> from `osKernelGetTickCount()`; override the `weak` time hook with the Apollo510
> STIMER for sub-millisecond stamps.
