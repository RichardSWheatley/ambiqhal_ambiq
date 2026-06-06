# ARB RIOT sample — differential-drive base

The closed-loop diff-drive node on the **RIOT OS** broker port. A control thread
and a command thread run alongside the main thread (which drains the broker),
created with `thread_create()` and paced by `xtimer`. Peripheral bindings reuse
the AmbiqSuite direct-HAL bindings.

## Build

```sh
make BOARD=<apollo510-board> \
     RIOTBASE=/path/to/RIOT \
     ARBDIR=/path/to/ambiq-robotics
```

The application `Makefile` lists the ARB core sources, the RIOT `platform.c`, and
the AmbiqSuite HAL bindings via `SRC` + `vpath`, and puts the ARB include paths
on `CFLAGS`.

> The RIOT port locks with `irq_disable()`/`irq_restore()` and reads the
> timebase from `xtimer_now_usec64()` (true microseconds). If your build uses
> ztimer instead of xtimer, swap the timebase call in `port/riot/platform.c` and
> the `USEMODULE` line here.
