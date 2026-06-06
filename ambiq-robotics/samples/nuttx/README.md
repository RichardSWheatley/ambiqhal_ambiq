# ARB NuttX sample — differential-drive base

The closed-loop diff-drive node on the **Apache NuttX** broker port. A POSIX
thread is the stand-in `/cmd_vel` source; the main task runs the 50 Hz control +
telemetry loop and drains the broker — showing cross-thread pub/sub on NuttX.
Peripheral bindings reuse the AmbiqSuite direct-HAL bindings.

## Install

Copy this directory into your NuttX apps tree, e.g.:

```
apps/examples/arb_diffdrive/
```

Then enable it in `menuconfig` (it adds an *ARB differential-drive sample* entry
via `Kconfig`), or set `CONFIG_ARB_DIFFDRIVE=y`. Point the Makefile at the
middleware checkout if it isn't a sibling of `apps/`:

```
make ARBDIR=/path/to/ambiq-robotics
```

The provided `Makefile`/`Make.defs`/`Kconfig` follow standard NuttX app
conventions and compile the OS-agnostic core, the NuttX `platform.c`, and the
AmbiqSuite HAL bindings into the application.

> The NuttX port uses `enter_critical_section()` for the broker lock and
> `CLOCK_MONOTONIC` for the timebase, so timestamps are real microseconds with no
> override needed.
