# Ambiq Robotics Broker (ARB)

A lightweight robotics middleware for the **Apollo510** that abstracts the
AmbiqSuite HAL and runs both on **bare-metal AmbiqSuite** and under **Zephyr**.
It borrows architectural DNA from ROS2/JAUS — publish/subscribe, request/response
services, a node-like presence on the network — but is sized for an MCU: static
allocation only, no IDL codegen, no heap. On Zephyr it can bridge upstream to
**micro-ROS** so the Apollo510 shows up as a ROS2 node.

## Layout

```
ambiq-robotics/
├── core/                  # OS-agnostic, pure C — the portable heart
│   ├── include/arb/
│   │   ├── msg.h          # flat message structs + type IDs
│   │   ├── topic.h        # pub/sub broker API
│   │   ├── service.h      # request/response API
│   │   ├── platform.h     # THE port-layer contract
│   │   ├── hal/           # robotics-oriented peripheral abstractions
│   │   │   ├── motor.h    # motor_set_duty(), ...
│   │   │   ├── encoder.h  # encoder_read() -> angle + velocity
│   │   │   └── imu.h      # imu_sample()
│   │   ├── control/       # reusable control building blocks
│   │   │   ├── pid.h      # PID w/ anti-windup
│   │   │   └── diff_drive.h # twist <-> wheel kinematics
│   │   └── bridge/        # external-network bridges
│   │       └── jaus.h     # JAUS (SAE AS-4) codec + topic mapping
│   └── src/               # topic.c, service.c, hal/*, control/*, bridge/*
├── port/                  # one small platform.c per OS (the only OS-specific code)
│   ├── ambiqsuite/        # no-OS super-loop + direct Ambiq HAL bindings
│   ├── zephyr/            # Zephyr module: k_msgq dispatch, device API, micro-ROS
│   ├── freertos/          # FreeRTOS (incl. the one bundled in AmbiqSuite)
│   ├── threadx/           # Eclipse ThreadX (Azure RTOS)
│   ├── nuttx/             # Apache NuttX
│   ├── riot/              # RIOT OS
│   ├── cmsis-rtos2/       # CMSIS-RTOS2 (Keil RTX5, etc.)
│   └── host/              # POSIX simulation + JAUS/UDP + OpenJAUS adapter
├── samples/
│   ├── ambiqsuite/        # closed-loop diff-drive base, super-loop
│   └── zephyr/            # closed-loop diff-drive base, dispatcher thread
└── tests/                 # host unit tests (ctest)
```

## Architecture

Three layers, two seams:

1. **Core (OS-agnostic).** Topic registry, service registry, and the HAL
   abstraction logic (clamping/scaling/differentiation). Knows nothing about an
   OS or a peripheral — only `platform.h` and the per-device backend ops.
2. **Port (`platform.h`).** The single contract between core and OS. A port
   implements the timebase, an ISR-safe `post`, a `dispatch` drain, and a lock.
3. **HAL bindings (`hal_bind.c`).** Per-port glue that fills in the motor /
   encoder / IMU backend ops with real peripheral calls (direct Ambiq HAL on
   bare-metal; Zephyr PWM/GPIO/sensor — or direct Ambiq HAL — on Zephyr).

### The port contract (`platform.h`)

```c
void     arb_platform_init(void);                 /* timebase + queue */
uint64_t arb_platform_time_us(void);              /* monotonic, ISR-safe */
int      arb_platform_post(arb_topic_id_t, const void *, size_t);  /* ISR-safe */
void     arb_platform_dispatch(void);             /* drain -> deliver */
arb_lock_t arb_platform_lock(void);               /* short critical section */
void     arb_platform_unlock(arb_lock_t);
```

`arb_topic_publish()` copies the message into the port queue via `post` (safe
from an ISR). Later, from a normal context, `dispatch` pulls each message and
calls `arb_topic_deliver()`, which fans it out to subscribers. That decoupling is
what lets the same code run on a bare-metal super-loop and on a Zephyr thread.

## Data flow

```
publisher --arb_topic_publish--> [port queue] --arb_platform_dispatch--> subscribers
ROS2 /cmd_vel --micro-ROS bridge--> arb_topic_publish --> motor controller
encoders --> arb_topic_publish(/odom) --> micro-ROS bridge --> ROS2 /odom
```

## Building

### Bare-metal AmbiqSuite

Add the makefile fragment to an AmbiqSuite SDK example and pull in the sources:

```make
ARB_ROOT := path/to/ambiq-robotics
include $(ARB_ROOT)/port/ambiqsuite/arb.mk
INCLUDES += $(ARB_INCLUDES)
SRC      += $(ARB_SRC)
```

See `samples/ambiqsuite/main.c` for a complete differential-drive node.

### Zephyr

Add this directory as a Zephyr module (it ships `zephyr/module.yml`) and enable
it in your `prj.conf`:

```
CONFIG_ARB=y
CONFIG_PWM=y
CONFIG_GPIO=y
CONFIG_SENSOR=y
```

Build the sample (provide a board overlay defining the `pwm-motor-l/r`,
`qdec-l/r`, and `robot-imu` aliases):

```
west build -b apollo510_evb samples/zephyr
```

Enable `CONFIG_ARB_MICRO_ROS_BRIDGE=y` (with the micro-ROS module present) to
mirror `/cmd_vel`, `/odom`, and `/imu` onto a ROS2 network.

## Supported platforms (ports)

The only OS-specific code is one small `platform.c` per target implementing the
`platform.h` contract. Adding a new OS is ~50-100 lines.

| Port | `post` (ISR-safe) | `dispatch` | lock | timebase |
|------|-------------------|-----------|------|----------|
| `ambiqsuite` (no-OS) | static ring + PRIMASK | super-loop drain | PRIMASK | STIMER (true µs) |
| `zephyr` | `k_msgq` | thread / manual | `irq_lock` | kernel ticks |
| `freertos` | `xQueueSendFromISR` | `xQueueReceive` | BASEPRI mask | ticks (override for µs) |
| `threadx` | block pool + `tx_queue_send` | `tx_queue_receive` | `TX_DISABLE` | ticks (override for µs) |
| `nuttx` | static ring + critical section | drain | `enter_critical_section` | `CLOCK_MONOTONIC` |
| `riot` | static ring + `irq_disable` | drain | `irq_disable` | xtimer 64-bit µs |
| `cmsis-rtos2` | `osMessageQueuePut` | `osMessageQueueGet` | PRIMASK | ticks (override for µs) |
| `host` | array ring | drain | no-op | `CLOCK_MONOTONIC` |

Tick-based ports expose a **weak** `arb_platform_time_us()`; override it with a
hardware timer (e.g. the Apollo510 STIMER) when you need sub-millisecond stamps.
*(Arm Mbed OS is intentionally omitted - Arm EOLs it in July 2026.)*

## JAUS interoperability

`core/src/bridge/jaus.c` speaks a useful subset of **JAUS (SAE AS-4)** so an ARB
node interoperates on a JAUS network, mirroring how the micro-ROS bridge exposes
it to ROS2:

```
JAUS SetWrenchEffort        -> ARB /cmd_vel
ARB  /odom                  -> JAUS ReportVelocityState + ReportLocalPose
JAUS Query{VelocityState,LocalPose,Identification,Heartbeat} -> matching Report
```

The codec is transport-agnostic (scaled-integer fields, command-code framing);
bytes leave through a send hook. Two transports are provided under
`port/host/bridge/`:

- **`jaus_udp.c`** - a simple UDP transport for ARB-to-ARB JAUS over a network
  or on the bench (functional, host-buildable).
- **`open_jaus.c`** - an **OpenJAUS SDK** adapter (built only with
  `-DARB_WITH_OPENJAUS`) that hands AS5669A transport, discovery, and node
  management to OpenJAUS while ARB owns robot behavior.

## Tuning (compile-time)

| Macro | Default | Meaning |
|-------|---------|---------|
| `ARB_MAX_TOPICS` | 16 | advertised topics |
| `ARB_MAX_SUBSCRIBERS` | 32 | total subscribers |
| `ARB_MAX_SERVICES` | 16 | registered services |
| `ARB_MSG_MAX_SIZE` | 64 | largest message (bytes) |
| `ARB_QUEUE_DEPTH` | 16 | port queue depth |

On Zephyr these map to `CONFIG_ARB_*` Kconfig options.

## Testing

The OS-agnostic core builds and runs on a host via the `port/host` simulation
port. Build and run the unit suite (broker, services, HAL logic, PID, kinematics):

```bash
cmake -S ambiq-robotics -B build -DARB_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

## License

BSD-3-Clause / Apache-2.0 (matching the surrounding AmbiqSuite HAL module).
