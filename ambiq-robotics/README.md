# Ambiq Robotics Broker (ARB)

A lightweight robotics middleware for the **Apollo510** that abstracts the
It borrows architectural DNA from ROS2/JAUS — publish/subscribe, request/response
services, a node-like presence on the network — but is sized for an MCU: static
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
│   │   └── bridge/        # external-link bridges
│   │       ├── jaus.h     # JAUS (SAE AS-4) codec + topic mapping
│   │       └── serial.h   # framed UART/SPI transport (telemetry + commands)
│   └── src/               # topic.c, service.c, hal/*, control/*, bridge/*
├── port/                  # one small platform.c per OS (the only OS-specific code)
│   ├── ambiqsuite/        # no-OS super-loop + direct Ambiq HAL bindings
│   ├── freertos/          # FreeRTOS (incl. the one bundled in AmbiqSuite)
│   ├── threadx/           # Eclipse ThreadX (Azure RTOS)
│   ├── nuttx/             # Apache NuttX
│   ├── riot/              # RIOT OS
│   ├── cmsis-rtos2/       # CMSIS-RTOS2 (Keil RTX5, etc.)
│   └── host/              # POSIX simulation + JAUS/UDP + OpenJAUS adapter
│                          #   riot, cmsis-rtos2, jaus
└── docs/                  # architecture.md, porting.md
│   ├── ambiqsuite/        # closed-loop diff-drive base, super-loop
└── tests/                 # host unit tests (ctest)
```

## Documentation

- [docs/architecture.md](docs/architecture.md) — layers, message flow, HAL,
  control, and bridges.
- [docs/porting.md](docs/porting.md) — bring ARB up on a new OS (the `platform.h`
  contract, queue patterns, build wiring, verification checklist).

## Architecture

Three layers, two seams:

1. **Core (OS-agnostic).** Topic registry, service registry, and the HAL
   abstraction logic (clamping/scaling/differentiation). Knows nothing about an
   OS or a peripheral — only `platform.h` and the per-device backend ops.
2. **Port (`platform.h`).** The single contract between core and OS. A port
   implements the timebase, an ISR-safe `post`, a `dispatch` drain, and a lock.
3. **HAL bindings (`hal_bind.c`).** Per-port glue that fills in the motor /
   encoder / IMU backend ops with real peripheral calls (direct Ambiq HAL on

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

## Supported platforms (ports)

The only OS-specific code is one small `platform.c` per target implementing the
`platform.h` contract. Adding a new OS is ~50-100 lines.

| Port | `post` (ISR-safe) | `dispatch` | lock | timebase |
|------|-------------------|-----------|------|----------|
| `ambiqsuite` (no-OS) | static ring + PRIMASK | super-loop drain | PRIMASK | STIMER (true µs) |
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


## Serial (UART) link

`core/src/bridge/serial.c` frames ARB topic messages over any byte stream
(0x7E + topic + len + payload + CRC16) so a remote host can stream telemetry out
and inject commands - the lightweight option when there is no IP network. The
codec is OS-agnostic; the AmbiqSuite no-OS UART wiring is
`port/ambiqsuite/transport_uart.c`. Subscribe `arb_uart_transport_stream` to the
topics you want to emit, and call `arb_uart_transport_poll()` from the loop.

## Tuning (compile-time)

| Macro | Default | Meaning |
|-------|---------|---------|
| `ARB_MAX_TOPICS` | 16 | advertised topics |
| `ARB_MAX_SUBSCRIBERS` | 32 | total subscribers |
| `ARB_MAX_SERVICES` | 16 | registered services |
| `ARB_MSG_MAX_SIZE` | 64 | largest message (bytes) |
| `ARB_QUEUE_DEPTH` | 16 | port queue depth |


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
