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
│   │   └── hal/           # robotics-oriented peripheral abstractions
│   │       ├── motor.h    # motor_set_duty(), ...
│   │       ├── encoder.h  # encoder_read() -> angle + velocity
│   │       └── imu.h      # imu_sample()
│   └── src/               # topic.c, service.c, hal/*.c
├── port/
│   ├── ambiqsuite/        # no-OS: super-loop dispatch + direct Ambiq HAL
│   └── zephyr/            # Zephyr module: k_msgq dispatch, device API, bridge
└── samples/
    ├── ambiqsuite/        # diff-drive base, super-loop
    └── zephyr/            # diff-drive base, dispatcher thread
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

## Tuning (compile-time)

| Macro | Default | Meaning |
|-------|---------|---------|
| `ARB_MAX_TOPICS` | 16 | advertised topics |
| `ARB_MAX_SUBSCRIBERS` | 32 | total subscribers |
| `ARB_MAX_SERVICES` | 16 | registered services |
| `ARB_MSG_MAX_SIZE` | 64 | largest message (bytes) |
| `ARB_QUEUE_DEPTH` | 16 | port queue depth |

On Zephyr these map to `CONFIG_ARB_*` Kconfig options.

## License

BSD-3-Clause / Apache-2.0 (matching the surrounding AmbiqSuite HAL module).
