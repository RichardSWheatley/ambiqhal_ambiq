# Ambiq Robotics Broker (ARB) — Zephyr module

Robotics building blocks for Zephyr, built on **zbus**. ARB does not ship its
own pub/sub: every topic is a statically-defined zbus channel. What ARB adds is
the part Zephyr doesn't have:

1. **Message vocabulary** (`arb/msg.h`) — flat, packed robotics message structs
   with type ids (twist, odom, imu, encoder, heartbeat, e-stop, …), the
   MCU-sized analog of ROS 2 `.msg` files.
2. **Node lifecycle + e-stop** (`arb/node.h`) — JAUS / ROS 2 managed-node style
   state machine (`INIT → READY → ACTIVE`, `ERROR`, `ESTOP`, `SHUTDOWN`) with
   heartbeats on `arb_chan_heartbeat` and a one-call
   `arb_node_estop_all()` that shuts down every active node and broadcasts on
   `arb_chan_estop`.
3. **Serial transport** (`arb/transport.h`) — bridges exported channels over a
   UART (`0x7E | topic | len | payload | crc16`, wire-compatible with the
   multi-OS ARB tree on `wheatley-robotics-updates`), so two boards — or a
   board and a host — share one topic space.
4. **micro-ROS bridge** (`bridge/micro_ros.c`, optional) — mirrors the
   channels into a ROS 2 graph.

Plus small pure-math control helpers (`arb/control/`): PID and diff-drive
kinematics.

## Layout

```
include/arb/        msg.h topics.h node.h serial.h transport.h err.h control/
src/                topics.c node.c serial.c transport.c control/
bridge/             micro_ros.c
samples/diff_drive/ closed-loop base on the Apollo510 EVB
tests/arb/          ztest suite (native_sim)
zephyr/module.yml
```

## Use

Add as a Zephyr module and enable:

```
CONFIG_ARB=y
CONFIG_ARB_TRANSPORT=y          # needs: chosen { arb,uart = &uartX; }
CONFIG_ARB_MICRO_ROS_BRIDGE=y   # needs micro_ros_zephyr
```

Publish/observe with plain zbus:

```c
#include "arb/topics.h"

arb_twist_t t = {0};
arb_header_init(&t.header, ARB_MSG_TWIST, 0, stamp_us(), seq++);
t.linear.x = 0.3f;
zbus_chan_pub(&arb_chan_cmd_vel, &t, K_MSEC(5));
```

## Sample

```
west build -b apollo510_evb ambiq-robotics/samples/diff_drive
```

Closed-loop differential drive: PID per wheel from quadrature encoders,
ICM-42688 IMU at 200 Hz, odometry at 50 Hz, everything on zbus, telemetry and
`cmd_vel` bridged over UART1.

## Tests

```
west twister -p native_sim -T ambiq-robotics/tests
```
