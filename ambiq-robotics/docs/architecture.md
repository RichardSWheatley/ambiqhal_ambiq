# ARB Architecture

The Ambiq Robotics Broker (ARB) is a static, MCU-sized robotics middleware:
ROS2/JAUS-style **publish/subscribe** and **request/response services** plus a
robotics **HAL abstraction**, with the same application code running on
bare-metal AmbiqSuite or on an RTOS, and optional bridges to ROS2 (micro-ROS)
and JAUS networks.

## Design principles

- **Static everything.** No `malloc`, no IDL codegen. Every limit is a
  compile-time constant (`ARB_MAX_TOPICS`, `ARB_MSG_MAX_SIZE`, ...).
- **One OS seam.** The OS-agnostic core only ever calls the six functions in
  `arb/platform.h`. Everything else (topics, services, control, bridges) is
  portable C.
- **ISR-safe publish, deferred delivery.** Publishing copies a message into a
  port queue (safe from an ISR); subscribers run later from a normal context.
- **POD messages.** Flat structs with an integer type id, copied by value.

## Layers

```
            application (samples, your robot code)
                 |            ^
      publish/call|            | subscribe/handler callbacks
                 v            |
   +-------------------------------------------+
   |  core (OS-agnostic)                       |
   |   topic registry · service registry       |
   |   HAL abstraction (motor/encoder/imu)     |
   |   control (pid, diff_drive)               |
   |   bridges (jaus codec, serial framer)     |
   +-------------------------------------------+
                 |   arb/platform.h  (the seam)
                 v
   +-------------------------------------------+
   |  port (one per OS)                         |
   |   time · post/dispatch queue · lock        |
   |   HAL bindings (peripheral I/O)            |
   |   transports (UDP, UART, micro-ROS)        |
   +-------------------------------------------+
                 |
                 v
        AmbiqSuite HAL / Zephyr / FreeRTOS / ...
```

## Message flow

```
arb_topic_publish(topic, msg)            [any context, incl. ISR]
      -> arb_platform_post(...)          copy into the port queue
            ...                          (returns immediately)
arb_platform_dispatch()                  [task / super-loop context]
      -> arb_topic_deliver(topic, msg)   fan out under the broker lock
            -> subscriber_a(msg)
            -> subscriber_b(msg)
```

Services are synchronous: `arb_service_call()` runs the registered handler in the
caller's context and returns its response, carrying a sequence id for tracing.

## HAL abstraction

Each device class (`motor`, `encoder`, `imu`) defines a small backend-ops vtable.
The portable core implements the *logic* (clamping/deadband/inversion for
motors, tick→angle/velocity for encoders, axis remap + message packing for IMUs);
the port fills in the ops with real peripheral calls. The same `arb_motor_t`
behaves identically whether it is driven by an Ambiq TIMER (bare-metal) or a
Zephyr PWM device.

## Control

`control/pid.c` is an allocation-free PID (output clamp, integral anti-windup,
derivative-on-measurement). `control/diff_drive.c` converts between a body twist
and per-wheel angular velocities. The samples compose them: `/cmd_vel` → wheel
setpoints → feedforward+PID against encoder feedback → motor duty, and the
measured wheel velocities → `/odom`.

## Bridges

Bridges connect ARB topics to the outside world and are transport-agnostic where
possible:

- **micro-ROS** (`port/zephyr/bridge/micro_ros.c`) — mirrors topics onto a ROS2
  graph.
- **JAUS** (`core/src/bridge/jaus.c`) — SAE AS-4 codec + topic mapping, with UDP
  transports for host and Zephyr and an OpenJAUS SDK adapter.
- **Serial** (`core/src/bridge/serial.c`) — framed UART/SPI link for telemetry
  and commands when there is no IP network.

See [porting.md](porting.md) to bring ARB up on a new OS.
