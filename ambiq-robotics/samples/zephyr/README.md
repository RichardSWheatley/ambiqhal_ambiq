# ARB Zephyr sample — differential-drive base

The same differential-drive node as the bare-metal sample, but using the Zephyr
device model for the HAL bindings and a dedicated dispatcher thread for delivery.

## Build

```
west build -b <your_apollo510_board> samples/zephyr
```

This directory (`ambiq-robotics/`) must be visible to Zephyr as a module — it
ships `zephyr/module.yml`. Add it under your west manifest's `projects:` or pass
it via `EXTRA_ZEPHYR_MODULES`.

## Required devicetree aliases

Provide a board overlay that defines:

| Alias | Node type |
|-------|-----------|
| `pwm-motor-l`, `pwm-motor-r` | PWM channel (`pwms` phandle-array) |
| `qdec-l`, `qdec-r` | quadrature decoder sensor device |
| `robot-imu` | 6-axis IMU sensor device |

Example overlay fragment:

```dts
/ {
    aliases {
        pwm-motor-l = &motor_l_pwm;
        pwm-motor-r = &motor_r_pwm;
        qdec-l = &qdec0;
        qdec-r = &qdec1;
        robot-imu = &icm42688;
    };
};
```

## JAUS

Set `CONFIG_ARB_JAUS_BRIDGE=y` to expose the node on a JAUS network over UDP
(the option pulls in networking). The sample then opens the bridge on UDP/3794
and a JAUS controller can drive it with `SetWrenchEffort` and receive
`ReportVelocityState`/`ReportLocalPose` — the same behavior as the host
`samples/jaus` node. Configure an IP for your board (DHCP or static) via the
usual Zephyr networking Kconfig.

## micro-ROS

Set `CONFIG_ARB_MICRO_ROS_BRIDGE=y` (uncomment in `prj.conf`) with the micro-ROS
Zephyr module available, then call `arb_micro_ros_bridge_init(&node, &executor)`
after setting up your rclc node/executor. The bridge maps:

```
ROS2 /cmd_vel  ->  ARB /cmd_vel
ARB  /odom     ->  ROS2 /odom
ARB  /imu      ->  ROS2 /imu
```
