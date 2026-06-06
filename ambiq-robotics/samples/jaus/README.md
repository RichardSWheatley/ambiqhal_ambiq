# ARB JAUS sample — JAUS-driven diff-drive node (host-runnable)

Wires the JAUS bridge to a (simulated) differential-drive robot so the whole
pipeline runs on a PC with no hardware:

```
JAUS SetWrenchEffort (UDP) --> bridge --> /cmd_vel --> robot sim
robot sim --> /odom --> bridge --> JAUS ReportVelocityState + ReportLocalPose
```

The robot is a kinematic integrator; point a JAUS controller, the OpenJAUS
adapter, or a second instance at it.

## Build & run

```bash
cmake -S ambiq-robotics -B build -DARB_BUILD_SAMPLES=ON
cmake --build build
./build/samples/jaus/arb_jaus_sample            # bind 3794, peer 127.0.0.1:3795
./build/samples/jaus/arb_jaus_sample 3794 127.0.0.1 3795 5   # run 5 s then exit
```

It prints pose at ~1 Hz. Send it a JAUS `SetWrenchEffort` (command code `0x0405`,
2-byte presence vector with bits 0 and 5 set, then two scaled-int16 effort
percentages over [-100, 100]) wrapped in the transport framing from
`port/host/bridge/jaus_udp.c`, and watch the pose move and reports come back.

> The UDP transport here is a simple stand-in for AS5669A JUDP (fine for
> ARB-to-ARB and bench testing). For a real JAUS network, build the OpenJAUS
> adapter (`port/host/bridge/open_jaus.c`, `-DARB_WITH_OPENJAUS`) instead.
