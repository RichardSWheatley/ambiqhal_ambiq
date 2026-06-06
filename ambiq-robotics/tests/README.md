# ARB host tests

OS-agnostic unit tests for the ARB core, built against the host platform port
(`port/host/platform.c`). They cover the broker (pub/sub, deferred dispatch,
multiple subscribers, error paths), services (req/resp, sequencing, not-found),
the HAL logic (motor scaling/inversion/deadband, encoder decode, IMU axis
remap), and the PID controller (clamping, anti-windup, convergence).

## Run with CMake/CTest

```bash
cmake -S . -B build -DARB_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

## Run directly with gcc

```bash
gcc -std=c11 -Wall -Wextra -I core/include \
    tests/test_arb.c port/host/platform.c \
    core/src/topic.c core/src/service.c \
    core/src/hal/motor.c core/src/hal/encoder.c core/src/hal/imu.c \
    core/src/control/pid.c \
    -lm -o /tmp/arb_tests && /tmp/arb_tests
```
