# Ambiq Robotics Broker (ARB) - OS-agnostic core as a standalone CMake target.
#
# Usable from host unit tests or any CMake-based firmware build. The Zephyr build
# does NOT use this file (see port/zephyr/CMakeLists.txt); it compiles the same
# sources directly into a zephyr_library.
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>

add_library(arb_core STATIC
  src/topic.c
  src/service.c
  src/hal/motor.c
  src/hal/encoder.c
  src/hal/imu.c
  src/control/pid.c
  src/control/diff_drive.c
)

target_include_directories(arb_core PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_compile_features(arb_core PUBLIC c_std_11)
