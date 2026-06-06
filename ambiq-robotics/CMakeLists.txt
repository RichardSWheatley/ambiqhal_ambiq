# Ambiq Robotics Broker (ARB) - top-level CMake (host / standalone builds).
#
# This drives host builds and the unit tests; firmware integrations use the
# per-target glue instead (port/zephyr/CMakeLists.txt, port/ambiqsuite/arb.mk).
#
#   cmake -S ambiq-robotics -B build -DARB_BUILD_TESTS=ON
#   cmake --build build
#   ctest --test-dir build --output-on-failure
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>

cmake_minimum_required(VERSION 3.20)
project(ambiq_robotics C)

option(ARB_BUILD_TESTS "Build the host unit tests" OFF)

add_subdirectory(core)

if(ARB_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
