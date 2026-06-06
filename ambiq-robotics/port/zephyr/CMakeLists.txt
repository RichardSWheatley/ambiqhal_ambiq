# Ambiq Robotics Broker (ARB) - Zephyr build glue
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>

if(CONFIG_ARB)
  set(ARB_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)

  zephyr_library_named(arb)

  # OS-agnostic core
  zephyr_include_directories(${ARB_ROOT}/core/include)
  zephyr_library_sources(
    ${ARB_ROOT}/core/src/topic.c
    ${ARB_ROOT}/core/src/service.c
    ${ARB_ROOT}/core/src/hal/motor.c
    ${ARB_ROOT}/core/src/hal/encoder.c
    ${ARB_ROOT}/core/src/hal/imu.c
  )

  # Zephyr port
  zephyr_include_directories(${CMAKE_CURRENT_LIST_DIR})
  zephyr_library_sources(
    ${CMAKE_CURRENT_LIST_DIR}/platform.c
    ${CMAKE_CURRENT_LIST_DIR}/hal_bind.c
  )

  # Map Kconfig limits onto the core's compile-time constants.
  zephyr_library_compile_definitions(
    ARB_MAX_TOPICS=${CONFIG_ARB_MAX_TOPICS}
    ARB_MAX_SUBSCRIBERS=${CONFIG_ARB_MAX_SUBSCRIBERS}
    ARB_MAX_SERVICES=${CONFIG_ARB_MAX_SERVICES}
  )

  zephyr_library_sources_ifdef(CONFIG_ARB_MICRO_ROS_BRIDGE
    ${CMAKE_CURRENT_LIST_DIR}/bridge/micro_ros.c
  )
endif()
