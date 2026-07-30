# Ambiq Robotics Broker (ARB) - Zephyr module
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>

if(CONFIG_ARB)

zephyr_include_directories(include)

zephyr_library_named(arb)
zephyr_library_sources(
  src/topics.c
  src/node.c
  src/serial.c
  src/time.c
  src/control/pid.c
  src/control/diff_drive.c
)
zephyr_library_sources_ifdef(CONFIG_ARB_TRANSPORT src/transport_core.c)
zephyr_library_sources_ifdef(CONFIG_ARB_TRANSPORT_SERIAL src/transport_serial.c)
zephyr_library_sources_ifdef(CONFIG_ARB_TRANSPORT_UDP src/transport_udp.c)

endif()
