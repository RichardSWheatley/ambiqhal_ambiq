# ARB JAUS host sample
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>

add_executable(arb_jaus_sample
  main.c
  ${CMAKE_CURRENT_LIST_DIR}/../../port/host/platform.c
  ${CMAKE_CURRENT_LIST_DIR}/../../port/host/bridge/jaus_udp.c
)

target_include_directories(arb_jaus_sample PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/../../port/host/bridge
)
target_link_libraries(arb_jaus_sample PRIVATE arb_core m)
target_compile_options(arb_jaus_sample PRIVATE -Wall -Wextra)
