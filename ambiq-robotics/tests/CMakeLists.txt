# ARB host unit tests
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>

add_executable(arb_tests
  test_arb.c
  ${CMAKE_CURRENT_LIST_DIR}/../port/host/platform.c
)

target_link_libraries(arb_tests PRIVATE arb_core m)
target_compile_options(arb_tests PRIVATE -Wall -Wextra)

add_test(NAME arb_tests COMMAND arb_tests)
