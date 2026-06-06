# ARB Zephyr sample - differential-drive base.
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(arb_diffdrive)

target_sources(app PRIVATE src/main.c)
