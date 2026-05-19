# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(weaver_scenarios)

target_sources(app PRIVATE
	src/main.c
	src/scen_imu_overrun.c
	src/scen_ble_deadline.c
	src/scen_ppg_latency.c
	src/scen_burst_shed.c
	src/scen_display_defer.c
)
