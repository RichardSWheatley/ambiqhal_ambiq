# ARB makefile fragment - CMSIS-RTOS2 on Apollo (AmbiqSuite SDK projects).
#
# Uses the CMSIS-RTOS2 platform port for the broker and reuses the AmbiqSuite
# direct-HAL bindings for motor/encoder/IMU. Works with any CMSIS-RTOS2 RTOS
# (Keil RTX5, etc.) running on the Apollo510.
#
#   ARB_ROOT := ../../path/to/ambiq-robotics
#   include $(ARB_ROOT)/port/cmsis-rtos2/arb.mk
#   INCLUDES += $(ARB_INCLUDES)
#   SRC      += $(ARB_SRC)
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>

ARB_CMSIS_DIR := $(dir $(lastword $(MAKEFILE_LIST)))
ARB_ROOT      ?= $(ARB_CMSIS_DIR)/../..

ARB_INCLUDES := \
	$(ARB_ROOT)/core/include \
	$(ARB_ROOT)/port/ambiqsuite

ARB_SRC := \
	$(ARB_ROOT)/core/src/topic.c \
	$(ARB_ROOT)/core/src/service.c \
	$(ARB_ROOT)/core/src/hal/motor.c \
	$(ARB_ROOT)/core/src/hal/encoder.c \
	$(ARB_ROOT)/core/src/hal/imu.c \
	$(ARB_ROOT)/core/src/control/pid.c \
	$(ARB_ROOT)/core/src/control/diff_drive.c \
	$(ARB_ROOT)/core/src/bridge/jaus.c \
	$(ARB_ROOT)/core/src/bridge/serial.c \
	$(ARB_CMSIS_DIR)/platform.c \
	$(ARB_ROOT)/port/ambiqsuite/hal_bind.c
