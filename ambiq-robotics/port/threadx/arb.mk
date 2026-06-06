# ARB makefile fragment - Eclipse ThreadX on Apollo (AmbiqSuite SDK projects).
#
# Uses the ThreadX platform port for the broker and reuses the AmbiqSuite
# direct-HAL bindings for motor/encoder/IMU.
#
#   ARB_ROOT := ../../path/to/ambiq-robotics
#   include $(ARB_ROOT)/port/threadx/arb.mk
#   INCLUDES += $(ARB_INCLUDES)
#   SRC      += $(ARB_SRC)
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>

ARB_THREADX_DIR := $(dir $(lastword $(MAKEFILE_LIST)))
ARB_ROOT        ?= $(ARB_THREADX_DIR)/../..

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
	$(ARB_THREADX_DIR)/platform.c \
	$(ARB_ROOT)/port/ambiqsuite/hal_bind.c
