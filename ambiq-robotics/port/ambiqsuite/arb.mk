# Ambiq Robotics Broker (ARB) - makefile fragment for AmbiqSuite SDK projects.
#
# Include from an AmbiqSuite example Makefile:
#
#   ARB_ROOT := ../../path/to/ambiq-robotics
#   include $(ARB_ROOT)/port/ambiqsuite/arb.mk
#   INCLUDES += $(ARB_INCLUDES)
#   SRC      += $(ARB_SRC)
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>

ARB_PORT_DIR := $(dir $(lastword $(MAKEFILE_LIST)))
ARB_ROOT     ?= $(ARB_PORT_DIR)/../..

ARB_INCLUDES := \
	$(ARB_ROOT)/core/include \
	$(ARB_PORT_DIR)

ARB_SRC := \
	$(ARB_ROOT)/core/src/topic.c \
	$(ARB_ROOT)/core/src/service.c \
	$(ARB_ROOT)/core/src/hal/motor.c \
	$(ARB_ROOT)/core/src/hal/encoder.c \
	$(ARB_ROOT)/core/src/hal/imu.c \
	$(ARB_ROOT)/core/src/control/pid.c \
	$(ARB_ROOT)/core/src/control/diff_drive.c \
	$(ARB_PORT_DIR)/platform.c \
	$(ARB_PORT_DIR)/hal_bind.c
