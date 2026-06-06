# ARB NuttX sample - apps/ Makefile
#
# Drop this directory in your NuttX apps tree (e.g. apps/examples/arb_diffdrive)
# and point ARBDIR at the ambiq-robotics checkout. It compiles the OS-agnostic
# core, the NuttX broker port, and the AmbiqSuite HAL bindings into the app.
#
# SPDX-License-Identifier: Apache-2.0

include $(APPDIR)/Make.defs

# Location of the ambiq-robotics middleware (override on the command line).
ARBDIR ?= $(APPDIR)/../ambiq-robotics

PROGNAME  = $(CONFIG_ARB_DIFFDRIVE_PROGNAME)
PRIORITY  = $(CONFIG_ARB_DIFFDRIVE_PRIORITY)
STACKSIZE = $(CONFIG_ARB_DIFFDRIVE_STACKSIZE)
MODULE    = $(CONFIG_ARB_DIFFDRIVE)

CFLAGS += -I$(ARBDIR)/core/include
CFLAGS += -I$(ARBDIR)/port/ambiqsuite

MAINSRC = arb_diffdrive_main.c

CSRCS += $(ARBDIR)/core/src/topic.c
CSRCS += $(ARBDIR)/core/src/service.c
CSRCS += $(ARBDIR)/core/src/hal/motor.c
CSRCS += $(ARBDIR)/core/src/hal/encoder.c
CSRCS += $(ARBDIR)/core/src/hal/imu.c
CSRCS += $(ARBDIR)/core/src/control/pid.c
CSRCS += $(ARBDIR)/core/src/control/diff_drive.c
CSRCS += $(ARBDIR)/core/src/bridge/jaus.c
CSRCS += $(ARBDIR)/core/src/bridge/serial.c
CSRCS += $(ARBDIR)/port/nuttx/platform.c
CSRCS += $(ARBDIR)/port/ambiqsuite/hal_bind.c

include $(APPDIR)/Application.mk
