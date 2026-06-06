# ARB RIOT sample - application Makefile
#
# Build for an Apollo510 BOARD. Point RIOTBASE at your RIOT checkout and ARBDIR
# at the ambiq-robotics middleware.
#
#   make BOARD=<apollo510-board> RIOTBASE=/path/to/RIOT ARBDIR=/path/to/ambiq-robotics
#
# SPDX-License-Identifier: Apache-2.0

APPLICATION = arb_diffdrive
BOARD      ?= apollo510-evb
RIOTBASE   ?= $(CURDIR)/../../../RIOT
ARBDIR     ?= $(CURDIR)/../..

USEMODULE += xtimer

CFLAGS += -I$(ARBDIR)/core/include
CFLAGS += -I$(ARBDIR)/port/ambiqsuite

# Explicit source list (otherwise RIOT compiles only *.c in this directory).
SRC  = main.c
SRC += topic.c service.c motor.c encoder.c imu.c
SRC += pid.c diff_drive.c jaus.c serial.c
SRC += platform.c hal_bind.c

vpath %.c $(ARBDIR)/core/src \
          $(ARBDIR)/core/src/hal \
          $(ARBDIR)/core/src/control \
          $(ARBDIR)/core/src/bridge \
          $(ARBDIR)/port/riot \
          $(ARBDIR)/port/ambiqsuite

include $(RIOTBASE)/Makefile.include
