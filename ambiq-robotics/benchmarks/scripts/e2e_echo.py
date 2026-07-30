#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
"""Byte-true echo for the e2e_cmd_vel host-RTT benchmark.

Echoes every byte straight back, so frames (and their CRCs) return intact
and the board measures a full serial round trip.

Usage: e2e_echo.py PORT [BAUD]   (default baud 921600)
"""
import sys

import serial

port = sys.argv[1]
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 921600

with serial.Serial(port, baud, timeout=0.1) as s:
    print(f"echoing on {port} @ {baud}")
    while True:
        data = s.read(4096)
        if data:
            s.write(data)
