# ARB sample: closed-loop differential drive (Apollo510 EVB)

Part of the [ARB Zephyr module](../../README.md). Everything runs on zbus; the
only way in or out of the board is the ARB serial transport on UART1.

```
cmd_vel (UART frame or local pub) ──► PID per wheel ──► PWM + DIR
encoders (GPIO IRQ, x4 quadrature) ──► arb_chan_encoder @ 50 Hz ──► UART
ICM-42688-P (I2C, 200 Hz)          ──► arb_chan_imu              ──► UART
wheel odometry (integrated)        ──► arb_chan_odom    @ 50 Hz  ──► UART
node heartbeat                     ──► arb_chan_heartbeat @ 2 Hz ──► UART
arb_chan_estop (UART frame in)     ──► motors brake, node → ESTOP
```

## Hardware

| Item | Notes |
|---|---|
| Apollo510 EVB | the target |
| DRV8833 or TB6612FNG carrier (Pololu #2130 / #713) | dual H-bridge, jumpered. MikroE DC Motor Click is single-motor (muxed DRV8833) — wrong topology for two wheels |
| 2× N20 gear motor with quadrature encoder | 6 V, 360 CPR line count assumed (`ENC_CPR`) |
| MikroE 6DOF IMU 14 Click (MIKROE-4237) | ICM-42688-P; drops into the EVB mikroBUS socket (J14/J15). COMM SEL jumpers to I2C, ADDR SEL low → 0x68. Any other ICM-42688-P breakout on I2C works too |
| USB-UART adapter, 3.3 V (CP2102/FTDI) | the ARB link to the host |
| Motor supply | 2S LiPo or 4×AA to H-bridge VM |
| Jumper wires, breadboard | |

Chassis alternative: a Pololu Romi kit (chassis + 120:1 gearmotors), the Romi
Motor Driver and Power Distribution Board (#3543, dual DRV8838 — same PWM+DIR
control as this sample), and the Romi Encoder Pair Kit (#3542, 1440
counts/wheel-rev at x4, matching `ENC_CPR 360`) replaces the H-bridge, motors,
encoders, and battery wiring in one stack — leaving just the IMU Click and the
USB-UART adapter.

### Wiring

| Signal | Apollo510 GPIO | Goes to |
|---|---|---|
| Motor L PWM | 12 (CTIMER0 out0) | AIN1 |
| Motor L DIR | 13 | AIN2 |
| Motor R PWM | 18 (CTIMER1 out0) | BIN1 |
| Motor R DIR | 19 | BIN2 |
| Encoder L A / B | 24 / 25 | encoder outputs (3.3 V) |
| Encoder R A / B | 26 / 27 | encoder outputs (3.3 V) |
| IMU SDA / SCL | 5 / 6 (IOM0) | jumpered breakout; for the Click in the mikroBUS socket, match the overlay's I2C node to the IOM the socket routes (QSG Fig. 6) |
| ARB link TX / RX | 48 / 49 (UART1) | adapter RX / TX, 921600 8N1 |
| Console | EVB USB (UART0) | Zephyr log output |

The overlay assumes IOM0 on GPIO5/6 for I2C; if the mikroBUS socket routes a
different IOM, point the overlay's I2C node and pinctrl at that instance.
Pin changes go in [`boards/apollo510_evb.overlay`](boards/apollo510_evb.overlay);
geometry and limits (`WHEEL_BASE_M`, `WHEEL_RADIUS_M`, `ENC_CPR`,
`MAX_WHEEL_RADPS`) at the top of [`src/main.c`](src/main.c).

## Software (host side)

- Zephyr workspace with this module registered (`west`, Zephyr SDK)
- A serial terminal for the console (UART0): `west espressif monitor`-style
  tools, PuTTY, or `minicom` — 115200 8N1
- Python 3 + `pyserial` for the ARB link (UART1) — the frames are binary, a
  plain terminal shows garbage

Build and flash:

```
west build -b apollo510_evb ambiq-robotics/samples/diff_drive
west flash
```

## What to expect

Console (UART0) on boot:

```
[00:00:00.xxx] <inf> arb: node 'diff_drive' created (id=1)
[00:00:00.xxx] <inf> arb: node 'diff_drive' -> READY
[00:00:00.xxx] <inf> arb: node 'diff_drive' -> ACTIVE
[00:00:00.xxx] <inf> arb: transport up on uart@...
[00:00:00.xxx] <inf> diff_drive: ARB diff-drive sample (zbus)
```

`WHO_AM_I 0x00` instead means the IMU wiring/address is wrong; the sample runs
without it.

ARB link (UART1), immediately and unprompted:

| Frame | Topic id | Payload | Rate |
|---|---|---|---|
| heartbeat | 10 | `arb_heartbeat_t` (node=1, state=2 ACTIVE) | 2 Hz |
| odom | 2 | `arb_odom_t` | 50 Hz |
| encoder | 5 | `arb_encoder_msg_t` ×2 (ids 0, 1) | 50 Hz each |
| imu | 3 | `arb_imu_t` | 200 Hz |

With motors idle, odom stays ~0 and encoder counts are static. Spin a wheel by
hand: its `count`/`velocity_rad_s` move and odom integrates.

## Wire format

```
0x7E | topic(u16 LE) | len(u16 LE) | payload[len] | crc16(u16 LE)
```

CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over topic+len+payload. Payloads
are the little-endian structs from [`arb/msg.h`](../../include/arb/msg.h) —
naturally aligned, no padding on Cortex-M. Every payload starts with the 16-byte
header: `stamp_us(u64) seq(u32) type(u16) source(u16)`.

`arb_twist_t` (topic 1, 40 bytes): header, then `linear.xyz` and `angular.xyz`
as 6 floats. Drive with `linear.x` (m/s) and `angular.z` (rad/s).

## Driving it from a host

```python
#!/usr/bin/env python3
# pip install pyserial
import serial, struct, time

def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc

def frame(topic, payload):
    body = struct.pack('<HH', topic, len(payload)) + payload
    return b'\x7e' + body + struct.pack('<H', crc16(body))

def twist(vx, wz, seq=0):
    hdr = struct.pack('<QIHH', int(time.time() * 1e6), seq, 1, 0)
    return hdr + struct.pack('<6f', vx, 0, 0, 0, 0, wz)

s = serial.Serial('COM5', 921600)          # your adapter port

s.write(frame(1, twist(0.15, 0.0)))        # forward, 0.15 m/s
time.sleep(2)
s.write(frame(1, twist(0.0, 0.0)))         # stop

# read one frame back
def read_frame(s):
    while s.read(1) != b'\x7e':
        pass
    topic, ln = struct.unpack('<HH', s.read(4))
    payload = s.read(ln)
    rx_crc, = struct.unpack('<H', s.read(2))
    ok = rx_crc == crc16(struct.pack('<HH', topic, ln) + payload)
    return topic, payload, ok

topic, payload, ok = read_frame(s)
print('topic', topic, 'len', len(payload), 'crc', ok)
```

E-stop from the host: send topic 11 with an `arb_estop_t` payload — header
with `type=11`, then `source(u16) reason(u8)`, zero-padded to the struct size
of 24 bytes (payload length must equal `sizeof` the C struct, tail padding
included; the same applies to any inbound frame). Motors brake, the node
latches ESTOP, and the board broadcasts the stop back out. Recovery is a
reset.

## Safety

Wheels off the ground for first power-up. `MAX_WHEEL_RADPS` clamps setpoints,
but PID gains (`arb_pid_init` calls in `main()`) are starting values for N20s —
retune for your motors.
