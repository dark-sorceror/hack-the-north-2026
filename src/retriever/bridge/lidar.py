"""Slamtec RPLIDAR A2M12 on the Pi: a small driver on pyserial, and a fake.

This is a SAFETY BUBBLE sensor, not a mapping sensor. Navigation stays on
AprilTags + wheel odometry; the lidar only lets the Pi refuse to drive into
things (bridge/safety.py). No SLAM here, on purpose.

Import cost: stdlib only. pyserial and gpiozero are imported lazily, inside the
functions that open the hardware, so the bridge (and every laptop-side test)
imports this module with nothing installed.

WIRING (primary path: no USB adapter, the lidar's own 5-wire XH2.54-5P cable
straight onto the Pi 5 header; wire it with the Pi POWERED OFF, the lidar's
input capacitor pulls up to 2.5 A inrush [DS]):

    lidar wire        Pi 5 header
    red    VCC   ->   pin 2  (5 V)
    black  GND   ->   pin 6  (GND)
    yellow TX    ->   pin 10 (GPIO15, UART0 RXD)
    green  RX    ->   pin 8  (GPIO14, UART0 TXD)
    blue MOTOCTL ->   pin 12 (GPIO18), driven high by GpioMotor

  Port: /dev/ttyAMA0 after `dtoverlay=uart0-pi5` in /boot/firmware/config.txt
  [P4]. Do NOT set enable_uart=1 on a Pi 5: with no debug cable it routes kernel
  log output to GPIO14/15 [P2], i.e. into the lidar's RX.

  MOTOCTL has an internal pull-down [DS]: a floating or low pin means motor OFF,
  so an unconfigured GPIO18 at boot, a crashed process and a released pin all
  stop the motor. A DC high level runs the motor "at the highest speed" [DS];
  speed control needs a 24.5-25.5 kHz PWM [DS], which lgpio's software PWM can
  not produce, so this driver only switches it on and off.

  Secondary path: Slamtec's USB adapter (CP2102 [KIT], /dev/ttyUSB*). The
  adapter drives MOTOCTL itself. On OUR adapter MOTOCTL follows DTR (V): the
  motor runs whenever the port is CLOSED (DTR deasserted) and stops only while
  a program holds the port open with DTR asserted. So on the adapter "stop the
  motor on exit" cannot last past the exit: unplug the adapter to stop it for
  good. The SDK additionally supports adapters that take SET_MOTOR_PWM
  [SDK]; AdapterMotor tries that first and falls back to DTR. On the robot the
  GPIO path has none of this: MOTOCTL is pulled down, off unless driven.

FACTS THE CODE RELIES ON (V = verified on our unit by the team, 2026-09-19,
over a USB adapter with rplidar-roboticia and Slamtec's SDK v2.1.0;
D = datasheet/protocol/SDK source, not yet seen on our hardware):

  V  256000 baud, 8N1. GET_INFO: model 44 (0x2C, major 2 = A2 family),
     firmware 1.32, hardware 6. GET_HEALTH: Good, 0.
  V  Scan modes (Slamtec SDK getAllSupportedScanModes on our unit): Standard
     252 us/sample (3968/s), Express 126 us, Boost 63 us, Sensitivity 63 us
     (the typical/default one), Stability 100 us; max distance 16 m in all.
  V  Standard SCAN (0x20), the mode this driver uses: ~130 VALID points per
     revolution at ~7.5-11.7 rev/s: rays ~1 deg apart, ~60% of them return
     nothing. First revolution ~1.4 s after motor start. A 2 cm chair leg can
     fall between valid returns; see the unknown-sector rule in safety.py.
  D  Range 0.2 m minimum, 12 m white / 10 m black targets [DS]. Anything
     closer than 0.2 m to the lidar is invisible.
  D  Scan rate 5-15 Hz, 10 Hz typical [DS]. Speed at MOTOCTL DC high:
     "highest speed", number not given [DS]. UNVERIFIED: measure it with
     scripts/lidar_check.py.
  D  The dense modes cannot use standard packets at 256000 baud: 15873
     samples/s fits the link (25.6 kB/s) only as ultra-capsules (132 bytes per
     96 samples, 0x84 [SDK sl_lidar_cmd.h]). Not implemented here.
  D  Angles increase CLOCKWISE seen from above, 0 deg points away from the
     cable ("interface lead") [DS fig 2-4][PROTO fig 4-7]. This module turns
     them into the usual counter-clockwise radians at the parser, once.
  D  Power 4.9-5.2 V, <=50 mV ripple, 450 mA typ / 600 mA max running, up to
     1.5 A at start-up, 2.5 A inrush [DS]. From the Pi's 5 V pin this bypasses
     the 600 mA USB limit but not the PSU: use the 5 V / 5 A supply.
  D  RX is a "current control type" input, logic high >= 1.6 V [DS]: the Pi's
     3.3 V TXD drives it directly.

Protocol (standard scan only; express/boost need capsule decoding and are not
implemented, see the notes at RPLidarLink):

  request   A5 cmd                                    no payload
            A5 cmd len payload.. xor(A5,cmd,len,payload..)   cmd & 0x80  [PROTO p6][SDK codec]
  response  A5 5A len(30 bits)|mode(2 bits) (little-endian u32) type   [PROTO p7]
  SCAN      A5 20 -> A5 5A 05 00 00 40 81, then 5-byte packets:          [PROTO p15]
              b0 = quality(6) | !S | S      S: first packet of a revolution
              b1 = angle_q6[6:0] << 1 | C   C: check bit, always 1
              b2 = angle_q6[14:7]           angle = angle_q6 / 64 deg
              b3,b4 = distance_q2 (LE)      distance = q2 / 4 mm, 0 = no return
  STOP      A5 25, no response, wait >= 1 ms                             [PROTO p13]
  RESET     A5 40, no response, wait >= 2 ms (then the core reboots)     [PROTO p14]
  GET_INFO  A5 50 -> 20 bytes: model, fw_minor, fw_major, hw, serial[16] [PROTO p33]
  GET_HEALTH A5 52 -> 3 bytes: status (0 good, 1 warning, 2 error), code u16 LE [PROTO p35][SDK]
  GET_SAMPLERATE A5 59 -> 4 bytes: us per sample, standard and express u16 LE [PROTO p36]
  SET_MOTOR_PWM  A5 F0 02 pwm(u16 LE) xor   adapter only, 0..1023    [SDK][RPLIDAR-PY]
  GET_ACC_BOARD_FLAG A5 FF 04 00000000 xor -> u32, bit 0 = PWM motor ctl [SDK]

Standard packets carry no checksum: the only integrity checks are S != !S,
C == 1 [PROTO p16], and angle_q6 < 360*64. A packet that fails them is a
framing slip: drop ONE byte and try again, the resync the SDK's
handler_normalnode.cpp also does. quality == 0 or distance == 0 is a valid
packet meaning "no return here" and is left out of the scan.

Sources:
  [DS]    Slamtec LD310 RPLIDAR A2M12 datasheet v1.0 (2022-04-08).
          https://bucket-download.slamtec.com/f65f8e37026796c56ddd512d33c7d4308d9edf94/LD310_SLAMTEC_rplidar_datasheet_A2M12_v1.0_en.pdf
  [PROTO] Slamtec LR001 RPLIDAR interface protocol v2.1 (2019-03-28).
          http://bucket.download.slamtec.com/ccb3c2fc1e66bb00bd4370e208b670217c8b55fa/LR001_SLAMTEC_rplidar_protocol_v2.1_en.pdf
  [SDK]   github.com/Slamtec/rplidar_sdk @ 99478e5 (sdk/include/sl_lidar_cmd.h,
          sl_lidar_protocol.h, src/sl_lidar_driver.cpp setMotorSpeed /
          checkMotorCtrlSupport / stop, src/sl_lidarprotocol_codec.cpp,
          src/dataunpacker/unpacker/handler_normalnode.cpp,
          src/arch/linux/net_serial.cpp "Clear the DTR bit to let the motor spin").
  [ROS]   github.com/Slamtec/rplidar_ros @ 24cc9b6: launch/rplidar_a2m12_launch.py
          serial_baudrate 256000 (a2m8: 115200); scripts/rplidar.rules
          USB id 10c4:ea60 (Silicon Labs CP210x).
  [KIT]   Slamtec LM310 RPLIDAR A2 development kit manual v1.1: adapter uses a
          CP2102; baud dial switch 115200/256000; MOTOCTL pull-down, active high.
  [RPLIDAR-PY] github.com/SkoltechRobotics/rplidar @ fff48d4: DEFAULT_MOTOR_PWM 660.
  [P2]    Raspberry Pi docs, configuration/interfaces.adoc "Configure UARTs"
          (raspberrypi/documentation @ ecd7a81): Pi 5 primary UART is the debug
          header; enable_uart=1 without a debug cable sends kernel logging to
          GPIO14/15; /dev/ttyAMA0 is UART0.
  [P4]    raspberrypi/linux rpi-6.12.y arch/arm/boot/dts/overlays/README:
          "uart0-pi5: Enable uart 0 on GPIOs 14-15. Pi 5 only."
"""

from __future__ import annotations

import glob
import logging
import math
import os
import random
import struct
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Iterable, Iterator, Protocol, Sequence

log = logging.getLogger("retriever.bridge.lidar")

TAU = 2.0 * math.pi

# ---------------------------------------------------------------- protocol constants

DEFAULT_PORT = "/dev/ttyAMA0"      # Pi 5 UART0 on GPIO14/15, after dtoverlay=uart0-pi5
DEFAULT_BAUD = 256000              # A2M12 [DS], verified on our unit. (A2M8: 115200.)
DEFAULT_ADAPTER_PWM = 660          # [RPLIDAR-PY]; the SDK falls back to 600
MAX_ADAPTER_PWM = 1023

SYNC1, SYNC2 = 0xA5, 0x5A
CMD_STOP = 0x25
CMD_RESET = 0x40
CMD_SCAN = 0x20
CMD_GET_INFO = 0x50
CMD_GET_HEALTH = 0x52
CMD_GET_SAMPLERATE = 0x59
CMD_SET_MOTOR_PWM = 0xF0
CMD_GET_ACC_BOARD_FLAG = 0xFF
CMD_HAS_PAYLOAD = 0x80

ANS_DEVINFO = 0x04
ANS_DEVHEALTH = 0x06
ANS_MEASUREMENT = 0x81
ANS_SAMPLE_RATE = 0x15
ANS_ACC_BOARD_FLAG = 0xFF

PACKET_LEN = 5
MAX_ANGLE_Q6 = 360 * 64
HEALTH_NAMES = {0: "good", 1: "warning", 2: "error"}

MIN_RANGE_M = 0.20                 # [DS]; closer targets return nothing useful
MAX_RANGE_M = 12.0                 # [DS], white target


class LidarError(RuntimeError):
    """The lidar said something we did not expect, or nothing at all."""


# ---------------------------------------------------------------- packets


def request(cmd: int, payload: bytes = b"") -> bytes:
    """One request packet. Commands with bit 7 set carry a length, the payload
    and an XOR checksum over every byte before it [PROTO p6]; the others are
    two bytes, and a payload for them is a bug."""
    if not 0 <= cmd <= 0xFF:
        raise ValueError(f"command {cmd!r} is not a byte")
    if not cmd & CMD_HAS_PAYLOAD:
        if payload:
            raise ValueError(f"command 0x{cmd:02X} takes no payload")
        return bytes((SYNC1, cmd))
    if len(payload) > 255:
        raise ValueError("payload longer than 255 bytes")
    body = bytes((SYNC1, cmd, len(payload))) + bytes(payload)
    checksum = 0
    for b in body:
        checksum ^= b
    return body + bytes((checksum,))


@dataclass(frozen=True)
class Descriptor:
    length: int      # bytes per data response
    mode: int        # 0 single response, 1 multiple (a stream)
    dtype: int


def parse_descriptor(raw: bytes) -> Descriptor:
    if len(raw) != 7 or raw[0] != SYNC1 or raw[1] != SYNC2:
        raise LidarError(f"bad response descriptor {bytes(raw).hex(' ')}")
    word = struct.unpack("<I", bytes(raw[2:6]))[0]
    return Descriptor(length=word & 0x3FFFFFFF, mode=word >> 30, dtype=raw[6])


@dataclass(frozen=True)
class DeviceInfo:
    model: int
    firmware: tuple[int, int]    # (major, minor)
    hardware: int
    serial: str

    def __str__(self) -> str:
        return (f"model {self.model} (0x{self.model:02X}), firmware "
                f"{self.firmware[0]}.{self.firmware[1]:02d}, hardware {self.hardware}, "
                f"serial {self.serial}")


def parse_info(raw: bytes) -> DeviceInfo:
    if len(raw) < 20:
        raise LidarError(f"GET_INFO answer is {len(raw)} bytes, expected 20")
    return DeviceInfo(model=raw[0], firmware=(raw[2], raw[1]), hardware=raw[3],
                      serial=bytes(raw[4:20]).hex().upper())


@dataclass(frozen=True)
class Health:
    status: int        # 0 good, 1 warning, 2 error (= protection stop)
    error_code: int

    @property
    def name(self) -> str:
        return HEALTH_NAMES.get(self.status, f"unknown({self.status})")

    def __str__(self) -> str:
        return f"{self.name} (code {self.error_code})"


def parse_health(raw: bytes) -> Health:
    if len(raw) < 3:
        raise LidarError(f"GET_HEALTH answer is {len(raw)} bytes, expected 3")
    # Little-endian, like every multi-byte field [PROTO p6][SDK struct]. The
    # SkoltechRobotics library reads it big-endian; the SDK is authoritative.
    return Health(status=raw[0], error_code=raw[1] | (raw[2] << 8))


@dataclass(frozen=True)
class Measurement:
    start: bool          # first packet of a new revolution
    quality: int         # 0..63; 0 = no return
    angle_deg: float     # RPLIDAR convention: CLOCKWISE from the 0 mark, [0, 360)
    range_mm: float      # 0 = no return

    @property
    def valid(self) -> bool:
        return self.quality > 0 and self.range_mm > 0.0


def parse_measurement(pkt: bytes | bytearray | Sequence[int]) -> Measurement | None:
    """One 5-byte standard-scan packet, or None if its check bits say it is a
    framing slip rather than a packet."""
    b0, b1, b2, b3, b4 = pkt[0], pkt[1], pkt[2], pkt[3], pkt[4]
    s = b0 & 0x01
    s_inv = (b0 >> 1) & 0x01
    if s == s_inv or not b1 & 0x01:
        return None
    angle_q6 = (b1 >> 1) | (b2 << 7)
    if angle_q6 >= MAX_ANGLE_Q6:
        return None
    return Measurement(start=bool(s), quality=b0 >> 2, angle_deg=angle_q6 / 64.0,
                       range_mm=(b3 | (b4 << 8)) / 4.0)


def encode_measurement(start: bool, quality: int, angle_deg: float, range_mm: float) -> bytes:
    """The inverse of parse_measurement. Tests and the byte-level fake use it."""
    q = max(0, min(63, int(quality)))
    angle_q6 = int(round(angle_deg * 64.0)) % MAX_ANGLE_Q6
    dist_q2 = max(0, min(0xFFFF, int(round(range_mm * 4.0))))
    b0 = (q << 2) | (0 if start else 0x02) | (0x01 if start else 0)
    return bytes((b0, ((angle_q6 & 0x7F) << 1) | 0x01, angle_q6 >> 7,
                  dist_q2 & 0xFF, dist_q2 >> 8))


class MeasurementParser:
    """Byte stream -> Measurements, resynchronising after a slip.

    Bytes arrive in arbitrary chunks. Five bytes that fail the check bits mean
    we are not on a packet boundary: drop one byte and look again. `bad` counts
    dropped bytes; a steady trickle is noise on the wire, a flood is the wrong
    baud rate.

    The check bits are weak (two bits and a range), so random bytes pass them
    about one time in ten. Out of sync (at the start, and after any bad byte)
    a packet is therefore accepted only if the five bytes after it are a valid
    packet too; in sync, one at a time. A tested case: five garbage bytes
    before the stream produced a phantom return without this.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self._synced = False
        self.good = 0
        self.bad = 0

    def reset(self) -> None:
        self._buf.clear()
        self._synced = False

    def feed(self, data: bytes) -> list[Measurement]:
        self._buf += data
        out: list[Measurement] = []
        buf = self._buf
        i = 0
        n = len(buf)
        while n - i >= PACKET_LEN:
            if not self._synced:
                if n - i < 2 * PACKET_LEN:
                    break                       # need the next packet to confirm
                m = parse_measurement(buf[i:i + PACKET_LEN])
                after = buf[i + PACKET_LEN:i + 2 * PACKET_LEN]
                if m is None or parse_measurement(after) is None:
                    i += 1
                    self.bad += 1
                    continue
                self._synced = True
            else:
                m = parse_measurement(buf[i:i + PACKET_LEN])
                if m is None:
                    self._synced = False
                    i += 1
                    self.bad += 1
                    continue
            out.append(m)
            self.good += 1
            i += PACKET_LEN
        del buf[:i]
        return out


# ---------------------------------------------------------------- scans


@dataclass(frozen=True)
class LidarScan:
    """One revolution.

    points are (angle_rad, range_m, quality) in the SENSOR frame, angles
    counter-clockwise from the lidar's 0 mark (the side away from its cable),
    in [0, 2*pi). Only valid returns are kept. `t` is the clock time the
    revolution was complete: compare it with the same clock (time.monotonic
    unless told otherwise).
    """

    t: float
    points: tuple[tuple[float, float, int], ...]
    rev_hz: float | None = None
    n_raw: int = 0                  # packets in the revolution, returns or not
    # Every sample of the revolution, valid or not, counted per SAMPLE_BIN_DEG
    # bin of sensor angle (same frame as points). With the valid points this
    # says how much of each direction the lidar actually SAW: a ray that
    # returns nothing is open space, a dark or shiny surface, or something
    # closer than 0.2 m, and the bubble treats it as unknown, not clear.
    samples: tuple[int, ...] = ()

    def age(self, now: float) -> float:
        return now - self.t


SAMPLE_BIN_DEG = 2.0
N_SAMPLE_BINS = 180


def sensor_angle(angle_deg_cw: float) -> float:
    """RPLIDAR's clockwise degrees -> counter-clockwise radians in [0, 2*pi)."""
    return (-math.radians(angle_deg_cw)) % TAU


def sample_bin(angle_rad: float) -> int:
    """Sensor-frame angle (CCW radians) -> index into LidarScan.samples."""
    return int((angle_rad % TAU) / math.radians(SAMPLE_BIN_DEG)) % N_SAMPLE_BINS


class ScanAssembler:
    """Measurements -> one LidarScan per revolution.

    A revolution ends when the next one starts: a packet with S set, or, if that
    packet was lost on the wire, the angle wrapping from ~360 back to ~0. The
    first revolution after start-up is partial and is dropped by the coverage
    check, as is any fragment a lost S flag leaves behind.
    """

    def __init__(self, min_points: int = 20, min_coverage_deg: float = 270.0) -> None:
        self.min_points = min_points
        self.min_coverage_deg = min_coverage_deg
        self.rev_hz: float | None = None
        self.dropped = 0
        self._reset_rev()
        self._t_full: float | None = None

    def _reset_rev(self) -> None:
        self._pts: list[tuple[float, float, int]] = []
        self._samples = [0] * N_SAMPLE_BINS
        self._n = 0
        self._first: float | None = None
        self._last: float | None = None

    def add(self, m: Measurement, now: float) -> LidarScan | None:
        wrapped = self._last is not None and m.angle_deg < self._last - 180.0
        out = None
        if (m.start or wrapped) and self._n:
            out = self._close(now)
        if self._first is None:
            self._first = m.angle_deg
        self._last = m.angle_deg
        self._n += 1
        a = sensor_angle(m.angle_deg)
        self._samples[sample_bin(a)] += 1
        if m.valid:
            self._pts.append((a, m.range_mm / 1000.0, m.quality))
        return out

    def _close(self, now: float) -> LidarScan | None:
        coverage = (self._last or 0.0) - (self._first or 0.0)
        n, pts, samples = self._n, self._pts, self._samples
        self._reset_rev()
        if n < self.min_points or coverage < self.min_coverage_deg:
            self.dropped += 1
            return None
        if self._t_full is not None and now > self._t_full:
            hz = 1.0 / (now - self._t_full)
            self.rev_hz = hz if self.rev_hz is None else 0.7 * self.rev_hz + 0.3 * hz
        self._t_full = now
        return LidarScan(t=now, points=tuple(pts), rev_hz=self.rev_hz, n_raw=n,
                         samples=tuple(samples))


# ---------------------------------------------------------------- mounting


@dataclass(frozen=True)
class LidarMount:
    """Where the lidar sits on the base. Base frame: x forward, y left, metres,
    origin at the odometry centre (between the wheels).

    yaw_rad is the base-frame bearing of the lidar's 0 mark (the side away from
    its cable). Cable pointing at the robot's rear: yaw 0. Measure it rather
    than trusting a drawing: scripts/lidar_check.py --find-front.
    inverted: mounted upside down (e.g. hanging under the chassis), which
    mirrors the sweep. With raw RPLIDAR degrees (clockwise):
        upright   bearing_base = yaw - raw
        inverted  bearing_base = yaw + raw
    Both mounts in use or considered (front-centre upright; under the chassis,
    inverted, between the wheels) are just values of these four fields.
    """

    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0
    inverted: bool = False

    def bearing(self, sensor_angle_rad: float) -> float:
        """Sensor-frame CCW angle -> base-frame bearing (CCW, radians)."""
        a = -sensor_angle_rad if self.inverted else sensor_angle_rad
        return self.yaw_rad + a

    def sensor_angle_of(self, bearing_rad: float) -> float:
        """The inverse of bearing(): base-frame bearing (as seen FROM the lidar)
        -> sensor-frame CCW angle in [0, 2*pi)."""
        a = bearing_rad - self.yaw_rad
        return (-a if self.inverted else a) % TAU

    def to_base(self, sensor_angle_rad: float, range_m: float) -> tuple[float, float]:
        b = self.bearing(sensor_angle_rad)
        return self.x_m + range_m * math.cos(b), self.y_m + range_m * math.sin(b)


# The two conversions a consumer outside the bridge (navigation, the dashboard)
# should use. Conventions, once: the RPLIDAR reports degrees CLOCKWISE seen from
# above [DS]; the robot's base frame is x forward, y LEFT, bearings CCW-positive
# (types.Pose, navigation/geometry.py). The parser already flipped CW -> CCW
# (sensor_angle), so with the lidar's 0 mark facing forward (yaw 0) a raw
# reading at 270 deg lands at bearing +90 deg: the robot's LEFT. In one line:
#     bearing_base = mount.yaw - radians(raw_cw_deg)     (inverted: + instead of -)


def scan_to_base(scan: LidarScan,
                 mount: LidarMount = LidarMount()) -> list[tuple[float, float, int]]:
    """(x_m, y_m, quality) in the BASE frame for every valid return."""
    return [(*mount.to_base(a, r), q) for a, r, q in scan.points]


def scan_to_base_polar(scan: LidarScan,
                       mount: LidarMount = LidarMount()) -> list[tuple[float, float]]:
    """(bearing_rad, range_m) from the BASE origin, bearing CCW from forward in
    (-pi, pi]: the same convention as types.Target.bearing_rad."""
    out = []
    for a, r, _ in scan.points:
        x, y = mount.to_base(a, r)
        out.append((math.atan2(y, x), math.hypot(x, y)))
    return out


# ---------------------------------------------------------------- the source contract


class LidarSource(Protocol):
    """What the bridge needs from a lidar. latest_scan() is called from the
    server's event loop at the state rate, so it must return at once: a real
    driver reads the serial port on its own thread and only hands over the
    last finished revolution here."""

    def latest_scan(self) -> LidarScan | None: ...

    def close(self) -> None: ...


def staleness(scan: LidarScan | None, now: float) -> float:
    """Seconds since the scan completed; inf when there is none."""
    return math.inf if scan is None else max(0.0, now - scan.t)


# ---------------------------------------------------------------- motor control


class MotorControl(Protocol):
    """Starts and stops the scan motor. on/off get the link for the adapter
    variant, which speaks to its accessory board over the same serial port."""

    controllable: bool

    def on(self, link: Any = None) -> None: ...

    def off(self, link: Any = None) -> None: ...

    def close(self) -> None: ...


def _gpiozero() -> Any:
    try:
        import gpiozero
    except ImportError as exc:
        raise ImportError(
            "gpiozero is not importable. It ships with Raspberry Pi OS for the system "
            "python3; in a venv, create it with --system-site-packages. RPi.GPIO does "
            "not work on the Pi 5."
        ) from exc
    return gpiozero


class GpioMotor:
    """MOTOCTL on a Pi GPIO pin (the direct-wiring path; GPIO18 in our build).

    Off when constructed, off on off()/close(). A plain high level runs the
    motor at its highest speed [DS]; nothing here tries to PWM it.
    After close() gpiozero leaves the pin a floating input, and MOTOCTL's
    internal pull-down [DS] keeps the motor off; the same holds after a crash
    or kill -9, which is why this pin needs no external resistor.
    """

    controllable = True

    def __init__(self, pin: int | str | None = None, *, pin_factory: Any = None,
                 device: Any = None) -> None:
        if device is None:
            if pin is None:
                raise ValueError("GpioMotor needs a pin (or a device)")
            device = _gpiozero().DigitalOutputDevice(
                pin, active_high=True, initial_value=False, pin_factory=pin_factory)
        self.device = device
        self.pin = pin
        self.running = False
        self.device.off()

    def on(self, link: Any = None) -> None:
        self.device.on()
        self.running = True

    def off(self, link: Any = None) -> None:
        self.device.off()
        self.running = False

    def close(self) -> None:
        try:
            self.off()
        finally:
            self.device.close()


class ExternalMotor:
    """MOTOCTL tied high (or to anything we do not control): the motor runs
    whenever the lidar has power. STOP still switches the laser off."""

    controllable = False
    running = True

    def on(self, link: Any = None) -> None:
        pass

    def off(self, link: Any = None) -> None:
        pass

    def close(self) -> None:
        pass


class AdapterMotor:
    """Slamtec's USB adapter drives MOTOCTL. What the SDK does [SDK
    setMotorSpeed, checkMotorCtrlSupport]: ask GET_ACC_BOARD_FLAG; if bit 0 is
    set, command a PWM duty with SET_MOTOR_PWM, else use DTR (deasserted =
    spin). DTR is also cleared at open, as the SDK does.

    Measured on our adapter: MOTOCTL follows DTR, so off() only holds while
    the port stays open; closing it (any exit, clean or not) deasserts DTR and
    the motor spins again. Unplug the adapter to stop it for good."""

    controllable = True
    clear_dtr_at_open = True

    def __init__(self, pwm: int = DEFAULT_ADAPTER_PWM) -> None:
        if not 0 < pwm <= MAX_ADAPTER_PWM:
            raise ValueError(f"adapter pwm must be 1..{MAX_ADAPTER_PWM}")
        self.pwm = pwm
        self.running = False

    def on(self, link: Any = None) -> None:
        if link is None:
            raise LidarError("the adapter motor is commanded over the serial link")
        if link.acc_board_pwm is None:
            link.acc_board_pwm = link.probe_accessory_board()
        link.set_dtr(False)
        if link.acc_board_pwm:
            link.set_motor_pwm(self.pwm)
        self.running = True

    def off(self, link: Any = None) -> None:
        if link is None:
            return
        try:
            if link.acc_board_pwm:
                link.set_motor_pwm(0)
        finally:
            link.set_dtr(True)
            self.running = False

    def close(self) -> None:
        pass


# ---------------------------------------------------------------- serial


def open_serial(port: str, baud: int, timeout: float, dtr: bool | None = None) -> Any:
    """pyserial, imported here and only here."""
    try:
        import serial
    except ImportError as exc:
        raise LidarError(
            "pyserial is not importable. On the Pi: sudo apt install python3-serial "
            "(or pip install pyserial into the venv the bridge runs in)"
        ) from exc
    s = serial.Serial()
    s.port = port
    s.baudrate = baud
    s.timeout = timeout
    s.write_timeout = 1.0
    if dtr is not None:
        # applied at open(); a bare Pi UART has no DTR and pyserial ignores it
        s.dtr = dtr
    try:
        s.open()
    except (OSError, serial.SerialException) as exc:
        raise LidarError(f"can't open {port}: {exc}") from None
    return s


def find_ports() -> list[str]:
    """Likely lidar ports, best guess first: the Pi 5 header UART, then USB
    serial adapters (Linux by-id names are stable across reboots), then macOS."""
    out: list[str] = []
    for pattern in ("/dev/ttyAMA0", "/dev/serial/by-id/*CP210*", "/dev/serial/by-id/*",
                    "/dev/ttyUSB*", "/dev/cu.usbserial*", "/dev/cu.SLAB_USBtoUART*"):
        for p in sorted(glob.glob(pattern)):
            if p not in out and os.path.exists(p):
                out.append(p)
    return out


def default_motor(port: str, pin: int | str | None = None,
                  kind: str | None = None) -> MotorControl:
    """gpio when a pin is given; adapter for a USB port; else external (tied high)."""
    if kind is None:
        if pin is not None:
            kind = "gpio"
        elif any(s in port for s in ("ttyUSB", "usbserial", "SLAB", "by-id", "ttyACM")):
            kind = "adapter"
        else:
            kind = "external"
    if kind == "gpio":
        return GpioMotor(pin)
    if kind == "adapter":
        return AdapterMotor()
    if kind == "external":
        return ExternalMotor()
    raise ValueError(f"unknown lidar motor control {kind!r}: gpio, adapter or external")


class RPLidarLink:
    """The protocol over one serial port. Synchronous and BLOCKING: the bridge
    uses it only from RPLidar's thread; scripts/lidar_check.py uses it directly.

    Standard SCAN only. Express/boost modes would give 4-16x the points but
    need capsule decoding plus GET_LIDAR_CONF to learn which capsule format the
    firmware picked; standard mode is the path verified on our unit.
    """

    def __init__(
        self,
        port: str = DEFAULT_PORT,
        baud: int = DEFAULT_BAUD,
        *,
        motor: MotorControl | None = None,
        serial_factory: Callable[..., Any] | None = None,
        timeout_s: float = 0.05,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.port = port
        self.baud = baud
        self.motor: MotorControl = motor if motor is not None else ExternalMotor()
        self._factory = serial_factory or open_serial
        self.timeout_s = timeout_s
        self.sleep = sleep
        self.parser = MeasurementParser()
        self.acc_board_pwm: bool | None = None
        self.scanning = False
        self._ser: Any = None

    # -- lifecycle --------------------------------------------------------

    def open(self) -> RPLidarLink:
        dtr = False if getattr(self.motor, "clear_dtr_at_open", False) else None
        self._ser = self._factory(self.port, self.baud, self.timeout_s, dtr=dtr)
        return self

    def close(self) -> None:
        """Laser off (STOP), motor off, port closed. Never raises: it runs on
        every exit path, including the ones where the port just vanished."""
        if self._ser is not None:
            try:
                self.stop_scan()
            except Exception as exc:
                log.debug("STOP on close failed: %s", exc)
        try:
            self.motor.off(self if self._ser is not None else None)
        except Exception as exc:
            log.warning("lidar motor off failed: %s", exc)
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self.scanning = False

    def __enter__(self) -> RPLidarLink:
        return self.open()

    def __exit__(self, *exc: object) -> None:
        self.close()

    # -- raw I/O ----------------------------------------------------------

    @property
    def serial(self) -> Any:
        if self._ser is None:
            raise LidarError("lidar port is not open")
        return self._ser

    def send(self, cmd: int, payload: bytes = b"") -> None:
        self.serial.write(request(cmd, payload))

    def flush_input(self) -> None:
        self.serial.reset_input_buffer()
        self.parser.reset()

    def _read_some(self) -> bytes:
        s = self.serial
        n = getattr(s, "in_waiting", 0) or 1
        return s.read(n)

    def _read_exact(self, n: int, deadline: float) -> bytes:
        out = bytearray()
        while len(out) < n:
            if time.monotonic() > deadline:
                raise LidarError(f"timed out: got {len(out)} of {n} bytes")
            out += self.serial.read(n - len(out))
        return bytes(out)

    def _read_descriptor(self, deadline: float) -> Descriptor:
        """Skip anything before A5 5A (the tail of a stopped scan), then parse."""
        window = bytearray()
        while True:
            if time.monotonic() > deadline:
                raise LidarError("no answer from the lidar (wrong port or baud? powered?)")
            b = self.serial.read(1)
            if not b:
                continue
            window += b
            if len(window) > 2:
                del window[0]
            if window == bytes((SYNC1, SYNC2)):
                rest = self._read_exact(5, deadline)
                return parse_descriptor(bytes((SYNC1, SYNC2)) + rest)

    def _request(self, cmd: int, dtype: int, payload: bytes = b"",
                 timeout: float = 1.0) -> bytes:
        self.flush_input()
        self.send(cmd, payload)
        deadline = time.monotonic() + timeout
        d = self._read_descriptor(deadline)
        if d.dtype != dtype:
            raise LidarError(f"command 0x{cmd:02X} answered type 0x{d.dtype:02X}, "
                             f"expected 0x{dtype:02X}")
        return self._read_exact(d.length, deadline)

    # -- commands ---------------------------------------------------------

    def stop_scan(self) -> None:
        self.send(CMD_STOP)
        self.sleep(0.05)          # >= 1 ms required [PROTO]; the SDK waits 100 ms
        self.flush_input()
        self.scanning = False

    def reset_core(self, boot_s: float = 1.0) -> None:
        """Reboot the scan core, the documented way out of protection stop
        [PROTO p35]. boot_s is a guess (>= 2 ms is the documented minimum)."""
        self.send(CMD_RESET)
        self.sleep(boot_s)
        self.flush_input()
        self.scanning = False

    def get_info(self, timeout: float = 1.0) -> DeviceInfo:
        return parse_info(self._request(CMD_GET_INFO, ANS_DEVINFO, timeout=timeout))

    def get_health(self, timeout: float = 1.0) -> Health:
        return parse_health(self._request(CMD_GET_HEALTH, ANS_DEVHEALTH, timeout=timeout))

    def get_sample_rate(self, timeout: float = 1.0) -> tuple[int, int]:
        """(standard, express) microseconds per sample [PROTO p36]."""
        raw = self._request(CMD_GET_SAMPLERATE, ANS_SAMPLE_RATE, timeout=timeout)
        if len(raw) < 4:
            raise LidarError("short GET_SAMPLERATE answer")
        return struct.unpack("<HH", raw[:4])

    def probe_accessory_board(self, timeout: float = 0.5) -> bool:
        """True if a USB adapter answers that it drives the motor PWM [SDK]."""
        try:
            raw = self._request(CMD_GET_ACC_BOARD_FLAG, ANS_ACC_BOARD_FLAG,
                                payload=b"\x00\x00\x00\x00", timeout=timeout)
        except LidarError:
            return False
        return len(raw) >= 4 and bool(struct.unpack("<I", raw[:4])[0] & 0x1)

    def set_motor_pwm(self, pwm: int) -> None:
        self.send(CMD_SET_MOTOR_PWM, struct.pack("<H", max(0, min(MAX_ADAPTER_PWM, int(pwm)))))
        self.sleep(0.01)

    def set_dtr(self, value: bool) -> None:
        try:
            self.serial.dtr = value
        except (OSError, AttributeError, ValueError) as exc:
            log.debug("DTR not settable on %s: %s", self.port, exc)

    def motor_on(self) -> None:
        self.motor.on(self)

    def motor_off(self) -> None:
        self.motor.off(self)

    def start_scan(self, timeout: float = 1.0) -> None:
        self.flush_input()
        self.send(CMD_SCAN)
        d = self._read_descriptor(time.monotonic() + timeout)
        if d.dtype != ANS_MEASUREMENT or d.length != PACKET_LEN or d.mode != 1:
            raise LidarError(f"SCAN answered {d}, expected a 5-byte measurement stream")
        self.scanning = True

    def read_measurements(self) -> list[Measurement]:
        """Whatever has arrived, parsed. Blocks at most timeout_s."""
        return self.parser.feed(self._read_some())

    def scans(self, clock: Callable[[], float] = time.monotonic,
              stop: threading.Event | None = None,
              assembler: ScanAssembler | None = None) -> Iterator[LidarScan]:
        asm = assembler or ScanAssembler()
        while stop is None or not stop.is_set():
            ms = self.read_measurements()
            if not ms:
                continue
            now = clock()
            for m in ms:
                scan = asm.add(m, now)
                if scan is not None:
                    yield scan


def bring_up(link: RPLidarLink,
             log_fn: Callable[[str], None] = log.info) -> tuple[DeviceInfo, Health]:
    """The start sequence Slamtec recommends [PROTO p44]: STOP whatever a
    previous process left running, read info, check health, RESET once out of
    a protection stop, spin the motor, start the scan."""
    link.stop_scan()
    info = link.get_info()
    health = link.get_health()
    if health.status == 2:
        log_fn(f"lidar reports {health}: resetting the scan core")
        link.reset_core()
        health = link.get_health()
        if health.status == 2:
            raise LidarError(f"lidar is in protection stop after a reset ({health}); "
                             "power-cycle it. Repeated protection stops mean damage [PROTO].")
    try:
        std_us, _ = link.get_sample_rate()
        rate = 1e6 / max(1, std_us)
        log_fn(f"lidar standard mode: {std_us} us/sample ({rate:.0f} samples/s)")
    except LidarError:
        pass
    link.motor_on()
    link.start_scan()
    return info, health


class RPLidar:
    """The bridge's lidar: RPLidarLink on its own thread.

    latest_scan() only returns a reference under a lock, so the server's event
    loop never waits on the serial port. If the port disappears or the stream
    stops, the thread logs it, stops the motor, and retries every retry_s;
    meanwhile the scan goes stale and the safety bubble limits motion to creep.
    close() stops the motor and releases the port and the GPIO.
    """

    def __init__(
        self,
        port: str = DEFAULT_PORT,
        baud: int = DEFAULT_BAUD,
        *,
        motor: MotorControl | None = None,
        clock: Callable[[], float] = time.monotonic,
        serial_factory: Callable[..., Any] | None = None,
        retry_s: float = 2.0,
        first_scan_timeout_s: float = 6.0,
        scan_timeout_s: float = 1.5,
        min_points: int = 20,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.port = port
        self.baud = baud
        self.motor: MotorControl = motor if motor is not None else ExternalMotor()
        self.clock = clock
        self._factory = serial_factory
        self.retry_s = retry_s
        self.first_scan_timeout_s = first_scan_timeout_s
        self.scan_timeout_s = scan_timeout_s
        self.min_points = min_points
        self.sleep = sleep

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._scan: LidarScan | None = None
        self._closed = False
        self.state = "idle"             # idle | starting | scanning | error | closed
        self.error: str | None = None
        self.info: DeviceInfo | None = None
        self.health: Health | None = None
        self.scans = 0
        self.bad_bytes = 0

    def start(self) -> RPLidar:
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="lidar", daemon=True)
            self._thread.start()
        return self

    def latest_scan(self) -> LidarScan | None:
        with self._lock:
            return self._scan

    def staleness(self, now: float | None = None) -> float:
        return staleness(self.latest_scan(), self.clock() if now is None else now)

    def status(self) -> dict[str, Any]:
        scan = self.latest_scan()
        return {"state": self.state, "error": self.error,
                "info": str(self.info) if self.info else None,
                "health": str(self.health) if self.health else None,
                "rev_hz": scan.rev_hz if scan else None,
                "points": len(scan.points) if scan else 0,
                "scans": self.scans, "bad_bytes": self.bad_bytes}

    def close(self, timeout: float = 3.0) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout)
        if self._thread is None:
            try:
                self.motor.close()
            except Exception as exc:
                log.warning("lidar motor close failed: %s", exc)
        self.state = "closed"

    def __enter__(self) -> RPLidar:
        return self.start()

    def __exit__(self, *exc: object) -> None:
        self.close()

    # -- the thread -------------------------------------------------------

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                link = RPLidarLink(self.port, self.baud, motor=self.motor,
                                   serial_factory=self._factory, sleep=self.sleep)
                try:
                    self.state, self.error = "starting", None
                    link.open()
                    self._session(link)
                except Exception as exc:
                    self.state, self.error = "error", f"{type(exc).__name__}: {exc}"
                    log.warning("lidar on %s: %s (retrying in %.0f s)",
                                self.port, self.error, self.retry_s)
                finally:
                    self.bad_bytes += link.parser.bad
                    link.close()           # laser off, motor off, port released
                self._stop.wait(self.retry_s)
        finally:
            try:
                self.motor.close()
            except Exception as exc:
                log.warning("lidar motor close failed: %s", exc)

    def _session(self, link: RPLidarLink) -> None:
        self.info, self.health = bring_up(link)
        log.info("lidar on %s @ %d: %s, health %s",
                 self.port, self.baud, self.info, self.health)
        asm = ScanAssembler(min_points=self.min_points)
        started = last = self.clock()
        first = True
        while not self._stop.is_set():
            ms = link.read_measurements()
            now = self.clock()
            for m in ms:
                scan = asm.add(m, now)
                if scan is not None:
                    with self._lock:
                        self._scan = scan
                    self.scans += 1
                    last = now
                    if first:
                        first = False
                        self.state = "scanning"
                        log.info("lidar scanning: %d points/rev", len(scan.points))
            limit = self.first_scan_timeout_s if first else self.scan_timeout_s
            if now - (started if first else last) > limit:
                raise LidarError(f"no complete revolution for {limit:.1f} s "
                                 f"({link.parser.bad} bad bytes; motor spinning?)")


# ---------------------------------------------------------------- the fake


def _ray_segment(ox: float, oy: float, dx: float, dy: float,
                 x0: float, y0: float, x1: float, y1: float) -> float | None:
    ex, ey = x1 - x0, y1 - y0
    denom = dx * ey - dy * ex
    if abs(denom) < 1e-12:
        return None
    wx, wy = x0 - ox, y0 - oy
    t = (wx * ey - wy * ex) / denom
    u = (wx * dy - wy * dx) / denom
    if t >= 0.0 and -1e-9 <= u <= 1.0 + 1e-9:
        return t
    return None


def _ray_circle(ox: float, oy: float, dx: float, dy: float,
                cx: float, cy: float, r: float) -> float | None:
    fx, fy = ox - cx, oy - cy
    b = dx * fx + dy * fy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    if disc < 0.0:
        return None
    root = math.sqrt(disc)
    t = -b - root
    if t < 0.0:
        t = -b + root
    return t if t >= 0.0 else None


class FakeWorld:
    """Walls (segments) and round posts in the WORLD frame, for FakeLidar."""

    def __init__(self, walls: Iterable[tuple[float, float, float, float]] = (),
                 posts: Iterable[tuple[float, float, float]] = ()) -> None:
        self.walls = [tuple(map(float, w)) for w in walls]
        self.posts = [tuple(map(float, p)) for p in posts]

    @classmethod
    def room(cls, x0: float = -2.5, y0: float = -2.4, x1: float = 3.0, y1: float = 2.0,
             posts: Iterable[tuple[float, float, float]] = ()) -> FakeWorld:
        return cls([(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)],
                   posts)

    def add_wall(self, x0: float, y0: float, x1: float, y1: float) -> FakeWorld:
        self.walls.append((float(x0), float(y0), float(x1), float(y1)))
        return self

    def add_post(self, x: float, y: float, r: float) -> FakeWorld:
        self.posts.append((float(x), float(y), float(r)))
        return self

    def raycast(self, ox: float, oy: float, angle: float,
                extra_posts: Iterable[tuple[float, float, float]] = ()) -> float | None:
        dx, dy = math.cos(angle), math.sin(angle)
        best: float | None = None
        for w in self.walls:
            t = _ray_segment(ox, oy, dx, dy, *w)
            if t is not None and (best is None or t < best):
                best = t
        for p in (*self.posts, *extra_posts):
            t = _ray_circle(ox, oy, dx, dy, *p)
            if t is not None and (best is None or t < best):
                best = t
        return best


class FakeTankPose:
    """The world pose of a FakeTankDriver, integrated from its unwrapped
    wheel travel (so no rollover ambiguity at any call rate).
    Call it from the thread that owns the driver: the server's event loop."""

    def __init__(self, driver: Any,
                 start: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        self.driver = driver
        self.x, self.y, self.theta = start
        self._last: tuple[float, float] | None = None

    def __call__(self) -> tuple[float, float, float]:
        drv = self.driver
        drv.read_state()                      # catches the simulation up to now
        pos = (float(drv.wheel_travel_m[0]), float(drv.wheel_travel_m[1]))
        if self._last is None:
            self._last = pos
            return self.x, self.y, self.theta
        dl = pos[0] - self._last[0]
        dr = pos[1] - self._last[1]
        self._last = pos
        ds = (dl + dr) / 2.0
        dth = (dr - dl) / drv.geo.effective_track_m
        if abs(dth) < 1e-9:
            self.x += ds * math.cos(self.theta)
            self.y += ds * math.sin(self.theta)
        else:
            r = ds / dth
            self.x += r * (math.sin(self.theta + dth) - math.sin(self.theta))
            self.y -= r * (math.cos(self.theta + dth) - math.cos(self.theta))
        self.theta = (self.theta + dth + math.pi) % TAU - math.pi
        return self.x, self.y, self.theta


class FakeLidar:
    """A lidar in a synthetic room. Same contract as RPLidar.

    Lazy, like FakeTankDriver: latest_scan() makes a new revolution when one
    is due on `clock`, cast from wherever pose_fn() says the robot is. No
    thread, so tests on a fake clock are exact. `self_parts` are circles in the
    BASE frame that ride with the robot (an arm, a mast): what the self-mask is
    for. freeze() simulates a dead lidar: no new scans, the last one ages.
    """

    def __init__(
        self,
        world: FakeWorld | None = None,
        *,
        pose_fn: Callable[[], tuple[float, float, float]] | None = None,
        mount: LidarMount = LidarMount(),
        rate_hz: float = 10.0,
        beams: int = 360,
        clock: Callable[[], float] = time.monotonic,
        noise_m: float = 0.0,
        seed: int = 0,
        self_parts: Iterable[tuple[float, float, float]] = (),
        min_range_m: float = MIN_RANGE_M,
        max_range_m: float = MAX_RANGE_M,
        quality: int = 47,
    ) -> None:
        self.world = world if world is not None else FakeWorld.room()
        self.pose_fn = pose_fn or (lambda: (0.0, 0.0, 0.0))
        self.mount = mount
        self.period = 1.0 / rate_hz
        self.beams = beams
        self.clock = clock
        self.noise_m = noise_m
        self._rng = random.Random(seed)
        self.self_parts = [tuple(map(float, p)) for p in self_parts]
        self.min_range_m = min_range_m
        self.max_range_m = max_range_m
        self.quality = quality
        # dark(sensor_angle) -> True: that ray returns nothing (a black or
        # shiny surface, or open space beyond max range). Tests use it.
        self.dark: Callable[[float], bool] | None = None
        self.frozen = False
        self.scans = 0
        self._scan: LidarScan | None = None

    def latest_scan(self) -> LidarScan | None:
        now = self.clock()
        if not self.frozen and (self._scan is None or now - self._scan.t >= self.period):
            self._scan = self.scan_now(now)
        return self._scan

    def scan_now(self, now: float | None = None) -> LidarScan:
        now = self.clock() if now is None else now
        px, py, pth = self.pose_fn()
        c, s = math.cos(pth), math.sin(pth)
        ox = px + c * self.mount.x_m - s * self.mount.y_m
        oy = py + s * self.mount.x_m + c * self.mount.y_m
        parts = [(px + c * x - s * y, py + s * x + c * y, r) for x, y, r in self.self_parts]
        pts: list[tuple[float, float, int]] = []
        samples = [0] * N_SAMPLE_BINS
        for i in range(self.beams):
            a = TAU * i / self.beams
            samples[sample_bin(a)] += 1
            if self.dark is not None and self.dark(a):
                continue
            r = self.world.raycast(ox, oy, pth + self.mount.bearing(a), parts)
            if r is None:
                continue
            if self.noise_m:
                r += self._rng.gauss(0.0, self.noise_m)
            if self.min_range_m <= r <= self.max_range_m:
                pts.append((a, r, self.quality))
        self.scans += 1
        return LidarScan(t=now, points=tuple(pts), rev_hz=1.0 / self.period, n_raw=self.beams,
                         samples=tuple(samples))

    def freeze(self) -> None:
        self.frozen = True

    def unfreeze(self) -> None:
        self.frozen = False

    def close(self) -> None:
        pass


__all__ = [
    "DEFAULT_BAUD",
    "DEFAULT_PORT",
    "N_SAMPLE_BINS",
    "SAMPLE_BIN_DEG",
    "AdapterMotor",
    "DeviceInfo",
    "ExternalMotor",
    "FakeLidar",
    "FakeTankPose",
    "FakeWorld",
    "GpioMotor",
    "Health",
    "LidarError",
    "LidarMount",
    "LidarScan",
    "LidarSource",
    "Measurement",
    "MeasurementParser",
    "RPLidar",
    "RPLidarLink",
    "ScanAssembler",
    "bring_up",
    "default_motor",
    "encode_measurement",
    "find_ports",
    "open_serial",
    "parse_descriptor",
    "parse_health",
    "parse_info",
    "parse_measurement",
    "request",
    "sample_bin",
    "scan_to_base",
    "scan_to_base_polar",
    "sensor_angle",
    "staleness",
]
