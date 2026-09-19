"""The D435i's gyro, for heading. Raw USB HID, stdlib only, no librealsense.

    python3 -m retriever.bridge.imu          # on the Pi: rates, bias, live heading

WHY RAW HID. librealsense's pip wheel reaches the D435i's motion module
through the kernel's HID sensor drivers (IIO), and the Raspberry Pi kernel
doesn't ship them: pyrealsense2 sees the depth and colour sensors and no
IMU at all. The IMU is still there as a plain HID interface (/dev/hidrawN),
and this speaks its protocol directly, as librealsense's own libusb backend
does (src/hid/hid-device.cpp, hid-types.h; D400 FW >= 5.16 reports):

  * feature report per sensor (1 accel, 2 gyro): 9 bytes
      reportId, connectionType, sensorState, power, minReport, report(u16 ms), sensitivity(u16)
    get it, set power D0 (2) and the interval 1000/fps, set it; D4 (6) stops it.
  * input reports, 38 bytes: id, ?, timestamp u64, x y z int32, 16 bytes of custom values.
    (FW < 5.16: 32 bytes with int16 x y z.)
  * gyro: FW >= 5.16 counts 0.0001 deg/s, older 0.1 deg/s. accel: 0.001 g.

HEADING. The robot turns about "up", whatever way the camera is mounted, and
the accelerometer says where up is (at rest it measures the floor pushing up).
So yaw rate = gyro . up, with the gyro's bias learned while the wheels are
still. Integrated, that is a heading that doesn't care how much the tyres
slide: the skid-steer's weak spot. It still drifts slowly (bias), which the
wheels can't fix and the lidar map later can.

Permissions: /dev/hidrawN is root-only by default. A udev rule for it (group
plugdev) fixes that for good; for a quick test, `sudo chmod 666 /dev/hidraw0`.
"""

from __future__ import annotations

import errno
import glob
import logging
import math
import os
import struct
import threading
import time
from dataclasses import dataclass
from typing import Callable

log = logging.getLogger("retriever.bridge.imu")

VID, PID = 0x8086, 0x0B3A
REPORT_ACCEL, REPORT_GYRO = 1, 2
POWER_ON, POWER_OFF = 2, 6                  # DEVICE_POWER_D0, DEVICE_POWER_D4
FEATURE = struct.Struct("<BBBBBHH")          # 9 bytes
REPORT_32BIT = struct.Struct("<BBQiii")      # FW >= 5.16: 38 bytes, first 22 used
REPORT_16BIT = struct.Struct("<BBQhhh")      # FW <  5.16: 32 bytes, first 16 used
GYRO_DEG_PER_LSB = {38: 1e-4, 32: 0.1}
ACCEL_MPS2_PER_LSB = 0.001 * 9.80665


def _ioc(direction: int, kind: str, nr: int, size: int) -> int:
    return (direction << 30) | (size << 16) | (ord(kind) << 8) | nr


def HIDIOCSFEATURE(n: int) -> int:  # noqa: N802 - the kernel's name
    return _ioc(3, "H", 0x06, n)


def HIDIOCGFEATURE(n: int) -> int:  # noqa: N802
    return _ioc(3, "H", 0x07, n)


def find_hidraw(sysfs: str = "/sys/class/hidraw") -> str | None:
    """/dev/hidrawN of the D435i's IMU, or None."""
    want = f"HID_ID=0003:{VID:08X}:{PID:08X}"
    for d in sorted(glob.glob(os.path.join(sysfs, "hidraw*"))):
        try:
            with open(os.path.join(d, "device", "uevent")) as f:
                if want in f.read().upper():
                    return "/dev/" + os.path.basename(d)
        except OSError:
            continue
    return None


def parse_report(buf: bytes) -> tuple[int, int, tuple[float, float, float]] | None:
    """(report id, device timestamp, (x, y, z) in SI: rad/s or m/s^2), or None."""
    n = len(buf)
    if n == 38:
        rid, _, ts, x, y, z = REPORT_32BIT.unpack_from(buf)
    elif n == 32:
        rid, _, ts, x, y, z = REPORT_16BIT.unpack_from(buf)
    else:
        return None
    if rid == REPORT_GYRO:
        k = math.radians(GYRO_DEG_PER_LSB[n])
    elif rid == REPORT_ACCEL:
        k = ACCEL_MPS2_PER_LSB
    else:
        return None
    return rid, ts, (x * k, y * k, z * k)


@dataclass(frozen=True)
class YawConfig:
    still_rate: float = 0.03        # rad/s: slower than this with the wheels stopped = still
    still_for_s: float = 0.4        # ...for this long before the bias starts learning
    bias_tau_s: float = 4.0         # bias time constant while still
    up_tau_s: float = 1.0           # gravity direction time constant
    max_dt_s: float = 0.05          # a gap longer than this is a dropout, not a turn


class YawTracker:
    """Heading from gyro samples: rate about up (from the accelerometer), bias
    removed while still. Pure: feed it samples with their times."""

    def __init__(self, config: YawConfig = YawConfig()) -> None:
        self.config = config
        self.yaw = 0.0                          # rad, unwrapped, CCW about up
        self.rate = 0.0                         # rad/s about up, bias removed
        self.bias = [0.0, 0.0, 0.0]             # rad/s, gyro frame
        self.up: list[float] | None = None      # unit vector, gyro/accel frame
        self.bias_ready = False
        self._bias_n = 0
        self._last_t: float | None = None
        self._still_since: float | None = None
        self.samples = 0

    def accel(self, a: tuple[float, float, float], dt: float = 0.016) -> None:
        n = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
        if n < 5.0 or n > 15.0:                 # a bump, not gravity
            return
        u = [a[0] / n, a[1] / n, a[2] / n]
        if self.up is None:
            self.up = u
            return
        k = min(1.0, dt / self.config.up_tau_s)
        v = [self.up[i] + k * (u[i] - self.up[i]) for i in range(3)]
        m = math.sqrt(sum(x * x for x in v)) or 1.0
        self.up = [x / m for x in v]

    def gyro(self, w: tuple[float, float, float], t: float, wheels_still: bool = True) -> float:
        """One gyro sample at time t (seconds). Returns the yaw."""
        c = self.config
        self.samples += 1
        dt = 0.0 if self._last_t is None else t - self._last_t
        self._last_t = t
        if not 0.0 < dt <= c.max_dt_s:
            dt = 0.0
        unbiased = [w[i] - self.bias[i] for i in range(3)]
        mag = math.sqrt(sum(x * x for x in unbiased))
        if wheels_still and (mag < c.still_rate or not self.bias_ready):
            if self._still_since is None:
                self._still_since = t
            if t - self._still_since >= c.still_for_s or not self.bias_ready:
                # the first second: a plain average; after: a slow EMA
                self._bias_n += 1
                k = 1.0 / self._bias_n if self._bias_n < 200 else min(1.0, dt / c.bias_tau_s)
                self.bias = [self.bias[i] + k * (w[i] - self.bias[i]) for i in range(3)]
                if self._bias_n >= 100:
                    self.bias_ready = True
        else:
            self._still_since = None
        if self.up is None or not self.bias_ready:
            self.rate = 0.0
            return self.yaw
        unbiased = [w[i] - self.bias[i] for i in range(3)]
        self.rate = sum(unbiased[i] * self.up[i] for i in range(3))
        self.yaw += self.rate * dt
        return self.yaw

    @property
    def ready(self) -> bool:
        return self.bias_ready and self.up is not None


class D435iImu:
    """Reads the D435i's gyro + accel on a thread; `yaw()` any time.

    wheels_still: called per gyro sample; True while the wheels are commanded
    to zero (the bridge knows), so the bias only learns while really still."""

    def __init__(self, path: str | None = None, gyro_hz: int = 200, accel_hz: int = 63,
                 config: YawConfig = YawConfig(),
                 wheels_still: Callable[[], bool] = lambda: True,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.path = path
        self.gyro_hz, self.accel_hz = gyro_hz, accel_hz
        self.tracker = YawTracker(config)
        self.wheels_still = wheels_still
        self.clock = clock
        self._fd: int | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.error: str | None = None
        self.last_sample: float | None = None
        self.report_size: int | None = None
        self.counts = {REPORT_GYRO: 0, REPORT_ACCEL: 0}

    def start(self) -> D435iImu:
        path = self.path or find_hidraw()
        if path is None:
            raise FileNotFoundError("no D435i IMU (HID 8086:0b3a) found: is the camera plugged in?")
        self.path = path
        try:
            self._fd = os.open(path, os.O_RDWR)
        except PermissionError:
            raise PermissionError(
                f"can't open {path}: it needs a udev rule (group plugdev), or for now "
                f"`sudo chmod 666 {path}`") from None
        self._feature(REPORT_ACCEL, POWER_ON, self.accel_hz)
        self._feature(REPORT_GYRO, POWER_ON, self.gyro_hz)
        self._thread = threading.Thread(target=self._read_loop, name="d435i-imu", daemon=True)
        self._thread.start()
        return self

    def _feature(self, report_id: int, power: int, fps: int = 0) -> None:
        import fcntl

        assert self._fd is not None
        buf = bytearray(FEATURE.size)
        buf[0] = report_id
        fcntl.ioctl(self._fd, HIDIOCGFEATURE(len(buf)), buf, True)
        rid, conn, state, _, min_report, report, sens = FEATURE.unpack(bytes(buf))
        if fps > 0:
            report = int(1000 / fps)
        if report_id == REPORT_GYRO:
            sens = 0                            # librealsense sends int(0.1): the default
        new = bytearray(FEATURE.pack(report_id, conn, state, power, min_report, report, sens))
        fcntl.ioctl(self._fd, HIDIOCSFEATURE(len(new)), new, True)
        check = bytearray(FEATURE.size)
        check[0] = report_id
        fcntl.ioctl(self._fd, HIDIOCGFEATURE(len(check)), check, True)
        if check[3] != power:
            raise OSError(f"the D435i IMU didn't take power {power} for report {report_id}")

    def _read_loop(self) -> None:
        fd = self._fd
        last_accel = None
        while not self._stop.is_set() and fd is not None:
            try:
                buf = os.read(fd, 64)
            except OSError as exc:
                if exc.errno in (errno.EINTR, errno.EAGAIN):
                    continue
                self.error = f"IMU read failed: {exc}"
                log.error("%s", self.error)
                return
            parsed = parse_report(buf)
            if parsed is None:
                continue
            rid, _ts, v = parsed
            now = self.clock()
            with self._lock:
                self.report_size = len(buf)
                self.counts[rid] += 1
                self.last_sample = now
                if rid == REPORT_ACCEL:
                    self.tracker.accel(v, 0.0 if last_accel is None else now - last_accel)
                    last_accel = now
                else:
                    self.tracker.gyro(v, now, bool(self.wheels_still()))

    def yaw(self) -> float | None:
        """Heading in rad (unwrapped, CCW), or None until ready or if stale."""
        with self._lock:
            if not self.tracker.ready or self.last_sample is None:
                return None
            if self.clock() - self.last_sample > 0.25:
                return None
            return self.tracker.yaw

    def status(self) -> dict:
        with self._lock:
            t = self.tracker
            return {"ready": t.ready, "yaw": t.yaw, "rate": t.rate, "bias": list(t.bias),
                    "up": t.up, "counts": dict(self.counts), "report_size": self.report_size,
                    "error": self.error}

    def close(self) -> None:
        self._stop.set()
        if self._fd is not None:
            for rid in (REPORT_ACCEL, REPORT_GYRO):
                try:
                    self._feature(rid, POWER_OFF)
                except OSError:
                    pass
            fd, self._fd = self._fd, None
            os.close(fd)                        # wakes the blocked read
        if self._thread is not None:
            self._thread.join(1.0)


def main() -> int:
    """Probe: start the IMU, keep still 2 s for the bias, then print heading."""
    logging.basicConfig(level=logging.INFO)
    imu = D435iImu().start()
    print(f"opened {imu.path}; keep the robot still for the bias...")
    t0 = time.monotonic()
    last = dict(imu.counts)
    try:
        while True:
            time.sleep(1.0)
            s = imu.status()
            dt = time.monotonic() - t0
            rates = {k: s["counts"][k] - last[k] for k in s["counts"]}
            last = dict(s["counts"])
            up = s["up"]
            print(f"{dt:5.1f}s  gyro {rates[REPORT_GYRO]:3d}/s accel {rates[REPORT_ACCEL]:3d}/s  "
                  f"report {s['report_size']}B  ready {s['ready']}  "
                  f"heading {math.degrees(s['yaw']):8.2f} deg  rate {math.degrees(s['rate']):6.2f} deg/s  "
                  f"bias {[round(math.degrees(b), 3) for b in s['bias']]} deg/s  "
                  f"up {[round(u, 2) for u in up] if up else None}")
    except KeyboardInterrupt:
        pass
    finally:
        imu.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
