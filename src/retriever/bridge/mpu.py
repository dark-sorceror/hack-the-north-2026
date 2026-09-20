"""An InvenSense MPU on the Pi's I2C (6050, 6500, 9250, 9255), for heading.
Stdlib only.

    python3 -m retriever.bridge.mpu            # on the Pi: rates, bias, live heading

The robot's dedicated IMU, wired to the header:

    VCC -> pin 1 (3.3V)   SDA -> pin 3 (GPIO2)
    GND -> pin 9          SCL -> pin 5 (GPIO3)      AD0 -> GND (address 0x68)

WHY NOT THE CAMERA'S. The D435i's gyro works (bridge/imu.py) and stays as a
fallback, but the camera belongs to perception and the arm now. A $10 part
bolted to the chassis is also better placed than one on a moving camera mount:
the gyro must feel what the CHASSIS does.

WHAT IT GIVES. Gyro and accelerometer only. The magnetometer is left alone: a
compass 10 cm from four motors reads the motors. Heading comes from the gyro's
rate about gravity's axis, exactly as for the camera's IMU, and the same
YawTracker (imu.py) removes the bias and holds the heading still while the
wheels are stopped.

I2C, not SPI: four wires on a breadboard, and 200 Hz of 14-byte reads is about
2% of a core. The gyro and accelerometer registers are the same across the
family (the MPU-6050 map, RM-MPU-6000A rev 4.2, carried into the 9250's
RM-MPU-9250A-00): +-250 deg/s at 131 LSB per deg/s, +-2 g at 16384 LSB per g,
both through the 41 Hz low-pass so wheel buzz doesn't alias. WHO_AM_I decides
the two places they differ: the 6050 has no separate accelerometer low-pass
register, and the temperature scale changed.
"""

from __future__ import annotations

import fcntl
import logging
import math
import os
import struct
import threading
import time
from typing import Any, Callable

from retriever.bridge.imu import YawConfig, YawTracker

log = logging.getLogger("retriever.bridge.mpu")

I2C_SLAVE = 0x0703                  # linux/i2c-dev.h
DEFAULT_BUS = "/dev/i2c-1"          # the 40-pin header's bus (dtparam=i2c_arm=on)
ADDR_LOW, ADDR_HIGH = 0x68, 0x69    # AD0 low / high

# registers
WHO_AM_I = 0x75
PWR_MGMT_1, PWR_MGMT_2 = 0x6B, 0x6C
SMPLRT_DIV, CONFIG = 0x19, 0x1A
GYRO_CONFIG, ACCEL_CONFIG, ACCEL_CONFIG2 = 0x1B, 0x1C, 0x1D
ACCEL_XOUT_H = 0x3B                 # 14 bytes: accel xyz, temp, gyro xyz

# what the part answers to WHO_AM_I; all of these speak the registers above
KNOWN_IDS = {0x71: "MPU-9250", 0x73: "MPU-9255", 0x70: "MPU-6500", 0x68: "MPU-6050",
             0x75: "ICM-20689", 0x12: "ICM-20948 (untested)"}

GYRO_LSB_PER_DPS = 131.0            # +-250 deg/s
ACCEL_LSB_PER_G = 16384.0           # +-2 g
G = 9.80665
_SAMPLE = struct.Struct(">hhhhhhh")  # ax ay az temp gx gy gz, big-endian
MPU6050 = 0x68                      # its WHO_AM_I; the one part without ACCEL_CONFIG2


def to_si(raw: bytes, who: int = 0x71) -> tuple[tuple[float, float, float],
                                                tuple[float, float, float], float]:
    """14 register bytes -> (accel m/s^2, gyro rad/s, temperature C)."""
    ax, ay, az, temp, gx, gy, gz = _SAMPLE.unpack(raw)
    accel = tuple(v / ACCEL_LSB_PER_G * G for v in (ax, ay, az))
    gyro = tuple(math.radians(v / GYRO_LSB_PER_DPS) for v in (gx, gy, gz))
    celsius = temp / 340.0 + 36.53 if who == MPU6050 else temp / 333.87 + 21.0
    return accel, gyro, celsius


class I2CDevice:
    """One device on an I2C bus. `open()` is the only part that touches Linux,
    so tests hand the driver their own object with read/write/close."""

    def __init__(self, fd: int) -> None:
        self._fd = fd

    @classmethod
    def open(cls, bus: str = DEFAULT_BUS, address: int = ADDR_LOW) -> I2CDevice:
        try:
            fd = os.open(bus, os.O_RDWR)
        except FileNotFoundError:
            raise FileNotFoundError(
                f"{bus} is missing: the Pi's header I2C is off. Add 'dtparam=i2c_arm=on' to "
                "/boot/firmware/config.txt and reboot (hardware/nav_pi/setup.sh does it)") from None
        except PermissionError:
            raise PermissionError(f"can't open {bus}: the user needs the i2c group") from None
        try:
            fcntl.ioctl(fd, I2C_SLAVE, address)
        except OSError:
            os.close(fd)
            raise
        return cls(fd)

    def write(self, register: int, value: int) -> None:
        os.write(self._fd, bytes((register, value)))

    def read(self, register: int, length: int = 1) -> bytes:
        os.write(self._fd, bytes((register,)))
        data = os.read(self._fd, length)
        if len(data) != length:
            raise OSError(f"short I2C read: {len(data)} of {length} bytes")
        return data

    def close(self) -> None:
        if self._fd >= 0:
            os.close(self._fd)
            self._fd = -1


def find_device(bus: str = DEFAULT_BUS,
                opener: Callable[[str, int], I2CDevice] = I2CDevice.open) -> tuple[I2CDevice, int, str]:
    """(device, address, part name) for whichever address answers, or raise."""
    last: Exception | None = None
    for address in (ADDR_LOW, ADDR_HIGH):
        try:
            dev = opener(bus, address)
        except OSError as exc:
            last = exc
            continue
        try:
            who = dev.read(WHO_AM_I)[0]
        except OSError as exc:
            dev.close()
            last = exc
            continue
        if who in KNOWN_IDS:
            return dev, address, KNOWN_IDS[who]
        dev.close()
        last = OSError(f"0x{address:02x} answered WHO_AM_I 0x{who:02x}, which is not an MPU")
    raise (last or FileNotFoundError(f"no MPU on {bus} at 0x68 or 0x69"))


class MpuImu:
    """Reads gyro + accel on a thread; `yaw()` any time. Same shape as
    imu.D435iImu, so the bridge takes either one.

    wheels_still: the bridge points this at its own 'wheels commanded to zero',
    which is what lets the bias re-learn while the robot is stopped."""

    who = 0x71                          # WHO_AM_I, read at start(); picks the two variations

    def __init__(self, bus: str = DEFAULT_BUS, rate_hz: float = 200.0,
                 config: YawConfig = YawConfig(),
                 wheels_still: Callable[[], bool] = lambda: True,
                 clock: Callable[[], float] = time.monotonic,
                 device: I2CDevice | None = None) -> None:
        self.bus = bus
        self.rate_hz = rate_hz
        self.tracker = YawTracker(config)
        self.wheels_still = wheels_still
        self.clock = clock
        self.part = "?"
        self.address = 0
        self._dev = device
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.error: str | None = None
        self.last_sample: float | None = None
        self.samples = 0
        self.bad_reads = 0

    # -- lifecycle --------------------------------------------------------

    def start(self) -> MpuImu:
        if self._dev is None:
            self._dev, self.address, self.part = find_device(self.bus)
        else:
            self.part = KNOWN_IDS.get(self._dev.read(WHO_AM_I)[0], "?")
        self.who = self._dev.read(WHO_AM_I)[0]
        self._configure()
        self._thread = threading.Thread(target=self._read_loop, name="mpu9250", daemon=True)
        self._thread.start()
        return self

    def _configure(self) -> None:
        d = self._dev
        assert d is not None
        d.write(PWR_MGMT_1, 0x80)           # reset
        time.sleep(0.1)
        d.write(PWR_MGMT_1, 0x01)           # wake, clock from the gyro's PLL
        time.sleep(0.01)
        d.write(PWR_MGMT_2, 0x00)           # all six axes on
        d.write(CONFIG, 0x03)               # gyro low-pass 41 Hz, 1 kHz internal rate
        d.write(GYRO_CONFIG, 0x00)          # +-250 deg/s, low-pass in use
        d.write(ACCEL_CONFIG, 0x00)         # +-2 g
        if self.who != MPU6050:             # the 6050 has no separate accel low-pass
            d.write(ACCEL_CONFIG2, 0x03)    # accel low-pass 41 Hz
        d.write(SMPLRT_DIV, 0x04)           # 1 kHz / (1 + 4) = 200 Hz
        time.sleep(0.02)

    def _read_loop(self) -> None:
        period = 1.0 / self.rate_hz
        next_at = time.monotonic()
        while not self._stop.is_set():
            try:
                raw = self._dev.read(ACCEL_XOUT_H, 14)      # type: ignore[union-attr]
            except OSError as exc:
                self.bad_reads += 1
                if self.bad_reads in (1, 50) or self.bad_reads % 500 == 0:
                    log.warning("MPU read failed (%d so far): %s", self.bad_reads, exc)
                if self.bad_reads > 200 and self.samples == 0:
                    self.error = f"the IMU on {self.bus} never answered: {exc}"
                    return
                self._stop.wait(0.01)
                continue
            accel, gyro, _ = to_si(raw, self.who)
            now = self.clock()
            with self._lock:
                self.samples += 1
                self.last_sample = now
                self.tracker.accel(accel, period)
                self.tracker.gyro(gyro, now, bool(self.wheels_still()))
            next_at += period
            delay = next_at - time.monotonic()
            if delay < -0.05:                                # fell behind: don't spiral
                next_at = time.monotonic()
                delay = 0.0
            self._stop.wait(max(0.0, delay))

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(1.0)
        if self._dev is not None:
            try:
                self._dev.write(PWR_MGMT_1, 0x40)            # sleep
            except OSError:
                pass
            self._dev.close()
            self._dev = None

    # -- what the bridge asks ---------------------------------------------

    def yaw(self) -> float | None:
        with self._lock:
            if not self.tracker.ready or self.last_sample is None:
                return None
            if self.clock() - self.last_sample > 0.25:
                return None
            return self.tracker.yaw

    def status(self) -> dict[str, Any]:
        with self._lock:
            t = self.tracker
            return {"part": self.part, "address": self.address, "ready": t.ready,
                    "yaw": t.yaw, "rate": t.rate, "bias": list(t.bias), "up": t.up,
                    "holding": t.holding, "samples": self.samples,
                    "bad_reads": self.bad_reads, "error": self.error}


def main() -> int:
    """Probe: keep the robot still for a second, then watch the heading."""
    logging.basicConfig(level=logging.INFO)
    imu = MpuImu().start()
    print(f"{imu.part} at 0x{imu.address:02x} on {imu.bus}; keep it still for the bias...")
    t0, last = time.monotonic(), 0
    try:
        while True:
            time.sleep(1.0)
            s = imu.status()
            rate, last = s["samples"] - last, s["samples"]
            up = s["up"]
            print(f"{time.monotonic() - t0:5.1f}s  {rate:3d} samples/s  ready {s['ready']}  "
                  f"holding {s['holding']}  heading {math.degrees(s['yaw']):8.2f} deg  "
                  f"rate {math.degrees(s['rate']):6.2f} deg/s  "
                  f"bias {[round(math.degrees(b), 3) for b in s['bias']]} deg/s  "
                  f"up {[round(u, 2) for u in up] if up else None}  bad reads {s['bad_reads']}")
    except KeyboardInterrupt:
        pass
    finally:
        imu.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
