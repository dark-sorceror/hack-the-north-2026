"""The robot's own IMU on I2C (bridge/mpu.py), against a simulated MPU chip.
Stdlib only; no hardware, no clock."""

from __future__ import annotations

import math
import struct
import sys
import threading
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.bridge.mpu import (
    ACCEL_CONFIG,
    ACCEL_CONFIG2,
    ACCEL_XOUT_H,
    ADDR_HIGH,
    ADDR_LOW,
    CONFIG,
    GYRO_CONFIG,
    MPU6050,
    PWR_MGMT_1,
    SMPLRT_DIV,
    WHO_AM_I,
    MpuImu,
    find_device,
    to_si,
)


class FakeMpu:
    """An MPU on a bus: answers WHO_AM_I, records writes, and streams motion."""

    def __init__(self, who=MPU6050, up=(0.0, 0.0, 1.0), rate_about_up=0.0, bias=(0.01, -0.02, 0.0)):
        self.who, self.up, self.rate, self.bias = who, up, rate_about_up, bias
        self.writes: list[tuple[int, int]] = []
        self.reads = 0
        self.closed = False
        self.on_read = None            # called with the read count, for tests that need to stop

    def write(self, register, value):
        self.writes.append((register, value))

    def read(self, register, length=1):
        if register == WHO_AM_I:
            return bytes((self.who,))
        if register == ACCEL_XOUT_H and length == 14:
            self.reads += 1
            if self.on_read:
                self.on_read(self.reads)
            a = [u * 9.80665 for u in self.up]                   # at rest: the floor pushes up
            g = [self.bias[i] + self.rate * self.up[i] for i in range(3)]
            return struct.pack(">hhhhhhh",
                               round(a[0] / 9.80665 * 16384), round(a[1] / 9.80665 * 16384),
                               round(a[2] / 9.80665 * 16384), 0,
                               round(math.degrees(g[0]) * 131), round(math.degrees(g[1]) * 131),
                               round(math.degrees(g[2]) * 131))
        return bytes(length)

    def close(self):
        self.closed = True


class TestScaling(unittest.TestCase):
    def test_full_scale_and_gravity(self):
        raw = struct.pack(">hhhhhhh", 0, 0, 16384, 0, 32750, -131, 0)
        accel, gyro, _ = to_si(raw)
        self.assertAlmostEqual(accel[2], 9.80665, places=3)
        self.assertAlmostEqual(math.degrees(gyro[0]), 250.0, places=1)
        self.assertAlmostEqual(math.degrees(gyro[1]), -1.0, places=3)

    def test_temperature_scale_differs_by_part(self):
        raw = struct.pack(">hhhhhhh", 0, 0, 0, 0, 0, 0, 0)
        self.assertAlmostEqual(to_si(raw, MPU6050)[2], 36.53, places=2)      # 6050
        self.assertAlmostEqual(to_si(raw, 0x71)[2], 21.0, places=2)          # 9250


class TestFindDevice(unittest.TestCase):
    def opener(self, devices):
        def open_(bus, address):
            if address not in devices:
                raise OSError(f"nothing at 0x{address:02x}")
            return devices[address]
        return open_

    def test_finds_it_at_either_address(self):
        for address in (ADDR_LOW, ADDR_HIGH):
            dev, found, part = find_device("/dev/fake", self.opener({address: FakeMpu()}))
            self.assertEqual(found, address)
            self.assertEqual(part, "MPU-6050")

    def test_a_9250_is_named(self):
        _, _, part = find_device("/dev/fake", self.opener({ADDR_LOW: FakeMpu(who=0x71)}))
        self.assertEqual(part, "MPU-9250")

    def test_something_else_on_the_bus_is_refused(self):
        with self.assertRaises(OSError):
            find_device("/dev/fake", self.opener({ADDR_LOW: FakeMpu(who=0x3C)}))

    def test_an_empty_bus_raises(self):
        with self.assertRaises(OSError):
            find_device("/dev/fake", self.opener({}))


class TestConfigure(unittest.TestCase):
    def start(self, who):
        dev = FakeMpu(who=who)
        imu = MpuImu(device=dev, rate_hz=10_000.0)
        imu.start()
        imu.close()
        return dev

    def test_wakes_it_and_sets_the_ranges(self):
        w = dict(self.start(MPU6050).writes)
        self.assertEqual(w[GYRO_CONFIG], 0x00)          # +-250 deg/s
        self.assertEqual(w[ACCEL_CONFIG], 0x00)         # +-2 g
        self.assertEqual(w[CONFIG], 0x03)               # 41 Hz low-pass
        self.assertEqual(w[SMPLRT_DIV], 0x04)           # 200 Hz
        self.assertEqual(w[PWR_MGMT_1], 0x40)           # ...and asleep again after close()

    def test_the_6050_is_not_given_the_9250s_accel_filter(self):
        self.assertNotIn(ACCEL_CONFIG2, dict(self.start(MPU6050).writes))
        self.assertIn(ACCEL_CONFIG2, dict(self.start(0x71).writes))

    def test_close_puts_it_to_sleep_and_closes_the_bus(self):
        dev = FakeMpu()
        imu = MpuImu(device=dev, rate_hz=10_000.0).start()
        imu.close()
        self.assertTrue(dev.closed)


class TestReading(unittest.TestCase):
    """The read loop on a fake clock: every sample steps time by one period."""

    def run_loop(self, dev, samples, rate_hz=200.0, still=True):
        t = {"now": 0.0}
        imu = MpuImu(device=dev, rate_hz=rate_hz, clock=lambda: t["now"],
                     wheels_still=lambda: still)
        imu.start()
        done = threading.Event()

        def tick(n):
            t["now"] += 1.0 / rate_hz
            if n >= samples:
                imu._stop.set()
                done.set()
        dev.on_read = tick
        done.wait(5.0)
        imu.close()
        return imu

    def test_still_learns_the_bias_and_holds_the_heading(self):
        imu = self.run_loop(FakeMpu(bias=(0.02, 0.0, -0.01)), samples=600)   # 3 s still
        s = imu.status()
        self.assertTrue(s["ready"])
        self.assertAlmostEqual(math.degrees(s["yaw"]), 0.0, delta=0.5)
        self.assertTrue(s["holding"])
        self.assertIsNotNone(imu.yaw())

    def test_a_turn_with_the_wheels_moving_is_integrated(self):
        dev = FakeMpu(up=(0.0, -0.6, -0.8), bias=(0.01, 0.01, 0.01))
        t = {"now": 0.0}
        imu = MpuImu(device=dev, rate_hz=200.0, clock=lambda: t["now"], wheels_still=lambda: True)
        imu.start()
        done = threading.Event()

        def tick(n):
            t["now"] += 0.005
            if n == 400:                       # 2 s still: the bias settles
                dev.rate = math.radians(45)    # then a real turn...
                imu.wheels_still = lambda: False
            if n >= 800:                       # ...for 2 s = 90 degrees
                imu._stop.set()
                done.set()
        dev.on_read = tick
        done.wait(5.0)
        imu.close()
        self.assertAlmostEqual(math.degrees(imu.status()["yaw"]), 90.0, delta=2.0)

    def test_a_dead_bus_is_reported_not_crashed(self):
        class Dead(FakeMpu):
            def read(self, register, length=1):
                if register == WHO_AM_I:
                    return bytes((MPU6050,))
                raise OSError("no ack")

        dev = Dead()
        imu = MpuImu(device=dev, rate_hz=10_000.0).start()
        deadline = threading.Event()
        deadline.wait(0.5)
        imu.close()
        self.assertIsNone(imu.yaw())
        self.assertGreater(imu.bad_reads, 0)


if __name__ == "__main__":
    unittest.main()
