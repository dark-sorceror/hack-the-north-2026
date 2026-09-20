"""Gyro heading from the Pi to the laptop's odometry: the State.yaw field, the
bridge server passing an IMU through, and TankOdometry taking its turns from
the gyro. Stdlib only."""

from __future__ import annotations

import math
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.bridge.client import BridgeRobot
from retriever.bridge.fake_driver import FakeTankDriver
from retriever.bridge.protocol import SERVER_MESSAGES, State, decode, encode
from retriever.bridge.server import BridgeServer, ServerThread
from retriever.navigation.kinematics import TankGeometry
from retriever.navigation.odometry import TankOdometry
from retriever.types import Action


class TestWire(unittest.TestCase):
    def test_yaw_round_trips_and_is_left_off_without_a_gyro(self):
        s = State(seq=1, t=0.5, left_ticks=0, right_ticks=0, yaw=-1.25)
        self.assertEqual(decode(encode(s), SERVER_MESSAGES).yaw, -1.25)
        line = encode(State(seq=1, t=0.5, left_ticks=0, right_ticks=0))
        self.assertNotIn(b"yaw", line)                      # byte-identical to a v1 Pi
        self.assertIsNone(decode(line, SERVER_MESSAGES).yaw)


class TestOdometry(unittest.TestCase):
    geo = TankGeometry(wheel_radius_m=0.05, track_width_m=0.3, scrub_factor=1.0)
    cpr = 4096

    def ticks_for(self, rad_left, rad_right):
        k = self.cpr / (2 * math.pi)
        return round(rad_left * k), round(rad_right * k)

    def spin(self, odo, wheel_turn_rad, gyro_turn_rad, steps=100, dt=0.02):
        """Spin on the spot: the wheels claim wheel_turn_rad, the gyro measures gyro_turn_rad."""
        rim = wheel_turn_rad * self.geo.effective_track_m / 2.0     # arc per side
        wheel_rad = rim / self.geo.wheel_radius_m
        odo.update(0, 0, 1.0, 0.0)
        for k in range(1, steps + 1):
            left, right = self.ticks_for(-wheel_rad * k / steps, wheel_rad * k / steps)
            odo.update(left % self.cpr, right % self.cpr, dt, gyro_turn_rad * k / steps)
        return odo.pose

    def test_the_gyro_decides_the_turn(self):
        odo = TankOdometry(geo=self.geo, counts_per_rev=self.cpr)
        pose = self.spin(odo, wheel_turn_rad=math.radians(100), gyro_turn_rad=math.radians(90))
        self.assertAlmostEqual(math.degrees(pose.theta), 90.0, delta=0.5)
        self.assertEqual(odo.heading_source, "gyro")
        # the wheels said ~11% more than the gyro: the slip, measured live
        self.assertAlmostEqual(odo.turn_wheels_rad / odo.turn_gyro_rad, 100 / 90, delta=0.02)

    def test_no_gyro_is_the_wheels_as_before(self):
        odo = TankOdometry(geo=self.geo, counts_per_rev=self.cpr)
        odo.update(0, 0, 1.0)
        rim = math.radians(90) * self.geo.effective_track_m / 2.0 / self.geo.wheel_radius_m
        for k in range(1, 101):
            left, right = self.ticks_for(-rim * k / 100, rim * k / 100)
            odo.update(left % self.cpr, right % self.cpr, 0.02)
        self.assertAlmostEqual(math.degrees(odo.pose.theta), 90.0, delta=0.5)
        self.assertEqual(odo.heading_source, "wheels")

    def test_a_gap_in_the_gyro_is_not_counted_twice(self):
        odo = TankOdometry(geo=self.geo, counts_per_rev=self.cpr)
        odo.update(0, 0, 1.0, 0.0)
        odo.update(0, 0, 0.02, 0.0)
        odo.update(0, 0, 0.02, None)                        # gyro missing for a step...
        odo.update(0, 0, 0.02, 1.0)                         # ...back, far away: re-seed only
        self.assertAlmostEqual(odo.pose.theta, 0.0, places=6)
        odo.update(0, 0, 0.02, 1.1)                         # from here, changes count again
        self.assertAlmostEqual(odo.pose.theta, 0.1, places=6)


class FakeImu:
    """yaw() = what the robot REALLY turned (from the fake tank's true pose),
    while the wheels over-count: a skid-steer slipping."""

    def __init__(self, driver, slip=0.8):
        self.driver, self.slip = driver, slip
        self.wheels_still = None
        self.closed = False

    def yaw(self):
        return self.slip * self.driver_theta()

    def driver_theta(self):
        left, right = self.driver.wheel_travel_m      # unwrapped rim distance, metres
        return (right - left) / self.driver.geo.effective_track_m

    def close(self):
        self.closed = True


class TestEndToEnd(unittest.TestCase):
    def test_the_laptop_heading_follows_the_gyro_not_the_wheels(self):
        drv = FakeTankDriver()
        imu = FakeImu(drv, slip=0.8)
        server = BridgeServer(drv, "127.0.0.1", 0, timeout_ms=300, motion_timeout_ms=500,
                              state_hz=50.0, imu=imu)
        st = ServerThread(server)
        port = st.start()
        self.addCleanup(st.stop)
        self.assertTrue(callable(imu.wheels_still))           # the core told it when it's stopped
        self.assertTrue(st.call(imu.wheels_still))

        bot = BridgeRobot("127.0.0.1", port)
        self.addCleanup(bot.close)
        t0 = bot.observe().t
        obs = bot.observe()
        while obs.t - t0 < 1.0:
            bot.act(Action(base_wz=1.0))                      # spin left for a second
            obs = bot.observe()
        bot.act(Action())
        time.sleep(0.2)
        obs = bot.observe()
        wheels = st.call(imu.driver_theta)
        self.assertGreater(wheels, 0.5)
        self.assertEqual(bot.odometry.heading_source, "gyro")
        self.assertAlmostEqual(obs.base.theta, 0.8 * wheels, delta=0.03)   # the gyro's, not the wheels'

    def test_server_close_closes_the_gyro(self):
        drv = FakeTankDriver()
        imu = FakeImu(drv)
        st = ServerThread(BridgeServer(drv, "127.0.0.1", 0, imu=imu))
        st.start()
        st.stop()
        self.assertTrue(imu.closed)


if __name__ == "__main__":
    unittest.main()
