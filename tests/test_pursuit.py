"""Pure pursuit along drawn waypoints (navigation/pursuit.py), and the D435i
gyro over raw HID (bridge/imu.py). No robot, no clock."""

from __future__ import annotations

import math
import os
import struct
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.bridge.imu import (
    REPORT_16BIT,
    REPORT_32BIT,
    REPORT_ACCEL,
    REPORT_GYRO,
    D435iImu,
    HIDIOCGFEATURE,
    HIDIOCSFEATURE,
    YawTracker,
    find_hidraw,
    parse_report,
)
from retriever.navigation.drive import Limits
from retriever.navigation.odometry import integrate_twist
from retriever.navigation.pursuit import PathTracker, path_length, waypoint_curve
from retriever.types import Observation, Pose

WAYPOINTS = [(0.0, 0.0), (1.0, 0.4), (1.8, -0.2), (2.6, 0.5)]


def drive(ctl, belief, true=None, slip=1.0, dt=0.05, t_max=90.0):
    """Closed loop on perfect kinematics; `true` turns only `slip` of what the
    odometry (belief) thinks: a skid-steer's wheels sliding."""
    true = true or belief
    t, statuses, off = 0.0, [], 0.0
    while t < t_max:
        a, done = ctl.step_observation(Observation(joints={}, base=belief, t=t))
        if done:
            return True, belief, true, statuses, off
        statuses.append(ctl.plan.status)
        belief = integrate_twist(belief, a.base_vx, 0.0, a.base_wz, dt)
        true = integrate_twist(true, a.base_vx, 0.0, a.base_wz * slip, dt)
        off = max(off, min(math.dist((belief.x, belief.y), p) for p in ctl.path))
        t += dt
    return False, belief, true, statuses, off


class TestCurve(unittest.TestCase):
    def test_passes_through_every_click(self):
        c = waypoint_curve(WAYPOINTS)
        for w in WAYPOINTS:
            self.assertLess(min(math.dist(w, p) for p in c), 0.005)
        self.assertEqual(c[0], WAYPOINTS[0])
        self.assertAlmostEqual(c[-1][0], WAYPOINTS[-1][0], places=6)

    def test_is_sampled_finely_and_never_loops_back(self):
        c = waypoint_curve([(0, 0), (0.05, 0.0), (1.0, 1.0)])     # two clicks very close
        steps = [math.dist(a, b) for a, b in zip(c, c[1:])]
        self.assertLess(max(steps), 0.06)
        # centripetal: no overshoot beyond the clicks' bounding box by more than a little
        self.assertGreater(min(p[0] for p in c), -0.05)

    def test_two_points_is_a_line_and_duplicates_are_dropped(self):
        c = waypoint_curve([(0, 0), (0, 0), (1, 0)])
        self.assertTrue(all(abs(p[1]) < 1e-9 for p in c))
        self.assertAlmostEqual(path_length(c), 1.0, places=6)


class TestTracker(unittest.TestCase):
    def limits(self):
        return Limits(v_max=0.3, w_max=1.0, pos_tol=0.06)

    def test_follows_a_curve_smoothly_to_its_end(self):
        curve = waypoint_curve(WAYPOINTS)
        done, belief, _, statuses, off = drive(PathTracker(curve, self.limits()), Pose())
        self.assertTrue(done)
        self.assertLess(math.dist((belief.x, belief.y), curve[-1]), 0.1)
        self.assertLess(off, 0.15)                               # stays on the curve
        switches = sum(1 for a, b in zip(statuses, statuses[1:]) if a != b)
        self.assertLessEqual(switches, 2)                        # no turn/drive flicker

    def test_turns_on_the_spot_first_when_the_path_is_behind(self):
        curve = waypoint_curve([(0, 0), (-1.0, 0.1), (-2.0, 0.0)])
        done, belief, _, statuses, _ = drive(PathTracker(curve, self.limits()), Pose())
        self.assertTrue(done)
        self.assertEqual(statuses[0], "turning")
        self.assertLess(math.dist((belief.x, belief.y), curve[-1]), 0.1)

    def test_reversing_back_cancels_turn_slip(self):
        """Out with 10% of every turn lost to slip, then back the same curve in
        reverse: each turn back undoes one out, so it ends where it started.
        Turning round instead adds a 180 that nothing undoes."""
        curve = waypoint_curve(WAYPOINTS)
        for reverse, limit in ((True, 0.12), (False, None)):
            with self.subTest(reverse=reverse):
                _, b, tr, _, _ = drive(PathTracker(curve, self.limits()), Pose(), slip=0.9)
                back = list(reversed(curve))
                done, b2, tr2, _, _ = drive(PathTracker(back, self.limits(), reverse=reverse),
                                            b, tr, slip=0.9)
                self.assertTrue(done)
                err = math.hypot(tr2.x, tr2.y)
                if limit is not None:
                    self.assertLess(err, limit)
                else:
                    self.assertGreater(err, 0.3)                 # the turn-round doesn't cancel

    def test_reverse_drives_backwards(self):
        curve = [(0.0, 0.0), (-1.0, 0.0)]
        ctl = PathTracker(curve, self.limits(), reverse=True)
        a, _ = ctl.step_observation(Observation(joints={}, base=Pose(), t=0.0))
        self.assertLess(a.base_vx, 0.0)
        self.assertEqual(ctl.plan.status, "following")

    def test_short_paths_are_refused(self):
        with self.assertRaises(ValueError):
            PathTracker([(0, 0)])


def gyro_report(x, y, z, size=38, rid=REPORT_GYRO):
    if size == 38:
        return REPORT_32BIT.pack(rid, 0, 123456, x, y, z) + bytes(16)
    return REPORT_16BIT.pack(rid, 0, 123456, x, y, z) + bytes(16)


class TestImuReports(unittest.TestCase):
    def test_ioctl_numbers_match_the_kernel(self):
        self.assertEqual(HIDIOCGFEATURE(9), 0xC0094807)
        self.assertEqual(HIDIOCSFEATURE(9), 0xC0094806)

    def test_new_firmware_gyro_is_a_ten_thousandth_of_a_degree(self):
        rid, _, (x, y, z) = parse_report(gyro_report(900000, -450000, 0))
        self.assertEqual(rid, REPORT_GYRO)
        self.assertAlmostEqual(x, math.radians(90.0), places=6)
        self.assertAlmostEqual(y, math.radians(-45.0), places=6)

    def test_old_firmware_gyro_is_a_tenth_of_a_degree(self):
        _, _, (x, _, _) = parse_report(gyro_report(900, 0, 0, size=32))
        self.assertAlmostEqual(x, math.radians(90.0), places=6)

    def test_accel_is_milli_g(self):
        _, _, (_, y, _) = parse_report(gyro_report(0, -1000, 0, rid=REPORT_ACCEL))
        self.assertAlmostEqual(y, -9.80665, places=4)

    def test_anything_else_is_ignored(self):
        self.assertIsNone(parse_report(b"\x01" * 20))
        self.assertIsNone(parse_report(gyro_report(0, 0, 0, rid=3)))

    def test_finds_the_camera_in_sysfs(self):
        with tempfile.TemporaryDirectory() as d:
            for name, hid in (("hidraw0", "0003:0000046D:0000C52B"), ("hidraw1", "0003:00008086:00000B3A")):
                os.makedirs(os.path.join(d, name, "device"))
                with open(os.path.join(d, name, "device", "uevent"), "w") as f:
                    f.write(f"HID_ID={hid}\nHID_NAME=x\n")
            self.assertEqual(find_hidraw(d), "/dev/hidraw1")
            self.assertIsNone(find_hidraw(os.path.join(d, "nothing")))


class TestYaw(unittest.TestCase):
    UP = (0.0, -0.6, -0.8)       # a camera pitched down: "up" is not any one axis

    def feed(self, tr, seconds, rate_about_up, bias=(0.01, -0.02, 0.005), still=True, t0=0.0,
             hz=200):
        g = [-9.80665 * u for u in self.UP]          # the floor pushes up: accel reads +up
        g = [-x for x in g]
        t = t0
        for k in range(int(seconds * hz)):
            if k % 3 == 0:
                tr.accel(tuple(g), 3 / hz)
            w = tuple(bias[i] + rate_about_up * self.UP[i] for i in range(3))
            tr.gyro(w, t, still)
            t += 1 / hz
        return t

    def test_heading_about_up_with_the_bias_removed(self):
        tr = YawTracker()
        t = self.feed(tr, 1.0, 0.0)                   # still: learns the bias
        self.assertTrue(tr.ready)
        self.assertAlmostEqual(tr.yaw, 0.0, places=3)
        t = self.feed(tr, 2.0, math.radians(45), still=False, t0=t)   # turns left 90 deg
        self.assertAlmostEqual(math.degrees(tr.yaw), 90.0, delta=1.0)
        self.feed(tr, 2.0, math.radians(-45), still=False, t0=t)      # and back
        self.assertAlmostEqual(math.degrees(tr.yaw), 0.0, delta=1.0)

    def test_bias_does_not_learn_while_the_wheels_turn(self):
        tr = YawTracker()
        t = self.feed(tr, 1.0, 0.0)
        bias = list(tr.bias)
        self.feed(tr, 3.0, 0.01, still=False, t0=t)  # a slow real turn, wheels moving
        self.assertEqual(tr.bias, bias)
        self.assertGreater(tr.yaw, 0.02)

    def test_nothing_before_the_bias_is_known(self):
        tr = YawTracker()
        self.assertFalse(tr.ready)
        tr.gyro((0.5, 0.0, 0.0), 0.0, False)
        self.assertEqual(tr.yaw, 0.0)


class TestImuReader(unittest.TestCase):
    """The read loop, on a datagram socket standing in for /dev/hidrawN: like
    hidraw, and unlike a pipe, one read returns exactly one report."""

    def test_reads_reports_into_a_heading(self):
        import socket

        r, w = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.addCleanup(w.close)
        imu = D435iImu(path="/dev/null")
        imu._fd = r.fileno()
        th = threading.Thread(target=imu._read_loop, daemon=True)
        th.start()
        accel = gyro_report(0, 0, 1000, rid=REPORT_ACCEL)          # up = +z
        for k in range(300):                                        # ~1.5 s still at 200 Hz
            if k % 3 == 0:
                w.send(accel)
            w.send(gyro_report(0, 0, 0))
            time.sleep(0.005)
        time.sleep(0.1)
        self.assertEqual(imu.report_size, 38)
        self.assertEqual(imu.counts[REPORT_GYRO], 300)
        self.assertEqual(imu.counts[REPORT_ACCEL], 100)
        self.assertIsNotNone(imu.yaw())
        imu._stop.set()
        w.send(gyro_report(0, 0, 0))                                # wake the blocked read
        th.join(1.0)
        r.close()


if __name__ == "__main__":
    unittest.main()
