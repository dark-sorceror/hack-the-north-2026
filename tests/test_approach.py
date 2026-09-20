"""`/approach`: a robot-frame fix from another machine becomes a world goal.

This is the seam the central Pi drives the robot through. The two things worth
pinning down are the frame conversion (the caller says "1.5 m ahead, 0.3 m to my
left"; the Mac turns that into the odometry frame using the pose it HAD when the
camera saw it) and the dating, which must work without the two machines sharing
a clock.
"""

from __future__ import annotations

import logging
import math
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.bridge.client import BridgeRobot
from retriever.bridge.fake_driver import FakeTankDriver
from retriever.bridge.server import BridgeServer, ServerThread
from retriever.teleop import TeleopSession

_QUIET = logging.NullHandler()


def setUpModule():
    logging.getLogger("retriever.bridge").addHandler(_QUIET)


def tearDownModule():
    logging.getLogger("retriever.bridge").removeHandler(_QUIET)


class ApproachCase(unittest.TestCase):
    def setUp(self):
        self.drv = FakeTankDriver()
        self.server = BridgeServer(self.drv, "127.0.0.1", 0, timeout_ms=300,
                                   motion_timeout_ms=500, state_hz=50.0)
        self.st = ServerThread(self.server)
        self.port = self.st.start()
        self.addCleanup(self.st.stop)

    def session(self, **kw):
        s = TeleopSession(lambda: BridgeRobot("127.0.0.1", self.port), **kw).start()
        self.addCleanup(s.close)
        self.assertTrue(s.wait_connected(3.0), s.link_detail)
        return s

    def settled(self, **kw):
        """A session that has accumulated some pose history.

        Worth knowing rather than hiding: for the first instants after the link
        comes up there is no history, so a *dated* approach is refused rather
        than placed against a pose nobody recorded. That is the right answer --
        it just means the first detection cannot arrive in the same breath as
        the connection.
        """
        s = self.session(**kw)
        time.sleep(0.3)                       # ~15 states at 50 Hz
        return s


class TestFrameConversion(ApproachCase):
    def test_straight_ahead_becomes_a_goal_straight_ahead(self):
        s = self.session()
        out = s.approach(2.0, 0.0)
        # Fresh pose is the origin facing +x, so the goal is just 2 m along it.
        self.assertAlmostEqual(out["x"], 2.0, places=2)
        self.assertAlmostEqual(out["y"], 0.0, places=2)
        self.assertAlmostEqual(out["range_m"], 2.0, places=2)

    def test_y_is_left_not_right(self):
        """The sign that mirrors the whole world if it is wrong."""
        s = self.session()
        out = s.approach(0.0, 1.0)
        self.assertGreater(out["y"], 0.5, "+y must be LEFT in the robot frame")

    def test_standoff_stops_short_along_the_same_line(self):
        s = self.session()
        out = s.approach(2.0, 0.0, standoff_m=0.5)
        self.assertAlmostEqual(out["x"], 1.5, places=2)
        self.assertAlmostEqual(out["standoff_m"], 0.5, places=3)
        # It stops short; it does not move sideways off the line of approach.
        self.assertAlmostEqual(out["y"], 0.0, places=2)

    def test_standoff_never_overshoots_backwards(self):
        """A standoff bigger than the range must not put the goal behind us."""
        s = self.session()
        out = s.approach(0.30, 0.0, standoff_m=5.0)
        self.assertGreater(out["x"], 0.0)
        self.assertLess(out["standoff_m"], 0.30)

    def test_it_reports_the_pose_it_used(self):
        """So a wrong destination can be blamed on the right half."""
        s = self.session()
        out = s.approach(1.0, 0.0)
        self.assertEqual(len(out["from"]), 3)


class TestDating(ApproachCase):
    def test_age_s_is_accepted_from_a_machine_with_no_shared_clock(self):
        s = self.settled()
        out = s.approach(1.0, 0.0, age_s=0.2)
        self.assertAlmostEqual(out["range_m"], 1.0, places=2)

    def test_age_s_beyond_the_pose_history_is_refused_not_guessed(self):
        s = self.session()
        with self.assertRaises(RuntimeError) as ctx:
            s.approach(1.0, 0.0, age_s=30.0)
        self.assertIn("ago", str(ctx.exception))

    def test_t_wins_when_both_are_given(self):
        s = self.settled()
        now = time.monotonic()
        out = s.approach(1.0, 0.0, now, age_s=30.0)   # age_s would have failed
        self.assertAlmostEqual(out["range_m"], 1.0, places=2)

    def test_negative_age_is_rejected(self):
        s = self.session()
        with self.assertRaises(ValueError):
            s.approach(1.0, 0.0, age_s=-1.0)

    def test_no_dating_at_all_means_now(self):
        s = self.session()
        out = s.approach(1.0, 0.0)
        self.assertAlmostEqual(out["range_m"], 1.0, places=2)


class TestRefusals(ApproachCase):
    def test_non_finite_is_rejected(self):
        s = self.session()
        for bad in (float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                s.approach(bad, 0.0)

    def test_where_the_robot_already_is_is_refused(self):
        s = self.session()
        with self.assertRaises(RuntimeError):
            s.approach(0.0, 0.0)


if __name__ == "__main__":
    unittest.main()
