"""Pure pursuit along drawn waypoints (navigation/pursuit.py), and the D435i
gyro over raw HID (bridge/imu.py). No robot, no clock."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

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


if __name__ == "__main__":
    unittest.main()
