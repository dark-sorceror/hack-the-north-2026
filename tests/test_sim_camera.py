"""SimCamera: what a depth camera would report as obstacles, for the navigation
session's simulator tests (the same shape as CameraObstacleSource)."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.navigation.simscan import box
from retriever.perception.sim_camera import SimCamera
from retriever.types import Pose


def nearest(points):
    return min(points, key=lambda p: math.hypot(*p))


class TestSimCamera(unittest.TestCase):
    def cam(self, world, pose=Pose(), **kw):
        return SimCamera(world, lambda: pose, **kw)

    def test_a_box_ahead_is_seen_at_its_distance(self):
        x, y = nearest(self.cam(box(1.5, 0.0, 0.4, 0.4)).latest().points)
        self.assertAlmostEqual(x, 1.3, delta=0.02)          # its front face
        self.assertAlmostEqual(y, 0.0, delta=0.05)

    def test_a_black_box_is_seen_too(self):
        # p_return=0: the lidar gets nothing back from it, a depth camera sees it
        self.assertTrue(self.cam(box(1.5, 0.0, 0.4, 0.4, p_return=0.0)).latest().points)

    def test_nothing_outside_the_field_of_view(self):
        self.assertEqual(self.cam(box(-1.5, 0.0, 0.4, 0.4)).latest().points, ())   # behind
        self.assertEqual(self.cam(box(0.0, 1.5, 0.4, 0.4)).latest().points, ())    # beside

    def test_points_are_in_the_robot_frame_wherever_it_is(self):
        pose = Pose(2.0, 1.0, math.pi / 2)                  # facing +y in the world
        x, y = nearest(self.cam(box(2.0, 2.5, 0.4, 0.4), pose).latest().points)
        self.assertAlmostEqual(x, 1.3, delta=0.02)
        self.assertAlmostEqual(y, 0.0, delta=0.05)

    def test_a_box_on_the_left_has_positive_y(self):
        pts = self.cam(box(1.2, 0.6, 0.3, 0.3)).latest().points
        self.assertTrue(pts)
        self.assertTrue(all(y > 0 for _, y in pts))

    def test_one_frame_per_period(self):
        ticks = iter([0.0, 0.05, 0.1])
        cam = self.cam(box(1.5, 0.0, 0.4, 0.4), hz=10.0, clock=lambda: next(ticks))
        first = cam.latest()
        self.assertIs(cam.latest(), first)                  # 0.05: same frame
        second = cam.latest()                               # 0.10: a new one
        self.assertIsNot(second, first)
        self.assertEqual(second.t, 0.1)

    def test_age_and_describe_match_the_contract(self):
        cam = self.cam(box(1.5, 0.0, 0.4, 0.4))
        self.assertEqual(cam.age(), 0.0)                    # simulated: always fresh
        self.assertIn("ahead 1.3", cam.describe())


if __name__ == "__main__":
    unittest.main()
