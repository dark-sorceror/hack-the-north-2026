"""Depth grid -> obstacle points (perception/depth_obstacles.py).

Every test renders a synthetic depth image by ray casting a flat floor and
axis-aligned boxes from a known camera mount, so the expected answer is
geometry, not a recording. numpy only.
"""

import math
import sys
import unittest
from importlib.util import find_spec
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

HAVE_NUMPY = find_spec("numpy") is not None
if HAVE_NUMPY:
    import numpy as np

from retriever.perception.depth_obstacles import (
    fit_floor,
    grid_intrinsics,
    obstacle_points,
)
from retriever.perception.projection import CameraMount, Intrinsics, camera_to_base, deproject

W, H = 170, 96                                  # an 848x480 D435i grid at 5 px cells
INTR = Intrinsics(fx=90.0, fy=90.0, cx=84.5, cy=47.5)
MOUNT = CameraMount(height_m=0.30, forward_m=0.20, pitch_rad=0.15)


def render(mount, boxes=(), floor=True, w=W, h=H, intr=INTR, far=8.0):
    """Depth (metres along the camera's z; 0 = no reading) of a flat floor and
    axis-aligned boxes (base frame: x0, x1, y0, y1, z0, z1; y > 0 is LEFT)."""
    v, u = np.mgrid[0:h, 0:w].astype(np.float64)
    # A ray whose camera-frame z is 1: its parameter along the ray IS the depth.
    dx, dy, dz = camera_to_base((u - intr.cx) / intr.fx, (v - intr.cy) / intr.fy,
                                np.ones_like(u), CameraMount(0.0, 0.0, mount.pitch_rad))
    ox, oy, oz = mount.forward_m, 0.0, mount.height_m
    depth = np.full((h, w), np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        if floor:
            t = -oz / dz
            depth = np.where((dz < 0) & (t > 0), t, depth)
        for x0, x1, y0, y1, z0, z1 in boxes:
            tx = ((x0 - ox) / dx, (x1 - ox) / dx)
            ty = ((y0 - oy) / dy, (y1 - oy) / dy)
            tz = ((z0 - oz) / dz, (z1 - oz) / dz)
            tmin = np.maximum.reduce([np.minimum(*tx), np.minimum(*ty), np.minimum(*tz)])
            tmax = np.minimum.reduce([np.maximum(*tx), np.maximum(*ty), np.maximum(*tz)])
            hit = (tmax >= tmin) & (tmin > 0)
            depth = np.where(hit & (tmin < depth), tmin, depth)
    depth[~np.isfinite(depth) | (depth > far)] = 0.0
    return depth


def nearest(points):
    return min(points, key=lambda p: math.hypot(*p))


@unittest.skipUnless(HAVE_NUMPY, "numpy not installed")
class TestObstaclePoints(unittest.TestCase):
    def test_a_flat_floor_makes_no_points(self):
        self.assertEqual(obstacle_points(render(MOUNT), INTR, MOUNT), [])

    def test_the_floor_stays_floor_with_the_camera_tilted_further_down(self):
        m = CameraMount(height_m=0.30, forward_m=0.20, pitch_rad=0.35)
        self.assertEqual(obstacle_points(render(m), INTR, m), [])

    def test_a_box_ahead_is_about_a_metre_ahead(self):
        box = (1.0, 1.3, -0.15, 0.15, 0.0, 0.25)          # 25 cm tall, front face 1 m out
        x, y = nearest(obstacle_points(render(MOUNT, [box]), INTR, MOUNT))
        self.assertAlmostEqual(math.hypot(x, y), 1.0, delta=0.05)
        self.assertLess(abs(math.atan2(y, x)), math.radians(10))

    def test_a_box_on_the_left_has_positive_y(self):
        box = (1.0, 1.3, 0.25, 0.55, 0.0, 0.25)           # y > 0 is LEFT
        pts = obstacle_points(render(MOUNT, [box]), INTR, MOUNT)
        self.assertTrue(pts)
        self.assertTrue(all(y > 0 for _, y in pts))

    def test_a_table_top_within_the_robots_height_is_kept(self):
        top = (0.8, 1.6, -0.4, 0.4, 0.60, 0.62)           # a slab the lidar's slice misses
        pts = obstacle_points(render(MOUNT, [top]), INTR, MOUNT)
        self.assertTrue(pts)
        self.assertGreater(math.hypot(*nearest(pts)), 0.9)

    def test_something_above_the_robot_is_dropped(self):
        ceiling = (0.8, 2.5, -1.0, 1.0, 0.90, 0.92)
        self.assertEqual(obstacle_points(render(MOUNT, [ceiling]), INTR, MOUNT), [])

    def test_one_bad_pixel_is_not_an_obstacle(self):
        d = render(MOUNT)
        d[40, 85] = 0.8
        self.assertEqual(obstacle_points(d, INTR, MOUNT), [])

    def test_kth_nearest_points_in_one_bin_are(self):
        d = render(MOUNT)
        d[38:41, 85] = 0.8                                # three readings, one bearing
        pts = obstacle_points(d, INTR, MOUNT)
        self.assertEqual(len(pts), 1)
        self.assertAlmostEqual(math.hypot(*pts[0]), 1.0, delta=0.05)

    def test_grid_intrinsics_put_cell_centres_where_their_pixels_are(self):
        full = Intrinsics(446.8, 433.0, 423.5, 239.5)
        g = grid_intrinsics(full, 5)
        # cell (84, 47) is full-res pixels 420..424 x 235..239: centre (422, 237)
        a = deproject(422.0, 237.0, 1.0, full)
        b = deproject(84.0, 47.0, 1.0, g)
        for p, q in zip(a, b):
            self.assertAlmostEqual(p, q, places=9)

    def test_projection_accepts_arrays(self):
        xs, ys, zs = np.array([0.1, -0.2]), np.array([0.05, 0.3]), np.array([1.0, 2.0])
        f, l, u = camera_to_base(xs, ys, zs, MOUNT)
        for i in range(2):
            fi, li, ui = camera_to_base(float(xs[i]), float(ys[i]), float(zs[i]), MOUNT)
            self.assertAlmostEqual(f[i], fi)
            self.assertAlmostEqual(l[i], li)
            self.assertAlmostEqual(u[i], ui)


@unittest.skipUnless(HAVE_NUMPY, "numpy not installed")
class TestFitFloor(unittest.TestCase):
    def test_recovers_height_and_pitch(self):
        for h, p in ((0.30, 0.15), (0.45, 0.30)):
            m = CameraMount(height_m=h, forward_m=0.2, pitch_rad=p)
            got_h, got_p = fit_floor(render(m), INTR)
            self.assertAlmostEqual(got_h, h, delta=0.005)
            self.assertAlmostEqual(got_p, p, delta=0.005)

    def test_ignores_a_box_in_the_frame(self):
        got_h, got_p = fit_floor(render(MOUNT, [(0.8, 1.0, -0.1, 0.1, 0.0, 0.2)]), INTR)
        self.assertAlmostEqual(got_h, 0.30, delta=0.01)
        self.assertAlmostEqual(got_p, 0.15, delta=0.01)

    def test_needs_floor_in_view(self):
        with self.assertRaises(ValueError):
            fit_floor(np.zeros((H, W)), INTR)


if __name__ == "__main__":
    unittest.main()
