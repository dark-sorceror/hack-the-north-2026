"""Camera geometry (perception/projection.py): the frames, the pitch sign, and the
ground-projected range an approach controller is allowed to drive.

Every expected number here is hand-computed from the pinhole model, not recorded from a
camera, so a sign that flips shows up as a failure and not as a slightly odd constant.
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.perception.projection import (
    CameraMount,
    Intrinsics,
    camera_to_base,
    deproject,
    pixel_to_target,
)

# A 320x240 camera with square pixels and its principal point in the middle.
INTR = Intrinsics(fx=100.0, fy=100.0, cx=160.0, cy=120.0)


def base_to_pixel(forward: float, left: float, up: float, intr: Intrinsics,
                  mount: CameraMount) -> tuple[float, float, float]:
    """The inverse of deproject + camera_to_base, written out here so the round trip is
    checked against independent algebra rather than against the module itself."""
    cos_p, sin_p = math.cos(mount.pitch_rad), math.sin(mount.pitch_rad)
    f, u_off = forward - mount.forward_m, up - mount.height_m
    x_right = -left
    z_fwd = f * cos_p - u_off * sin_p
    y_down = -(f * sin_p + u_off * cos_p)
    return (intr.cx + intr.fx * x_right / z_fwd, intr.cy + intr.fy * y_down / z_fwd, z_fwd)


class TestDeproject(unittest.TestCase):
    def test_hand_computed_pixels(self):
        self.assertEqual(deproject(260.0, 120.0, 2.0, INTR), (2.0, 0.0, 2.0))
        self.assertEqual(deproject(160.0, 220.0, 1.5, INTR), (0.0, 1.5, 1.5))
        self.assertEqual(deproject(110.0, 70.0, 4.0, INTR), (-2.0, -2.0, 4.0))

    def test_the_principal_point_is_straight_down_the_optical_axis(self):
        self.assertEqual(deproject(INTR.cx, INTR.cy, 3.0, INTR), (0.0, 0.0, 3.0))

    def test_x_is_right_and_y_is_down(self):
        x_right, y_down, _ = deproject(200.0, 200.0, 1.0, INTR)
        self.assertGreater(x_right, 0.0)  # right of the centre of the image
        self.assertGreater(y_down, 0.0)  # and below it: +Y points DOWN

    def test_depth_is_along_z_not_along_the_ray(self):
        # The far corner of the image at 2 m of depth is more than 2 m away from the lens.
        x_right, y_down, z_fwd = deproject(0.0, 0.0, 2.0, INTR)
        self.assertEqual(z_fwd, 2.0)
        self.assertGreater(math.sqrt(x_right**2 + y_down**2 + z_fwd**2), 2.0)


class TestCameraToBase(unittest.TestCase):
    def test_a_level_camera_only_moves_the_origin(self):
        mount = CameraMount(height_m=0.5, forward_m=0.1, pitch_rad=0.0)
        self.assertEqual(camera_to_base(0.0, 0.0, 2.0, mount), (2.1, 0.0, 0.5))

    def test_camera_right_is_base_right_which_is_negative_left(self):
        mount = CameraMount(height_m=0.5, forward_m=0.0, pitch_rad=0.0)
        forward, left, up = camera_to_base(1.0, 0.0, 2.0, mount)
        self.assertEqual((forward, left, up), (2.0, -1.0, 0.5))

    def test_camera_down_is_below_the_lens(self):
        mount = CameraMount(height_m=0.5, forward_m=0.0, pitch_rad=0.0)
        forward, left, up = camera_to_base(0.0, 0.4, 2.0, mount)
        self.assertEqual((forward, left), (2.0, 0.0))
        self.assertAlmostEqual(up, 0.1)

    def test_positive_pitch_means_tilted_down_toward_the_floor(self):
        """The sign everyone gets backwards: pitch_rad > 0 looks DOWN.

        A point 2 m along the optical axis of a camera 1 m up and pitched 0.3 rad down is
        nearer the base than 2 m and BELOW the lens; pitched up by the same angle it is
        above it. Get this backwards and the floor in front of the robot reads as a wall.
        """
        down = CameraMount(height_m=1.0, forward_m=0.0, pitch_rad=0.3)
        forward, left, up = camera_to_base(0.0, 0.0, 2.0, down)
        self.assertAlmostEqual(forward, 2.0 * math.cos(0.3))
        self.assertAlmostEqual(left, 0.0)
        self.assertAlmostEqual(up, 1.0 - 2.0 * math.sin(0.3))
        self.assertLess(up, 1.0)

        upward = CameraMount(height_m=1.0, forward_m=0.0, pitch_rad=-0.3)
        self.assertAlmostEqual(camera_to_base(0.0, 0.0, 2.0, upward)[2],
                               1.0 + 2.0 * math.sin(0.3))

    def test_a_pitched_camera_still_puts_the_floor_at_zero_height(self):
        # 1 m up, pitched 0.4 down: the floor point on the optical axis is 1/sin(0.4) away.
        mount = CameraMount(height_m=1.0, forward_m=0.0, pitch_rad=0.4)
        _, _, up = camera_to_base(0.0, 0.0, 1.0 / math.sin(0.4), mount)
        self.assertAlmostEqual(up, 0.0)

    def test_pitch_does_not_move_a_point_sideways(self):
        mount = CameraMount(height_m=0.3, forward_m=0.0, pitch_rad=0.5)
        self.assertAlmostEqual(camera_to_base(0.7, 0.2, 2.0, mount)[1], -0.7)


class TestPixelToTarget(unittest.TestCase):
    def test_a_pixel_dead_ahead_has_no_bearing(self):
        mount = CameraMount(height_m=0.2, forward_m=0.1, pitch_rad=0.0)
        target = pixel_to_target(INTR.cx, INTR.cy, 3.0, INTR, mount, "mug")
        self.assertEqual(target.bearing_rad, 0.0)
        self.assertAlmostEqual(target.range_m, 3.1)
        self.assertAlmostEqual(target.height_m, 0.2)
        self.assertEqual(target.label, "mug")

    def test_the_range_is_across_the_floor_not_down_the_ray(self):
        """A slant range fed to an approach controller overshoots by however far the
        camera is above the thing: here a 2.83 m ray to a mug 2 m away on the floor."""
        mount = CameraMount(height_m=2.0, forward_m=0.0, pitch_rad=0.0)
        target = pixel_to_target(160.0, 220.0, 2.0, INTR, mount, "mug")
        x_right, y_down, z_fwd = deproject(160.0, 220.0, 2.0, INTR)
        slant = math.sqrt(x_right**2 + y_down**2 + z_fwd**2)
        self.assertAlmostEqual(slant, 2.0 * math.sqrt(2.0))
        self.assertAlmostEqual(target.range_m, 2.0)
        self.assertAlmostEqual(target.height_m, 0.0)
        self.assertLess(target.range_m, slant)

    def test_a_pixel_left_of_centre_has_a_positive_bearing(self):
        mount = CameraMount()
        left_target = pixel_to_target(60.0, 120.0, 2.0, INTR, mount, "ball")
        right_target = pixel_to_target(260.0, 120.0, 2.0, INTR, mount, "ball")
        self.assertGreater(left_target.bearing_rad, 0.0)
        self.assertLess(right_target.bearing_rad, 0.0)
        self.assertAlmostEqual(left_target.bearing_rad, -right_target.bearing_rad)
        self.assertAlmostEqual(left_target.bearing_rad, math.atan2(2.0, 2.1))

    def test_a_thing_on_a_table_is_reported_at_its_height(self):
        mount = CameraMount(height_m=0.2, forward_m=0.0, pitch_rad=0.0)
        # 0.55 m above the lens, 1.5 m of depth ahead: a mug on a 0.75 m table.
        v = INTR.cy - 0.55 / 1.5 * INTR.fy
        target = pixel_to_target(INTR.cx, v, 1.5, INTR, mount, "mug")
        self.assertAlmostEqual(target.height_m, 0.75)
        self.assertAlmostEqual(target.range_m, 1.5)

    def test_the_labelling_fields_are_carried_through(self):
        target = pixel_to_target(160.0, 120.0, 1.0, INTR, CameraMount(), "keys",
                                 confidence=0.8, instance_id="keys-1", bbox=(1, 2, 3, 4))
        self.assertEqual((target.confidence, target.instance_id, target.bbox),
                         (0.8, "keys-1", (1, 2, 3, 4)))

    def test_a_base_frame_point_survives_the_round_trip(self):
        mount = CameraMount(height_m=0.35, forward_m=0.12, pitch_rad=0.25)
        for forward, left, up in ((1.6, 0.4, 0.05), (2.5, -0.8, 0.9), (0.9, 0.0, 0.0)):
            with self.subTest(point=(forward, left, up)):
                u, v, depth = base_to_pixel(forward, left, up, INTR, mount)
                got = camera_to_base(*deproject(u, v, depth, INTR), mount)
                for expected, actual in zip((forward, left, up), got):
                    self.assertAlmostEqual(actual, expected, places=9)
                target = pixel_to_target(u, v, depth, INTR, mount, "thing")
                self.assertAlmostEqual(target.range_m, math.hypot(forward, left), places=9)
                self.assertAlmostEqual(target.bearing_rad, math.atan2(left, forward), places=9)
                self.assertAlmostEqual(target.height_m, up, places=9)


class TestValidation(unittest.TestCase):
    def test_a_zero_focal_length_is_refused_where_it_is_written(self):
        with self.assertRaises(ValueError):
            Intrinsics(0.0, 100.0, 160.0, 120.0)
        with self.assertRaises(ValueError):
            Intrinsics(100.0, 0.0, 160.0, 120.0)

    def test_non_finite_intrinsics_are_refused(self):
        for field in range(4):
            values = [100.0, 100.0, 160.0, 120.0]
            values[field] = math.nan
            with self.subTest(field=field), self.assertRaises(ValueError):
                Intrinsics(*values)

    def test_a_depth_that_is_not_a_distance_is_refused(self):
        for depth in (0.0, -1.0, math.nan, math.inf, -math.inf):
            with self.subTest(depth=depth), self.assertRaises(ValueError):
                deproject(160.0, 120.0, depth, INTR)

    def test_a_non_finite_pixel_is_refused(self):
        with self.assertRaises(ValueError):
            deproject(math.nan, 120.0, 1.0, INTR)
        with self.assertRaises(ValueError):
            deproject(160.0, math.inf, 1.0, INTR)

    def test_a_non_finite_mount_is_refused(self):
        with self.assertRaises(ValueError):
            CameraMount(height_m=math.nan)
        with self.assertRaises(ValueError):
            CameraMount(pitch_rad=math.inf)

    def test_pixel_to_target_refuses_a_missing_depth_reading(self):
        with self.assertRaises(ValueError):
            pixel_to_target(160.0, 120.0, 0.0, INTR, CameraMount(), "mug")


if __name__ == "__main__":
    unittest.main()
