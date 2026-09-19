"""Skid-steer kinematics, exact odometry and the frame helpers everything else leans on."""

from __future__ import annotations

import math
import unittest

from retriever.navigation.geometry import (
    TAU,
    angle_diff,
    base_to_world,
    clamp_velocity,
    distance,
    pose_error,
    target_offset,
    world_to_base,
    wrap_angle,
)
from retriever.navigation.kinematics import (
    TankGeometry,
    assert_no_strafe,
    tank_body_to_wheels,
    tank_wheels_to_body,
)
from retriever.navigation.odometry import TankOdometry, integrate_twist, unwrap_ticks
from retriever.types import Pose, Target


class PoseAssertions(unittest.TestCase):
    def assertPoseAlmostEqual(self, got: Pose, want: Pose, tol: float = 1e-9) -> None:
        self.assertLess(math.hypot(got.x - want.x, got.y - want.y), tol, (got, want))
        self.assertLess(abs(angle_diff(got.theta, want.theta)), tol, (got, want))


class UnwrapTicksTest(unittest.TestCase):
    def test_forward_across_rollover(self) -> None:
        self.assertEqual(unwrap_ticks(5, 4090, 4096), 11)

    def test_backward_across_rollover(self) -> None:
        self.assertEqual(unwrap_ticks(4090, 5, 4096), -11)

    def test_plain_and_cumulative_counts(self) -> None:
        self.assertEqual(unwrap_ticks(1100, 1000, 4096), 100)
        self.assertEqual(unwrap_ticks(50_000, 49_000, 4096), 1000)
        self.assertEqual(unwrap_ticks(-20, 30, 4096), -50)
        self.assertEqual(unwrap_ticks(7, 7, 4096), 0)


class IntegrateTwistTest(PoseAssertions):
    def test_one_big_step_equals_many_small_ones(self) -> None:
        start, (vx, vy, wz) = Pose(1.0, -2.0, 0.3), (0.4, 0.1, 0.9)
        one_step = integrate_twist(start, vx, vy, wz, 2.0)
        many_steps = start
        for _ in range(1000):
            many_steps = integrate_twist(many_steps, vx, vy, wz, 0.002)
        self.assertPoseAlmostEqual(one_step, many_steps)

        # Euler's straight chord lands somewhere else entirely; that is what exactness buys.
        euler = Pose(
            start.x + (vx * math.cos(start.theta) - vy * math.sin(start.theta)) * 2.0,
            start.y + (vx * math.sin(start.theta) + vy * math.cos(start.theta)) * 2.0,
        )
        self.assertGreater(distance(euler, one_step), 0.1)

    def test_full_circle_returns_to_the_start(self) -> None:
        start = Pose(0.5, 0.5, 1.0)
        self.assertPoseAlmostEqual(integrate_twist(start, 0.3, 0.0, 1.0, TAU), start)

    def test_straight_line_follows_the_heading(self) -> None:
        end = integrate_twist(Pose(1.0, 1.0, math.pi / 2), 0.5, 0.0, 0.0, 2.0)
        self.assertPoseAlmostEqual(end, Pose(1.0, 2.0, math.pi / 2))

    def test_spin_in_place_turns_without_moving(self) -> None:
        end = integrate_twist(Pose(2.0, 3.0, 0.0), 0.0, 0.0, 1.0, 3 * math.pi / 2)
        self.assertPoseAlmostEqual(end, Pose(2.0, 3.0, -math.pi / 2))
        self.assertEqual(end.theta, wrap_angle(end.theta))

    def test_a_barely_turning_arc_is_a_straight_line(self) -> None:
        arc = integrate_twist(Pose(), 1.0, 0.0, 1e-12, 1.0)
        self.assertPoseAlmostEqual(arc, Pose(1.0, 0.0, 1e-12))


class TankOdometryTest(PoseAssertions):
    GEO = TankGeometry(wheel_radius_m=0.05, track_width_m=0.3)
    METRES_PER_TICK = TAU * 0.05 / 4096

    def test_first_update_only_records_ticks(self) -> None:
        odometry = TankOdometry(self.GEO)
        self.assertEqual(odometry.update(1234, 777, 0.02), Pose())

    def test_equal_ticks_drive_straight(self) -> None:
        odometry = TankOdometry(self.GEO)
        for step in range(5):  # one wheel revolution, a quarter turn per reading
            odometry.update(step * 1024, step * 1024, 0.02)
        self.assertPoseAlmostEqual(odometry.pose, Pose(TAU * 0.05, 0.0, 0.0))
        self.assertAlmostEqual(odometry.distance_travelled_m, TAU * 0.05)

    def test_opposite_ticks_spin_in_place(self) -> None:
        odometry = TankOdometry(self.GEO)
        odometry.update(0, 0, 0.02)
        odometry.update(-1000, 1000, 0.02)
        turn = 2 * 1000 * self.METRES_PER_TICK / 0.3
        self.assertPoseAlmostEqual(odometry.pose, Pose(0.0, 0.0, turn))
        self.assertEqual(odometry.distance_travelled_m, 0.0)

    def test_rollover_does_not_teleport(self) -> None:
        odometry = TankOdometry(self.GEO)
        odometry.update(4090, 4090, 0.02)
        odometry.update(5, 5, 0.02)
        self.assertPoseAlmostEqual(odometry.pose, Pose(11 * self.METRES_PER_TICK, 0.0, 0.0))

    def test_reset_reseeds_the_pose_and_forgets_the_last_ticks(self) -> None:
        odometry = TankOdometry(self.GEO)
        odometry.update(0, 0, 0.02)
        odometry.reset(Pose(1.0, 2.0, 0.5))
        self.assertEqual(odometry.update(3000, 100, 0.02), Pose(1.0, 2.0, 0.5))

    def test_bad_dt_is_refused_without_losing_the_motion(self) -> None:
        odometry = TankOdometry(self.GEO)
        odometry.update(0, 0, 0.02)
        with self.assertRaises(ValueError):
            odometry.update(100, 100, 0.0)
        odometry.update(100, 100, 0.02)
        self.assertAlmostEqual(odometry.pose.x, 100 * self.METRES_PER_TICK)


class KinematicsTest(unittest.TestCase):
    def test_round_trip(self) -> None:
        geo = TankGeometry(scrub_factor=1.3)
        for vx, wz in [(0.0, 0.0), (0.5, 0.0), (0.0, 1.2), (-0.3, 0.7), (0.4, -2.0)]:
            with self.subTest(vx=vx, wz=wz):
                back = tank_wheels_to_body(*tank_body_to_wheels(vx, wz, geo), geo)
                self.assertAlmostEqual(back[0], vx)
                self.assertAlmostEqual(back[1], wz)

    def test_forward_is_equal_wheels_and_a_left_turn_speeds_up_the_right(self) -> None:
        self.assertEqual(tank_body_to_wheels(0.5, 0.0), (0.5, 0.5))
        v_left, v_right = tank_body_to_wheels(0.0, 1.0)
        self.assertAlmostEqual(v_left, -0.15)
        self.assertAlmostEqual(v_right, 0.15)

    def test_scrub_factor_slows_rotation_but_not_forward_speed(self) -> None:
        ideal, scrubby = TankGeometry(), TankGeometry(scrub_factor=1.5)
        self.assertAlmostEqual(scrubby.effective_track_m, 0.45)
        vx_ideal, wz_ideal = tank_wheels_to_body(0.1, 0.4, ideal)
        vx_scrub, wz_scrub = tank_wheels_to_body(0.1, 0.4, scrubby)
        self.assertEqual(vx_scrub, vx_ideal)
        self.assertAlmostEqual(wz_scrub, wz_ideal / 1.5)

    def test_geometry_refuses_impossible_numbers(self) -> None:
        for bad in (
            {"wheel_radius_m": 0.0},
            {"track_width_m": -0.3},
            {"scrub_factor": 0.9},
            {"wheel_radius_m": math.nan},
            {"track_width_m": math.inf},
        ):
            with self.subTest(**bad), self.assertRaises(ValueError):
                TankGeometry(**bad)

    def test_assert_no_strafe_refuses_sideways_velocity_loudly(self) -> None:
        for vy in (0.1, -0.1, math.nan):
            with (
                self.subTest(vy=vy),
                self.assertRaisesRegex(ValueError, "cannot move sideways"),
            ):
                assert_no_strafe(vy)
        assert_no_strafe(0.0)
        assert_no_strafe(1e-9)


class GeometryTest(unittest.TestCase):
    def test_wrap_angle_edges(self) -> None:
        cases = {
            0.0: 0.0,
            math.pi: -math.pi,
            -math.pi: -math.pi,
            3 * math.pi: -math.pi,
            -3 * math.pi: -math.pi,
            TAU: 0.0,
            math.pi / 2 + 4 * TAU: math.pi / 2,
        }
        for angle, wrapped in cases.items():
            with self.subTest(angle=angle):
                self.assertAlmostEqual(wrap_angle(angle), wrapped)
        for angle in (math.nextafter(-math.pi, -4.0), math.nextafter(math.pi, 4.0), 1e6):
            with self.subTest(angle=angle):
                self.assertTrue(-math.pi <= wrap_angle(angle) < math.pi)

    def test_angle_diff_takes_the_short_way_round(self) -> None:
        self.assertAlmostEqual(
            angle_diff(math.radians(-170), math.radians(170)), math.radians(20)
        )
        self.assertAlmostEqual(
            angle_diff(math.radians(170), math.radians(-170)), math.radians(-20)
        )
        self.assertAlmostEqual(angle_diff(0.5, 0.2), 0.3)

    def test_world_to_base_and_back(self) -> None:
        facing_north = math.pi / 2
        forward, left = world_to_base(0.0, 1.0, facing_north)
        self.assertAlmostEqual(forward, 1.0)
        self.assertAlmostEqual(left, 0.0)
        forward, left = world_to_base(1.0, 0.0, facing_north)
        self.assertAlmostEqual(forward, 0.0)
        self.assertAlmostEqual(left, -1.0)
        x, y = base_to_world(*world_to_base(0.3, -1.7, 2.1), 2.1)
        self.assertAlmostEqual(x, 0.3)
        self.assertAlmostEqual(y, -1.7)

    def test_target_offset_puts_positive_bearing_on_the_left(self) -> None:
        forward, left = target_offset(
            Target(label="keys", bearing_rad=math.pi / 2, range_m=2.0)
        )
        self.assertAlmostEqual(forward, 0.0)
        self.assertAlmostEqual(left, 2.0)
        self.assertEqual(
            target_offset(Target(label="mug", bearing_rad=0.0, range_m=1.5)), (1.5, 0.0)
        )

    def test_pose_error_is_in_the_base_frame(self) -> None:
        forward, left, heading = pose_error(Pose(1.0, 1.0, math.pi / 2), Pose(0.0, 3.0, 0.0))
        self.assertAlmostEqual(forward, 2.0)
        self.assertAlmostEqual(left, 1.0)
        self.assertAlmostEqual(heading, -math.pi / 2)

    def test_distance_ignores_heading(self) -> None:
        self.assertEqual(distance(Pose(1.0, 1.0, 0.3), Pose(4.0, 5.0, -2.0)), 5.0)

    def test_clamp_velocity_scales_but_keeps_direction(self) -> None:
        self.assertEqual(clamp_velocity(0.3, 0.4, 1.0), (0.3, 0.4))
        vx, vy = clamp_velocity(3.0, -4.0, 1.0)
        self.assertAlmostEqual(vx, 0.6)
        self.assertAlmostEqual(vy, -0.8)
        self.assertEqual(clamp_velocity(1.0, 1.0, 0.0), (0.0, 0.0))
        with self.assertRaises(ValueError):
            clamp_velocity(1.0, 0.0, -1.0)


if __name__ == "__main__":
    unittest.main()
