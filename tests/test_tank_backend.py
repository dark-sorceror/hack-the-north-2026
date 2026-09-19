"""TankFakeRobot and its encoders: the robot we are building, minus the robot."""

from __future__ import annotations

import math
import unittest

from retriever.backends.base import RobotBackend
from retriever.backends.fake import MAX_JOINT_RATE
from retriever.backends.tank import COUNTS_PER_REV, TankEncoderSim, TankFakeRobot
from retriever.navigation.geometry import TAU, angle_diff, distance
from retriever.navigation.kinematics import TankGeometry
from retriever.types import Action, Pose

DT = 0.02


def drive(robot: TankFakeRobot, action: Action, seconds: float) -> None:
    for _ in range(round(seconds / DT)):
        robot.act(action)


class TankEncoderSimTest(unittest.TestCase):
    RADIUS_M = 0.05
    TICK_M = TAU * RADIUS_M / COUNTS_PER_REV

    def test_slow_wheels_still_accumulate_ticks(self) -> None:
        encoders = TankEncoderSim(self.RADIUS_M)
        quarter_tick_per_step = 0.25 * self.TICK_M / DT
        for _ in range(10):
            ticks = encoders.step(quarter_tick_per_step, 0.0, DT)
        self.assertEqual(ticks, (2, 0))
        self.assertEqual(encoders.read(), ticks)

    def test_counts_wrap_in_both_directions(self) -> None:
        encoders = TankEncoderSim(self.RADIUS_M)
        forward = (COUNTS_PER_REV + 3.5) * self.TICK_M / DT
        backward = -1.5 * self.TICK_M / DT
        self.assertEqual(encoders.step(forward, backward, DT), (3, COUNTS_PER_REV - 2))


class TankFakeRobotTest(unittest.TestCase):
    def test_is_a_robot_backend(self) -> None:
        self.assertIsInstance(TankFakeRobot(), RobotBackend)

    def test_refuses_strafe_before_anything_moves(self) -> None:
        robot = TankFakeRobot()
        drive(robot, Action(base_vx=0.3, base_wz=0.5), seconds=0.5)
        before = (robot.base, robot.ticks, robot.observe())
        with self.assertRaisesRegex(ValueError, "cannot move sideways"):
            robot.act(Action(joints={"elbow_flex": 1.0}, base_vx=0.3, base_vy=0.2))
        self.assertEqual((robot.base, robot.ticks, robot.observe()), before)

    def test_estimate_tracks_truth_when_the_geometry_matches(self) -> None:
        robot = TankFakeRobot()
        drive(robot, Action(base_vx=0.4), seconds=3.0)
        drive(robot, Action(base_vx=0.3, base_wz=0.8), seconds=4.0)
        drive(robot, Action(base_wz=-1.5), seconds=2.0)
        drive(robot, Action(base_vx=-0.2, base_wz=0.3), seconds=3.0)
        estimate, truth = robot.observe().base, robot.base
        self.assertGreater(distance(Pose(), truth), 0.5)
        self.assertLess(distance(estimate, truth), 1e-3)
        self.assertLess(abs(angle_diff(estimate.theta, truth.theta)), 1e-3)

    def test_miscalibrated_scrub_drifts_the_heading_but_not_the_distance(self) -> None:
        robot = TankFakeRobot(true_geo=TankGeometry(scrub_factor=1.4))
        drive(robot, Action(base_vx=0.5), seconds=2.0)
        self.assertAlmostEqual(robot.observe().base.x, robot.base.x, delta=1e-3)

        drive(robot, Action(base_wz=1.0), seconds=math.pi / 2)
        believed, actual = robot.observe().base.theta, robot.base.theta
        self.assertAlmostEqual(believed, math.pi / 2, delta=0.02)
        self.assertAlmostEqual(actual, believed / 1.4, delta=0.01)

    def test_wrong_wheel_radius_shows_up_as_distance(self) -> None:
        robot = TankFakeRobot(true_geo=TankGeometry(wheel_radius_m=0.05))
        drive(robot, Action(base_vx=0.5), seconds=2.0)
        self.assertAlmostEqual(robot.observe().base.x, 1.0, delta=1e-3)
        self.assertAlmostEqual(robot.base.x, 1.0 * 0.05 / 0.048, delta=1e-3)

    def test_encoder_wraps_on_a_long_drive_do_not_corrupt_the_estimate(self) -> None:
        robot = TankFakeRobot()
        drive(robot, Action(base_vx=0.5), seconds=20.0)  # about 33 wheel revolutions
        self.assertAlmostEqual(robot.observe().base.x, robot.base.x, delta=1e-3)
        self.assertAlmostEqual(robot.base.x, 10.0, places=6)
        drive(robot, Action(base_vx=-0.5), seconds=20.0)
        self.assertLess(distance(robot.observe().base, Pose()), 1e-3)
        self.assertTrue(all(0 <= count < COUNTS_PER_REV for count in robot.ticks))

    def test_arm_and_gripper_behave_like_the_fake_robot(self) -> None:
        robot = TankFakeRobot(objects_at={"keys": Pose(0.1, 0.0)})
        robot.act(Action(joints={"elbow_flex": 1.0}))
        self.assertAlmostEqual(robot.observe().joints["elbow_flex"], MAX_JOINT_RATE * DT)
        drive(robot, Action(joints={"gripper": 0.0}), seconds=1.0)
        self.assertEqual(robot.holding, "keys")
        self.assertAlmostEqual(robot.observe().gripper_load, 0.6, places=1)


if __name__ == "__main__":
    unittest.main()
