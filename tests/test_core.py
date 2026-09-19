"""Shared types, the FakeRobot every skill is developed against, and the robot's memory."""

from __future__ import annotations

import dataclasses
import math
import tempfile
import unittest
from pathlib import Path

from retriever.backends.base import RobotBackend
from retriever.backends.fake import GRIPPER_CLOSED, MAX_JOINT_RATE, FakeRobot
from retriever.types import Action, Observation, Pose, Result, Station, Target


def drive(robot: FakeRobot, action: Action, seconds: float, dt: float = 0.02) -> None:
    for _ in range(round(seconds / dt)):
        robot.act(action)


class ResultTest(unittest.TestCase):
    def test_confidence_must_be_a_probability(self) -> None:
        for bad in (-0.01, 1.01, math.nan):
            with self.subTest(confidence=bad), self.assertRaises(ValueError):
                Result(ok=True, confidence=bad)
        self.assertEqual(Result(ok=True, confidence=0.0).confidence, 0.0)
        self.assertEqual(Result(ok=True).confidence, 1.0)

    def test_failed_has_zero_confidence_and_keeps_data(self) -> None:
        result = Result.failed("I closed on nothing", attempts=2)
        self.assertFalse(result.ok)
        self.assertEqual(result.confidence, 0.0)
        self.assertEqual(result.detail, "I closed on nothing")
        self.assertEqual(result.data, {"attempts": 2})

    def test_results_do_not_share_a_data_dict(self) -> None:
        self.assertIsNot(Result(ok=True).data, Result(ok=True).data)


class FrozenTypesTest(unittest.TestCase):
    def test_every_shared_type_is_immutable(self) -> None:
        instances = [
            Result(ok=True),
            Pose(1.0, 2.0, 0.5),
            Observation(joints={}, base=Pose()),
            Action(base_vx=0.1),
            Target(label="keys", bearing_rad=0.2, range_m=1.0),
            Station(name="kitchen", tag_id=3, pose=Pose()),
        ]
        for instance in instances:
            first_field = dataclasses.fields(instance)[0].name
            with (
                self.subTest(type=type(instance).__name__),
                self.assertRaises(dataclasses.FrozenInstanceError),
            ):
                setattr(instance, first_field, None)


class FakeRobotTest(unittest.TestCase):
    def test_is_a_robot_backend(self) -> None:
        self.assertIsInstance(FakeRobot(), RobotBackend)

    def test_drives_straight_forward(self) -> None:
        robot = FakeRobot()
        drive(robot, Action(base_vx=0.5), seconds=2.0)
        self.assertAlmostEqual(robot.base.x, 1.0, places=9)
        self.assertAlmostEqual(robot.base.y, 0.0, places=9)
        self.assertAlmostEqual(robot.observe().t, 2.0, places=9)

    def test_constant_twist_traces_an_exact_arc(self) -> None:
        # A quarter circle of radius vx/wz ends at (R, R) facing +y, however it is sliced.
        robot = FakeRobot(dt=0.1)
        drive(robot, Action(base_vx=0.5, base_wz=math.pi / 2), seconds=1.0, dt=0.1)
        radius = 0.5 / (math.pi / 2)
        self.assertAlmostEqual(robot.base.x, radius, places=9)
        self.assertAlmostEqual(robot.base.y, radius, places=9)
        self.assertAlmostEqual(robot.base.theta, math.pi / 2, places=9)

    def test_base_is_holonomic(self) -> None:
        robot = FakeRobot()
        drive(robot, Action(base_vy=0.25), seconds=1.0)
        self.assertAlmostEqual(robot.base.x, 0.0, places=9)
        self.assertAlmostEqual(robot.base.y, 0.25, places=9)

    def test_joints_slew_at_the_rate_limit_and_stop_on_target(self) -> None:
        robot = FakeRobot(dt=0.02)
        robot.act(Action(joints={"elbow_flex": 1.0}))
        self.assertAlmostEqual(robot.observe().joints["elbow_flex"], MAX_JOINT_RATE * 0.02)
        drive(robot, Action(), seconds=1.0)  # the target persists without being resent
        self.assertEqual(robot.observe().joints["elbow_flex"], 1.0)
        self.assertEqual(robot.observe().joints["shoulder_pan"], 0.0)

    def test_closing_on_nothing_reads_no_load(self) -> None:
        robot = FakeRobot(objects_at={"keys": Pose(2.0, 0.0)})
        drive(robot, Action(joints={"gripper": 0.0}), seconds=1.0)
        observation = robot.observe()
        self.assertLessEqual(observation.joints["gripper"], GRIPPER_CLOSED)
        self.assertIsNone(robot.holding)
        self.assertAlmostEqual(observation.gripper_load, 0.0)

    def test_closing_near_an_object_holds_it_until_opened(self) -> None:
        robot = FakeRobot(objects_at={"keys": Pose(0.1, 0.05), "mug": Pose(0.2, 0.0)})
        drive(robot, Action(joints={"gripper": 0.0}), seconds=1.0)
        self.assertEqual(robot.holding, "keys")  # the nearer of the two
        self.assertAlmostEqual(robot.observe().gripper_load, 0.6, places=1)
        drive(robot, Action(joints={"gripper": 1.0}), seconds=1.0)
        self.assertIsNone(robot.holding)
        self.assertAlmostEqual(robot.observe().gripper_load, 0.0)

    def test_driving_onto_an_object_with_the_gripper_shut_does_not_grab_it(self) -> None:
        robot = FakeRobot(objects_at={"keys": Pose(1.0, 0.0)})
        drive(robot, Action(joints={"gripper": 0.0}), seconds=1.0)
        drive(robot, Action(base_vx=0.5), seconds=2.0)
        self.assertIsNone(robot.holding)

    def test_carried_object_is_dropped_where_the_robot_is(self) -> None:
        robot = FakeRobot(objects_at={"keys": Pose(0.0, 0.0)})
        drive(robot, Action(joints={"gripper": 0.0}), seconds=1.0)
        drive(robot, Action(base_vx=1.0), seconds=2.0)
        drive(robot, Action(joints={"gripper": 1.0}), seconds=1.0)
        drive(robot, Action(joints={"gripper": 0.0}), seconds=1.0)
        self.assertEqual(robot.holding, "keys")

    def test_refuses_unknown_joints_and_non_finite_commands(self) -> None:
        robot = FakeRobot()
        for action in (Action(joints={"elbow": 1.0}), Action(base_vx=math.inf)):
            with self.subTest(action=action), self.assertRaises(ValueError):
                robot.act(action)
        self.assertEqual(robot.base, Pose())
        self.assertEqual(robot.observe().t, 0.0)

    def test_cycles_camera_frames_from_a_directory(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            Path(tmp, "0001.jpg").write_bytes(b"first")
            Path(tmp, "0002.jpg").write_bytes(b"second")
            Path(tmp, "notes.txt").write_bytes(b"ignored")
            robot = FakeRobot(frames_dir=tmp)
            frames = [robot.observe().cam_front for _ in range(3)]
        self.assertEqual(frames, [b"first", b"second", b"first"])

    def test_no_frames_dir_means_no_camera(self) -> None:
        self.assertIsNone(FakeRobot().observe().cam_front)

    def test_missing_frames_dir_fails_loudly(self) -> None:
        with tempfile.TemporaryDirectory() as tmp, self.assertRaises(FileNotFoundError):
            FakeRobot(frames_dir=Path(tmp, "typo"))


if __name__ == "__main__":
    unittest.main()
