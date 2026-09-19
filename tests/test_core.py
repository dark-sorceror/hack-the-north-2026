"""Shared types, the FakeRobot every skill is developed against, and the robot's memory."""

from __future__ import annotations

import dataclasses
import math
import tempfile
import time
import unittest
from pathlib import Path

from retriever.backends.base import RobotBackend
from retriever.backends.fake import GRIPPER_CLOSED, MAX_JOINT_RATE, FakeRobot
from retriever.memory import Memory, Sighting
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


class MemoryTest(unittest.TestCase):
    def setUp(self) -> None:
        self.memory = Memory()
        self.addCleanup(self.memory.close)

    def test_teach_round_trips_within_float32_precision(self) -> None:
        embedding = [0.1, -2.5, 1e-3, 123.456]
        object_id = self.memory.teach("keys", embedding)
        [(stored_id, name, stored)] = self.memory.taught()
        self.assertEqual((stored_id, name), (object_id, "keys"))
        for want, got in zip(embedding, stored, strict=True):
            self.assertTrue(math.isclose(want, got, rel_tol=1e-6), (want, got))

    def test_reteaching_a_name_replaces_it_and_keeps_its_id(self) -> None:
        first = self.memory.teach("keys", [1.0, 0.0])
        self.memory.teach("mug", [0.0, 1.0])
        second = self.memory.teach("Keys", [0.5, 0.5])
        self.assertEqual(first, second)
        taught = {name.lower(): embedding for _, name, embedding in self.memory.taught()}
        self.assertEqual(taught, {"keys": [0.5, 0.5], "mug": [0.0, 1.0]})

    def test_teach_refuses_empty_or_non_finite_embeddings(self) -> None:
        for embedding in ([], [1.0, math.nan]):
            with self.subTest(embedding=embedding), self.assertRaises(ValueError):
                self.memory.teach("keys", embedding)

    def test_last_seen_is_the_newest_sighting_not_the_last_written(self) -> None:
        self.memory.saw("keys", Pose(1.0, 1.0), 0.9, at=2000.0)
        self.memory.saw("keys", Pose(3.0, 3.0), 0.7, object_id="k1", at=1000.0)
        self.assertEqual(
            self.memory.last_seen("keys"), Sighting("keys", Pose(1.0, 1.0), 0.9, seen_at=2000.0)
        )

    def test_sighting_age(self) -> None:
        sighting = Sighting("keys", Pose(), 0.9, seen_at=2000.0)
        self.assertEqual(sighting.age_s(now=2060.0), 60.0)
        self.assertGreater(sighting.age_s(), 0.0)

    def test_labels_are_case_insensitive(self) -> None:
        self.memory.saw("Keys", Pose(1.0, 0.0), 0.8, at=1000.0)
        expected = Sighting("Keys", Pose(1.0, 0.0), 0.8, seen_at=1000.0)
        self.assertEqual(self.memory.last_seen("keys"), expected)
        self.assertEqual(self.memory.last_seen("KEYS"), expected)

    def test_never_seen_is_none(self) -> None:
        self.assertIsNone(self.memory.last_seen("unicorn"))

    def test_saw_refuses_an_invalid_confidence(self) -> None:
        with self.assertRaises(ValueError):
            self.memory.saw("keys", Pose(), 1.5)

    def test_summary_lists_the_newest_sighting_per_label_newest_first(self) -> None:
        now = time.time()
        self.memory.saw("keys", Pose(9.0, 9.0), 0.5, at=now - 3600)
        self.memory.saw("Keys", Pose(1.2, 0.4), 0.82, at=now - 180)
        self.memory.saw("mug", Pose(0.5, -1.0), 0.6, at=now - 2 * 3600)
        self.memory.saw("remote", Pose(0.0, 2.0), 0.95, at=now - 1)
        self.assertEqual(
            self.memory.summary().splitlines(),
            [
                "remote: (0.00, 2.00) just now, conf 0.95",
                "Keys: (1.20, 0.40) 3 min ago, conf 0.82",
                "mug: (0.50, -1.00) 2 h ago, conf 0.60",
            ],
        )
        self.assertEqual(len(self.memory.summary(limit=2).splitlines()), 2)

    def test_summary_says_plainly_when_memory_is_empty(self) -> None:
        self.assertIn("empty", self.memory.summary().lower())

    def test_file_backed_memory_persists_and_creates_its_directory(self) -> None:
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        path = Path(tmp.name, "runs", "today", "memory.db")
        memory = Memory(path)
        object_id = memory.teach("keys", [1.0, 2.0])
        memory.saw("keys", Pose(1.0, 2.0, 0.5), 0.9, object_id=object_id, at=1000.0)
        memory.close()

        reopened = Memory(path)
        self.addCleanup(reopened.close)
        self.assertEqual(reopened.taught(), [(object_id, "keys", [1.0, 2.0])])
        self.assertEqual(
            reopened.last_seen("keys"),
            Sighting("keys", Pose(1.0, 2.0, 0.5), 0.9, seen_at=1000.0, object_id=object_id),
        )


if __name__ == "__main__":
    unittest.main()
