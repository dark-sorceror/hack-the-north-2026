"""Shared types, the FakeRobot every skill is developed against, and the robot's memory."""

from __future__ import annotations

import dataclasses
import math
import unittest

from retriever.types import Action, Observation, Pose, Result, Station, Target


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


if __name__ == "__main__":
    unittest.main()
