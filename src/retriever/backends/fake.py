"""A robot that is not there, honest about the few things skills get wrong.

Skills are written against this long before they move a motor, so it models exactly the
behaviour a skill could otherwise assume away. Joints slew at a finite rate instead of
teleporting, so a skill that expects the arm to arrive instantly fails here and not on
the robot. The gripper reports load only when it actually closed on an object, so a skill
has to check its grasp and can honestly say "I closed on nothing". The base is holonomic
with a perfect pose and integrates each twist exactly, so any error a test sees belongs
to the code under test, not to the simulator. Time advances a fixed `dt` per `act()`,
which keeps every run deterministic.
"""

from __future__ import annotations

import math
from pathlib import Path

from retriever.navigation.geometry import distance
from retriever.navigation.odometry import integrate_twist
from retriever.types import JOINTS, Action, Observation, Pose

MAX_JOINT_RATE = 2.5  # rad/s: joints slew toward targets, never teleport
GRIPPER_CLOSED = 0.15  # gripper position at/below which it counts as closed

_GRIPPER_OPEN = 1.0  # where the gripper starts
_GRASP_REACH_M = 0.25  # an object this close to the base can be grasped
_HELD_LOAD = 0.6  # normalised servo current while something is in the gripper


class FakeRobot:
    """A robot that is not there: holonomic base with a perfect pose, one arm and a gripper."""

    def __init__(
        self,
        frames_dir: Path | str | None = None,
        objects_at: dict[str, Pose] | None = None,
        dt: float = 0.02,
    ) -> None:
        if not dt > 0.0:
            raise ValueError(f"dt must be positive, got {dt!r}")
        self._dt = dt
        self._frames = _jpegs_in(Path(frames_dir)) if frames_dir is not None else []
        self._frames_shown = 0
        self._objects = dict(objects_at or {})
        self._base = Pose()
        self._joints = {name: 0.0 for name in JOINTS}
        self._joints["gripper"] = _GRIPPER_OPEN
        self._targets = dict(self._joints)
        self._holding: str | None = None
        self._steps = 0

    @property
    def base(self) -> Pose:
        """Ground-truth pose of the base."""
        return self._base

    @property
    def holding(self) -> str | None:
        """Label of the object in the gripper, or None."""
        return self._holding

    @property
    def objects(self) -> dict[str, Pose]:
        """Where the fake world's objects are now, as a snapshot.

        A simulated detector reads this to decide what the robot can see, and it changes
        as things are picked up and put down, so it is read fresh every tick. A copy, so
        nothing outside can move an object without going through the robot.
        """
        return dict(self._objects)

    def observe(self) -> Observation:
        """Snapshot the robot; each call shows the next camera frame, cycling."""
        frame = None
        if self._frames:
            frame = self._frames[self._frames_shown % len(self._frames)].read_bytes()
            self._frames_shown += 1
        return Observation(
            joints=dict(self._joints),
            base=self._base,
            cam_front=frame,
            gripper_load=_HELD_LOAD if self._holding is not None else 0.0,
            t=self._steps * self._dt,
        )

    def act(self, action: Action) -> None:
        """Apply `action` and advance the simulation by one `dt`."""
        _check(action)
        self._base = integrate_twist(
            self._base, action.base_vx, action.base_vy, action.base_wz, self._dt
        )
        was_closed = self._gripper_closed()
        self._targets.update(action.joints)
        max_step = MAX_JOINT_RATE * self._dt
        for name, target in self._targets.items():
            error = target - self._joints[name]
            if abs(error) <= max_step:
                self._joints[name] = target
            else:
                self._joints[name] += math.copysign(max_step, error)
        self._update_grasp(was_closed)
        self._steps += 1

    def close(self) -> None:
        """Nothing to release: a robot that is not there is always stopped."""

    def _gripper_closed(self) -> bool:
        return self._joints["gripper"] <= GRIPPER_CLOSED

    def _update_grasp(self, was_closed: bool) -> None:
        # Grasping happens on the closing edge: driving up to an object with the gripper
        # already shut must not pick it up.
        if self._gripper_closed() and not was_closed:
            self._holding = self._object_in_reach()
            if self._holding is not None:
                del self._objects[self._holding]
        elif not self._gripper_closed() and self._holding is not None:
            self._objects[self._holding] = self._base  # dropped where the robot is
            self._holding = None

    def _object_in_reach(self) -> str | None:
        nearest = min(
            ((distance(self._base, pose), label) for label, pose in self._objects.items()),
            default=None,
        )
        return nearest[1] if nearest is not None and nearest[0] <= _GRASP_REACH_M else None


def _jpegs_in(frames_dir: Path) -> list[Path]:
    # A mistyped path must fail loudly, not quietly produce a robot with no camera.
    if not frames_dir.is_dir():
        raise FileNotFoundError(f"frames_dir {frames_dir} is not a directory")
    return sorted(frames_dir.glob("*.jpg"))


def _check(action: Action) -> None:
    """Refuse what a real arm or base would choke on, before any state changes."""
    unknown = sorted(set(action.joints) - set(JOINTS))
    if unknown:
        raise ValueError(f"unknown joints {unknown}; this arm has {list(JOINTS)}")
    numbers = (action.base_vx, action.base_vy, action.base_wz, *action.joints.values())
    if not all(math.isfinite(value) for value in numbers):
        raise ValueError(f"non-finite value in {action!r}")
