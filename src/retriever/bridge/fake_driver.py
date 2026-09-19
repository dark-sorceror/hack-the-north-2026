"""A HardwareDriver with nothing attached, running on the wall clock.

It exists so the whole bridge, and the laptop on the other end of it, can run with no robot
in the room. It runs on real time, not per-call steps: every call first catches the
simulation up to `clock()`, so wheels commanded to turn keep turning (and encoders keep
counting) until something explicitly stops them. A stalled link therefore really does let
the fake keep driving, which is what makes the watchdog and deadman tests honest. Encoders
are TankEncoderSim, the same quantised, wrapping counts the laptop's odometry must survive.
`stop()` freezes the arm where it IS, not where it was going, because that is what holding
torque on a real servo does.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Iterable
from typing import Any

from retriever.backends.fake import GRIPPER_CLOSED, MAX_JOINT_RATE
from retriever.backends.tank import COUNTS_PER_REV, TankEncoderSim
from retriever.navigation.kinematics import TankGeometry
from retriever.types import JOINTS

ARM_JOINTS = tuple(j for j in JOINTS if j != "gripper")

_GRIPPER_OPEN = 1.0  # where the gripper starts
_HELD_LOAD = 0.6  # normalised servo current while the gripper is closed on an object


class FakeTankDriver:
    """HardwareDriver for a skid-steer base and one arm that are not there."""

    def __init__(
        self,
        geo: TankGeometry = TankGeometry(),
        counts_per_rev: int = COUNTS_PER_REV,
        joint_names: Iterable[str] = ARM_JOINTS,
        clock: Callable[[], float] = time.monotonic,
        max_joint_rate: float = MAX_JOINT_RATE,
        battery_life_s: float = 3600.0,
    ) -> None:
        if not battery_life_s > 0.0:
            raise ValueError(f"battery_life_s must be positive, got {battery_life_s!r}")
        self.counts_per_rev = counts_per_rev
        self._encoders = TankEncoderSim(geo.wheel_radius_m, counts_per_rev)
        self._joint_names = frozenset(joint_names)
        self._clock = clock
        self._max_joint_rate = max_joint_rate
        self._battery_life_s = battery_life_s
        self._joints = {name: 0.0 for name in self._joint_names}
        self._joints["gripper"] = _GRIPPER_OPEN
        self._targets = dict(self._joints)
        self._started_at = self._caught_up_to = clock()
        # Test affordances.
        self.wheel_speeds = (0.0, 0.0)  # last commanded (left, right) rim speeds, m/s
        self.stop_count = 0
        self.vacuum = False
        self.object_in_gripper = False  # closing the gripper on it reports load

    def set_wheels(self, left_mps: float, right_mps: float) -> None:
        """Spin the wheels at these rim speeds from now until told otherwise."""
        self._catch_up()
        self.wheel_speeds = (left_mps, right_mps)

    def set_joints(self, targets: dict[str, float]) -> None:
        """Slew the named joints toward `targets`; an unknown name raises before any moves."""
        unknown = sorted(set(targets) - self._joint_names)
        if unknown:
            known = sorted(self._joint_names)
            raise ValueError(f"unknown joints {unknown}; this arm has {known}")
        self._catch_up()
        self._targets.update(targets)

    def set_gripper(self, position: float) -> None:
        """Slew the gripper toward `position`."""
        self._catch_up()
        self._targets["gripper"] = position

    def set_vacuum(self, on: bool) -> None:
        """Switch the (imaginary) vacuum."""
        self._catch_up()
        self.vacuum = on

    def read_state(self) -> dict[str, Any]:
        """Wrapped encoder counts, joint positions, gripper load and battery, as of now."""
        self._catch_up()
        left_ticks, right_ticks = self._encoders.read()
        closed_on_object = self.object_in_gripper and self._joints["gripper"] <= GRIPPER_CLOSED
        elapsed = self._caught_up_to - self._started_at
        return {
            "left_ticks": left_ticks,
            "right_ticks": right_ticks,
            "joints": dict(self._joints),
            "gripper_load": _HELD_LOAD if closed_on_object else 0.0,
            "battery": max(0.0, 1.0 - elapsed / self._battery_life_s),
        }

    def stop(self) -> None:
        """Wheels to zero; arm and gripper freeze where they are; the vacuum is untouched."""
        self._catch_up()
        self.wheel_speeds = (0.0, 0.0)
        self._targets = dict(self._joints)
        self.stop_count += 1

    def _catch_up(self) -> None:
        """Advance the wheels and joints to `clock()` under the current commands."""
        now = self._clock()
        dt = now - self._caught_up_to
        if dt <= 0.0:
            return
        self._caught_up_to = now
        self._encoders.step(*self.wheel_speeds, dt)
        max_step = self._max_joint_rate * dt
        for name, target in self._targets.items():
            error = target - self._joints[name]
            if abs(error) <= max_step:
                self._joints[name] = target
            else:
                self._joints[name] += math.copysign(max_step, error)
