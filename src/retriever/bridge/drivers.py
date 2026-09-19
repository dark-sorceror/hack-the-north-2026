"""One HardwareDriver assembled from the parts the robot actually has.

The bridge wants a single driver, but the robot is a base on one serial bus, arms on
another and maybe a vacuum on a GPIO pin, each written and tested on its own. This module
composes them. Routing is decided once, at construction: a joint name owned by two arms is
a wiring mistake and fails at startup, not when the arm first moves. The one rule that
matters is in `stop()`: the base stops FIRST, because it is the part that can hit someone,
and every part gets its stop even if an earlier one raised; the first error is re-raised
afterwards so the failure is still heard.
"""

from __future__ import annotations

import logging
from typing import Any, Protocol

from retriever.navigation.kinematics import TankGeometry

logger = logging.getLogger(__name__)


class BaseDriver(Protocol):
    """The drive base: two sides of wheels with encoders."""

    counts_per_rev: int

    def set_wheels(self, left_mps: float, right_mps: float) -> None:
        """Command left and right rim speeds in m/s."""
        ...

    def read_ticks(self) -> tuple[int, int]:
        """The (left, right) encoder counts."""
        ...

    def battery(self) -> float | None:
        """Charge in 0..1, or None if the base cannot tell."""
        ...

    def stop(self) -> None:
        """Wheels to zero now."""
        ...


class ArmDriver(Protocol):
    """One arm, optionally with a gripper."""

    joint_names: tuple[str, ...]
    has_gripper: bool

    def set_joints(self, targets: dict[str, float]) -> None:
        """Move these joints toward targets in radians."""
        ...

    def set_gripper(self, position: float) -> None:
        """Move the gripper toward `position`."""
        ...

    def read_joints(self) -> dict[str, float]:
        """Joint positions in radians, with "gripper" if the arm has one."""
        ...

    def gripper_load(self) -> float:
        """Normalised gripper servo current, 0..1."""
        ...

    def hold(self) -> None:
        """Stop moving and keep torque on, so the arm holds where it is."""
        ...


class VacuumDriver(Protocol):
    """A suction pump on a relay."""

    def set_vacuum(self, on: bool) -> None:
        """Switch the pump on or off."""
        ...


class CompositeDriver:
    """One HardwareDriver from a base, any number of arms and an optional vacuum."""

    def __init__(
        self,
        base: BaseDriver,
        arms: dict[str, ArmDriver] | None = None,
        vacuum: VacuumDriver | None = None,
    ) -> None:
        self._base = base
        self._arms = dict(arms or {})
        self._vacuum = vacuum
        self.counts_per_rev = base.counts_per_rev
        self._owner: dict[str, str] = {}  # joint name -> name of the arm that has it
        for arm_name, arm in self._arms.items():
            for joint in arm.joint_names:
                if joint in self._owner:
                    raise ValueError(
                        f"joint {joint!r} belongs to both arm {self._owner[joint]!r} and arm "
                        f"{arm_name!r}; joint names must be unique across arms"
                    )
                self._owner[joint] = arm_name
        self._gripper_arm = next((arm for arm in self._arms.values() if arm.has_gripper), None)

    def set_wheels(self, left_mps: float, right_mps: float) -> None:
        """Command the base's rim speeds."""
        self._base.set_wheels(left_mps, right_mps)

    def set_joints(self, targets: dict[str, float]) -> None:
        """Send each joint target to the arm that owns it; an unknown name moves nothing."""
        unknown = sorted(set(targets) - set(self._owner))
        if unknown:
            raise ValueError(f"unknown joints {unknown}; the arms have {sorted(self._owner)}")
        per_arm: dict[str, dict[str, float]] = {}
        for joint, angle in targets.items():
            per_arm.setdefault(self._owner[joint], {})[joint] = angle
        for arm_name, arm_targets in per_arm.items():
            self._arms[arm_name].set_joints(arm_targets)

    def set_gripper(self, position: float) -> None:
        """Move the gripper of the first arm that has one."""
        if self._gripper_arm is None:
            raise ValueError("no arm has a gripper")
        self._gripper_arm.set_gripper(position)

    def set_vacuum(self, on: bool) -> None:
        """Switch the vacuum; raises if none is attached."""
        if self._vacuum is None:
            raise ValueError("no vacuum is attached")
        self._vacuum.set_vacuum(on)

    def read_state(self) -> dict[str, Any]:
        """Ticks and battery from the base; joints, gripper and load from the arms."""
        left_ticks, right_ticks = self._base.read_ticks()
        joints: dict[str, float] = {}
        for arm in self._arms.values():
            reading = arm.read_joints()
            joints.update((name, angle) for name, angle in reading.items() if name != "gripper")
            if arm is self._gripper_arm:
                joints["gripper"] = reading["gripper"]
        load = 0.0 if self._gripper_arm is None else self._gripper_arm.gripper_load()
        battery = self._base.battery()
        return {
            "left_ticks": left_ticks,
            "right_ticks": right_ticks,
            "joints": joints,
            "gripper_load": load,
            "battery": 1.0 if battery is None else battery,
        }

    def stop(self) -> None:
        """Base first, then every arm holds; each part is stopped even if one raises."""
        stops = [("the base", self._base.stop)]
        stops += [(f"arm {name!r}", arm.hold) for name, arm in self._arms.items()]
        first_error: Exception | None = None
        for part, stop in stops:
            try:
                stop()
            except Exception as exc:
                logger.exception("stopping %s failed", part)
                if first_error is None:
                    first_error = exc
        if first_error is not None:
            raise first_error


def build_real_driver(
    wheel_port: str, arm_port: str | None = None, geo: TankGeometry = TankGeometry()
) -> CompositeDriver:
    """The robot's real hardware; not written yet, so this always raises."""
    raise NotImplementedError(
        f"the DDSM115 wheel driver (for {wheel_port}) and the SO-101 arm driver are not "
        "written yet; run the bridge with --driver fake"
    )
