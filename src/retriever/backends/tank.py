"""The fake for the robot we are actually building: a skid steer that only knows its encoders.

FakeRobot's holonomic base and perfect pose would hide the failures that matter most on
this chassis, so `TankFakeRobot` refuses sideways velocity before anything moves, drives
its wheels through tank kinematics, and reports the pose its own quantised, wrapping
encoders imply rather than the truth. `true_geo` gives the "real" chassis different numbers
from the ones the controller believes, so a miscalibrated scrub factor shows up the way it
does on the floor: straight lines look perfect while the heading drifts. The arm, gripper
and camera are FakeRobot's, unchanged. `TankEncoderSim` is shared with the Pi's fake
driver, so both ends of the bridge count ticks the same way.
"""

from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

from retriever.backends.fake import FakeRobot
from retriever.navigation.geometry import TAU
from retriever.navigation.kinematics import (
    TankGeometry,
    assert_no_strafe,
    tank_body_to_wheels,
    tank_wheels_to_body,
)
from retriever.navigation.odometry import TankOdometry
from retriever.types import Action, Observation, Pose

COUNTS_PER_REV = 4096


class TankEncoderSim:
    """Left and right wrapping 12-bit wheel encoders, driven by rim speed."""

    def __init__(self, wheel_radius_m: float, counts_per_rev: int = COUNTS_PER_REV) -> None:
        if not wheel_radius_m > 0.0:
            raise ValueError(f"wheel_radius_m must be positive, got {wheel_radius_m!r}")
        if counts_per_rev <= 0:
            raise ValueError(f"counts_per_rev must be positive, got {counts_per_rev!r}")
        self._counts_per_rev = counts_per_rev
        self._counts_per_m = counts_per_rev / (TAU * wheel_radius_m)
        # Unwrapped and fractional: a wheel creeping at a fraction of a tick per step still
        # adds up, instead of rounding to zero every step and never moving at all.
        self._left = 0.0
        self._right = 0.0

    def step(self, v_left: float, v_right: float, dt: float) -> tuple[int, int]:
        """Turn the wheels at rim speeds (m/s) for `dt` seconds; return the wrapped counts."""
        self._left += v_left * dt * self._counts_per_m
        self._right += v_right * dt * self._counts_per_m
        return self.read()

    def read(self) -> tuple[int, int]:
        """The counts an encoder would report now, in 0 .. counts_per_rev - 1."""
        return (
            math.floor(self._left) % self._counts_per_rev,
            math.floor(self._right) % self._counts_per_rev,
        )


class TankFakeRobot:
    """FakeRobot's surface on a skid-steer base that knows its pose only from its encoders."""

    def __init__(
        self,
        frames_dir: Path | str | None = None,
        objects_at: dict[str, Pose] | None = None,
        dt: float = 0.02,
        geo: TankGeometry = TankGeometry(),
        true_geo: TankGeometry | None = None,
        counts_per_rev: int = COUNTS_PER_REV,
    ) -> None:
        # The world: ground-truth pose, arm, gripper and camera.
        self._world = FakeRobot(frames_dir, objects_at, dt)
        self._dt = dt
        self._geo = geo
        self._true_geo = geo if true_geo is None else true_geo
        self._encoders = TankEncoderSim(self._true_geo.wheel_radius_m, counts_per_rev)
        self._odometry = TankOdometry(geo, counts_per_rev)
        self._odometry.update(*self._encoders.read(), dt)  # the first reading only records

    @property
    def base(self) -> Pose:
        """Ground truth, which the robot itself never sees."""
        return self._world.base

    @property
    def ticks(self) -> tuple[int, int]:
        """The wrapped (left, right) encoder counts."""
        return self._encoders.read()

    @property
    def holding(self) -> str | None:
        """Label of the object in the gripper, or None."""
        return self._world.holding

    def observe(self) -> Observation:
        """Snapshot the robot; `base` is the odometry estimate, not the ground truth."""
        return replace(self._world.observe(), base=self._odometry.pose)

    def act(self, action: Action) -> None:
        """Refuse any sideways velocity, then drive the wheels and advance one `dt`."""
        assert_no_strafe(action.base_vy)
        v_left, v_right = tank_body_to_wheels(action.base_vx, action.base_wz, self._geo)
        # The motor controllers spin each wheel at the rate the believed radius implies; the
        # chassis then rolls on its true radius and turns about its true, scrubbed track.
        wheel_scale = self._true_geo.wheel_radius_m / self._geo.wheel_radius_m
        rim_left, rim_right = v_left * wheel_scale, v_right * wheel_scale
        vx, wz = tank_wheels_to_body(rim_left, rim_right, self._true_geo)
        self._world.act(replace(action, base_vx=vx, base_vy=0.0, base_wz=wz))
        self._odometry.update(*self._encoders.step(rim_left, rim_right, self._dt), self._dt)

    def close(self) -> None:
        """Close the simulated world underneath."""
        self._world.close()
