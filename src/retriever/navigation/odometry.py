"""Wheel odometry for the skid steer: where the robot believes it is between tag fixes.

Two things wreck odometry without raising an error. Encoders wrap: a 12-bit wheel encoder
rolls over every revolution, so 4090 -> 5 is eleven ticks forward, not four thousand back,
and a naive difference teleports the robot. `unwrap_ticks` takes the short way round,
which is right as long as a wheel turns less than half a revolution between readings. And
Euler integration cuts every corner with a straight line, so the pose would depend on how
often states happen to arrive. `integrate_twist` follows the exact arc of a constant twist
instead, so one long step and a thousand short ones land in the same place.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from retriever.navigation.geometry import TAU, base_to_world, wrap_angle
from retriever.navigation.kinematics import TankGeometry, tank_wheels_to_body
from retriever.types import Pose

_STRAIGHT_TURN_RAD = 1e-9  # below this heading change the arc is a straight line


def unwrap_ticks(new: int, old: int, counts_per_rev: int) -> int:
    """The shortest signed tick delta from `old` to `new` on an encoder that wraps."""
    if counts_per_rev <= 0:
        raise ValueError(f"counts_per_rev must be positive, got {counts_per_rev!r}")
    delta = (new - old) % counts_per_rev
    return delta - counts_per_rev if 2 * delta >= counts_per_rev else delta


def integrate_twist(pose: Pose, vx: float, vy: float, wz: float, dt: float) -> Pose:
    """Where `pose` ends up after holding body twist (vx, vy, wz) for `dt` seconds.

    Exact for a constant twist: the path is an arc, not the chord Euler would draw.
    """
    turn = wz * dt
    if abs(turn) < _STRAIGHT_TURN_RAD:
        forward, left = vx * dt, vy * dt
    else:
        along, across = math.sin(turn) / wz, (1.0 - math.cos(turn)) / wz
        forward, left = vx * along - vy * across, vx * across + vy * along
    dx, dy = base_to_world(forward, left, pose.theta)
    return Pose(pose.x + dx, pose.y + dy, wrap_angle(pose.theta + turn))


@dataclass
class TankOdometry:
    """Pose from left/right encoder ticks, integrated exactly between readings."""

    geo: TankGeometry = field(default_factory=TankGeometry)
    counts_per_rev: int = 4096
    pose: Pose = field(default_factory=Pose)
    distance_travelled_m: float = 0.0  # path length of the base centre; survives reset()
    # With a gyro: how much turning the wheels claimed vs what the gyro measured,
    # over the same steps. Their ratio is the skid-steer's slip, live.
    turn_wheels_rad: float = 0.0
    turn_gyro_rad: float = 0.0
    heading_source: str = "wheels"
    _last_ticks: tuple[int, int] | None = field(
        default=None, init=False, repr=False, compare=False
    )
    _last_heading: float | None = field(default=None, init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        if self.counts_per_rev <= 0:
            raise ValueError(f"counts_per_rev must be positive, got {self.counts_per_rev!r}")

    def reset(self, pose: Pose = Pose()) -> None:
        """Re-seed the pose (e.g. from an AprilTag fix); the next update only records ticks."""
        self.pose = pose
        self._last_ticks = None
        self._last_heading = None

    def update(self, left_ticks: int, right_ticks: int, dt: float,
               heading: float | None = None) -> Pose:
        """Fold in an encoder reading taken `dt` seconds after the previous one.

        The first reading after construction or reset() only records where the ticks are.

        heading: a gyro's heading (radians, CCW, any zero). When given, the step's
        turn is the gyro's change in heading, not the wheels' guess: distance from
        the wheels, rotation from the gyro. A step without one falls back to the
        wheels, and the next one only re-seeds the gyro (its change over the gap was
        already counted by the wheels).
        """
        if self._last_ticks is None:
            self._last_ticks = (left_ticks, right_ticks)
            self._last_heading = heading
            return self.pose
        # Checked before any state changes, so the next good reading still covers this motion.
        if not dt > 0.0:
            raise ValueError(f"dt must be positive, got {dt!r}")
        metres_per_tick = TAU * self.geo.wheel_radius_m / self.counts_per_rev
        last_left, last_right = self._last_ticks
        d_left = unwrap_ticks(left_ticks, last_left, self.counts_per_rev) * metres_per_tick
        d_right = unwrap_ticks(right_ticks, last_right, self.counts_per_rev) * metres_per_tick
        vx, wz = tank_wheels_to_body(d_left / dt, d_right / dt, self.geo)
        if heading is not None and self._last_heading is not None:
            gyro_dth = math.remainder(heading - self._last_heading, TAU)
            if abs(wz * dt) > 1e-4 or abs(gyro_dth) > 1e-4:
                self.turn_wheels_rad += abs(wz * dt)
                self.turn_gyro_rad += abs(gyro_dth)
            wz = gyro_dth / dt
            self.heading_source = "gyro"
        else:
            self.heading_source = "wheels"
        self._last_heading = heading
        self.pose = integrate_twist(self.pose, vx, 0.0, wz, dt)
        self.distance_travelled_m += abs(vx * dt)
        self._last_ticks = (left_ticks, right_ticks)
        return self.pose
