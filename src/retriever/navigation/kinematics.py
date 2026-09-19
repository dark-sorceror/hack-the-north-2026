"""Skid-steer kinematics: body twist to left/right rim speeds and back, never sideways.

This robot is a four-wheel skid steer, so the only motions it can honestly make are a
forward speed and a yaw rate. It also drags its wheels sideways in every turn, which makes
it rotate slower than its track width predicts. Modelling that as a wider *effective*
track (the scrub factor) is the standard correction, and it has to be measured, not
derived: get it wrong and straight lines still look perfect while the heading drifts, a
failure that reads exactly like bad localisation. A sideways velocity is refused loudly
by `assert_no_strafe` rather than dropped, because a silently dropped vy means a
holonomic controller is steering a robot that cannot do what it is being asked.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class TankGeometry:
    """A four-wheel skid steer; non-holonomic."""

    wheel_radius_m: float = 0.048
    track_width_m: float = 0.30
    scrub_factor: float = 1.0  # >= 1; measured by spinning in place, not derived

    def __post_init__(self) -> None:
        for name in ("wheel_radius_m", "track_width_m", "scrub_factor"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"{name} must be positive and finite, got {value!r}")
        if self.scrub_factor < 1.0:
            raise ValueError(
                f"scrub_factor must be >= 1 (scrub only ever slows a turn), "
                f"got {self.scrub_factor!r}"
            )

    @property
    def effective_track_m(self) -> float:
        """The track width the robot turns as if it had: track_width_m * scrub_factor."""
        return self.track_width_m * self.scrub_factor


def tank_body_to_wheels(
    vx: float, wz: float, geo: TankGeometry = TankGeometry()
) -> tuple[float, float]:
    """Rim speeds (v_left, v_right) in m/s that give forward speed `vx` and yaw rate `wz`."""
    half_turn = wz * geo.effective_track_m / 2.0
    return vx - half_turn, vx + half_turn


def tank_wheels_to_body(
    v_left: float, v_right: float, geo: TankGeometry = TankGeometry()
) -> tuple[float, float]:
    """Forward speed and yaw rate (vx, wz) that rim speeds produce; inverts the above."""
    return (v_left + v_right) / 2.0, (v_right - v_left) / geo.effective_track_m


def assert_no_strafe(vy: float, tol: float = 1e-6) -> None:
    """Raise ValueError if `vy` asks the skid-steer base to move sideways."""
    if not abs(vy) <= tol:  # written this way round so NaN is refused too
        raise ValueError(
            f"base_vy={vy!r}: a skid-steer base cannot move sideways. A holonomic "
            "controller is pointed at a tank base; command forward speed and yaw rate only."
        )
