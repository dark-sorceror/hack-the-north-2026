"""Planar frame maths: the only module that writes sin and cos for frames.

Sign conventions are where navigation bugs hide. A flipped bearing turns "steer toward the
keys" into "steer away", and it looks like a tuning problem for an hour before anyone
suspects the maths. So every frame transform lives here, written and tested once: x is
forward, y is left, theta is counter-clockwise from the arena's +x axis, and every angle
that comes out is wrapped into [-pi, pi).
"""

from __future__ import annotations

import math

from retriever.types import Pose, Target

TAU = 2 * math.pi


def wrap_angle(a: float) -> float:
    """`a` wrapped into [-pi, pi)."""
    wrapped = (a + math.pi) % TAU - math.pi
    # Rounding sends inputs a hair below -pi to exactly +pi; keep the interval half-open.
    return wrapped if wrapped < math.pi else -math.pi


def angle_diff(goal: float, current: float) -> float:
    """The shortest signed rotation from `current` to `goal`; positive is CCW (left)."""
    return wrap_angle(goal - current)


def world_to_base(dx: float, dy: float, theta: float) -> tuple[float, float]:
    """A world-frame offset as seen from a base heading `theta`, as (forward, left)."""
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    return dx * cos_t + dy * sin_t, -dx * sin_t + dy * cos_t


def base_to_world(forward: float, left: float, theta: float) -> tuple[float, float]:
    """A base-frame offset rotated into the world frame; the inverse of `world_to_base`."""
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    return forward * cos_t - left * sin_t, forward * sin_t + left * cos_t


def target_offset(t: Target) -> tuple[float, float]:
    """Where a seen target is, as (forward, left) metres in the base frame."""
    return t.range_m * math.cos(t.bearing_rad), t.range_m * math.sin(t.bearing_rad)


def pose_error(base: Pose, goal: Pose) -> tuple[float, float, float]:
    """Where `goal` is relative to `base`, in the base frame: (forward, left, heading)."""
    forward, left = world_to_base(goal.x - base.x, goal.y - base.y, base.theta)
    return forward, left, angle_diff(goal.theta, base.theta)


def distance(base: Pose, goal: Pose) -> float:
    """Straight-line distance between two poses, ignoring heading."""
    return math.hypot(goal.x - base.x, goal.y - base.y)


def clamp_velocity(vx: float, vy: float, v_max: float) -> tuple[float, float]:
    """Scale (vx, vy) down to at most `v_max` in magnitude, keeping its direction."""
    if not v_max >= 0.0:
        raise ValueError(f"v_max must be non-negative, got {v_max!r}")
    speed = math.hypot(vx, vy)
    if speed <= v_max:
        return vx, vy
    scale = v_max / speed
    return vx * scale, vy * scale
