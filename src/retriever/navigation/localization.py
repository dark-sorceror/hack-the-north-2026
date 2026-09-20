"""SE(2) pose composition: the algebra every frame change on this robot goes through.

The base does not fly, so a pose is (x, y, theta) and composing two of them is a 2x2
rotation and an add. Written down once, here, because getting the sign of the rotation
wrong is the bug that makes a skill drive to the mirror image of where it meant to go.

Pure functions over `Pose` only: no odometry, no sensors, no clock. That keeps them
importable anywhere and testable by hand.
"""

from __future__ import annotations

import math

from retriever.navigation.geometry import angle_diff, wrap_angle
from retriever.types import Pose


def compose(a: Pose, b: Pose) -> Pose:
    """a ⊕ b — b expressed in a's frame, lifted into a's parent frame."""
    c, s = math.cos(a.theta), math.sin(a.theta)
    return Pose(
        x=a.x + b.x * c - b.y * s,
        y=a.y + b.x * s + b.y * c,
        theta=wrap_angle(a.theta + b.theta),
    )


def invert(a: Pose) -> Pose:
    """The transform that undoes a."""
    c, s = math.cos(a.theta), math.sin(a.theta)
    return Pose(x=-(a.x * c + a.y * s), y=-(-a.x * s + a.y * c), theta=wrap_angle(-a.theta))


def base_from_tag(tag_in_world: Pose, tag_in_base: Pose) -> Pose:
    """Where the base must be, given where a tag is and where we see it.

    base_in_world = tag_in_world ⊕ inverse(tag_in_base)
    """
    return compose(tag_in_world, invert(tag_in_base))


def blend(a: Pose, b: Pose, t: float) -> Pose:
    """Move a fraction t of the way from a to b, taking the short way round."""
    return Pose(
        x=a.x + (b.x - a.x) * t,
        y=a.y + (b.y - a.y) * t,
        theta=wrap_angle(a.theta + angle_diff(b.theta, a.theta) * t),
    )
