"""A pixel and a depth -> where that thing is on the floor, relative to the base.

Two frames meet here, and mixing them up is the bug nobody finds by staring at the code:

  * the CAMERA frame is the one every depth SDK uses (RealSense, OpenCV): +X right,
    +Y **down**, +Z forward, origin at the lens;
  * the BASE frame is the robot's, the one navigation and the skills already speak:
    +X forward, +Y left, +Z up, origin on the floor under the middle of the base.

So this module is the only place that writes the camera's sines and cosines, the way
navigation/geometry.py is the only place that writes the arena's. A camera tilted DOWN
toward the floor has POSITIVE pitch — that is the sign people get backwards, and getting
it backwards puts every obstacle above the robot instead of on the floor in front of it.

`pixel_to_target` hands back the GROUND-PROJECTED range, not the slant range down the ray.
An approach controller drives the range it is given: fed 1.8 m of slant range to a mug on
the floor 1.5 m away, it drives 1.8 m and hits the table. Only `deproject` and
`camera_to_base` are pure arithmetic, so numpy arrays flow through them elementwise, which
is how a whole depth grid becomes base-frame points without a Python loop.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real

from retriever.types import Target


@dataclass(frozen=True)
class Intrinsics:
    """A pinhole camera's focal lengths and principal point, in pixels."""

    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self) -> None:
        for name in ("fx", "fy", "cx", "cy"):
            _require_finite(name, getattr(self, name))
        # A zero focal length is an unset field, not a camera: catch it where it was written.
        if self.fx == 0.0 or self.fy == 0.0:
            raise ValueError(
                f"focal lengths must be non-zero, got fx={self.fx!r}, fy={self.fy!r}")


@dataclass(frozen=True)
class CameraMount:
    """Where a camera sits on the base: metres up and forward, and its tilt in radians.

    `pitch_rad` is POSITIVE when the camera is tilted DOWN toward the floor.
    """

    height_m: float = 0.2
    forward_m: float = 0.1
    pitch_rad: float = 0.0

    def __post_init__(self) -> None:
        for name in ("height_m", "forward_m", "pitch_rad"):
            _require_finite(name, getattr(self, name))


def deproject(u: float, v: float, depth_m: float,
              intr: Intrinsics) -> tuple[float, float, float]:
    """Pixel (u, v) at `depth_m` along the camera's +Z -> (X right, Y down, Z forward) metres.

    `depth_m` is the depth a depth camera reports (distance along +Z), not the distance
    along the ray. Scalars are checked; numpy arrays pass straight through the arithmetic,
    and their caller is the one that has to drop the cells with no reading.
    """
    _require_finite("u", u)
    _require_finite("v", v)
    _require_positive_depth(depth_m)
    return (u - intr.cx) / intr.fx * depth_m, (v - intr.cy) / intr.fy * depth_m, depth_m


def camera_to_base(x_right: float, y_down: float, z_fwd: float,
                   mount: CameraMount) -> tuple[float, float, float]:
    """A camera-frame point -> the base frame, as (forward, left, up) metres.

    The camera is rotated by `mount.pitch_rad` about the base's left axis (down is
    positive) and then offset to where it is bolted, so `up` is height above the floor.
    """
    cos_p, sin_p = math.cos(mount.pitch_rad), math.sin(mount.pitch_rad)
    forward = mount.forward_m + z_fwd * cos_p - y_down * sin_p
    left = -x_right
    up = mount.height_m - (y_down * cos_p + z_fwd * sin_p)
    return forward, left, up


def pixel_to_target(u: float, v: float, depth_m: float, intr: Intrinsics, mount: CameraMount,
                    label: str, confidence: float = 0.0, instance_id: str | None = None,
                    bbox: tuple[int, int, int, int] | None = None) -> Target:
    """A pixel and its depth -> a `Target` the base can drive at.

    `range_m` is the distance across the FLOOR to the thing (the slant range would make
    every approach overshoot), `bearing_rad` is positive to the left, and `height_m` says
    how far above the floor it is, so a caller can tell a mug on a table from one underneath.
    """
    x_right, y_down, z_fwd = deproject(u, v, depth_m, intr)
    forward, left, up = camera_to_base(x_right, y_down, z_fwd, mount)
    return Target(
        label=label,
        bearing_rad=math.atan2(left, forward),
        range_m=math.hypot(forward, left),
        confidence=confidence,
        instance_id=instance_id,
        bbox=bbox,
        height_m=up,
    )


def _require_finite(name: str, value: float) -> None:
    """Reject a non-finite scalar; anything that is not a plain number is left to the caller."""
    if isinstance(value, Real) and not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value!r}")


def _require_positive_depth(depth_m: float) -> None:
    """A depth of zero or less is 'no reading', and projecting it invents an obstacle."""
    if isinstance(depth_m, Real) and (not math.isfinite(depth_m) or depth_m <= 0.0):
        raise ValueError(f"depth_m must be a positive, finite distance, got {depth_m!r}")
