"""The nouns every layer shares: skills, backends, the Pi bridge client and the planner.

They are frozen so a snapshot cannot change underneath a skill that is still reasoning
about it, and stdlib-only so the Pi can import them with nothing installed. Units are SI
throughout (metres, radians, seconds) so no layer has to guess.

`Result.detail` is spoken aloud verbatim, and its confidence is validated on construction:
a skill that reports 1.3 confidence has a bug, and it should surface where it was written,
not three layers up. `Action.base_vy` exists so a holonomic controller's intent stays
visible; the tank layers refuse a non-zero value instead of silently dropping it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


@dataclass(frozen=True)
class Result:
    """What every skill returns; `detail` is spoken to the user as written."""

    ok: bool
    confidence: float = 1.0
    detail: str = ""
    data: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not 0.0 <= self.confidence <= 1.0:  # also rejects NaN
            raise ValueError(f"confidence must be within [0, 1], got {self.confidence!r}")

    @classmethod
    def failed(cls, detail: str, **data: Any) -> Result:
        """A failure with zero confidence; keyword arguments become `data`."""
        return cls(ok=False, confidence=0.0, detail=detail, data=data)


@dataclass(frozen=True)
class Pose:
    """A planar pose in the arena frame: metres, and radians CCW from +x."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


@dataclass(frozen=True)
class Observation:
    """One synchronous snapshot of the robot."""

    joints: dict[str, float]  # radians, including "gripper"
    base: Pose
    cam_front: bytes | None = None
    cam_wrist: bytes | None = None
    gripper_load: float = 0.0  # 0..1 normalised servo current
    battery: float = 1.0  # 0..1
    t: float = 0.0


@dataclass(frozen=True)
class Action:
    """Commanded state. Joints left out keep their last target; the base velocity is in the
    base frame (vx forward, vy left, wz CCW)."""

    joints: dict[str, float] = field(default_factory=dict)  # may include "gripper"
    base_vx: float = 0.0
    base_vy: float = 0.0  # exists for the type; tank layers reject non-zero
    base_wz: float = 0.0


@dataclass(frozen=True)
class Target:
    """Something perception can see right now, relative to the base."""

    label: str
    bearing_rad: float  # + is left (CCW), base frame
    range_m: float
    confidence: float = 0.0
    instance_id: str | None = None
    bbox: tuple[int, int, int, int] | None = None
    height_m: float | None = None


@dataclass(frozen=True)
class Station:
    """A named place in the arena, fixed by an AprilTag."""

    name: str
    tag_id: int
    pose: Pose
