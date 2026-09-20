"""A simulated depth camera for fake robots: CameraObstacles from simulated shapes.

For the navigation session's simulator tests. It answers the same three calls as
depth_obstacles.CameraObstacleSource (latest / age / describe), without a camera,
a stream or any depth maths: those are covered by tests/test_depth_obstacles.py.

It ray-casts shapes with a `.hit(ox, oy, dx, dy)` method (navigation/simscan's
Circle and Segment) in the plane, one ray per bin_deg across fov_deg, from a lens
forward_m ahead of the base centre. It sees every shape whatever its p_return: a
black box that returns nothing to the lidar is plain to a depth camera. To model
something BELOW the lidar's plane, give SimCamera a world that contains it and
the SimLidar one that does not: the lidar then sees the wall behind it as clear.
"""
from __future__ import annotations

import math
import time
from typing import Any, Callable, Sequence

from retriever.perception.depth_obstacles import CameraObstacles
from retriever.types import Pose


class SimCamera:
    def __init__(self, world: Sequence[Any], pose_fn: Callable[[], Pose], *,
                 forward_m: float = 0.20, fov_deg: float = 87.0, bin_deg: float = 2.0,
                 min_range_m: float = 0.25, max_range_m: float = 3.0, hz: float = 15.0,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.world = list(world)
        self.pose_fn = pose_fn
        self.forward_m = forward_m
        self.fov_deg = fov_deg
        self.bin_deg = bin_deg
        self.min_range_m = min_range_m
        self.max_range_m = max_range_m
        self.hz = hz
        self.clock = clock
        self._obs: CameraObstacles | None = None

    @classmethod
    def on(cls, robot: Any, world: Sequence[Any], **kw: Any) -> SimCamera:
        """Ride on a FakeRobot / TankFakeRobot: its ground truth."""
        return cls(world, lambda: robot.base, **kw)

    def frame_at(self, pose: Pose, t: float) -> CameraObstacles:
        """One sweep from `pose`, as robot-frame (x forward, y left) points."""
        c, s = math.cos(pose.theta), math.sin(pose.theta)
        ox, oy = pose.x + self.forward_m * c, pose.y + self.forward_m * s
        pts = []
        for k in range(int(self.fov_deg / self.bin_deg) + 1):
            rel = math.radians(-self.fov_deg / 2.0 + k * self.bin_deg)
            dx, dy = math.cos(pose.theta + rel), math.sin(pose.theta + rel)
            hits = [d for shape in self.world if (d := shape.hit(ox, oy, dx, dy)) is not None]
            near = min(hits, default=math.inf)
            if self.min_range_m <= near <= self.max_range_m:
                pts.append((self.forward_m + near * math.cos(rel), near * math.sin(rel)))
        return CameraObstacles(t, tuple(pts))

    def latest(self) -> CameraObstacles:
        """The newest frame, a new one at most every 1/hz, like a real camera."""
        now = self.clock()
        if self._obs is None or now - self._obs.t >= 1.0 / self.hz - 1e-9:
            self._obs = self.frame_at(self.pose_fn(), now)
        return self._obs

    def age(self) -> float:
        """Always fresh: a simulated camera never goes quiet."""
        return 0.0

    def describe(self) -> str:
        """The same one-liner as the real source, for a page or a test."""
        obs = self.latest()
        near = {"left": math.inf, "ahead": math.inf, "right": math.inf}
        for x, y in obs.points:
            bearing = math.degrees(math.atan2(y, x))
            side = "left" if bearing > 15 else ("right" if bearing < -15 else "ahead")
            near[side] = min(near[side], math.hypot(x, y))
        parts = [f"{side} {d:.2f} m" for side, d in near.items() if d < math.inf]
        return f"{len(obs.points)} points" + (": " + ", ".join(parts) if parts else "")

    def stop(self) -> None:
        """Nothing to close: here so it can stand in for the real source."""
