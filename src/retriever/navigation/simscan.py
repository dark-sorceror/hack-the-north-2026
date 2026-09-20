"""A simulated RPLIDAR: ray-cast a flat world of circles and segments.

Speaks the REAL unit's convention -- degrees increasing CLOCKWISE seen from
above, 0 at the front mark -- and goes through the same avoid.Mount the robot
uses, so a sign error in that conversion shows up here as a robot steering into
the box instead of round it.

Imitates what was measured on our A2M12:

  * 360 rays a revolution at ~10 Hz, starting at a random angle each time;
  * ~60% of rays return nothing (dropout, independent per ray);
  * ~1 cm range noise; nothing nearer than 0.2 m or farther than 16 m;
  * p_return per shape, so a black or chrome chair can return nothing at all;
  * occluders fixed to the robot (the four wheels round an under-chassis
    mount). They return the robot itself until the mask removes them, and they
    hide whatever is behind them, which is what a blind sector really is.

The world is a horizontal slice at the scan height: a chair is four legs, not a
seat. Stdlib only.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, replace
from typing import Any, Callable, Sequence

from retriever.navigation.avoid import Mount, Scan, scan_from_rplidar
from retriever.navigation.geometry import TAU, wrap_angle
from retriever.types import Pose

# ------------------------------------------------------------------ shapes


@dataclass(frozen=True)
class Circle:
    """A post, a chair leg, a person's leg, a wheel. p_return None = the lidar's
    usual chance of a return; 0.0 = invisible (black, chrome)."""

    x: float
    y: float
    r: float
    p_return: float | None = None

    def hit(self, ox: float, oy: float, dx: float, dy: float) -> float | None:
        """Distance along the unit ray (dx, dy) from (ox, oy) to the circle."""
        fx, fy = ox - self.x, oy - self.y
        b = fx * dx + fy * dy
        c = fx * fx + fy * fy - self.r * self.r
        disc = b * b - c
        if disc < 0.0:
            return None
        sq = math.sqrt(disc)
        t = -b - sq if -b - sq >= 0.0 else -b + sq
        return t if t >= 0.0 else None


@dataclass(frozen=True)
class Segment:
    """A wall or one face of a box."""

    x0: float
    y0: float
    x1: float
    y1: float
    p_return: float | None = None

    def hit(self, ox: float, oy: float, dx: float, dy: float) -> float | None:
        ex, ey = self.x1 - self.x0, self.y1 - self.y0
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            return None  # parallel
        wx, wy = self.x0 - ox, self.y0 - oy
        t = (wx * ey - wy * ex) / den
        u = (wx * dy - wy * dx) / den
        return t if t >= 0.0 and 0.0 <= u <= 1.0 else None


Shape = Circle | Segment


def wall(x0: float, y0: float, x1: float, y1: float, p_return: float | None = None) -> Segment:
    return Segment(x0, y0, x1, y1, p_return)


def room(x0: float, y0: float, x1: float, y1: float) -> list[Segment]:
    """Four walls. Almost every test wants one: in a world with nothing to hit,
    every ray returns nothing, and nothing is seen as free."""
    return [wall(x0, y0, x1, y0), wall(x1, y0, x1, y1), wall(x1, y1, x0, y1), wall(x0, y1, x0, y0)]


def box(cx: float, cy: float, size_x: float, size_y: float,
        p_return: float | None = None) -> list[Segment]:
    """An axis-aligned box centred on (cx, cy)."""
    x0, x1 = cx - size_x / 2, cx + size_x / 2
    y0, y1 = cy - size_y / 2, cy + size_y / 2
    return [wall(x0, y0, x1, y0, p_return), wall(x1, y0, x1, y1, p_return),
            wall(x1, y1, x0, y1, p_return), wall(x0, y1, x0, y0, p_return)]


def chair(cx: float, cy: float, size: float = 0.45, leg_r: float = 0.012,
          p_return: float | None = None) -> list[Circle]:
    """What a lidar 15-25 cm off the floor sees of a chair: four thin legs."""
    h = size / 2
    return [Circle(cx + sx * h, cy + sy * h, leg_r, p_return) for sx in (-1, 1) for sy in (-1, 1)]


def person(cx: float, cy: float) -> list[Circle]:
    """Two legs, side by side along y."""
    return [Circle(cx, cy - 0.1, 0.06), Circle(cx, cy + 0.1, 0.06)]


def wheels(front_x: float = 0.18, rear_x: float = -0.18, half_track: float = 0.20,
           r: float = 0.05) -> list[Circle]:
    """Four wheels in the BASE frame, as occluders for an under-chassis lidar."""
    return [Circle(x, y, r, 1.0) for x in (front_x, rear_x) for y in (half_track, -half_track)]


def occluder_mask(occluders: Sequence[Circle], mount: Mount,
                  pad_deg: float = 3.0) -> tuple[tuple[float, float], ...]:
    """The mask lidar_check.py --record-mask would record for these occluders:
    (start, end) radians in the lidar's own counter-clockwise frame."""
    out = []
    pad = math.radians(pad_deg)
    for o in occluders:
        dx, dy = o.x - mount.x_m, o.y - mount.y_m
        d = math.hypot(dx, dy)
        half = math.asin(min(1.0, o.r / d)) + pad if d > 0 else math.pi
        bearing = math.atan2(dy, dx)
        # base bearing -> lidar CCW angle: the inverse of Mount.ray_bearing
        a = (mount.yaw_rad - bearing) if mount.inverted else (bearing - mount.yaw_rad)
        out.append(((a - half) % TAU, (a + half) % TAU))
    return tuple(out)


# ------------------------------------------------------------------ the lidar


class SimLidar:
    """A scan source for fake robots: call it, get the latest avoid.Scan.

    pose_fn gives the robot's TRUE pose (the lidar sees the world, not the
    odometry's belief); clock_fn the backend's clock. Revolutions come at `hz`;
    between them the same Scan is handed back, as a real driver would.
    `bubble_blocked` lets a test play the Pi's safety bubble.
    """

    def __init__(
        self,
        world: Sequence[Shape],
        pose_fn: Callable[[], Pose],
        clock_fn: Callable[[], float],
        mount: Mount = Mount(),
        *,
        occluders: Sequence[Circle] = (),
        rays: int = 360,
        hz: float = 10.0,
        dropout: float = 0.6,
        noise_m: float = 0.01,
        min_range_m: float = 0.20,
        max_range_m: float = 16.0,
        seed: int = 0,
    ) -> None:
        self.world = list(world)
        self.pose_fn = pose_fn
        self.clock_fn = clock_fn
        self.mount = mount
        self.occluders = list(occluders)
        self.rays = rays
        self.hz = hz
        self.dropout = dropout
        self.noise_m = noise_m
        self.min_range_m = min_range_m
        self.max_range_m = max_range_m
        self.rng = random.Random(seed)
        self.bubble_blocked = False
        self.last_sweep: list[tuple[float, float]] = []
        self.sweeps = 0
        self._scan: Scan | None = None

    @classmethod
    def on(cls, robot: Any, world: Sequence[Shape], **kw: Any) -> SimLidar:
        """Ride on a FakeRobot / TankFakeRobot: its ground truth and its clock."""
        return cls(world, lambda: robot.base, lambda: robot.observe().t, **kw)

    def sweep(self, pose: Pose) -> list[tuple[float, float]]:
        """One revolution in the RPLIDAR's own terms: (angle_cw_deg, range_m),
        range 0.0 = no return."""
        m = self.mount
        c, s = math.cos(pose.theta), math.sin(pose.theta)
        lx, ly = pose.x + m.x_m * c - m.y_m * s, pose.y + m.x_m * s + m.y_m * c
        shapes: list[Shape] = list(self.world)
        shapes += [Circle(pose.x + o.x * c - o.y * s, pose.y + o.x * s + o.y * c, o.r, o.p_return)
                   for o in self.occluders]
        step = 360.0 / self.rays
        start = self.rng.uniform(0.0, step)
        out = []
        for k in range(self.rays):
            angle_cw = start + k * step
            heading = wrap_angle(pose.theta + m.ray_bearing(angle_cw))
            dx, dy = math.cos(heading), math.sin(heading)
            near, what = math.inf, None
            for shape in shapes:
                d = shape.hit(lx, ly, dx, dy)
                if d is not None and d < near:
                    near, what = d, shape
            r = 0.0
            if what is not None and near <= self.max_range_m:
                p = what.p_return if what.p_return is not None else 1.0 - self.dropout
                if self.rng.random() < p:
                    d = near + self.rng.gauss(0.0, self.noise_m)
                    if self.min_range_m <= d <= self.max_range_m:
                        r = d
            out.append((angle_cw, r))
        return out

    def scan_at(self, pose: Pose, t: float = 0.0) -> Scan:
        """One revolution from `pose`, converted exactly as the robot would."""
        self.last_sweep = self.sweep(pose)
        self.sweeps += 1
        return scan_from_rplidar(t, self.last_sweep, self.mount,
                                 min_range_m=self.min_range_m, max_range_m=self.max_range_m)

    def __call__(self) -> Scan:
        now = self.clock_fn()
        if self._scan is None or now - self._scan.t >= 1.0 / self.hz - 1e-9:
            self._scan = self.scan_at(self.pose_fn(), now)
        if self._scan.bubble_blocked != self.bubble_blocked:
            return replace(self._scan, bubble_blocked=self.bubble_blocked)
        return self._scan


__all__ = [
    "Circle", "Segment", "SimLidar", "box", "chair", "occluder_mask", "person", "room",
    "wall", "wheels",
]
