"""Pure pursuit along a given path, and the smooth curve through clicked waypoints.

    curve = waypoint_curve([(0, 0), (1, 0.5), (2, 0), (2.5, 1)])   # through every point
    ctl = PathTracker(curve, Limits(v_max=0.3, w_max=1.0))
    run_goto(robot, Pose(*curve[-1], 0.0), controller=ctl)      # or teleop's path mode

No map and no lidar needed: this follows the curve it is given, on odometry.
(navigator.PathFollower is the one that plans round obstacles; the Pi's
bubble still has the last word on contact either way.)

THE CURVE. Centripetal Catmull-Rom through the waypoints, sampled every few
centimetres. It passes through every click, and the centripetal form never
loops or overshoots between two close points the way a uniform spline does.

PURE PURSUIT. Aim at the point `lookahead` further along the curve and steer
the arc that reaches it: curvature = 2 sin(alpha) / L. The lookahead grows
with speed, which is what makes it smooth: at speed it aims further ahead and
takes corners as gentle arcs; slow, it aims close and hugs the curve.

  * Progress only goes forward along the curve (a window ahead of the last
    nearest point), so a curve that passes near itself is never short-cut.
  * A tank can't swing its nose round while driving, so an aim point more than
    rotate_deg off the nose is turned to on the spot first, and it keeps
    turning until within rotate_until_deg (no flicker between the two).
  * Speed falls with curvature and with the distance left, and with cos(alpha)
    so it is mostly lined up before it goes fast.
  * reverse=True drives the curve backwards (the rear leads). Coming home that
    way skips the 180-degree turn-around: the one turn where a skid-steer's
    wheel slip costs the most heading.

Odometry is the only truth it has. Pure pursuit makes the drive smooth; how
close the robot really ends up is up to the odometry (calibrate turning; a gyro).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Sequence

from retriever.navigation.drive import Limits
from retriever.navigation.geometry import wrap_angle
from retriever.types import Action, Observation, Pose

Point = tuple[float, float]


def waypoint_curve(points: Sequence[Sequence[float]], spacing_m: float = 0.04) -> list[Point]:
    """A smooth curve through every waypoint (centripetal Catmull-Rom), sampled
    about every spacing_m. Consecutive duplicates are dropped; one or two points
    give a straight line."""
    pts: list[Point] = []
    for p in points:
        q = (float(p[0]), float(p[1]))
        if not pts or math.dist(q, pts[-1]) > 1e-6:
            pts.append(q)
    if len(pts) < 2:
        return list(pts)
    if len(pts) == 2:
        n = max(1, int(math.dist(*pts) / spacing_m))
        return [(pts[0][0] + (pts[1][0] - pts[0][0]) * k / n,
                 pts[0][1] + (pts[1][1] - pts[0][1]) * k / n) for k in range(n + 1)]
    # mirror the ends so the curve starts and ends on the first and last points
    ext = ([(2 * pts[0][0] - pts[1][0], 2 * pts[0][1] - pts[1][1])] + pts
           + [(2 * pts[-1][0] - pts[-2][0], 2 * pts[-1][1] - pts[-2][1])])
    out: list[Point] = [pts[0]]
    for i in range(1, len(ext) - 2):
        p0, p1, p2, p3 = ext[i - 1], ext[i], ext[i + 1], ext[i + 2]
        t0 = 0.0
        t1 = t0 + math.dist(p0, p1) ** 0.5 or t0 + 1e-6
        t2 = t1 + math.dist(p1, p2) ** 0.5 or t1 + 1e-6
        t3 = t2 + math.dist(p2, p3) ** 0.5 or t2 + 1e-6
        n = max(1, int(math.dist(p1, p2) / spacing_m))
        for k in range(1, n + 1):
            t = t1 + (t2 - t1) * k / n
            out.append(_cr(p0, p1, p2, p3, t0, t1, t2, t3, t))
    return out


def _cr(p0: Point, p1: Point, p2: Point, p3: Point,
        t0: float, t1: float, t2: float, t3: float, t: float) -> Point:
    def lerp(a: Point, b: Point, ta: float, tb: float) -> Point:
        if tb - ta < 1e-9:
            return a
        wa, wb = (tb - t) / (tb - ta), (t - ta) / (tb - ta)
        return a[0] * wa + b[0] * wb, a[1] * wa + b[1] * wb
    a1, a2, a3 = lerp(p0, p1, t0, t1), lerp(p1, p2, t1, t2), lerp(p2, p3, t2, t3)
    b1, b2 = lerp(a1, a2, t0, t2), lerp(a2, a3, t1, t3)
    return lerp(b1, b2, t1, t2)


def path_length(path: Sequence[Point]) -> float:
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


@dataclass(frozen=True)
class PursuitConfig:
    lookahead_min_m: float = 0.30
    lookahead_max_m: float = 0.70
    lookahead_per_mps: float = 1.2    # L = min + this * |speed|, capped at max
    rotate_deg: float = 60.0          # aim point this far off the nose: turn on the spot...
    rotate_until_deg: float = 15.0    # ...until it is this close
    k_turn: float = 2.2               # rad/s per rad, turning on the spot
    min_turn_wz: float = 0.25         # slower than this and a skid-steer just sits there
    curvature_slow: float = 1.0       # v <= v_max / (1 + this * |curvature|)
    arrive_gain: float = 1.2          # v <= this * distance left
    creep_mps: float = 0.05
    search_ahead_m: float = 1.5       # nearest-point search window, ahead of the last one


@dataclass(frozen=True)
class TrackPlan:
    """What the tracker is doing, for a page (the keys of avoid.Plan.as_dict)."""

    status: str
    reason: str
    steer_rad: float = 0.0
    speed_scale: float = 0.0

    def as_dict(self) -> dict[str, Any]:
        return {"status": self.status, "reason": self.reason,
                "steer_deg": round(math.degrees(self.steer_rad), 1),
                "speed": round(self.speed_scale, 3)}


class PathTracker:
    """Controller (drive.py's shape): follow `path` with pure pursuit. The goal
    passed to step_observation is ignored; the path's end is the goal."""

    def __init__(self, path: Sequence[Sequence[float]], limits: Limits = Limits(),
                 config: PursuitConfig = PursuitConfig(), reverse: bool = False) -> None:
        self.path: list[Point] = [(float(p[0]), float(p[1])) for p in path]
        if len(self.path) < 2:
            raise ValueError("a path needs at least two points")
        self.limits = limits
        self.config = config
        self.reverse = reverse
        self.plan: TrackPlan | None = None
        self.failure: tuple[str, dict] | None = None
        self.remaining_m: float | None = None     # along the path, after each step
        self._seg = 0
        self._turning = False
        self._speed = 0.0
        self._cum = [0.0]
        for a, b in zip(self.path, self.path[1:]):
            self._cum.append(self._cum[-1] + math.dist(a, b))
        self.length_m = self._cum[-1]

    def _nearest(self, x: float, y: float) -> tuple[int, float, float]:
        """(segment, metres along it, distance) of the nearest point, searching
        only forward from the last one, within search_ahead_m."""
        best = (self._seg, 0.0, math.inf)
        start_s = self._cum[self._seg]
        for i in range(self._seg, len(self.path) - 1):
            if self._cum[i] - start_s > self.config.search_ahead_m:
                break
            (ax, ay), (bx, by) = self.path[i], self.path[i + 1]
            vx, vy = bx - ax, by - ay
            L2 = vx * vx + vy * vy
            s = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / L2))
            d = math.hypot(ax + s * vx - x, ay + s * vy - y)
            if d < best[2]:
                best = (i, s * math.sqrt(L2), d)
        return best

    def _at(self, s: float) -> Point:
        """The point s metres along the path (clamped to its ends)."""
        s = max(0.0, min(self.length_m, s))
        lo, hi = 0, len(self._cum) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self._cum[mid] <= s:
                lo = mid
            else:
                hi = mid
        L = self._cum[hi] - self._cum[lo]
        f = 0.0 if L == 0 else (s - self._cum[lo]) / L
        (ax, ay), (bx, by) = self.path[lo], self.path[hi]
        return ax + f * (bx - ax), ay + f * (by - ay)

    def step(self, base: Pose, goal: Pose) -> tuple[Action, bool]:
        return self.step_observation(Observation(joints={}, base=base, t=0.0), goal)

    def step_observation(self, obs: Observation, goal: Pose | None = None) -> tuple[Action, bool]:
        c, lim = self.config, self.limits
        p = obs.base
        seg, along, _ = self._nearest(p.x, p.y)
        self._seg = seg
        s_here = self._cum[seg] + along
        left = self.length_m - s_here
        self.remaining_m = left
        end = self.path[-1]
        if left <= lim.pos_tol and math.hypot(end[0] - p.x, end[1] - p.y) <= lim.pos_tol * 1.5:
            self.plan = TrackPlan("arrived", "")
            self._speed = 0.0
            return Action(), True

        L = min(c.lookahead_max_m, c.lookahead_min_m + c.lookahead_per_mps * abs(self._speed))
        cx, cy = self._at(s_here + L)
        if left < L:                                  # aim past the end along its last direction
            cx, cy = end
        heading = p.theta + (math.pi if self.reverse else 0.0)
        alpha = wrap_angle(math.atan2(cy - p.y, cx - p.x) - heading)

        if abs(alpha) > math.radians(c.rotate_deg):
            self._turning = True
        elif abs(alpha) < math.radians(c.rotate_until_deg):
            self._turning = False
        if self._turning:
            wz = max(-lim.w_max, min(lim.w_max, c.k_turn * alpha))
            wz = math.copysign(max(abs(wz), c.min_turn_wz), alpha)
            self._speed = 0.0
            self.plan = TrackPlan("turning", "lining up with the path", alpha, 0.0)
            return Action(base_wz=wz), False

        ld = max(0.05, math.hypot(cx - p.x, cy - p.y))
        curvature = 2.0 * math.sin(alpha) / ld
        v = lim.v_max / (1.0 + c.curvature_slow * abs(curvature))
        v = min(v, max(c.creep_mps, c.arrive_gain * math.hypot(end[0] - p.x, end[1] - p.y)))
        v *= max(0.0, math.cos(alpha))
        wz = v * curvature
        if abs(wz) > lim.w_max:                        # too tight at this speed: slow, keep the arc
            v *= lim.w_max / abs(wz)
            wz = math.copysign(lim.w_max, wz)
        self._speed = v
        self.plan = TrackPlan("following", "reversing along the path" if self.reverse else "",
                              alpha, v / lim.v_max if lim.v_max else 0.0)
        return Action(base_vx=-v if self.reverse else v, base_wz=wz), False
