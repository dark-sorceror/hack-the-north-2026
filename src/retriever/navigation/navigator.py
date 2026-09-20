"""Go to a spot on the map: keep the map, plan a route, follow it, re-plan.

    mapper = Mapper()                         # lives as long as the session: the map persists
    mapper.add_scan(pose_at_scan, scan_points)    # every scan, driving by hand or not
    ctl = PathFollower(mapper, Limits(v_max=0.3, w_max=1.0))
    run_goto(robot, goal, controller=ctl)     # or teleop's click-to-go

Same controller shape as drive.py's (step_observation / plan / failure), so it
drops in wherever AvoidingGotoController did.

FOLLOWING. Pure pursuit on the smoothed route: aim at the point lookahead_m
further along it. A tank cannot turn on a sixpence while driving, so a carrot
more than rotate_deg off the nose is turned to on the spot first. Forward speed
is regulated three ways: by the curvature (tight arcs slow down), by the
clearance under the robot (the distance field: near things, slower), and by the
distance left (it arrives, it does not overshoot).

RE-PLANNING. Every replan_s, at once if the robot is off the route or the
route ahead has become lethal (a person stepped in: the map already has them).
No route: stop and wait patience_s, backing off a little once or twice first
in case the robot is simply too close to something to turn, then give up and
say why. The Pi's bubble still has the last word on contact; when it refuses a
command, that counts as blocked too.
"""

from __future__ import annotations

import base64
import math
import threading
import time
import zlib
from dataclasses import dataclass
from typing import Any, Callable, Iterable, Sequence

import numpy as np

from retriever.navigation.drive import Limits
from retriever.navigation.geometry import wrap_angle
from retriever.navigation.grid import FREE, OCCUPIED, GridConfig, OccupancyGrid
from retriever.navigation.pathplan import Costmap, PathResult, PlannerConfig, build_costmap, plan_path
from retriever.types import Action, Observation, Pose

LETHAL_ZONE = 3   # page_view(): free cell the robot's centre may not enter
CAMERA = 4        # page_view(): blocked by the camera, not the lidar
CAMERA_HOLD_S = 2.0   # a camera cell blocks for this long after it was last seen


class Mapper:
    """The occupancy grid, fed from any thread, and its cached costmap."""

    def __init__(self, grid_config: GridConfig = GridConfig(),
                 planner_config: PlannerConfig = PlannerConfig(),
                 sensor_offset: tuple[float, float] = (0.0, 0.0),
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.grid = OccupancyGrid(grid_config)
        self.config = planner_config
        self.sensor_offset = sensor_offset
        self.clock = clock
        self._lock = threading.RLock()
        self._costmap: Costmap | None = None
        self._view: dict[str, Any] | None = None
        self._view_version = -1
        self._view_at = -math.inf
        self.scans = 0
        self.last_scan: float | None = None       # clock() of the last scan added
        # The camera layer: cell -> when it stops blocking. Kept apart from the
        # lidar's grid because the lidar clears every cell its beams cross, and a
        # box below its plane is exactly what they cross: written into the grid,
        # camera marks would be erased by the next revolution.
        self._camera: dict[int, float] = {}
        self.camera_points = 0
        self.last_camera: float | None = None

    def add_scan(self, pose: Pose, points_base: Iterable[Sequence[float]]) -> int:
        with self._lock:
            n = self.grid.insert_scan(pose, points_base, self.sensor_offset)
            self.scans += 1
            self.last_scan = self.clock()
            return n

    def add_camera_points(self, pose: Pose, points_base: Iterable[Sequence[float]],
                          hold_s: float = CAMERA_HOLD_S) -> int:
        """Obstacle points a camera saw (base frame, from the pose it was taken
        at). They block for hold_s and then stop, so someone who walks away
        stops blocking; seeing them again refreshes them."""
        pts = [(float(p[0]), float(p[1])) for p in points_base]
        with self._lock:
            now = self.clock()
            g = self.grid
            c, s = math.cos(pose.theta), math.sin(pose.theta)
            until = now + hold_s
            for bx, by in pts:
                x, y = pose.x + bx * c - by * s, pose.y + bx * s + by * c
                ix, iy = g.cell(x, y)
                if g.inside(ix, iy):
                    self._camera[int(iy) * g.n + int(ix)] = until
            self.camera_points = len(pts)
            self.last_camera = now
            self._costmap = None                  # the costmap is stale now
            return len(pts)

    def camera_mask(self) -> np.ndarray | None:
        """The camera cells that still block, or None if there are none."""
        with self._lock:
            now = self.clock()
            live = [k for k, until in self._camera.items() if until > now]
            if len(live) != len(self._camera):
                self._camera = {k: self._camera[k] for k in live}
                self._costmap = None
            if not live:
                return None
            mask = np.zeros(self.grid.n * self.grid.n, bool)
            mask[np.fromiter(live, dtype=np.int64, count=len(live))] = True
            return mask.reshape(self.grid.n, self.grid.n)

    def has_map(self) -> bool:
        """Anything to plan on at all: lidar scans, or live camera points."""
        with self._lock:
            return bool(self.scans) or any(u > self.clock() for u in self._camera.values())

    def scan_age(self) -> float:
        return math.inf if self.last_scan is None else self.clock() - self.last_scan

    def camera_age(self) -> float:
        return math.inf if self.last_camera is None else self.clock() - self.last_camera

    def reset(self) -> None:
        with self._lock:
            self.grid.clear()
            self._costmap = None
            self.scans = 0
            self._camera.clear()

    def costmap(self) -> Costmap:
        camera = self.camera_mask()               # also expires old cells
        with self._lock:
            if self._costmap is None or self._costmap.version != self.grid.version:
                self._costmap = build_costmap(self.grid, self.config, camera)
            return self._costmap

    def plan(self, start: tuple[float, float], goal: tuple[float, float]) -> PathResult:
        return plan_path(self.costmap(), start, goal)

    def page_view(self, min_period_s: float = 0.5) -> dict[str, Any] | None:
        """The map for a page: the known part only, one byte per 5 cm cell
        (0 unknown, 1 free, 2 occupied, 3 too close for the robot's centre),
        deflated and base64'd. Cached; rebuilt at most every min_period_s."""
        with self._lock:
            now = self.clock()
            if self._view is not None and (self._view_version == self.grid.version
                                           or now - self._view_at < min_period_s):
                return self._view
            g = self.grid
            states = g.states()
            cm = self.costmap()
            if cm.fine_lethal is not None:
                states[(states != OCCUPIED) & cm.fine_lethal] = LETHAL_ZONE
            camera = self.camera_mask()
            if camera is not None:
                states[camera] = CAMERA
            known = states != 0
            if not known.any():
                return None
            ys, xs = np.nonzero(known)
            pad = 4
            y0, y1 = max(0, ys.min() - pad), min(g.n, ys.max() + pad + 1)
            x0, x1 = max(0, xs.min() - pad), min(g.n, xs.max() + pad + 1)
            crop = np.ascontiguousarray(states[y0:y1, x0:x1])
            self._view = {
                "v": g.version, "res": g.res,
                "x0": round(g.x0 + x0 * g.res, 4), "y0": round(g.y0 + y0 * g.res, 4),
                "w": int(x1 - x0), "h": int(y1 - y0),      # w cells along x, h along y
                "data": base64.b64encode(zlib.compress(crop.tobytes(), 6)).decode(),
            }
            self._view_version, self._view_at = g.version, now
            return self._view


@dataclass(frozen=True)
class FollowConfig:
    lookahead_m: float = 0.45
    replan_s: float = 0.5
    rotate_deg: float = 45.0          # carrot further off the nose than this: turn on the spot
    k_turn: float = 2.2               # rad/s per rad of heading error, turning on the spot
    lat_accel_max: float = 0.5        # m/s^2 sideways in a bend (pursuit.py's rule)
    align_full_deg: float = 25.0      # full speed while the carrot is this close to the nose
    align_floor: float = 0.25         # ...and never less than this fraction of it: see below
    rotate_exit_deg: float = 25.0     # ...and turns until it is back inside THIS: hysteresis
    # Never outdrive the lidar. Speed is capped at what can still be stopped
    # inside the room actually visible, allowing for how old the scan is:
    #   room = v * reaction + v^2 / (2 * brake)
    # Solved for v, that is a smooth sqrt(room) curve instead of a step, which
    # is what makes a high gear usable: fast where it is open, slow where it is
    # tight, decided by physics rather than by a gear number.
    reaction_s: float = 0.25          # control period + a lidar revolution
    brake_mps2: float = 0.8           # what the robot can actually shed
    slow_clearance_m: float = 0.30    # (kept for the page's speed readout)
    min_speed_scale: float = 0.3
    creep_mps: float = 0.06
    arrive_gain: float = 1.2          # v <= this * distance left
    off_path_m: float = 0.30          # this far off the route: re-plan now
    check_ahead_m: float = 1.2        # route turning lethal this far ahead: re-plan now
    patience_s: float = 4.0           # no route this long: give up
    backoff_after_s: float = 0.8      # blocked this long: back off a little
    backoff_s: float = 1.5
    backoff_mps: float = 0.12
    max_backoffs: int = 4
    blocked_decay: float = 0.25       # how fast a good tick forgives a blocked one
    lock_end_m: float = 1.2           # this close: stop re-snapping the destination
    # Two ways round a chair are often within a few percent of each other, and
    # scan noise flips which one wins twice a second. The robot then turns
    # toward one, turns back toward the other and makes no ground. A new route
    # has to be this much shorter before it is worth abandoning the committed
    # one -- and it is taken anyway when the old one is gone or blocked.
    switch_margin: float = 0.20       # 20% better, or keep going
    # Turning on the spot sweeps the corners -- and with an arm on the front,
    # that circle is much bigger than the body. The wheels blank four wedges of
    # the lidar's view, so the thing a corner is about to hit is often one the
    # scan cannot see FROM HERE. The map remembers it, though, so ask the map
    # before spinning: no room means back out, not grind round.
    spin_radius_m: float = 0.0        # 0 disables the check (set from the footprint)
    spin_margin_m: float = 0.03
    turn_lookahead: float = 2.0       # turn-on-the-spot looks this much further than driving
    speed_rise_tau_s: float = 0.5     # how slowly the clearance speed limit is allowed back up
    # Turning has the same problem driving does: a gear sets one top rate, and
    # a small correction taken at the top rate overshoots. Same answer -- the
    # rate that can still be stopped in the angle that is left.
    # Measured, not assumed: at 1.5 rad/s the link and the ramp together carry
    # the nose ~20 deg past where the command changed, which is the overshoot
    # seen on gear 3. These are what the cap has to allow for.
    turn_decel: float = 1.2           # rad/s^2 the spin really sheds
    turn_reaction_s: float = 0.25     # a tick, the link, and the wheels answering
    turn_min: float = 0.25            # rad/s: never so slow it stalls
    # Back off turning, not straight: a three-point turn. Reversing buys room
    # in front AND swings the nose the way the route wants, so the retry starts
    # from a better angle instead of the one that just failed. Off: reverse
    # straight, which keeps the angle that got it stuck.
    reverse_to_turn: bool = True
    stale_scan_s: float = 1.0         # no scan this long: the map is out of date, wait


@dataclass(frozen=True)
class FollowPlan:
    """What the follower is doing, for a page (the same keys as avoid.Plan)."""

    status: str                       # following | turning | blocked | waiting
    reason: str
    steer_rad: float = 0.0            # base frame, + left: where it is aiming
    speed_scale: float = 0.0

    def as_dict(self) -> dict[str, Any]:
        return {"status": self.status, "reason": self.reason,
                "steer_deg": round(math.degrees(self.steer_rad), 1),
                "speed": round(self.speed_scale, 3)}


class PathFollower:
    """Controller: plan on the Mapper's map, follow with pure pursuit."""

    def __init__(self, mapper: Mapper, limits: Limits = Limits(),
                 config: FollowConfig = FollowConfig(),
                 bubble: Callable[[], Any] | None = None) -> None:
        """bubble: returns the Pi's BubbleStatus (or None); `state == "blocked"`
        means it refused the last command."""
        self.mapper = mapper
        self.limits = limits
        self.config = config
        self.bubble = bubble
        self.path: list[tuple[float, float]] | None = None
        self.result: PathResult | None = None
        self.plan: FollowPlan | None = None
        self.failure: tuple[str, dict] | None = None
        self._planned_at = -math.inf
        self._last_t: float | None = None
        self._blocked_s = 0.0
        self._why = ""
        self._backoffs = 0
        self._backoff_until: float | None = None
        self._backoff_wz = 0.0
        self._turning = False
        self._end_locked: tuple[float, float] | None = None
        self._end_goal: tuple[float, float] | None = None
        self._v_room: float | None = None

    # -- the route --------------------------------------------------------

    def _replan(self, p: Pose, goal: Pose, now: float) -> None:
        self._planned_at = now
        r = self.mapper.plan((p.x, p.y), (goal.x, goal.y))
        if r.ok and self.path is not None and self._path_still_good(p):
            keep = self._route_left(p)
            if r.length_m > keep * (1.0 - self.config.switch_margin):
                self._planned_at = now       # the committed route stands
                return
        self.result = r
        if r.ok:
            self.path = r.path
            # A goal the map calls lethal is snapped to the nearest free cell --
            # redone on EVERY replan, twice a second, so each new scan slides
            # the destination and the robot chases a point that keeps moving.
            # Once it is close, fix the endpoint: later scans may change the
            # ROUTE, never where it ends.
            if (self._end_locked is None and r.goal is not None
                    and math.hypot(goal.x - p.x, goal.y - p.y) <= self.config.lock_end_m):
                self._end_locked = r.goal
        else:
            self.path = None
            self._why = r.reason

    def _nearest(self, p: Pose) -> tuple[int, float, float]:
        """(segment index, distance along it, distance from p) of the route point nearest p."""
        best = (0, 0.0, math.inf)
        path = self.path or []
        for i, (a, b) in enumerate(zip(path, path[1:])):
            ax, ay = a
            vx, vy = b[0] - ax, b[1] - ay
            L2 = vx * vx + vy * vy
            s = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p.x - ax) * vx + (p.y - ay) * vy) / L2))
            d = math.hypot(ax + s * vx - p.x, ay + s * vy - p.y)
            if d < best[2]:
                best = (i, s * math.sqrt(L2), d)
        return best

    def _carrot(self, seg: int, along: float, ahead: float | None = None) -> tuple[float, float]:
        path = self.path or []
        left = self.config.lookahead_m if ahead is None else ahead
        i, off = seg, along
        while i < len(path) - 1:
            a, b = path[i], path[i + 1]
            L = math.dist(a, b)
            if off + left <= L:
                s = (off + left) / L if L else 0.0
                return a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1])
            left -= L - off
            i, off = i + 1, 0.0
        return path[-1]

    def _walk(self, seg: int, along: float, dist: float, step: float):
        """Points every `step` metres along the route, from (seg, along), for `dist`."""
        path = self.path or []
        i, off, gone = seg, along, 0.0
        while i < len(path) - 1 and gone <= dist:
            a, b = path[i], path[i + 1]
            L = math.dist(a, b)
            while off <= L and gone <= dist:
                s = off / L if L else 0.0
                yield a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1])
                off += step
                gone += step
            i, off = i + 1, off - L

    def _blocked_ahead(self, p: Pose, seg: int, along: float, cm: Costmap) -> bool:
        """Has the route just ahead turned lethal (something new in the way)?
        Not counted within escape_m of the robot: a route out of a tight spot
        starts inside the lethal zone on purpose."""
        near = cm.config.escape_m
        for x, y in self._walk(seg, along, self.config.check_ahead_m, cm.res * 0.5):
            if math.hypot(x - p.x, y - p.y) > near and cm.is_lethal(x, y):
                return True
        return False

    # -- one tick ---------------------------------------------------------

    def _past(self, p: Pose, end: tuple[float, float]) -> bool:
        """Has it driven PAST the goal? Distance alone never notices: if the
        robot overshoots (odometry under-reporting how far it has gone is
        enough), the gap starts growing again and it chases a point behind
        itself until something else stops it. Arriving is "close enough, OR the
        goal is behind me and was close": project the goal onto the nose."""
        if self.path is None or len(self.path) < 2:
            return False
        ax, ay = self.path[-2]
        bx, by = self.path[-1]
        hx, hy = bx - ax, by - ay                      # the last leg's direction
        n = math.hypot(hx, hy)
        if n < 1e-6:
            return False
        beyond = ((p.x - end[0]) * hx + (p.y - end[1]) * hy) / n
        return beyond > 0.0 and math.hypot(end[0] - p.x, end[1] - p.y) <= 3.0 * self.limits.pos_tol

    def _route_left(self, p: Pose) -> float:
        """Metres still to drive on the route it is committed to."""
        path = self.path or []
        if len(path) < 2:
            return math.inf
        seg, along, _ = self._nearest(p)
        total = -along
        for a, b in zip(path[seg:], path[seg + 1:]):
            total += math.dist(a, b)
        return max(0.0, total)

    def _path_still_good(self, p: Pose) -> bool:
        """Is the committed route worth keeping? Only if the robot is still on
        it and nothing has appeared across it."""
        if self.path is None or len(self.path) < 2:
            return False
        seg, along, off = self._nearest(p)
        if off > self.config.off_path_m:
            return False
        return not self._blocked_ahead(p, seg, along, self.mapper.costmap())

    def _start_backoff(self, p: Pose, cm: Costmap, now: float) -> bool:
        """Begin reversing out, if there is anywhere to reverse to and any tries
        left. Returns whether it started."""
        c = self.config
        if self._backoff_until is not None or self._backoffs >= c.max_backoffs:
            return False
        behind = (p.x - 0.15 * math.cos(p.theta), p.y - 0.15 * math.sin(p.theta))
        if cm.is_lethal(*behind):
            return False
        self._backoffs += 1
        self._blocked_s = 0.0
        self._backoff_until = now + c.backoff_s
        self._backoff_wz = self._turn_away(p) if c.reverse_to_turn else 0.0
        return True

    def _room_to_spin(self, p: Pose, cm: Costmap) -> bool:
        """Is there room for the corners to sweep? Asked of the MAP, not of the
        live scan, because the wheels blank wedges of the lidar's view and the
        obstacle a corner is about to meet is often in one of them."""
        r = self.config.spin_radius_m
        if r <= 0.0:
            return True
        return cm.clearance(p.x, p.y) >= r + self.config.spin_margin_m

    def _turn_rate(self, alpha: float, w_max: float) -> float:
        """How fast to swing the nose through `alpha`, in rad/s.

        Proportional, but capped by what can be stopped inside the angle that
        remains: w <= sqrt(2 * turn_decel * |alpha|), less the reaction time.
        Without that cap a high gear takes a 10 degree correction at 86 deg/s
        and sails past it; with it, big turns still go at the gear's rate and
        small ones ease in."""
        c = self.config
        a, t = max(c.turn_decel, 1e-3), c.turn_reaction_s
        stoppable = math.sqrt((a * t) ** 2 + 2.0 * a * abs(alpha)) - a * t
        w = min(abs(c.k_turn * alpha), max(stoppable, c.turn_min), w_max)
        return math.copysign(w, alpha)

    def _turn_away(self, p: Pose) -> float:
        """Which way to swing the nose while reversing. Toward the route, so the
        retry starts from a better angle than the one that just failed: backing
        straight out keeps the heading that got the robot stuck, which is how it
        ends up wedged against the next thing along. No route to aim at (the
        planner found none): reverse straight and let the re-plan decide."""
        if self.path is None or len(self.path) < 2:
            return 0.0
        seg, along, _ = self._nearest(p)
        cx, cy = self._carrot(seg, along)
        alpha = wrap_angle(math.atan2(cy - p.y, cx - p.x) - p.theta)
        w = self.limits.w_max
        return self._turn_rate(alpha, w)

    def _stop(self, status: str, reason: str) -> tuple[Action, bool]:
        self.plan = FollowPlan(status, reason)
        return Action(), False

    def step(self, base: Pose, goal: Pose) -> tuple[Action, bool]:
        raise TypeError("PathFollower needs the observation's clock: call "
                        "step_observation(obs, goal), or use run_goto")

    def step_observation(self, obs: Observation, goal: Pose) -> tuple[Action, bool]:
        c, lim = self.config, self.limits
        now = obs.t
        dt = 0.0 if self._last_t is None else max(0.0, min(0.5, now - self._last_t))
        self._last_t = now
        p = obs.base

        if self._end_goal != (goal.x, goal.y):         # a new destination: start again
            self._end_goal = (goal.x, goal.y)
            self._end_locked = None
        end = (self._end_locked
               or (self.result.goal if (self.result and self.result.ok and self.result.goal)
                   else (goal.x, goal.y)))
        d_end = math.hypot(end[0] - p.x, end[1] - p.y)
        if d_end <= lim.pos_tol or self._past(p, end):
            self.plan = FollowPlan("arrived", "")
            return Action(), True

        if self._backoff_until is not None:
            if now < self._backoff_until:
                self.plan = FollowPlan("waiting", "backing off to get room to turn",
                                       self._backoff_wz / max(c.k_turn, 1e-6), 0.0)
                return Action(base_vx=-c.backoff_mps, base_wz=self._backoff_wz), False
            self._backoff_until = None
            self._planned_at = -math.inf                # re-plan from where it ended up

        refused = False
        if self.bubble is not None:
            b = self.bubble()
            refused = b is not None and getattr(b, "state", "") == "blocked"

        if self.mapper.scan_age() > c.stale_scan_s:
            self._blocked_s += dt
            if self._blocked_s > c.patience_s:
                self.failure = ("My lidar map is out of date (no scan for "
                                f"{self.mapper.scan_age():.1f} s), so I've stopped.", {"blocked": True})
            return self._stop("waiting", "waiting for the lidar")

        cm = self.mapper.costmap()
        if self.path is not None:
            seg, along, off = self._nearest(p)
            if off > c.off_path_m or self._blocked_ahead(p, seg, along, cm) or refused:
                self._planned_at = -math.inf
        if now - self._planned_at >= c.replan_s:
            self._replan(p, goal, now)

        if self.path is None or refused:
            self._blocked_s += dt
            if self._blocked_s > c.backoff_after_s and self._start_backoff(p, cm, now):
                return self._stop("waiting", "backing off to get room to turn")
            if self._blocked_s > c.patience_s:
                why = ("my safety bubble keeps stopping me" if refused and self.path is not None
                       else self._why)
                self.failure = (f"I can't find a way there: {why}.", {"blocked": True})
            return self._stop("blocked", self._why if self.path is None else "the bubble stopped me")
        # Leaky, NOT symmetric. A refusal stops the robot, a stopped robot is
        # safe, so the bubble clears and it tries the same thing again: blocked
        # and clear ticks alternate. Subtracting dt cancelled them exactly, the
        # counter never reached backoff_after_s, and the robot sat there
        # flickering between "following" and "the bubble stopped me" instead of
        # backing out. Trouble has to outlive a good tick to count as over.
        self._blocked_s = max(0.0, self._blocked_s - c.blocked_decay * dt)

        seg, along, _ = self._nearest(p)
        cx, cy = self._carrot(seg, along)
        alpha = wrap_angle(math.atan2(cy - p.y, cx - p.x) - p.theta)
        # Hysteresis: it takes rotate_deg to START turning on the spot but only
        # rotate_exit_deg to stop. One threshold made it chatter across the line
        # -- turn, drive, turn, drive -- which is the jitter seen near a wall.
        gate = c.rotate_exit_deg if self._turning else c.rotate_deg
        if abs(alpha) > math.radians(gate):
            if not self._room_to_spin(p, cm):
                # Nose against something and a big turn wanted: reversing is the
                # obvious move, so make it the FIRST one. Waiting out
                # backoff_after_s here just grinds against the obstacle for a
                # second before reaching the same conclusion.
                self._why = "no room to turn here"
                if self._start_backoff(p, cm, now):
                    return self._stop("waiting", "backing out: no room to turn here")
                self._blocked_s += dt
                return self._stop("blocked", self._why)
            self._turning = True
            # Aim at a point further along than the driving carrot. Turning to
            # face a close one picks a heading that is right for the next half
            # metre and wrong for the corner after it, so the robot arrives
            # needing a second, harder turn. Looking further ahead commits to a
            # heading that serves both.
            fx, fy = self._carrot(seg, along, c.lookahead_m * c.turn_lookahead)
            far = wrap_angle(math.atan2(fy - p.y, fx - p.x) - p.theta)
            if abs(far) > math.radians(c.rotate_exit_deg) and far * alpha > 0:
                alpha = far                    # same way round, just further round
            wz = self._turn_rate(alpha, lim.w_max)
            self.plan = FollowPlan("turning", "turning to face the way", alpha, 0.0)
            return Action(base_wz=wz), False
        self._turning = False

        ld = max(0.05, math.hypot(cx - p.x, cy - p.y))
        curvature = 2.0 * math.sin(alpha) / ld
        k = max(abs(curvature), 1e-6)
        v = min(lim.v_max, lim.w_max / k, math.sqrt(c.lat_accel_max / k))
        # How fast may it go and still stop in the room it can see? A scan is up
        # to a revolution old and the loop reacts in a tick, so both count as
        # reaction time. Gear 3 in a corridor now slows itself; gear 1 in an
        # open room no longer crawls.
        room = max(0.0, cm.clearance(p.x, p.y) - cm.config.lethal_m)
        t_r = c.reaction_s + min(self.mapper.scan_age(), 0.5)
        a_b = max(c.brake_mps2, 1e-3)
        v_room = math.sqrt((a_b * t_r) ** 2 + 2.0 * a_b * room) - a_b * t_r
        # Drop instantly, rise slowly. Clearance is measured from a noisy scan,
        # and because wz = v * curvature, every wobble in it shook the STEERING
        # as well as the speed -- which is the jitter felt while curving round
        # something. Falling limits must still apply at once: that direction is
        # the safe one.
        if self._v_room is None or v_room <= self._v_room:
            self._v_room = v_room
        else:
            k = min(1.0, dt / max(c.speed_rise_tau_s, 1e-3))
            self._v_room += k * (v_room - self._v_room)
        v = min(v, max(c.creep_mps, self._v_room))
        left = math.hypot(end[0] - p.x, end[1] - p.y)
        v = min(v, max(c.creep_mps, c.arrive_gain * left))
        lo = math.cos(math.radians(c.rotate_deg))
        hi = math.cos(math.radians(c.align_full_deg))
        # Down to a FLOOR, not to zero. wz = v * curvature, so a factor that
        # reaches zero kills the turn along with the drive -- and it reached it
        # at exactly rotate_deg, the angle where turning on the spot takes over.
        # A carrot a hair inside that (measured on the robot: 54.9 deg of 55)
        # left the follower commanding 4 mm/s and 0 rad/s, so the angle never
        # changed and it sat there until the goal timed out.
        align = (math.cos(alpha) - lo) / (hi - lo)
        v *= c.align_floor + (1.0 - c.align_floor) * max(0.0, min(1.0, align))
        if v < c.creep_mps and abs(alpha) > math.radians(c.align_full_deg):
            wz = self._turn_rate(alpha, lim.w_max)     # arcing is pointless: turn instead
            self.plan = FollowPlan("turning", "turning to face the way", alpha, 0.0)
            return Action(base_wz=wz), False
        wz = v * curvature
        if abs(wz) > lim.w_max:                       # too tight for this speed: slow, keep the arc
            v *= lim.w_max / abs(wz)
            wz = math.copysign(lim.w_max, wz)
        self.plan = FollowPlan("following", "", alpha, v / lim.v_max if lim.v_max else 0.0)
        return Action(base_vx=v, base_wz=wz), False
