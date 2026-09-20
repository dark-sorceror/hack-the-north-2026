"""Lidar obstacle avoidance: which way to steer round what is in front, and how fast.

LEVEL 2, NOT A MAP. Navigation is still AprilTag waypoints + wheel odometry.
This only decides, scan by scan, how to get round a chair that stands between
the robot and wherever it was already going. No SLAM, no occupancy grid, no
global path. The Pi's safety bubble (bridge/safety.py) stays the last word on
contact; this layer's job is to make sure the bubble rarely has to speak.

THE ALGORITHM: VFH-lite (the binary polar histogram of VFH+), not follow-the-gap.
Follow-the-gap heads for the widest opening and needs a goal term bolted on.
Here the job is "get to THIS goal, going round what's in the way", and VFH's
"the free direction closest to the goal" is exactly that. It also gives, for
free, the two things this robot needs: inflation by the robot's size (the
enlargement angle) and a cost function to hang hysteresis on.

  1. Sectors. The circle round the robot is cut into 2-degree sectors.
  2. Inflate. Every return within lookahead_m blocks the sectors within
     asin((radius + margin) / range) of its bearing: a straight drive in any of
     those directions passes closer than radius + margin to that point.
  3. The goal is not an obstacle. Returns within goal_clear_m of the goal point
     ARE the goal (a bin, a hand-off station, the thing to pick up) and are
     dropped. Returns at or beyond the goal's range don't block directions that
     point at the goal: the trip ends before it reaches them. Without this the
     robot routes round the very bin it was sent to.
  4. Unknown is not free. A direction is FREE only if the rays that look down
     its corridor came back: at least 20% of them overall (the bubble's rule),
     no more than 25% masked, and no 10-degree stretch among them with no
     return at all. That last one is what a black or chrome box looks like: a
     hole in a scan that is otherwise full of returns. Anything else (dark,
     shiny, open space, a masked blind sector) is UNKNOWN: allowed, but only at
     creep speed.
  5. Choose. Among unblocked directions within max_detour_deg of the goal, the
     cheapest wins: angle from the goal, + unknown_penalty_deg if unseen, +
     hysteresis_deg to switch to the other side of the goal from the one it is
     already going round. That last term is what stops it flip-flopping between
     two equal gaps. No candidate at all -> BLOCKED, speed 0.
  6. Speed. Forward speed ramps down with the free distance straight ahead
     (stop_m..slow_m); into the unknown or onto the goal object it is capped at
     creep. Turning in place sweeps the corners, so the turn rate drops to creep
     when something is within turn_guard_m of them and part of the view is blind.

STAYING INSIDE THE BUBBLE. The bubble (safety.py rule 2) lets the robot drive at
the speed it can still stop from; at 0.35 m/s it starts slowing ~0.3 m from the
body, at creep it allows 3 cm. The planner passes things with 0.12 m of margin
(the bubble keeps 0.02 m) and ramps its speed to zero 0.10 m from the body, well
under the bubble's allowed speed at every clearance (tests/test_avoid.py checks
the curve). So the bubble should only ever fire on something the planner didn't
see, or on the goal itself; when it does, the controllers back off and re-plan.

Stdlib only; a plan over a 600-point scan takes about a millisecond.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, replace
from typing import Any, Callable, Iterable, Sequence

from retriever.navigation.geometry import TAU, angle_diff, world_to_base, wrap_angle
from retriever.types import Pose

CLEAR, STEERING, BLOCKED = "clear", "steering", "blocked"

# One letter per sector in Plan.sectors (compact enough to stream to a page).
FREE, OCCUPIED, UNKNOWN = "f", "b", "u"


def _in_arc(bearing: float, start: float, width: float) -> bool:
    return (bearing - start) % TAU <= width


# ------------------------------------------------------------------ the lidar


def parse_mask(text: str | None) -> tuple[tuple[float, float], ...]:
    """'100:140,350:10' -> ((a0, a1), ...) radians. Same format and frame as
    `--lidar-mask` (bridge/safety.parse_mask): degrees counter-clockwise from the
    lidar's own 0 mark, START:END, and 350:10 wraps through 0."""
    if not text:
        return ()
    out = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        try:
            a, b = (float(v) for v in part.split(":"))
        except ValueError:
            raise ValueError(f"bad mask interval {part!r}; expected START:END in degrees") from None
        out.append((math.radians(a) % TAU, math.radians(b) % TAU))
    return tuple(out)


@dataclass(frozen=True)
class Mount:
    """Where the lidar sits on the base, and which of its rays the robot blocks.

    Base frame: x forward, y left, metres from the base centre; bearings
    counter-clockwise, + is LEFT. The RPLIDAR reports angles CLOCKWISE seen from
    above, 0 at its front mark (the side away from its cable). So:

        right way up:              bearing_base = yaw_rad - angle_lidar
        upside down (inverted):    bearing_base = yaw_rad + angle_lidar

    This is the most dangerous sign in the task. Wrong, and a chair on the left
    is planned on the right, and the robot steers INTO it. ray_bearing() is the
    only place it is written; tests/test_avoid.py pins it with an obstacle on the
    left, both ways up.

    yaw_rad   base-frame bearing of the lidar's 0 mark (bridge LidarMount.yaw_rad).
    inverted  hung upside down under the chassis (bridge LidarMount.inverted).
    mask      blind sectors the robot's own wheels / chassis / arm cover: a list
              of (start, end) radians in the lidar's OWN frame, counter-clockwise
              from its 0 mark, start > end wrapping through 0 -- the same numbers
              as BubbleConfig.mask, so one recorded mask serves both
              (parse_mask reads the `--lidar-mask` string). Rays in a masked
              sector are thrown away and the sector counts as BLIND: unknown,
              never free.
    """

    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0
    inverted: bool = False
    mask: tuple[tuple[float, float], ...] = ()

    def ray_bearing(self, angle_cw_deg: float) -> float:
        """Base-frame direction of one raw RPLIDAR ray."""
        a = math.radians(angle_cw_deg)
        return wrap_angle(self.yaw_rad + a if self.inverted else self.yaw_rad - a)

    def to_base(self, angle_cw_deg: float, range_m: float) -> tuple[float, float]:
        """A raw return -> (x, y) in the base frame, mount offset included."""
        b = self.ray_bearing(angle_cw_deg)
        return self.x_m + range_m * math.cos(b), self.y_m + range_m * math.sin(b)

    def masked(self, angle_cw_deg: float) -> bool:
        a = (-math.radians(angle_cw_deg)) % TAU  # the lidar's own CCW frame, where masks live
        return any(_in_arc(a, s, w) for s, w in _arcs(self.mask))

    def blind(self) -> tuple[tuple[float, float], ...]:
        """The mask as base-frame (start, width) bearing arcs. Seen from the lidar,
        which is exact for a centred mount and close enough off-centre: a blind
        sector is a direction, not a place."""
        out = []
        for s, w in _arcs(self.mask):
            start = self.yaw_rad - s - w if self.inverted else self.yaw_rad + s
            out.append((wrap_angle(start), w))
        return tuple(out)


def _arcs(mask: Iterable[tuple[float, float]]) -> list[tuple[float, float]]:
    """(start, end) intervals -> (start, width), width in [0, 2*pi]."""
    out = []
    for a0, a1 in mask:
        w = (a1 - a0) % TAU
        if w == 0.0 and a1 != a0:
            w = TAU  # 0:360 means everything
        out.append((a0 % TAU, w))
    return out


@dataclass(frozen=True)
class Scan:
    """One lidar revolution the way the planner wants it: BASE frame.

    points are (bearing_rad, range_m) from the base centre, bearing + = left,
    returns only. ray_step_rad is the spacing of ALL rays, the ones that came
    back and the ones that didn't: with the returns it says how much of each
    direction the lidar actually saw. blind: (start, width) base-frame arcs the
    lidar cannot see at all (Mount.blind()). `t` must be on the same clock as
    Observation.t (on the bridge, both are the Pi's State.t clock).
    """

    t: float
    points: tuple[tuple[float, float], ...]
    ray_step_rad: float = math.radians(1.0)
    blind: tuple[tuple[float, float], ...] = ()
    bubble_blocked: bool = False  # the Pi's safety bubble refused the last command


def scan_from_rplidar(
    t: float,
    samples: Iterable[tuple[float, float]],
    mount: Mount = Mount(),
    *,
    n_rays: int | None = None,
    min_range_m: float = 0.20,
    max_range_m: float = 16.0,
) -> Scan:
    """Raw RPLIDAR samples -> Scan. samples are (angle_cw_deg, range_m), range 0
    meaning no return. Give n_rays when the list holds only the returns (as
    rplidar's iter_scans does); otherwise every sample counts as a ray."""
    pts = []
    n = 0
    for angle_cw, r in samples:
        n += 1
        if not r or r < min_range_m or r > max_range_m or mount.masked(angle_cw):
            continue
        x, y = mount.to_base(angle_cw, r)
        pts.append((math.atan2(y, x), math.hypot(x, y)))
    return Scan(t, tuple(pts), TAU / max(1, n_rays or n), mount.blind())


def scan_from_lidar_scan(scan: Any, mount: Mount = Mount(), min_range_m: float = 0.20) -> Scan:
    """bridge.lidar.LidarScan (sensor frame, counter-clockwise radians, returns
    only, n_raw = every packet) -> Scan. Duck-typed: nothing here imports the
    Pi-side driver."""
    samples = [((-math.degrees(a)) % 360.0, r) for a, r, *_ in scan.points]
    return scan_from_rplidar(scan.t, samples, mount, n_rays=scan.n_raw or len(samples),
                             min_range_m=min_range_m)


def scan_from_xy(
    t: float,
    xy: Iterable[tuple[float, float]],
    ray_step_rad: float,
    blind: tuple[tuple[float, float], ...] = (),
    bubble_blocked: bool = False,
) -> Scan:
    """Base-frame (x, y) points, e.g. the bridge's downsampled scan, one point
    per bin at most: then ray_step_rad is the bin width."""
    pts = tuple((math.atan2(y, x), math.hypot(x, y)) for x, y in xy)
    return Scan(t, pts, ray_step_rad, blind, bubble_blocked)


def bridge_scan_source(robot: Any, mount: Mount = Mount(), step_deg: float = 2.0) -> Callable[[], Scan | None]:
    """A scan source over BridgeRobot (bridge/client.py): its `scan` (base frame,
    already self- and mask-filtered on the Pi, 2-degree bins) and its `bubble`.
    `mount` only supplies the blind sectors; pass the same mask the Pi uses."""
    blind = mount.blind()
    step = math.radians(step_deg)

    def source() -> Scan | None:
        view = robot.scan
        if view is None:
            return None
        bubble = robot.bubble
        refused = bubble is not None and bubble.state == "blocked"
        return scan_from_xy(view.t, ((x, y) for x, y, *_ in view.points), step, blind, refused)

    return source


def reproject(scan: Scan, then: Pose, now: Pose) -> Scan:
    """The same returns, seen from where the robot is NOW rather than where it
    was when the scan was taken. Blind arcs are the robot's own, so they stay."""
    if then == now:
        return scan
    c, s = math.cos(then.theta), math.sin(then.theta)
    out = []
    for b, r in scan.points:
        px, py = r * math.cos(b), r * math.sin(b)
        wx, wy = then.x + px * c - py * s, then.y + px * s + py * c
        fwd, left = world_to_base(wx - now.x, wy - now.y, now.theta)
        out.append((math.atan2(left, fwd), math.hypot(fwd, left)))
    return replace(scan, points=tuple(out))


class ScanTracker:
    """Hands the planner the latest scan, as seen from where the robot is now.

    Two corrections for things that are true on the real robot:

      * Latency. A scan is 0.1-0.3 s old when it is used (a revolution, the
        wire at 5 Hz, WiFi). A tank turning at 1.2 rad/s turns 14 degrees in
        0.2 s: a chair 1 m away would be planned 25 cm from where it is. Every
        point is moved by the odometry change since its scan was taken.
      * Dropouts. ~60% of rays return nothing, so a chair leg is in one scan and
        gone from the next. The scans of the last memory_s are merged. That is a
        half-second persistence buffer, not a map: nothing older is kept, and
        nothing is looked up by position. memory_s=0 plans on one scan only.

    Returns None when the newest scan is older than stale_s: planning on a
    picture of the world from a second ago is worse than stopping.
    """

    def __init__(self, source: Callable[[], Scan | None], memory_s: float = 0.5,
                 stale_s: float = 0.6, history_s: float = 3.0) -> None:
        self.source = source
        self.memory_s = memory_s
        self.stale_s = stale_s
        self.history_s = history_s
        self.bubble_blocked = False
        self._poses: deque[tuple[float, Pose]] = deque()
        self._scans: list[tuple[Scan, Pose]] = []

    def update(self, t: float, pose: Pose) -> None:
        self._poses.append((t, pose))
        while self._poses and t - self._poses[0][0] > self.history_s:
            self._poses.popleft()
        try:
            scan = self.source()
        except Exception:  # a crashing source is a missing scan, not a crashed skill
            scan = None
        if scan is None:
            return
        self.bubble_blocked = scan.bubble_blocked
        if not self._scans or scan.t != self._scans[-1][0].t:
            self._scans.append((scan, self._pose_at(scan.t, pose)))
        newest = self._scans[-1][0].t
        self._scans = [(s, p) for s, p in self._scans if newest - s.t <= self.memory_s]

    def age(self, t: float) -> float:
        return math.inf if not self._scans else t - self._scans[-1][0].t

    def current(self, t: float, pose: Pose) -> Scan | None:
        if self.age(t) > self.stale_s:
            return None
        latest = self._scans[-1][0]
        pts: list[tuple[float, float]] = []
        for scan, then in self._scans:
            pts.extend(reproject(scan, then, pose).points)
        k = len(self._scans)
        return Scan(latest.t, tuple(pts), latest.ray_step_rad / k, latest.blind, self.bubble_blocked)

    def _pose_at(self, ts: float, fallback: Pose) -> Pose:
        """The odometry pose when the scan was taken: the last one recorded at or
        before ts. A scan older than the history gets the oldest pose we have."""
        best = None
        for t, p in self._poses:
            if t > ts:
                break
            best = p
        if best is None:
            best = self._poses[0][1] if self._poses else fallback
        return best


# ------------------------------------------------------------------ planner


@dataclass(frozen=True)
class AvoidConfig:
    """Every number the avoidance uses. Placeholders are marked; measure them."""

    # Geometry. radius_m is the body's CIRCUMSCRIBED radius: the corners, which a
    # skid-steer sweeps when it turns. Default = the bubble's placeholder
    # footprint 0.25/0.25/0.20 m. Use for_footprint() with the measured one.
    radius_m: float = 0.32
    margin_m: float = 0.12            # kept clear past the body; bubble keeps 0.02
    lookahead_m: float = 1.5          # returns farther than this are not obstacles
    sector_deg: float = 2.0
    max_detour_deg: float = 90.0      # a way round must still make progress to the goal
    goal_clear_m: float = 0.35        # returns this close to the goal ARE the goal
    self_radius_m: float = 0.0        # returns this close to the centre are the robot (0: trust the mask)

    # Speed, as fractions of the controller's limits.
    stop_m: float = 0.10              # free path ahead (past the body) where forward speed hits 0
    slow_m: float = 0.80              # ...and where it starts to fall
    creep_scale: float = 0.14         # ~0.05 m/s of 0.35: the bubble's creep speed
    turn_guard_m: float = 0.10        # nearer than this to the corners' circle: careful turns

    # What counts as seen. The first two are the bubble's rule 6 thresholds.
    min_valid_fraction: float = 0.20  # of the unmasked rays near a direction
    max_blind_fraction: float = 0.25  # of the rays near a direction that are masked
    hole_deg: float = 10.0            # this wide with no return at all = something dark (0: off)

    # Choosing.
    unknown_penalty_deg: float = 45.0  # a seen way round wins if within this of an unseen one
    hysteresis_deg: float = 15.0       # switching sides of the goal must win by this much

    # Scan handling (ScanTracker).
    memory_s: float = 0.5
    stale_s: float = 0.6

    # Giving up, and the bubble (drive.py).
    patience_s: float = 3.0           # blocked this long (a person gets to move) -> fail
    backoff_s: float = 1.5            # after the bubble refuses a command: back off...
    backoff_mps: float = 0.05         # ...at creep, ~7 cm
    max_refusals: int = 3             # bubble refusals per trip before giving up

    @classmethod
    def for_footprint(cls, front_m: float, rear_m: float, half_width_m: float, **kw: Any) -> AvoidConfig:
        """From the same rectangle as the bubble's --footprint FRONT,REAR,HALF_WIDTH."""
        return cls(radius_m=math.hypot(max(front_m, rear_m), half_width_m), **kw)


@dataclass(frozen=True)
class Plan:
    status: str                  # clear | steering | blocked
    steer_rad: float             # base-frame bearing to head for (+ left); the goal's own when clear
    speed_scale: float           # 0..1, multiplies forward speed
    turn_scale: float            # 0..1, multiplies turn rate
    reason: str                  # a sentence the robot can say
    obstacle_m: float | None = None   # body -> what blocks the straight line to the goal
    seen: bool = True            # enough rays came back toward steer_rad to call it free
    goal_rad: float = 0.0
    goal_m: float = 0.0
    sectors: str = ""            # sector i is centred on bearing i * sector_rad; f / b / u
    sector_rad: float = math.radians(2.0)

    def as_dict(self) -> dict[str, Any]:
        """Plain data for a page: degrees, rounded."""
        return {
            "status": self.status, "reason": self.reason,
            "steer_deg": round(math.degrees(self.steer_rad), 1),
            "speed": round(self.speed_scale, 3), "turn": round(self.turn_scale, 3),
            "seen": self.seen,
            "obstacle_m": None if self.obstacle_m is None else round(self.obstacle_m, 2),
            "goal_deg": round(math.degrees(self.goal_rad), 1), "goal_m": round(self.goal_m, 2),
            "sector_deg": round(math.degrees(self.sector_rad), 3), "sectors": self.sectors,
        }


def _ramp(x: float, lo: float, hi: float) -> float:
    if x <= lo:
        return 0.0
    if x >= hi:
        return 1.0
    return (x - lo) / (hi - lo)


def _towards(bearing: float) -> str:
    d = math.degrees(wrap_angle(bearing))
    if abs(d) <= 45.0:
        return "ahead"
    if abs(d) >= 135.0:
        return "behind me"
    return "to my left" if d > 0 else "to my right"


def _free_ahead(xy: Iterable[tuple[float, float]], radius: float) -> float:
    """How far the body disc can drive straight forward before touching a point."""
    best = math.inf
    r2 = radius * radius
    for x, y in xy:
        if x > 0.0 and abs(y) < radius:
            best = min(best, max(0.0, x - math.sqrt(r2 - y * y)))
    return best


class LocalPlanner:
    """VFH-lite on one Scan. Holds one piece of state: which side of the goal it
    is going round (the hysteresis). Use one planner per trip, or reset()."""

    def __init__(self, config: AvoidConfig = AvoidConfig()) -> None:
        self.config = config
        self.n = max(8, round(360.0 / config.sector_deg))
        self.w = TAU / self.n
        self._side = 0  # +1 going round on the left, -1 on the right, 0 straight at the goal

    def reset(self) -> None:
        self._side = 0

    def prefer_other_side(self) -> None:
        """The way round we were taking didn't work (the bubble stopped us): let
        the hysteresis favour the other side instead."""
        self._side = -self._side

    def _sector(self, bearing: float) -> int:
        return round(wrap_angle(bearing) / self.w) % self.n

    def no_scan(self, goal_rad: float, goal_m: float, why: str) -> Plan:
        return Plan(BLOCKED, wrap_angle(goal_rad), 0.0, 0.0,
                    f"I can't see where I'm going: {why}, so I've stopped.",
                    seen=False, goal_rad=wrap_angle(goal_rad), goal_m=goal_m, sector_rad=self.w)

    def turn_scale(self, scan: Scan) -> float:
        """How fast it may turn in place. The corners sweep the body's circle, so
        a point near that circle means careful turns; if part of the view is
        blind, whatever is near may reach into the blind part, so only creep.
        Every return counts here, the goal's too: this is about the corners
        hitting things, not about where to go."""
        c = self.config
        near = min((r - c.radius_m for _, r in scan.points if r >= c.self_radius_m),
                   default=math.inf)
        if near >= c.turn_guard_m:
            return 1.0
        if scan.blind or near <= 0.0:
            return c.creep_scale
        return max(c.creep_scale, near / c.turn_guard_m)

    def plan(self, scan: Scan, goal_rad: float, goal_m: float) -> Plan:
        c, n, w = self.config, self.n, self.w
        body = c.radius_m
        inflated = c.radius_m + c.margin_m
        # Already-wrapped goals (atan2's) pass through untouched: re-wrapping moves
        # the last bit, and "clear" must hand back exactly the controller's own bearing.
        if not -math.pi <= goal_rad <= math.pi:
            goal_rad = wrap_angle(goal_rad)
        goal_m = max(0.0, goal_m)
        gx, gy = goal_m * math.cos(goal_rad), goal_m * math.sin(goal_rad)
        # Directions that point at the goal zone. Inside it: everything forward of it.
        cone = (math.asin(c.goal_clear_m / goal_m) if goal_m > c.goal_clear_m else math.pi / 2)
        in_cone = [abs(angle_diff(i * w, goal_rad)) <= cone for i in range(n)]

        # 1. Sort the returns: evidence, obstacles, and the goal itself.
        seen = [0] * n
        obstacles: list[tuple[float, float, float, float]] = []   # bearing, range, x, y
        goal_pts: list[tuple[float, float]] = []
        for b, r in scan.points:
            if r < c.self_radius_m:
                continue
            seen[self._sector(b)] += 1
            if r > c.lookahead_m:
                continue
            x, y = r * math.cos(b), r * math.sin(b)
            if c.goal_clear_m > 0.0 and math.hypot(x - gx, y - gy) <= c.goal_clear_m:
                goal_pts.append((x, y))
            else:
                obstacles.append((b, r, x, y))

        # 2. Inflate: each obstacle blocks every sector a pass would come too close in.
        blocked_at = [math.inf] * n
        for b, r, _, _ in obstacles:
            half = math.asin(min(1.0, inflated / r))
            past_goal = r >= goal_m
            for k in range(math.ceil((b - half) / w), math.floor((b + half) / w) + 1):
                i = k % n
                if past_goal and in_cone[i]:
                    continue    # the trip to the goal ends before it gets there
                blocked_at[i] = min(blocked_at[i], r)

        # 3. Evidence: did enough rays come back near each unblocked direction?
        # The window is the corridor's angular width at the distance that matters.
        blind = [1 if any(_in_arc(i * w, s, bw) for s, bw in scan.blind) else 0 for i in range(n)]
        seen_sum, blind_sum = _circular_prefix(seen), _circular_prefix(blind)
        # A hole: hole_deg of unmasked rays with not one return between them.
        h = round(c.hole_deg / 2.0 / math.degrees(w))
        holes = [0] * n
        if h > 0:
            holes = [1 if _window(seen_sum, n, j, h) == 0 and _window(blind_sum, n, j, h) == 0
                     else 0 for j in range(n)]
        hole_sum = _circular_prefix(holes)
        state = []
        for i in range(n):
            if blocked_at[i] < math.inf:
                state.append(OCCUPIED)
                continue
            reach = min(c.lookahead_m, goal_m) if in_cone[i] else c.lookahead_m
            k = max(1, min(n // 4, round(math.asin(min(1.0, inflated / max(reach, 1e-6))) / w)))
            width = 2 * k + 1
            blind_frac = _window(blind_sum, n, i, k) / width
            rays = width * w / scan.ray_step_rad * (1.0 - blind_frac)
            got = _window(seen_sum, n, i, k)
            unseen = (blind_frac > c.max_blind_fraction
                      or got < c.min_valid_fraction * rays
                      or (h > 0 and _window(hole_sum, n, i, k + h) > 0))
            state.append(UNKNOWN if unseen else FREE)

        # 4. Choose: the cheapest unblocked direction that still heads for the goal.
        i_goal = self._sector(goal_rad)
        max_detour = math.radians(c.max_detour_deg)
        penalty_unknown = math.radians(c.unknown_penalty_deg)
        penalty_switch = math.radians(c.hysteresis_deg)
        best, best_key, best_side = None, None, 0
        for i in range(n):
            if state[i] == OCCUPIED:
                continue
            d = angle_diff(i * w, goal_rad)
            if abs(d) > max_detour:
                continue
            side = 0 if i == i_goal else (1 if d > 0 else -1)
            cost = abs(d)
            if state[i] == UNKNOWN:
                cost += penalty_unknown
            if self._side and side == -self._side:
                cost += penalty_switch
            key = (round(cost, 9), -d)  # exact ties go left, deterministically
            if best_key is None or key < best_key:
                best, best_key, best_side = i, key, side

        goal_blocked_m = (max(0.0, blocked_at[i_goal] - body)
                          if blocked_at[i_goal] < math.inf else None)
        sectors = "".join(state)
        common = {"goal_rad": goal_rad, "goal_m": goal_m, "sectors": sectors, "sector_rad": w}

        if best is None:
            self._side = 0
            where = _towards(goal_rad)
            what = (f"about {goal_blocked_m:.1f} m {where}" if goal_blocked_m is not None else where)
            return Plan(BLOCKED, goal_rad, 0.0, 0.0,
                        f"Something is blocking the way {what} and I can't find a way around it.",
                        obstacle_m=goal_blocked_m, seen=state[i_goal] == FREE, **common)

        self._side = best_side
        steer = goal_rad if best == i_goal else wrap_angle(best * w)
        is_seen = state[best] == FREE

        # 5. Speed: free path straight ahead; creep into the unknown and onto the goal.
        ahead = _free_ahead(((x, y) for _, _, x, y in obstacles), body)
        speed = _ramp(ahead, c.stop_m, c.slow_m)
        onto_goal = _free_ahead(goal_pts, body)
        creeping_to_goal = onto_goal < c.slow_m
        if creeping_to_goal:
            speed = min(speed, max(c.creep_scale, _ramp(onto_goal, c.stop_m, c.slow_m)))
        if not is_seen:
            speed = min(speed, c.creep_scale)
        turn = self.turn_scale(scan)

        # 6. Say why.
        if best == i_goal:
            status = CLEAR
            if not is_seen:
                reason = "I can't see much that way, so I'm creeping."
            elif creeping_to_goal and speed < 1.0:
                reason = "Creeping up to the goal."
            elif speed < 1.0:
                reason = f"Slowing down: something is {ahead:.1f} m ahead."
            else:
                reason = "The way is clear."
        else:
            status = STEERING
            way = "left" if best_side > 0 else "right"
            if goal_blocked_m is not None:
                reason = (f"Something is in the way about {goal_blocked_m:.1f} m "
                          f"{_towards(goal_rad)}, so I'm going round it on the {way}.")
            else:
                reason = f"I can't see clearly towards the goal, so I'm going round on the {way}."
            if not is_seen:
                reason += " I can't see much that way either, so I'm creeping."
        return Plan(status, steer, speed, turn, reason, obstacle_m=goal_blocked_m,
                    seen=is_seen, **common)


def _circular_prefix(values: Sequence[int]) -> list[int]:
    """Prefix sums over three copies, so a window can wrap either way."""
    out = [0]
    for v in list(values) * 3:
        out.append(out[-1] + v)
    return out


def _window(prefix: list[int], n: int, i: int, k: int) -> int:
    """Sum of values[i-k .. i+k], circular."""
    return prefix[n + i + k + 1] - prefix[n + i - k]


__all__ = [
    "BLOCKED", "CLEAR", "STEERING", "AvoidConfig", "LocalPlanner", "Mount", "Plan",
    "Scan", "ScanTracker", "bridge_scan_source", "parse_mask", "reproject",
    "scan_from_lidar_scan", "scan_from_rplidar", "scan_from_xy",
]
