"""SafetyBubble: the Pi refuses to drive into what the lidar sees.

The Pi's only autonomous decisions are STOP decisions (server.py). This keeps
to that rule: given the latest scan and the laptop's COMMANDED base velocity
(vx, wz), it returns the same command scaled by a factor k in [0, 1] and says
why. It never raises a command, never changes its direction or curvature,
never touches arm joints, gripper or vacuum, and has no way to clear an
e-stop. It is not navigation: no map, no planning, no SLAM.

THE RULES

1. Look where the command goes. The command (vx, wz) held constant moves the
   base along an arc (a line when wz == 0, a spin when vx == 0). The bubble
   slides the footprint rectangle, inflated by body_margin_m, along that path
   in 1 cm steps and finds how far it gets before it touches an obstacle
   point: `free_m`. Forward only looks forward, reverse only behind, a spin
   only sweeps the corners, so an obstacle ahead never stops a reverse or a
   turn that does not hit it. Distance along the path is measured at the
   footprint's fastest-moving corner, so a spin and a drive are held to the
   same stopping rule.

2. Stop distance scales with speed. Moving at u (fastest corner, m/s) needs

       need(u) = u * t_react + u^2 / (2 * decel) + stop_margin

   t_react = latency_s + the scan's age + one revolution period: the obstacle
   point may have been measured one revolution before the scan finished, the
   scan is `age` old by now, and latency_s covers the laptop's act period, the
   bubble tick and the wheel motors' response. Numbers (defaults):

       latency_s     0.10 s   ASSUMED (the DDSM115 response time is not known yet)
       revolution    0.13 s   measured: ~7.5 rev/s on our unit (0.15 s if unknown)
       decel_mps2    0.50     ASSUMED gentle braking: the wheel firmware's ramp is
                              not known, and a skid-steer carrying an arm should
                              not stop harder than this
       stop_margin   0.05 m
       body_margin   0.02 m   footprint inflation: tape-measure error + lidar noise
                              (~1% of range [DS] -> 1-2 cm at 1-2 m)

   At Limits.v_max 0.35 m/s with a fresh scan (t_react ~0.25-0.35 s):
   need ~0.27-0.30 m, i.e. the robot starts slowing ~0.3 m from the body.
   At 0.2 m/s: ~0.15 m. The allowed speed for a given free_m inverts need().

3. Creep. Below creep_speed_mps (0.05 m/s) the robot may close to
   creep_clearance_m (0.03 m past the inflated body, ~5 cm from the real
   body). Without this, the bubble would stop the robot ~0.3 m from every
   bin and hand-off station and it could never deposit anything. Worst-case
   creep stopping distance: 0.05 * 0.4 + 0.05^2 / 1.0 = 0.0225 m < 0.03 m, so
   even a creep never touches. BubbleConfig refuses settings that break this.

4. Stale or missing scan => creep only. No revolution for stale_after_s
   (0.5 s: ~3-4 revolutions at our rate) and every motion is capped at creep
   speed, and the reason says so. A stale scan can still slow the robot
   further; it can never permit anything.

5. Self-mask. Returns inside the robot's own footprint are ignored (they are
   the robot). Parts that block the view (wheels around an under-chassis
   mount, the chassis behind a front mount, the arm, a mast) are masked by
   sensor angle: a LIST of intervals, e.g. four wheel sectors.
   propose_mask() proposes it from scans recorded with nothing around
   (scripts/lidar_check.py --record-mask). A masked sector is BLIND, not free:
   see rule 6. Returns closer than min_range_m to the lidar are ignored too:
   the A2M12 cannot range below 0.2 m [DS], so anything that close is
   invisible, masked or not.

6. Unknown is not clear. Above creep speed the bubble looks at the rays that
   point into the region the command would sweep within its stop distance
   (the part not already under the robot). That region is UNKNOWN, and the
   command capped at unknown_speed_mps (creep), when
     - more than max_blind_fraction (25%) of those rays are masked: driving
       toward a blind sector, e.g. reversing with a front-mounted lidar whose
       rear view is the chassis; or
     - fewer than min_valid_fraction (20%) of the unmasked ones returned
       anything. A ray that returns nothing is open space beyond range, a
       black or shiny surface, or something closer than 0.2 m; in standard
       mode ~60% of all rays return nothing (measured). A dark chair in the
       path then slows the robot even though it shows up as a gap.
   Both are checked over the first near_view_m (10 cm) of the path and over
   the whole stop distance, so a long visible horizon cannot dilute a blind
   spot next to the robot. The 25% tolerance is for the wheel sectors of an
   under-chassis mount, which clip the corners of the forward view:
   straight driving passes, but spins and arcs do not, because the corners
   swing through the wheels' shadows (measured on the fake: 26-34% blind).
   That is a real blind spot, and creep is the honest answer to it;
   --bubble-max-blind raises the tolerance, 0 makes any overlap count.
   --bubble-min-valid 0 turns the dark-ray check off.

7. Floor returns (optional, off by default). A scan plane at height h above
   the floor meets the floor only when the chassis pitches, and then no
   closer than h / tan(pitch): h = 5 cm gives 0.95 m at 3 deg, 0.57 m at 5
   deg. The bubble reacts only to points within ~0.65 m of the base centre,
   so a low mount on a rigid skid-steer that pitches a degree or two when
   braking cannot trigger it. If it does (lidar_check.py shows an arc
   appear when the robot brakes), set floor_height_m: returns at or beyond
   h / tan(floor_pitch_deg) from the lidar then count only if the previous
   revolution saw something within 10 cm in the same direction. That delays
   a genuinely new obstacle in that band by one revolution, so one revolution
   is added to the reaction time. Nothing nearer than h / tan(floor_pitch_deg)
   is ever filtered: the floor cannot be there.

Everything here is pure: no clock, no I/O. The server passes `now`.
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, Sequence

from retriever.bridge.lidar import (
    DEFAULT_BAUD,
    N_SAMPLE_BINS,
    SAMPLE_BIN_DEG,
    TAU,
    FakeLidar,
    FakeTankPose,
    FakeWorld,
    LidarMount,
    LidarScan,
    RPLidar,
    default_motor,
    find_ports,
    sample_bin,
)
from retriever.bridge.protocol import BUBBLE_STATES

CLEAR, SLOWING, BLOCKED, STALE = BUBBLE_STATES
DEFAULT_REV_PERIOD_S = 0.15


# ---------------------------------------------------------------- geometry


@dataclass(frozen=True)
class Footprint:
    """The base as a rectangle in the base frame (x forward, y left, origin at
    the odometry centre). Include anything rigid that sticks out: bumpers,
    the arm in its stowed pose. MEASURE these; the defaults are placeholders
    for a ~0.5 x 0.4 m chassis."""

    front_m: float = 0.25
    rear_m: float = 0.25
    half_width_m: float = 0.20

    def __post_init__(self) -> None:
        if self.front_m <= 0 or self.rear_m <= 0 or self.half_width_m <= 0:
            raise ValueError("footprint dimensions must be positive")

    def clearance(self, x: float, y: float) -> float:
        """Distance from (x, y) to the rectangle; 0 inside."""
        dx = max(-self.rear_m - x, 0.0, x - self.front_m)
        dy = max(abs(y) - self.half_width_m, 0.0)
        return math.hypot(dx, dy)

    def corners(self, pad: float = 0.0) -> list[tuple[float, float]]:
        f, r, w = self.front_m + pad, self.rear_m + pad, self.half_width_m + pad
        return [(f, w), (f, -w), (-r, w), (-r, -w)]

    def radius(self, pad: float = 0.0) -> float:
        return max(math.hypot(x, y) for x, y in self.corners(pad))


@dataclass(frozen=True)
class BubbleConfig:
    footprint: Footprint = field(default_factory=Footprint)
    mount: LidarMount = field(default_factory=LidarMount)
    # Sensor-frame angle intervals (a0, a1), radians CCW from the lidar's 0
    # mark, in [0, 2*pi); a0 > a1 wraps through 0. See parse_mask().
    mask: tuple[tuple[float, float], ...] = ()
    latency_s: float = 0.10
    decel_mps2: float = 0.50
    stop_margin_m: float = 0.05
    body_margin_m: float = 0.02
    creep_speed_mps: float = 0.05
    creep_clearance_m: float = 0.03
    stale_after_s: float = 0.50
    min_range_m: float = 0.15
    step_m: float = 0.01
    # Rule 6: rays toward the swept region that must return something, and
    # how much of that view may be masked.
    min_valid_fraction: float = 0.20
    max_blind_fraction: float = 0.25
    near_view_m: float = 0.10
    unknown_speed_mps: float = 0.05
    unknown_min_rays: int = 6
    # Rule 7: lidar height above the floor (None = no floor filter).
    floor_height_m: float | None = None
    floor_pitch_deg: float = 3.0
    floor_match_m: float = 0.10
    # Worst case the creep rule is checked against: latency + 2 revolutions.
    worst_rev_period_s: float = DEFAULT_REV_PERIOD_S

    def __post_init__(self) -> None:
        if self.decel_mps2 <= 0 or self.creep_speed_mps < 0 or self.step_m <= 0:
            raise ValueError("decel and step must be positive, creep speed >= 0")
        t = self.latency_s + 2 * self.worst_rev_period_s
        v = self.creep_speed_mps
        creep_stop = v * t + v * v / (2 * self.decel_mps2)
        if self.creep_clearance_m < creep_stop:
            raise ValueError(
                f"creep_clearance_m {self.creep_clearance_m:.3f} m is less than the "
                f"distance a creep needs to stop ({creep_stop:.3f} m): the robot "
                "could touch what it creeps up to"
            )


@dataclass(frozen=True)
class BubbleDecision:
    vx: float
    wz: float
    state: str                  # clear | slowing | blocked | stale
    reason: str
    nearest_m: float | None     # clearance from the body to the nearest obstacle point
    stop_m: float               # clearance from the body this command needs
    scale: float                # vx, wz = commanded * scale
    free_m: float = math.inf    # free path length along the command, past body_margin


# ---------------------------------------------------------------- masks


def in_mask(angle: float, mask: Sequence[tuple[float, float]]) -> bool:
    a = angle % TAU
    for a0, a1 in mask:
        if (a0 <= a <= a1) if a0 <= a1 else (a >= a0 or a <= a1):
            return True
    return False


def parse_mask(text: str | None) -> tuple[tuple[float, float], ...]:
    """'100:140,350:10' (degrees, lidar frame, CCW from the 0 mark) -> radians.
    350:10 wraps through 0. The format lidar_check.py --record-mask prints."""
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
            raise ValueError(
                f"bad mask interval {part!r}; expected START:END in degrees") from None
        out.append((math.radians(a) % TAU, math.radians(b) % TAU))
    return tuple(out)


def format_mask(intervals_deg: Iterable[tuple[float, float]]) -> str:
    return ",".join(f"{a:.0f}:{b:.0f}" for a, b in intervals_deg)


def propose_mask(
    scans: Sequence[LidarScan],
    *,
    near_m: float = 0.5,
    min_fraction: float = 0.2,
    bin_deg: float = 1.0,
    pad_deg: float = 2.0,
    max_gap_deg: float = 30.0,
) -> list[tuple[float, float]]:
    """Self-mask from scans recorded with NOTHING within near_m of the lidar
    except the robot itself (and the arm moved through its carry poses).

    A 1-degree bin is the robot if it returned something nearer than near_m in
    at least min_fraction of the scans. A part closer than the lidar's 0.2 m
    minimum range returns NOTHING in its middle and something only at its
    edges, so a short gap (<= max_gap_deg) between two flagged runs is flagged
    too when its rays came back empty in most scans; a gap that returned
    farther things is a real window and stays open. (A wheel 23 cm from the
    lidar is dark across ~23 deg.) Record near walls 1-3 m away: rays into
    open space also come back empty, and would be masked, which only ever
    errs toward blind. Then every run is widened
    by pad_deg and merged. Returns (start_deg, end_deg) intervals in the
    sensor frame, CCW, one per part (four wheels -> four intervals); start >
    end wraps through 0. Empty if nothing was flagged.
    """
    nb = int(round(360.0 / bin_deg))
    if not scans:
        return []
    near_hits, any_hits = [0] * nb, [0] * nb
    for scan in scans:
        near_seen, any_seen = set(), set()
        for a, r, _ in scan.points:
            i = int(math.degrees(a) / bin_deg) % nb
            any_seen.add(i)
            if r < near_m:
                near_seen.add(i)
        for i in near_seen:
            near_hits[i] += 1
        for i in any_seen:
            any_hits[i] += 1
    need = max(1, math.ceil(min_fraction * len(scans)))
    flagged = [h >= need for h in near_hits]
    if not any(flagged):
        return []
    # Fill short, dark gaps between flagged runs (the unrangeable middle of a part).
    max_gap = int(max_gap_deg / bin_deg)
    i0 = flagged.index(True)
    j = 1
    while j <= nb:
        i = (i0 + j) % nb
        if flagged[i]:
            j += 1
            continue
        k = j
        while k <= nb and not flagged[(i0 + k) % nb]:
            k += 1
        gap = [(i0 + g) % nb for g in range(j, k)]
        if len(gap) <= max_gap and all(any_hits[g] < need for g in gap):
            for g in gap:
                flagged[g] = True
        j = k
    pad = int(math.ceil(pad_deg / bin_deg))
    wide = [any(flagged[(i + k) % nb] for k in range(-pad, pad + 1)) for i in range(nb)]
    if all(wide):
        return [(0.0, 360.0)]
    start = wide.index(False)
    runs: list[tuple[float, float]] = []
    run_from: int | None = None
    for j in range(1, nb + 1):
        i = (start + j) % nb
        if wide[i] and run_from is None:
            run_from = i
        elif not wide[i] and run_from is not None:
            last = (i - 1) % nb
            runs.append(((run_from * bin_deg) % 360.0, ((last + 1) * bin_deg) % 360.0))
            run_from = None
    return runs


# ---------------------------------------------------------------- the bubble


class SafetyBubble:
    def __init__(self, config: BubbleConfig | None = None) -> None:
        self.config = config or BubbleConfig()
        self._cached_scan: LidarScan | None = None
        self._cached_pts: list[tuple[float, float, float]] = []
        self._prev_bins: dict[int, list[float]] | None = None
        self.floor_dropped = 0

    @property
    def floor_range_m(self) -> float | None:
        """Nearest range from the lidar at which the floor can appear (rule 7)."""
        c = self.config
        if c.floor_height_m is None:
            return None
        return c.floor_height_m / math.tan(math.radians(c.floor_pitch_deg))

    # -- points -----------------------------------------------------------

    def obstacles(self, scan: LidarScan) -> list[tuple[float, float, float]]:
        """(x, y, clearance) in the base frame: valid, unmasked, far enough from
        the lidar to be trusted, and outside the robot's own footprint.
        Cached per scan: the server asks at 50 Hz, scans change at ~8 Hz."""
        if scan is self._cached_scan:
            return self._cached_pts
        c = self.config
        fp, mount = c.footprint, c.mount
        floor_r = self.floor_range_m
        prev, bins = self._prev_bins, {}
        pts = []
        for a, r, q in scan.points:
            if r < c.min_range_m or q <= 0 or in_mask(a, c.mask):
                continue
            b = sample_bin(a)
            bins.setdefault(b, []).append(r)
            if floor_r is not None and r >= floor_r and prev is not None and not any(
                    abs(r - r0) <= c.floor_match_m
                    for d in (-1, 0, 1) for r0 in prev.get((b + d) % N_SAMPLE_BINS, ())):
                self.floor_dropped += 1     # could be floor and nothing confirms it
                continue
            x, y = mount.to_base(a, r)
            clear = fp.clearance(x, y)
            if clear <= 0.0:
                continue                    # inside the chassis: that is the robot
            pts.append((x, y, clear))
        if floor_r is not None:
            self._prev_bins = bins
        self._cached_scan, self._cached_pts = scan, pts
        return pts

    # -- the rules --------------------------------------------------------

    def footprint_speed(self, vx: float, wz: float) -> float:
        """Speed of the fastest corner of the inflated footprint."""
        pad = self.config.body_margin_m
        return max(math.hypot(vx - wz * cy, wz * cx)
                   for cx, cy in self.config.footprint.corners(pad))

    def reaction_s(self, scan: LidarScan | None, now: float) -> float:
        c = self.config
        if scan is None:
            return c.latency_s + 2 * DEFAULT_REV_PERIOD_S
        period = 1.0 / scan.rev_hz if scan.rev_hz else DEFAULT_REV_PERIOD_S
        revs = 2 if c.floor_height_m is not None else 1     # rule 7 confirms over two
        return c.latency_s + max(0.0, now - scan.t) + revs * period

    def need_m(self, u: float, reaction_s: float) -> float:
        """Free path (past body_margin) needed to move at u and still stop."""
        c = self.config
        if u <= c.creep_speed_mps:
            return c.creep_clearance_m
        return u * reaction_s + u * u / (2.0 * c.decel_mps2) + c.stop_margin_m

    def allowed_speed(self, free_m: float, reaction_s: float) -> float:
        """The inverse of need_m: the fastest corner speed that can still stop
        within free_m. Never below creep unless free_m is within creep clearance."""
        c = self.config
        if math.isinf(free_m):
            return math.inf
        if free_m <= c.creep_clearance_m:
            return 0.0
        d = free_m - c.stop_margin_m
        v = 0.0
        if d > 0.0:
            a, t = c.decel_mps2, reaction_s
            v = a * (-t + math.sqrt(t * t + 2.0 * d / a))
        return max(v, c.creep_speed_mps)

    def free_path(self, pts: Sequence[tuple[float, float, float]], vx: float, wz: float,
                  horizon_m: float) -> float:
        """How far (fastest-corner path length) the inflated footprint moves
        along the command before touching a point; inf if not within horizon_m.

        A point already inside the margin band (closer than body_margin_m) gets
        a thinner margin of its own, so moving AWAY from it is never blocked.
        """
        c = self.config
        fp = c.footprint
        u = self.footprint_speed(vx, wz)
        if u <= 0.0 or not pts:
            return math.inf
        n = max(1, math.ceil(horizon_m / c.step_m))
        ds = horizon_m / n
        dt = ds / u
        reach = fp.radius(c.body_margin_m) + horizon_m + c.step_m
        cands = [(x, y, min(c.body_margin_m, max(0.0, clear - 0.002)))
                 for x, y, clear in pts if math.hypot(x, y) <= reach]
        if not cands:
            return math.inf
        front, rear, hw = fp.front_m, fp.rear_m, fp.half_width_m
        turning = abs(wz) > 1e-9
        radius = vx / wz if turning else 0.0
        for k in range(1, n + 1):
            t = k * dt
            if turning:
                th = wz * t
                px, py = radius * math.sin(th), radius * (1.0 - math.cos(th))
            else:
                th, px, py = 0.0, vx * t, 0.0
            cs, sn = math.cos(th), math.sin(th)
            for x, y, e in cands:
                dx, dy = x - px, y - py
                xr = cs * dx + sn * dy
                if xr > front + e or xr < -rear - e:
                    continue
                if abs(-sn * dx + cs * dy) <= hw + e:
                    return (k - 1) * ds
        return math.inf

    def _outline(self, spacing: float = 0.05) -> list[tuple[float, float]]:
        """Points every ~spacing m around the inflated footprint's edge."""
        pad = self.config.body_margin_m
        (fx, fy), _, (rx, _), _ = self.config.footprint.corners(pad)
        out = []
        for (x0, y0), (x1, y1) in (((fx, -fy), (fx, fy)), ((fx, fy), (rx, fy)),
                                   ((rx, fy), (rx, -fy)), ((rx, -fy), (fx, -fy))):
            n = max(1, math.ceil(math.hypot(x1 - x0, y1 - y0) / spacing))
            out += [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n)]
        return out

    def swept_bins(self, vx: float, wz: float, horizon_m: float) -> list[int]:
        return self.swept_views(vx, wz, horizon_m, horizon_m)[1]

    def swept_views(self, vx: float, wz: float, near_m: float,
                    horizon_m: float) -> tuple[list[int], list[int]]:
        """swept_bins() for the first near_m of the path and for all of
        horizon_m, in one pass: (near bins, all bins)."""
        """Sensor sample bins whose rays point into the region the command
        sweeps within horizon_m, leaving out what is already under the robot.

        The inflated outline is moved along the path in 2 cm steps; every edge
        of it that pokes outside today's footprint marks the bins between its
        two ends as seen from the lidar. A spin therefore marks only the four
        corner lobes, not the whole circle.
        """
        c = self.config
        fp, mount, pad = c.footprint, c.mount, c.body_margin_m
        u = self.footprint_speed(vx, wz)
        if u <= 0.0:
            return [], []
        n = max(1, math.ceil(horizon_m / 0.02))
        dt = horizon_m / n / u
        turning = abs(wz) > 1e-9
        radius = vx / wz if turning else 0.0
        outline = self._outline(0.02)
        near: list[int] | None = None
        # "Outside" = outside the inflated RECTANGLE the robot occupies now
        # (its corners are farther than pad from the chassis, so a clearance
        # test would count them as outside).
        fx, rx = fp.front_m + pad + 1e-6, -fp.rear_m - pad - 1e-6
        hy = fp.half_width_m + pad + 1e-6
        bins: set[int] = set()
        for k in range(1, n + 1):
            t = k * dt
            if turning:
                th = wz * t
                px, py = radius * math.sin(th), radius * (1.0 - math.cos(th))
            else:
                th, px, py = 0.0, vx * t, 0.0
            cs, sn = math.cos(th), math.sin(th)
            prev = None
            for ox, oy in outline + outline[:1]:
                x, y = px + cs * ox - sn * oy, py + sn * ox + cs * oy
                here = (mount.sensor_angle_of(math.atan2(y - mount.y_m, x - mount.x_m)),
                        x > fx or x < rx or abs(y) > hy)
                if prev is not None and (prev[1] or here[1]):
                    _mark_arc(bins, prev[0], here[0])
                prev = here
            if near is None and k * horizon_m / n >= near_m - 1e-9:
                near = sorted(bins)
        return (near if near is not None else sorted(bins)), sorted(bins)

    def visibility(self, scan: LidarScan, bins: Sequence[int]) -> tuple[int, int, float]:
        """Over the given sensor bins: (rays that returned something usable,
        unmasked rays, fraction of the bins that are masked)."""
        c = self.config
        if not bins:
            return 0, 0, 0.0
        step = math.radians(SAMPLE_BIN_DEG)
        live = {b for b in bins if not in_mask((b + 0.5) * step, c.mask)}
        blind = 1.0 - len(live) / len(bins)
        if not scan.samples:
            return 0, 0, blind
        total = sum(scan.samples[b] for b in live)
        valid = sum(1 for a, r, q in scan.points
                    if q > 0 and r >= c.min_range_m and sample_bin(a) in live)
        return valid, total, blind

    def unknown(self, scan: LidarScan, vx: float, wz: float, need_m: float) -> str:
        """Rule 6: why the region this command sweeps counts as unseen, or "".

        Checked twice: over the first near_view_m of the path, and over the
        whole stop distance. The near check keeps a long, mostly visible
        horizon from diluting a blind spot right next to the robot (the
        corners of a spin passing behind the wheels); without it a FAST spin
        passed where a slow one did not.
        """
        c = self.config
        where = _direction(vx, wz)
        horizon = need_m + c.body_margin_m
        views = self.swept_views(vx, wz, min(c.near_view_m, horizon), horizon)
        for bins in views:
            valid, total, blind = self.visibility(scan, bins)
            if blind > c.max_blind_fraction:
                return f"driving toward a masked (blind) sector {where}: creep only"
            if not scan.samples or c.min_valid_fraction <= 0.0:
                continue
            if total < c.unknown_min_rays:
                return f"too few lidar rays {where} ({total}): creep only"
            if valid < c.min_valid_fraction * total:
                return f"can't see {where} ({valid} of {total} rays returned): creep only"
        return ""

    def check(self, scan: LidarScan | None, vx: float, wz: float, now: float) -> BubbleDecision:
        """The command the base may have, and why."""
        c = self.config
        age = math.inf if scan is None else max(0.0, now - scan.t)
        stale = age > c.stale_after_s
        pts = self.obstacles(scan) if scan is not None else []
        nearest = min((p[2] for p in pts), default=None) if not stale else None
        react = self.reaction_s(scan, now)
        u = self.footprint_speed(vx, wz)
        stop_m = self.need_m(u, react) + c.body_margin_m

        k, free = 1.0, math.inf
        if u > 0.0 and pts:
            need = self.need_m(u, react)
            free = self.free_path(pts, vx, wz, need + 2 * c.step_m)
            allowed = self.allowed_speed(free, react)
            k = min(1.0, allowed / u) if not math.isinf(allowed) else 1.0
        where = _direction(vx, wz)

        if stale:
            if u > c.creep_speed_mps:
                k = min(k, c.creep_speed_mps / u)
            what = "no lidar scan yet" if scan is None else f"lidar scan {age:.1f} s old"
            reason = f"{what}: creep only ({c.creep_speed_mps * 100:.0f} cm/s)"
            if u > 0.0 and k <= 0.0:
                reason += f"; last scan shows an obstacle {where}"
            return BubbleDecision(vx * k, wz * k, STALE, reason, None, stop_m, k, free)

        blind = ""
        cap = c.unknown_speed_mps
        if scan is not None and u * k > cap:
            blind = self.unknown(scan, vx, wz, self.need_m(u, react))
            if blind:
                k = min(k, cap / u)

        if u <= 0.0 or k >= 1.0:
            return BubbleDecision(vx, wz, CLEAR, "", nearest, stop_m, 1.0, free)
        if blind and (math.isinf(free) or self.allowed_speed(free, react) >= cap):
            return BubbleDecision(vx * k, wz * k, SLOWING, blind, nearest, stop_m, k, free)
        dist = free + c.body_margin_m
        if k <= 0.0:
            return BubbleDecision(0.0, 0.0, BLOCKED,
                                  f"blocked: obstacle {dist * 100:.0f} cm {where}",
                                  nearest, stop_m, 0.0, free)
        return BubbleDecision(vx * k, wz * k, SLOWING,
                              f"slowing to {u * k:.2f} m/s: obstacle {dist:.2f} m {where}",
                              nearest, stop_m, k, free)

    # -- for the wire -----------------------------------------------------

    def wire_scan(self, scan: LidarScan, stop_m: float,
                  step_deg: float = 2.0) -> tuple[list[int], list[int]]:
        """Downsample to base-frame bearing bins: (ranges_cm, near).

        Bin i covers bearings [i*step, (i+1)*step) degrees CCW from the base's
        forward axis, measured from the base origin; it holds the nearest point
        in cm, 0 for none. `near` lists bins with a point closer to the body
        than stop_m. Masked and self points are left out.
        """
        n = int(round(360.0 / step_deg))
        step = TAU / n
        ranges = [0] * n
        near = [False] * n
        for x, y, clear in self.obstacles(scan):
            i = int((math.atan2(y, x) % TAU) / step) % n
            cm = max(1, min(65535, int(round(math.hypot(x, y) * 100.0))))
            if ranges[i] == 0 or cm < ranges[i]:
                ranges[i] = cm
            if clear < stop_m:
                near[i] = True
        return ranges, [i for i, f in enumerate(near) if f]


def _mark_arc(bins: set[int], a0: float, a1: float) -> None:
    """Add the sample bins along the short arc from a0 to a1 (radians)."""
    b, last = sample_bin(a0), sample_bin(a1)
    d = (a1 - a0 + math.pi) % TAU - math.pi
    step = 1 if d >= 0 else -1
    bins.add(b)
    while b != last:
        b = (b + step) % N_SAMPLE_BINS
        bins.add(b)


def _direction(vx: float, wz: float) -> str:
    turn = "left" if wz > 0 else "right"
    if abs(wz) <= 1e-9:
        return "ahead" if vx > 0 else "behind"
    if abs(vx) <= 1e-9:
        return f"turning {turn}"
    return f"{'ahead' if vx > 0 else 'behind'}, turning {turn}"


# ---------------------------------------------------------------- command line


def add_lidar_args(ap: argparse.ArgumentParser) -> None:
    """The bridge's lidar + bubble flags. scripts/fake_pi.py calls this."""
    g = ap.add_argument_group(
        "lidar safety bubble (RPLIDAR A2M12; off unless a lidar is given)")
    g.add_argument("--lidar-port", default=None,
                   help="lidar serial port: /dev/ttyAMA0 for the Pi 5 header UART (GPIO14/15), "
                        "/dev/ttyUSB0 for Slamtec's USB adapter, 'auto' to search")
    g.add_argument("--fake-lidar", action="store_true",
                   help="simulate a lidar in a walled room around the fake tank")
    g.add_argument("--lidar-baud", type=int, default=DEFAULT_BAUD, help="A2M12: 256000")
    g.add_argument("--lidar-motor-pin", default=None,
                   help="BCM GPIO wired to MOTOCTL (18 in our build); "
                        "omit if MOTOCTL is tied high")
    g.add_argument("--lidar-motor", choices=["gpio", "adapter", "external"], default=None,
                   help="motor control; default gpio with a pin, adapter for USB ports, "
                        "else external")
    g.add_argument("--lidar-x", type=float, default=0.0,
                   help="lidar position, m forward of the base centre")
    g.add_argument("--lidar-y", type=float, default=0.0,
                   help="lidar position, m left of the base centre")
    g.add_argument("--lidar-yaw-deg", type=float, default=0.0,
                   help="bearing of the lidar's 0 mark (away from its cable), "
                        "deg CCW from forward")
    g.add_argument("--lidar-inverted", "--lidar-upside-down", dest="lidar_inverted",
                   action="store_true", help="lidar mounted upside down (mirrors the sweep)")
    g.add_argument("--lidar-mask", default="",
                   help="blind sectors, lidar-frame degrees CCW, e.g. '100:140,350:10' "
                        "(scripts/lidar_check.py --record-mask prints this)")
    g.add_argument("--footprint", default="0.25,0.25,0.20",
                   help="FRONT,REAR,HALF_WIDTH in metres from the base centre (measure it)")
    g.add_argument("--bubble-decel", type=float, default=BubbleConfig.decel_mps2,
                   help="braking deceleration the stop distance assumes, m/s^2")
    g.add_argument("--bubble-latency", type=float, default=BubbleConfig.latency_s,
                   help="control + motor latency, s (scan age is added on top)")
    g.add_argument("--bubble-creep", type=float, default=BubbleConfig.creep_speed_mps,
                   help="speed below which the robot may close to "
                        "--bubble-creep-clearance, m/s")
    g.add_argument("--bubble-creep-clearance", type=float,
                   default=BubbleConfig.creep_clearance_m)
    g.add_argument("--bubble-stale", type=float, default=BubbleConfig.stale_after_s,
                   help="scan age after which only creep is allowed, s")
    g.add_argument("--bubble-min-valid", type=float, default=BubbleConfig.min_valid_fraction,
                   help="fraction of rays toward the path that must return something, "
                        "else only creep (0 disables)")
    g.add_argument("--bubble-max-blind", type=float, default=BubbleConfig.max_blind_fraction,
                   help="fraction of the view toward the path that may be masked before "
                        "only creep is allowed (0: any masked ray)")
    g.add_argument("--lidar-height", type=float, default=None,
                   help="scan plane height above the floor, m; turns on the floor-return "
                        "filter (safety.py rule 7). Leave unset unless floor arcs appear")
    g.add_argument("--floor-pitch-deg", type=float, default=BubbleConfig.floor_pitch_deg,
                   help="largest chassis pitch the floor filter allows for")
    g.add_argument("--scan-hz", type=float, default=5.0,
                   help="rate of the downsampled scan sent to subscribed laptops")


def parse_footprint(text: str) -> Footprint:
    try:
        front, rear, half = (float(v) for v in text.split(","))
    except ValueError:
        raise ValueError(f"--footprint wants FRONT,REAR,HALF_WIDTH, got {text!r}") from None
    return Footprint(front, rear, half)


def bubble_config_from_args(args: argparse.Namespace) -> BubbleConfig:
    return BubbleConfig(
        footprint=parse_footprint(args.footprint),
        mount=LidarMount(args.lidar_x, args.lidar_y, math.radians(args.lidar_yaw_deg),
                         args.lidar_inverted),
        mask=parse_mask(args.lidar_mask),
        latency_s=args.bubble_latency,
        decel_mps2=args.bubble_decel,
        creep_speed_mps=args.bubble_creep,
        creep_clearance_m=args.bubble_creep_clearance,
        stale_after_s=args.bubble_stale,
        min_valid_fraction=args.bubble_min_valid,
        max_blind_fraction=args.bubble_max_blind,
        floor_height_m=args.lidar_height,
        floor_pitch_deg=args.floor_pitch_deg,
    )


def lidar_from_args(
    args: argparse.Namespace,
    driver: Any = None,
    clock: Callable[[], float] = time.monotonic,
) -> tuple[Any, SafetyBubble | None]:
    """(lidar, bubble), or (None, None) when no lidar was asked for. A real
    lidar is started here (its own thread); BridgeServer.close() closes it.
    The motor GPIO is claimed here too, so a wrong pin fails at start-up.
    Raises ValueError / ImportError on a bad configuration: exit, don't guess.
    A lidar that is configured but unplugged is NOT an error: it retries, and
    the bubble holds the base to creep and says why until scans arrive."""
    if not args.lidar_port and not args.fake_lidar:
        return None, None
    config = bubble_config_from_args(args)
    bubble = SafetyBubble(config)
    if args.fake_lidar:
        tank = driver if hasattr(driver, "wheel_travel_m") else getattr(driver, "driver", None)
        if tank is None or not hasattr(tank, "wheel_travel_m"):   # VacuumOverlay wraps one
            raise ValueError(
                "--fake-lidar needs the fake tank driver (it rides on its encoders)")
        return FakeLidar(FakeWorld.room(posts=[(1.2, 0.0, 0.15)]), pose_fn=FakeTankPose(tank),
                         mount=config.mount, clock=clock), bubble
    try:
        import serial  # noqa: F401  (fail now, not as a lidar that silently never starts)
    except ImportError:
        raise ValueError("--lidar-port needs pyserial: sudo apt install python3-serial, "
                         "or pip install pyserial into the bridge's python") from None
    port = args.lidar_port
    if port == "auto":
        ports = find_ports()
        if not ports:
            raise ValueError("--lidar-port auto: no serial port found")
        port = ports[0]
    pin = args.lidar_motor_pin
    pin = int(pin) if isinstance(pin, str) and pin.isdigit() else pin
    motor = default_motor(port, pin, args.lidar_motor)
    return RPLidar(port, args.lidar_baud, motor=motor, clock=clock).start(), bubble


__all__ = [
    "BLOCKED", "CLEAR", "SLOWING", "STALE", "BubbleConfig", "BubbleDecision", "Footprint",
    "SafetyBubble", "add_lidar_args", "bubble_config_from_args", "format_mask", "in_mask",
    "lidar_from_args", "parse_footprint", "parse_mask", "propose_mask",
]
