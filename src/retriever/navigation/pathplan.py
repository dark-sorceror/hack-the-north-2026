"""A route across the map: costmap, A*, and a smoothed path.

WHY A CHAIR IS ONE THING. The planner treats the robot as a disc of its
circumscribed radius (a tank turns on the spot, sweeping its corners) plus a
margin, and no path may bring that disc's centre closer than `lethal_m` to an
occupied cell. A chair's legs are ~0.45 m apart; each one blocks a 0.38 m
radius round it, so the four close into one solid obstacle and the robot is
never routed between them (it would not fit under the seat with an arm on top
anyway). A gap narrower than 2 * lethal_m (~0.76 m) is closed the same way.

COST, NOT JUST WALLS. Beyond lethal_m the cost falls from 1 to 0 across
cost_band_m, and every step pays for it, so the route keeps to the middle of
open space when there is room and only shaves past things when it must. Cells
the lidar has never seen are allowed (the goal is often round a corner) at a
small extra cost per metre; the follower slows for them.

ESCAPING. If the robot is already inside the lethal zone (it drove there by
hand, or something moved up to it), cells near the start are passable at a
high price, so the plan leads OUT instead of reporting no path.

A GOAL INSIDE SOMETHING (a click on the chair itself) is moved to the nearest
cell the robot can stand in, within goal_snap_m, and the result says so.

Plain Python A* on a 10 cm grid (the map is 5 cm): a room-sized plan expands a
few thousand cells, a few tens of milliseconds.
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, field

import numpy as np

from retriever.navigation.grid import OccupancyGrid, distance_field


@dataclass(frozen=True)
class PlannerConfig:
    robot_radius_m: float = 0.32     # circumscribed: hypot(0.25, 0.20) for the default footprint
    margin_m: float = 0.06           # past the body: tape-measure error, odometry smear
    cost_band_m: float = 0.40        # beyond lethal, cost falls 1 -> 0 over this
    cost_weight: float = 4.0         # extra per metre at cost 1
    unknown_weight: float = 0.5      # extra per metre through never-seen cells
    plan_resolution_m: float = 0.10
    escape_m: float = 0.45           # lethal cells this near the start are passable (at a price)
    goal_snap_m: float = 0.8         # a goal inside something moves this far at most
    max_expansions: int = 120_000

    @property
    def lethal_m(self) -> float:
        return self.robot_radius_m + self.margin_m

    @classmethod
    def for_footprint(cls, front_m: float, rear_m: float, half_width_m: float,
                      **kw) -> PlannerConfig:
        return cls(robot_radius_m=math.hypot(max(front_m, rear_m), half_width_m), **kw)


@dataclass
class Costmap:
    """Planning-resolution view of the map. Arrays are [iy, ix]."""

    res: float
    x0: float
    y0: float
    dist_m: np.ndarray            # to the nearest occupied cell, metres (capped)
    lethal: np.ndarray            # bool: the robot's centre may not be here
    cost: np.ndarray              # 0..1
    unknown: np.ndarray           # bool: never seen
    config: PlannerConfig
    fine_lethal: np.ndarray | None = None   # at map resolution, for drawing
    version: int = 0

    @property
    def shape(self) -> tuple[int, int]:
        return self.lethal.shape

    def cell(self, x: float, y: float) -> tuple[int, int]:
        return int(math.floor((x - self.x0) / self.res)), int(math.floor((y - self.y0) / self.res))

    def centre(self, ix: int, iy: int) -> tuple[float, float]:
        return self.x0 + (ix + 0.5) * self.res, self.y0 + (iy + 0.5) * self.res

    def inside(self, ix: int, iy: int) -> bool:
        rows, cols = self.shape
        return 0 <= ix < cols and 0 <= iy < rows

    def clearance(self, x: float, y: float) -> float:
        """Metres from (x, y) to the nearest occupied cell (capped, inf outside)."""
        ix, iy = self.cell(x, y)
        if not self.inside(ix, iy):
            return math.inf
        return float(self.dist_m[iy, ix])

    def is_lethal(self, x: float, y: float) -> bool:
        ix, iy = self.cell(x, y)
        return self.inside(ix, iy) and bool(self.lethal[iy, ix])


def build_costmap(grid: OccupancyGrid, config: PlannerConfig = PlannerConfig(),
                  extra_occupied: np.ndarray | None = None) -> Costmap:
    """extra_occupied: cells another sensor says are blocked (the camera layer:
    navigator.Mapper). They inflate exactly as the lidar's do, so a route round
    a camera-only obstacle keeps the same clearance as one round a chair. They
    are never 'seen free': a camera can only make the robot more careful."""
    occ = grid.occupied()
    if extra_occupied is not None:
        occ = occ | extra_occupied
    reach = config.lethal_m + config.cost_band_m
    max_cells = int(math.ceil(reach / grid.res)) + 2
    dist_fine = distance_field(occ, max_cells) * grid.res
    unknown_fine = ~(occ | grid.seen_free())
    fine_lethal = dist_fine < config.lethal_m

    f = max(1, int(round(config.plan_resolution_m / grid.res)))
    n = grid.n
    m = (n // f) * f                      # crop to a multiple of f (the edge is walls-far)
    if f > 1:
        dist = dist_fine[:m, :m].reshape(m // f, f, m // f, f).min(axis=(1, 3))
        unknown = unknown_fine[:m, :m].reshape(m // f, f, m // f, f).all(axis=(1, 3))
    else:
        dist, unknown = dist_fine, unknown_fine
    lethal = dist < config.lethal_m
    cost = np.clip(1.0 - (dist - config.lethal_m) / config.cost_band_m, 0.0, 1.0)
    cost[lethal] = 1.0
    return Costmap(res=grid.res * f, x0=grid.x0, y0=grid.y0, dist_m=dist, lethal=lethal,
                   cost=cost.astype(np.float32), unknown=unknown, config=config,
                   fine_lethal=fine_lethal, version=grid.version)


@dataclass
class PathResult:
    ok: bool
    path: list[tuple[float, float]] = field(default_factory=list)   # start ... goal, metres
    reason: str = ""
    goal: tuple[float, float] | None = None       # where it plans to end (moved if inside something)
    moved_goal: bool = False
    length_m: float = 0.0
    expansions: int = 0
    ms: float = 0.0


_NEIGHBOURS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
               (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
               (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]


def _snap_goal(cm: Costmap, gx: int, gy: int, max_m: float) -> tuple[int, int] | None:
    """The free (non-lethal) cell nearest to (gx, gy), within max_m."""
    r = int(math.ceil(max_m / cm.res))
    rows, cols = cm.shape
    y0, y1 = max(0, gy - r), min(rows, gy + r + 1)
    x0, x1 = max(0, gx - r), min(cols, gx + r + 1)
    win = ~cm.lethal[y0:y1, x0:x1]
    if not win.any():
        return None
    yy, xx = np.nonzero(win)
    d = (yy + y0 - gy) ** 2 + (xx + x0 - gx) ** 2
    k = int(np.argmin(d))
    if d[k] > r * r:
        return None
    return int(xx[k] + x0), int(yy[k] + y0)


def plan_path(cm: Costmap, start: tuple[float, float], goal: tuple[float, float]) -> PathResult:
    """A route from start to goal, both in metres in the map frame."""
    t0 = time.perf_counter()
    c = cm.config
    rows, cols = cm.shape
    sx, sy = cm.cell(*start)
    gx, gy = cm.cell(*goal)
    if not cm.inside(sx, sy):
        return PathResult(False, reason="I'm off the edge of my map")
    if not cm.inside(gx, gy):
        return PathResult(False, reason="that spot is off the edge of my map")

    goal_xy = (float(goal[0]), float(goal[1]))
    moved = False
    if cm.lethal[gy, gx]:
        snapped = _snap_goal(cm, gx, gy, c.goal_snap_m)
        if snapped is None:
            return PathResult(False, reason="that spot is inside something, with no room near it")
        gx, gy = snapped
        goal_xy = cm.centre(gx, gy)
        moved = True

    # Flat Python lists: scalar indexing into numpy is ~10x slower than a list.
    lethal = cm.lethal.ravel().tolist()
    step_w = (1.0 + c.cost_weight * cm.cost + c.unknown_weight * cm.unknown).ravel().tolist()
    esc = int(math.ceil(c.escape_m / cm.res))
    esc2 = esc * esc
    escape_price = 25.0

    start_i, goal_i = sy * cols + sx, gy * cols + gx
    if start_i == goal_i:
        return PathResult(True, [start, goal_xy], goal=goal_xy, moved_goal=moved,
                          length_m=math.dist(start, goal_xy), ms=(time.perf_counter() - t0) * 1e3)

    def h(ix: int, iy: int) -> float:
        dx, dy = abs(ix - gx), abs(iy - gy)
        return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

    g = {start_i: 0.0}
    came: dict[int, int] = {}
    heap = [(h(sx, sy), 0.0, start_i)]
    closed = set()
    expansions = 0
    found = False
    while heap:
        _, gc, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        if cur == goal_i:
            found = True
            break
        closed.add(cur)
        expansions += 1
        if expansions > c.max_expansions:
            break
        cy, cx = divmod(cur, cols)
        for dx, dy, dl in _NEIGHBOURS:
            nx, ny = cx + dx, cy + dy
            if nx < 0 or ny < 0 or nx >= cols or ny >= rows:
                continue
            ni = ny * cols + nx
            if ni in closed:
                continue
            w = step_w[ni]
            if lethal[ni]:
                if (nx - sx) ** 2 + (ny - sy) ** 2 > esc2:
                    continue
                w += escape_price             # a way out of a tight spot, never a shortcut
            ng = gc + dl * w
            if ng < g.get(ni, math.inf):
                g[ni] = ng
                came[ni] = cur
                heapq.heappush(heap, (ng + h(nx, ny), ng, ni))

    ms = (time.perf_counter() - t0) * 1e3
    if not found:
        why = ("I looked too long for a way there" if expansions > c.max_expansions
               else "there's no way through that I can fit")
        return PathResult(False, reason=why, expansions=expansions, ms=ms)

    cells = [goal_i]
    while cells[-1] != start_i:
        cells.append(came[cells[-1]])
    cells.reverse()
    pts = [cm.centre(i % cols, i // cols) for i in cells]
    pts[0], pts[-1] = (float(start[0]), float(start[1])), goal_xy
    path = smooth_path(cm, pts)
    length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
    return PathResult(True, path, goal=goal_xy, moved_goal=moved, length_m=length,
                      expansions=expansions, ms=(time.perf_counter() - t0) * 1e3)


def _segment_cost(cm: Costmap, a: tuple[float, float], b: tuple[float, float]) -> float:
    """The worst cost along a straight segment (inf through a lethal cell)."""
    n = max(2, int(math.dist(a, b) / (cm.res * 0.5)) + 1)
    xs = np.linspace(a[0], b[0], n)
    ys = np.linspace(a[1], b[1], n)
    ix = np.floor((xs - cm.x0) / cm.res).astype(np.int64)
    iy = np.floor((ys - cm.y0) / cm.res).astype(np.int64)
    rows, cols = cm.shape
    if ix.min() < 0 or iy.min() < 0 or ix.max() >= cols or iy.max() >= rows:
        return math.inf
    if cm.lethal[iy, ix][1:].any():          # the start may be in a tight spot
        return math.inf
    return float(cm.cost[iy, ix].max())


def smooth_path(cm: Costmap, pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """String-pull the A* staircase into a few straight legs. A shortcut is
    taken only if it is no closer to anything than the stretch of path it
    replaces, so smoothing never trades the planner's clearance for length."""
    if len(pts) <= 2:
        return list(pts)
    costs = [cm.cost[cm.cell(*p)[1], cm.cell(*p)[0]] if cm.inside(*cm.cell(*p)) else 1.0
             for p in pts]
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = i + 1
        worst = max(costs[i], costs[j])
        while j + 1 < len(pts):
            worst_next = max(worst, costs[j + 1])
            if _segment_cost(cm, pts[i], pts[j + 1]) > worst_next + 0.05:
                break
            j += 1
            worst = worst_next
        out.append(pts[j])
        i = j
    return out
