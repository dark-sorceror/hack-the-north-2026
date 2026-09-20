"""The room as the lidar has seen it: an occupancy grid, and its distance field.

LEVEL 3: A MAP. avoid.py only knows the scan in front of it, so it steers
round a chair and then wedges itself beside the legs with every way forward
blocked (seen in the sim). This remembers: every return marks a cell, every
ray that passed through a cell clears it a little, and what the robot saw
three metres back is still there when it turns round. pathplan.py plans over
it.

The frame is the odometry frame (where the robot started, x = its first
heading). No scan matching yet, so the map is exactly as good as the odometry:
fine for a room, smeared by every slipped turn until the gyro lands.

LOG-ODDS, ASYMMETRIC. A chair leg is 2-3 cm thick in a 5 cm cell, and most
rays that pass through its cell miss it. With equal weights the misses would
erase it; so a hit counts three times as much as a miss, and in any one scan a
cell that took a return is never also cleared.

Only rays with a return clear anything. No return means nothing came back:
open space, or a black or chrome thing that swallows the beam. Unknown is not
free, here as in avoid.py.

numpy only (the laptop side; the Pi never imports this).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np

from retriever.types import Pose

UNKNOWN, FREE, OCCUPIED = 0, 1, 2


@dataclass(frozen=True)
class GridConfig:
    resolution_m: float = 0.05
    size_m: float = 16.0          # square, centred on the odometry origin
    hit: float = 0.9              # log-odds a return adds to its cell
    miss: float = -0.3            # ...and a ray takes from each cell it crosses
    lo: float = -2.0              # clamps: how sure it can get, so it can change its mind
    hi: float = 3.5
    occupied: float = 0.7         # above this: occupied (p > 0.67): one hit, until a ray clears it
    free: float = -0.6            # below this: seen free
    max_range_m: float = 6.0      # returns farther than this neither mark nor clear
    clear_stop_m: float = 0.10    # a ray clears up to this short of its return


class OccupancyGrid:
    """cells[iy, ix]; cell (ix, iy) covers x in [x0 + ix*res, x0 + (ix+1)*res)."""

    def __init__(self, config: GridConfig = GridConfig()) -> None:
        self.config = config
        self.res = config.resolution_m
        self.n = int(round(config.size_m / self.res))
        self.x0 = self.y0 = -self.n * self.res / 2.0
        self.logodds = np.zeros((self.n, self.n), np.float32)
        self.version = 0          # bumps on every insert: caches key on it

    # -- coordinates ------------------------------------------------------

    def cell(self, x: float | np.ndarray, y: float | np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        ix = np.floor((np.asarray(x) - self.x0) / self.res).astype(np.int64)
        iy = np.floor((np.asarray(y) - self.y0) / self.res).astype(np.int64)
        return ix, iy

    def centre(self, ix: int | np.ndarray, iy: int | np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return (self.x0 + (np.asarray(ix) + 0.5) * self.res,
                self.y0 + (np.asarray(iy) + 0.5) * self.res)

    def inside(self, ix: np.ndarray, iy: np.ndarray) -> np.ndarray:
        return (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)

    # -- updates ----------------------------------------------------------

    def insert(self, sensor: tuple[float, float], hits: np.ndarray | Sequence[Sequence[float]]) -> int:
        """One scan: `sensor` is where the rays start, `hits` the returns (N, 2),
        both in the map frame. Returns how many returns were used."""
        c = self.config
        h = np.asarray(hits, dtype=np.float64).reshape(-1, 2)
        if len(h) == 0:
            return 0
        sx, sy = float(sensor[0]), float(sensor[1])
        dx, dy = h[:, 0] - sx, h[:, 1] - sy
        d = np.hypot(dx, dy)
        keep = (d > 1e-3) & (d <= c.max_range_m)
        if not keep.any():
            return 0
        h, dx, dy, d = h[keep], dx[keep], dy[keep], d[keep]

        # Every ray, sampled every half cell from the sensor to clear_stop_m short
        # of its return: one flat array of sample points, no Python loop.
        step = self.res * 0.5
        counts = np.maximum(((d - c.clear_stop_m) / step).astype(np.int64), 0)
        total = int(counts.sum())
        miss_flat = np.empty(0, np.int64)
        if total:
            ray = np.repeat(np.arange(len(d)), counts)
            start = np.repeat(np.cumsum(counts) - counts, counts)
            t = (np.arange(total) - start) * step
            px = sx + dx[ray] / d[ray] * t
            py = sy + dy[ray] / d[ray] * t
            mix, miy = self.cell(px, py)
            ok = self.inside(mix, miy)
            miss_flat = np.unique(miy[ok] * self.n + mix[ok])

        hix, hiy = self.cell(h[:, 0], h[:, 1])
        ok = self.inside(hix, hiy)
        hit_flat = np.unique(hiy[ok] * self.n + hix[ok])
        # a cell that took a return this scan is not also cleared by it
        miss_flat = np.setdiff1d(miss_flat, hit_flat, assume_unique=True)

        flat = self.logodds.reshape(-1)
        flat[miss_flat] += c.miss
        flat[hit_flat] += c.hit
        np.clip(self.logodds, c.lo, c.hi, out=self.logodds)
        self.version += 1
        return int(len(d))

    def insert_scan(self, pose: Pose, points_base: Iterable[Sequence[float]],
                    sensor_offset: tuple[float, float] = (0.0, 0.0)) -> int:
        """A scan in the BASE frame (x forward, y left: the bridge's ScanView
        points), taken at `pose`. The rays start at the lidar: sensor_offset is
        where it sits on the base (0, 0 = the centre)."""
        pts = np.asarray([p[:2] for p in points_base], dtype=np.float64).reshape(-1, 2)
        c, s = math.cos(pose.theta), math.sin(pose.theta)
        rot = np.array([[c, -s], [s, c]])
        world = pts @ rot.T + np.array([pose.x, pose.y])
        ox, oy = sensor_offset
        sensor = (pose.x + c * ox - s * oy, pose.y + s * ox + c * oy)
        return self.insert(sensor, world)

    def clear(self) -> None:
        self.logodds[:] = 0.0
        self.version += 1

    # -- reading ----------------------------------------------------------

    def occupied(self) -> np.ndarray:
        return self.logodds > self.config.occupied

    def seen_free(self) -> np.ndarray:
        return self.logodds < self.config.free

    def states(self) -> np.ndarray:
        """uint8 per cell: UNKNOWN, FREE or OCCUPIED."""
        out = np.zeros(self.logodds.shape, np.uint8)
        out[self.seen_free()] = FREE
        out[self.occupied()] = OCCUPIED
        return out

    def is_occupied(self, x: float, y: float) -> bool:
        ix, iy = self.cell(x, y)
        return bool(self.inside(ix, iy) and self.logodds[iy, ix] > self.config.occupied)


def distance_field(occupied: np.ndarray, max_cells: int) -> np.ndarray:
    """Euclidean distance, in cells, from every cell to the nearest occupied
    one; exact up to max_cells and max_cells + 1 beyond (nothing needs more).

    Separable: first the distance straight up or down each column to the
    nearest occupied cell (two running max/min accumulations, no loop), then
    d^2(x) = min over |k| <= max_cells of k^2 + g(x + k)^2 along each row: one
    vectorised pass per k. About 2 * max_cells array ops; no scipy, no OpenCV.
    """
    rows, cols = occupied.shape
    cap = float(max_cells + 1)
    big = 10 ** 6
    idx = np.arange(rows, dtype=np.int64)[:, None]
    above = np.maximum.accumulate(np.where(occupied, idx, -big), axis=0)
    below = np.minimum.accumulate(np.where(occupied, idx, big)[::-1], axis=0)[::-1]
    g = np.minimum(np.minimum(idx - above, below - idx), cap).astype(np.float32)
    g2 = g * g
    d2 = g2.copy()
    for k in range(1, max_cells + 1):
        k2 = float(k * k)
        if k >= cols:
            break
        np.minimum(d2[:, k:], g2[:, :-k] + k2, out=d2[:, k:])
        np.minimum(d2[:, :-k], g2[:, k:] + k2, out=d2[:, :-k])
    return np.minimum(np.sqrt(d2), cap)
