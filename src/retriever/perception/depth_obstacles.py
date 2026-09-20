"""Depth from the robot's camera -> obstacle points for navigation's map.

The RPLIDAR sees one flat slice at its own height. It misses what is above or
below that slice (a table top, a low box: its rays pass over and CLEAR those
cells) and what returns nothing (black or chrome, which it can only call
unknown). The D435i's depth sees them. This turns a coarse depth grid (metres,
0 = no reading) into obstacle points in the ROBOT frame, (x forward, y left)
metres, for the camera layer of navigation's Mapper. They only ever block:
nothing here reports free space.

All camera geometry goes through perception/projection.py: `deproject` and
`camera_to_base` work on numpy arrays exactly as on floats, so there is still
one place the signs live. NumPy is imported inside the functions, so importing
this module needs nothing installed.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from retriever.perception.projection import CameraMount, Intrinsics, camera_to_base, deproject


@dataclass(frozen=True)
class CameraObstacleConfig:
    """Every number the depth -> obstacle step uses."""

    floor_m: float = 0.05        # above the floor plane before a point counts...
    floor_per_m: float = 0.02    # ...plus this per metre of range: a small pitch error lifts far floor
    max_height_m: float = 0.70   # the robot's top + 0.10: above this it passes under. MEASURE
    min_range_m: float = 0.25    # nearer D435i returns are unreliable (unverified)
    max_range_m: float = 3.0     # depth noise grows with range
    bin_deg: float = 2.0         # one point per 2 degrees of bearing, like the lidar's bins
    kth: int = 3                 # per bin, the kth-nearest point: one bad pixel is not an obstacle


# Where the D435i sits on the robot. PLACEHOLDERS: measure them, or point the
# camera at open floor and fit the height and tilt with `fit_floor`.
DEFAULT_MOUNT = CameraMount(height_m=0.30, forward_m=0.20, pitch_rad=0.15)


def grid_intrinsics(intr: Intrinsics, cell_px: int) -> Intrinsics:
    """Full-resolution intrinsics -> the depth grid's. Cell (i, j) is the median of
    full-res pixels [j*k, j*k + k) x [i*k, i*k + k), centred at full-res
    x = j*k + (k - 1)/2, so grid u = (x + 0.5)/k - 0.5."""
    k = float(cell_px)
    return Intrinsics(intr.fx / k, intr.fy / k, (intr.cx + 0.5) / k - 0.5, (intr.cy + 0.5) / k - 0.5)


def base_points(grid_m: Any, intr: Intrinsics, mount: CameraMount):
    """Every valid cell -> (forward, left, up) numpy arrays, robot frame, metres."""
    import numpy as np

    d = np.asarray(grid_m, dtype=np.float64)
    v, u = np.nonzero(np.isfinite(d) & (d > 0))
    x, y, z = deproject(u.astype(np.float64), v.astype(np.float64), d[v, u], intr)
    return camera_to_base(x, y, z, mount)


def obstacle_points(grid_m: Any, intr: Intrinsics, mount: CameraMount = DEFAULT_MOUNT,
                    cfg: CameraObstacleConfig = CameraObstacleConfig()) -> list[tuple[float, float]]:
    """Depth grid -> robot-frame (x, y) obstacle points: in each bin_deg of
    bearing, the kth-nearest point that is above the floor, below the robot's
    top and in range. Bins with fewer than kth such points give none. `intr`
    is the GRID's intrinsics (grid_intrinsics)."""
    import numpy as np

    fwd, left, up = base_points(grid_m, intr, mount)
    rng = np.hypot(fwd, left)
    keep = ((up > cfg.floor_m + cfg.floor_per_m * rng) & (up < cfg.max_height_m)
            & (rng >= cfg.min_range_m) & (rng <= cfg.max_range_m))
    if not np.any(keep):
        return []
    fwd, left, rng = fwd[keep], left[keep], rng[keep]
    bins = np.floor((np.arctan2(left, fwd) + math.pi) / math.radians(cfg.bin_deg)).astype(np.int64)
    order = np.lexsort((rng, bins))             # by bin, nearest first within a bin
    fwd, left, bins = fwd[order], left[order], bins[order]
    _, first, count = np.unique(bins, return_index=True, return_counts=True)
    pick = first[count >= cfg.kth] + (cfg.kth - 1)
    return [(float(x), float(y)) for x, y in zip(fwd[pick], left[pick])]


def fit_floor(grid_m: Any, intr: Intrinsics, rows: float = 0.5) -> tuple[float, float]:
    """(height_m, pitch_rad) of the camera above a flat floor, from a depth grid
    of open floor, using the lower `rows` of the image.

    A floor point satisfies  y_down*cos(p) + z_fwd*sin(p) = h  (camera_to_base's
    up = 0 with pitch p and height h). Fit y = A + B*z by least squares; then
    p = atan(-B) and h = A*cos(p). It refits once without points more than
    three robust deviations off the first line, so a shoe or a table leg in
    the frame doesn't tilt the answer. Raises ValueError without enough floor."""
    import numpy as np

    d = np.asarray(grid_m, dtype=np.float64)
    top = int(d.shape[0] * (1.0 - rows))
    sub = d[top:]
    v, u = np.nonzero(np.isfinite(sub) & (sub > 0))
    if v.size < 50:
        raise ValueError(f"only {v.size} depth cells in the lower image: "
                         "point the camera at open floor")
    _, y, z = deproject(u.astype(np.float64), (v + top).astype(np.float64), sub[v, u], intr)
    b, a = np.polyfit(z, y, 1)
    res = y - (a + b * z)
    mad = float(np.median(np.abs(res - np.median(res))))
    ok = np.abs(res) <= max(3.0 * 1.4826 * mad, 0.01)
    b, a = np.polyfit(z[ok], y[ok], 1)
    pitch = math.atan(-b)
    return float(a * math.cos(pitch)), float(pitch)
