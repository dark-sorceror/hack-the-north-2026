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

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable

from retriever.perception.projection import CameraMount, Intrinsics, camera_to_base, deproject

log = logging.getLogger(__name__)


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


def depth_camera_intrinsics(header: Any) -> tuple[Intrinsics, tuple[int, int]] | None:
    """The DEPTH camera's own intrinsics and (width, height) from the publisher's header
    message, or None when there are none to project with.

    None covers three cases that all mean the same thing to a caller — wait, do not
    guess: no header has arrived yet, the header carries no usable depth intrinsics, or
    it says the depth grid is ALIGNED to the colour image. Aligned depth is in the COLOUR
    camera's frame, so projecting it with the depth camera's numbers would put every
    point in the wrong place; a publisher that changes its mind about alignment must not
    be able to turn this into a silent, slightly-wrong map. Never raises: a header from
    an older publisher is a normal thing to meet, not an error.
    """
    if not isinstance(header, dict):
        return None
    depth = header.get("depth")
    if not isinstance(depth, dict) or depth.get("aligned"):
        return None
    numbers = depth.get("intrinsics")
    if not isinstance(numbers, dict):
        return None
    try:
        intr = Intrinsics(float(numbers["fx"]), float(numbers["fy"]),
                          float(numbers["cx"]), float(numbers["cy"]))
        width, height = int(numbers["width"]), int(numbers["height"])
    except (KeyError, TypeError, ValueError):
        return None
    if width <= 0 or height <= 0:
        return None
    return intr, (width, height)


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


@dataclass(frozen=True)
class CameraObstacles:
    """What the camera says is in the way: the hand-off to navigation.

    points: (x forward, y left) metres in the ROBOT frame at capture time.
    t:      capture time on the laptop's time.monotonic(), so navigation can
            look up the pose it had then, as it does for lidar scans.
    Block-only: an empty tuple means "nothing seen in the way", never "free".
    """

    t: float
    points: tuple[tuple[float, float], ...]


class CameraObstacleSource:
    """The newest depth grid on the robot-camera stream, as CameraObstacles.

    Dating. The stream's `t` is the publisher's wall clock and is never
    comparable with ours. What IS known: how long ago this laptop received the
    packet (the frame's `age`, our monotonic clock) and how old the frame was
    when it was sent (`source_age_s`, the publisher's monotonic clock). So it
    was captured about age + source_age_s ago. A packet is dated once, the
    first time it is seen: a stalled stream keeps handing back the same
    CameraObstacles with its old t, which is what lets navigation age it out.

    None without a packet, a depth grid, or depth intrinsics in the header (the
    last is logged once). Non-blocking and thread-safe: the control loop calls
    latest() about 20 times a second, and a dashboard may call describe() too.
    """

    def __init__(self, remote: Any, mount: CameraMount = DEFAULT_MOUNT,
                 cfg: CameraObstacleConfig = CameraObstacleConfig(),
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.remote = remote
        self.mount = mount
        self.cfg = cfg
        self.clock = clock
        self.last_points = 0
        self._lock = threading.Lock()
        self._pkt: Any = None
        self._obs: CameraObstacles | None = None
        self._warned = False

    def latest(self) -> CameraObstacles | None:
        """The newest frame's obstacle points, or None. Cheap to call: a packet
        already turned into points is handed back as it is."""
        with self._lock:
            pkt = self.remote.latest_packet()
            if pkt is None:
                return None
            if pkt is self._pkt:
                return self._obs
            found = depth_camera_intrinsics(self.remote.header)
            if found is None:
                if not self._warned:
                    log.warning("the camera stream's header has no depth-camera intrinsics: "
                                "no camera obstacles until it does")
                    self._warned = True
                return None
            cell = int((pkt.depth_grid or {}).get("cell_px") or 1)
            grid = pkt.depth_grid_m()
            if grid is None:
                return None
            pts = obstacle_points(grid, grid_intrinsics(found[0], cell), self.mount, self.cfg)
            self._pkt = pkt
            self._obs = CameraObstacles(self.clock() - pkt.age - (pkt.source_age_s or 0.0),
                                        tuple(pts))
            self.last_points = len(pts)
            return self._obs

    def age(self) -> float:
        """Seconds since the stream last delivered anything (inf: never)."""
        return self.remote.staleness()

    def describe(self) -> str:
        """One line for a dashboard or the bench: how many points, and the
        nearest one to the left (> 15 degrees), ahead, and to the right."""
        age = self.age()
        obs = self.latest()
        if obs is None:
            return ("waiting for the camera stream" if math.isinf(age)
                    else f"no points (stream {age:.1f} s old)")
        near = {"left": math.inf, "ahead": math.inf, "right": math.inf}
        for x, y in obs.points:
            bearing = math.degrees(math.atan2(y, x))
            side = "left" if bearing > 15 else ("right" if bearing < -15 else "ahead")
            near[side] = min(near[side], math.hypot(x, y))
        parts = [f"{side} {d:.2f} m" for side, d in near.items() if d < math.inf]
        return f"{len(obs.points)} points" + (": " + ", ".join(parts) if parts else "")

    def stop(self) -> None:
        self.remote.stop()


def calibrate(remote: Any, timeout_s: float = 10.0, poll_s: float = 0.05) -> tuple[float, float]:
    """(height_m, pitch_rad) of the camera from the stream's next depth grid,
    with open floor in front of it (fit_floor). Raises TimeoutError when no grid
    with depth-camera intrinsics arrives within timeout_s."""
    deadline = time.monotonic() + timeout_s
    while True:
        pkt = remote.latest_packet()
        found = depth_camera_intrinsics(remote.header)
        if pkt is not None and found is not None:
            cell = int((pkt.depth_grid or {}).get("cell_px") or 1)
            grid = pkt.depth_grid_m()
            if grid is not None:
                return fit_floor(grid, grid_intrinsics(found[0], cell))
        if time.monotonic() >= deadline:
            raise TimeoutError(f"no depth grid with depth-camera intrinsics within "
                               f"{timeout_s:.0f} s: is the depth publisher running?")
        time.sleep(poll_s)
