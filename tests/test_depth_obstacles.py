"""Depth grid -> obstacle points (perception/depth_obstacles.py).

Every test renders a synthetic depth image by ray casting a flat floor and
axis-aligned boxes from a known camera mount, so the expected answer is
geometry, not a recording. numpy only.
"""

import base64
import math
import sys
import time
import unittest
import zlib
from importlib.util import find_spec
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

HAVE_NUMPY = find_spec("numpy") is not None
if HAVE_NUMPY:
    import numpy as np

from retriever.perception.depth_obstacles import (
    DEFAULT_MOUNT,
    CameraObstacleSource,
    calibrate,
    check,
    depth_camera_intrinsics,
    fit_floor,
    grid_intrinsics,
    obstacle_points,
)
from retriever.perception.projection import CameraMount, Intrinsics, camera_to_base, deproject

W, H = 170, 96                                  # an 848x480 D435i grid at 5 px cells
INTR = Intrinsics(fx=90.0, fy=90.0, cx=84.5, cy=47.5)
MOUNT = CameraMount(height_m=0.30, forward_m=0.20, pitch_rad=0.15)


def render(mount, boxes=(), floor=True, w=W, h=H, intr=INTR, far=8.0):
    """Depth (metres along the camera's z; 0 = no reading) of a flat floor and
    axis-aligned boxes (base frame: x0, x1, y0, y1, z0, z1; y > 0 is LEFT)."""
    v, u = np.mgrid[0:h, 0:w].astype(np.float64)
    # A ray whose camera-frame z is 1: its parameter along the ray IS the depth.
    dx, dy, dz = camera_to_base((u - intr.cx) / intr.fx, (v - intr.cy) / intr.fy,
                                np.ones_like(u), CameraMount(0.0, 0.0, mount.pitch_rad))
    ox, oy, oz = mount.forward_m, 0.0, mount.height_m
    depth = np.full((h, w), np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        if floor:
            t = -oz / dz
            depth = np.where((dz < 0) & (t > 0), t, depth)
        for x0, x1, y0, y1, z0, z1 in boxes:
            tx = ((x0 - ox) / dx, (x1 - ox) / dx)
            ty = ((y0 - oy) / dy, (y1 - oy) / dy)
            tz = ((z0 - oz) / dz, (z1 - oz) / dz)
            tmin = np.maximum.reduce([np.minimum(*tx), np.minimum(*ty), np.minimum(*tz)])
            tmax = np.minimum.reduce([np.maximum(*tx), np.maximum(*ty), np.maximum(*tz)])
            hit = (tmax >= tmin) & (tmin > 0)
            depth = np.where(hit & (tmin < depth), tmin, depth)
    depth[~np.isfinite(depth) | (depth > far)] = 0.0
    return depth


def nearest(points):
    return min(points, key=lambda p: math.hypot(*p))


@unittest.skipUnless(HAVE_NUMPY, "numpy not installed")
class TestObstaclePoints(unittest.TestCase):
    def test_a_flat_floor_makes_no_points(self):
        self.assertEqual(obstacle_points(render(MOUNT), INTR, MOUNT), [])

    def test_the_floor_stays_floor_with_the_camera_tilted_further_down(self):
        m = CameraMount(height_m=0.30, forward_m=0.20, pitch_rad=0.35)
        self.assertEqual(obstacle_points(render(m), INTR, m), [])

    def test_a_box_ahead_is_about_a_metre_ahead(self):
        box = (1.0, 1.3, -0.15, 0.15, 0.0, 0.25)          # 25 cm tall, front face 1 m out
        x, y = nearest(obstacle_points(render(MOUNT, [box]), INTR, MOUNT))
        self.assertAlmostEqual(math.hypot(x, y), 1.0, delta=0.05)
        self.assertLess(abs(math.atan2(y, x)), math.radians(10))

    def test_a_box_on_the_left_has_positive_y(self):
        box = (1.0, 1.3, 0.25, 0.55, 0.0, 0.25)           # y > 0 is LEFT
        pts = obstacle_points(render(MOUNT, [box]), INTR, MOUNT)
        self.assertTrue(pts)
        self.assertTrue(all(y > 0 for _, y in pts))

    def test_a_table_top_within_the_robots_height_is_kept(self):
        top = (0.8, 1.6, -0.4, 0.4, 0.60, 0.62)           # a slab the lidar's slice misses
        pts = obstacle_points(render(MOUNT, [top]), INTR, MOUNT)
        self.assertTrue(pts)
        self.assertGreater(math.hypot(*nearest(pts)), 0.9)

    def test_something_above_the_robot_is_dropped(self):
        ceiling = (0.8, 2.5, -1.0, 1.0, 0.90, 0.92)
        self.assertEqual(obstacle_points(render(MOUNT, [ceiling]), INTR, MOUNT), [])

    def test_one_bad_pixel_is_not_an_obstacle(self):
        d = render(MOUNT)
        d[40, 85] = 0.8
        self.assertEqual(obstacle_points(d, INTR, MOUNT), [])

    def test_kth_nearest_points_in_one_bin_are(self):
        d = render(MOUNT)
        d[38:41, 85] = 0.8                                # three readings, one bearing
        pts = obstacle_points(d, INTR, MOUNT)
        self.assertEqual(len(pts), 1)
        self.assertAlmostEqual(math.hypot(*pts[0]), 1.0, delta=0.05)

    def test_grid_intrinsics_put_cell_centres_where_their_pixels_are(self):
        full = Intrinsics(446.8, 433.0, 423.5, 239.5)
        g = grid_intrinsics(full, 5)
        # cell (84, 47) is full-res pixels 420..424 x 235..239: centre (422, 237)
        a = deproject(422.0, 237.0, 1.0, full)
        b = deproject(84.0, 47.0, 1.0, g)
        for p, q in zip(a, b):
            self.assertAlmostEqual(p, q, places=9)

    def test_projection_accepts_arrays(self):
        xs, ys, zs = np.array([0.1, -0.2]), np.array([0.05, 0.3]), np.array([1.0, 2.0])
        f, l, u = camera_to_base(xs, ys, zs, MOUNT)
        for i in range(2):
            fi, li, ui = camera_to_base(float(xs[i]), float(ys[i]), float(zs[i]), MOUNT)
            self.assertAlmostEqual(f[i], fi)
            self.assertAlmostEqual(l[i], li)
            self.assertAlmostEqual(u[i], ui)


@unittest.skipUnless(HAVE_NUMPY, "numpy not installed")
class TestFitFloor(unittest.TestCase):
    def test_recovers_height_and_pitch(self):
        for h, p in ((0.30, 0.15), (0.45, 0.30)):
            m = CameraMount(height_m=h, forward_m=0.2, pitch_rad=p)
            got_h, got_p = fit_floor(render(m), INTR)
            self.assertAlmostEqual(got_h, h, delta=0.005)
            self.assertAlmostEqual(got_p, p, delta=0.005)

    def test_ignores_a_box_in_the_frame(self):
        got_h, got_p = fit_floor(render(MOUNT, [(0.8, 1.0, -0.1, 0.1, 0.0, 0.2)]), INTR)
        self.assertAlmostEqual(got_h, 0.30, delta=0.01)
        self.assertAlmostEqual(got_p, 0.15, delta=0.01)

    def test_needs_floor_in_view(self):
        with self.assertRaises(ValueError):
            fit_floor(np.zeros((H, W)), INTR)


def grid_message(depth_m, cell_px=1, age_s=0.05, seq=1):
    """A wire-shaped message from the depth publisher carrying `depth_m` as its grid."""
    mm = np.clip(np.rint(depth_m * 1000.0), 0, 65535).astype("<u2")
    h, w = depth_m.shape
    return {"t": 0.0, "seq": seq, "frame_w": w * cell_px, "frame_h": h * cell_px,
            "detections": [], "age_s": age_s,
            "depth_grid": {"b64": base64.b64encode(zlib.compress(mm.tobytes(), 1)).decode(),
                           "w": w, "h": h, "cell_px": cell_px, "encoding": "zlib+<u2",
                           "scale_m": 0.001}}


def camstream_header(intr=INTR, w=W, h=H, aligned=False):
    """The publisher's header message, the one depth_camera_intrinsics reads."""
    return {"type": "header", "frame_w": w, "frame_h": h, "camera": "d435i-qnx",
            "detector": None, "intrinsics": None,
            "depth": {"aligned": aligned,
                      "intrinsics": {"fx": intr.fx, "fy": intr.fy, "cx": intr.cx,
                                     "cy": intr.cy, "width": w, "height": h}}}


class StubFrame:
    """One packet off the camera stream, with what the source reads from it: how long
    ago it reached this laptop, how old it was when sent, and its depth grid."""

    def __init__(self, message, received_at):
        self.message = message
        self.received_at = received_at
        self.depth_grid = message.get("depth_grid")
        self.source_age_s = message.get("age_s")

    @property
    def age(self):
        return time.monotonic() - self.received_at

    def depth_grid_m(self):
        grid = self.depth_grid
        if grid is None:
            return None
        raw = zlib.decompress(base64.b64decode(grid["b64"]))
        return np.frombuffer(raw, "<u2").reshape(grid["h"], grid["w"]) * grid["scale_m"]


class StubRemote:
    def __init__(self, header=None, packet=None, staleness=0.0):
        self.header, self.packet, self._stale, self.stopped = header, packet, staleness, False

    def latest_packet(self):
        return self.packet

    def staleness(self):
        return self._stale

    def stop(self):
        self.stopped = True


class TestDepthCameraIntrinsics(unittest.TestCase):
    """The depth camera's own numbers out of the publisher's header: without them
    nothing can be projected, and guessing would quietly bend the whole map."""

    def test_reads_the_depth_cameras_own_intrinsics_and_size(self):
        found = depth_camera_intrinsics(camstream_header())
        self.assertIsNotNone(found)
        self.assertEqual(found[0], INTR)
        self.assertEqual(found[1], (W, H))

    def test_depth_aligned_to_colour_is_refused(self):
        # Aligned depth is in the COLOUR camera's frame: these numbers would misplace it.
        self.assertIsNone(depth_camera_intrinsics(camstream_header(aligned=True)))

    def test_no_header_yet_is_none_not_a_crash(self):
        self.assertIsNone(depth_camera_intrinsics(None))
        self.assertIsNone(depth_camera_intrinsics("header"))

    def test_a_header_without_depth_intrinsics_is_none(self):
        header = camstream_header()
        header.pop("depth")
        self.assertIsNone(depth_camera_intrinsics(header))
        self.assertIsNone(depth_camera_intrinsics({"depth": {"aligned": False}}))
        self.assertIsNone(depth_camera_intrinsics({"depth": {"intrinsics": None}}))

    def test_malformed_numbers_are_none(self):
        for spoil in ({"fx": None}, {"fy": "wide"}, {"fx": 0.0}, {"width": 0},
                      {"cx": float("nan")}):
            with self.subTest(spoil=spoil):
                header = camstream_header()
                header["depth"]["intrinsics"].update(spoil)
                self.assertIsNone(depth_camera_intrinsics(header))
        missing = camstream_header()
        missing["depth"]["intrinsics"].pop("height")
        self.assertIsNone(depth_camera_intrinsics(missing))


@unittest.skipUnless(HAVE_NUMPY, "numpy not installed")
class TestCameraObstacleSource(unittest.TestCase):
    """The contract navigation consumes: latest(), age(), describe(), all on the
    laptop's monotonic clock."""

    def packet(self, depth, received_ago=0.2, age_s=0.05, seq=1):
        return StubFrame(grid_message(depth, age_s=age_s, seq=seq),
                         received_at=time.monotonic() - received_ago)

    def source(self, remote, clock=lambda: 100.0):
        return CameraObstacleSource(remote, MOUNT, clock=clock)

    def test_a_box_in_the_stream_becomes_points_dated_at_capture_time(self):
        box = (1.0, 1.3, -0.15, 0.15, 0.0, 0.25)
        src = self.source(StubRemote(camstream_header(), self.packet(render(MOUNT, [box]))))
        obs = src.latest()
        # captured = now - how long ago it arrived - how old it was when sent
        self.assertAlmostEqual(obs.t, 100.0 - 0.2 - 0.05, delta=0.02)
        self.assertAlmostEqual(math.hypot(*nearest(obs.points)), 1.0, delta=0.05)
        self.assertEqual(src.last_points, len(obs.points))

    def test_one_packet_is_dated_once(self):
        src = self.source(StubRemote(camstream_header(), self.packet(render(MOUNT))))
        self.assertIs(src.latest(), src.latest())

    def test_no_packet_or_no_depth_intrinsics_means_none(self):
        self.assertIsNone(self.source(StubRemote(camstream_header(), None)).latest())
        no_intr = StubRemote(None, self.packet(render(MOUNT)))
        with self.assertLogs("retriever.perception.depth_obstacles", "WARNING"):
            self.assertIsNone(self.source(no_intr).latest())

    def test_age_and_stop_pass_through_to_the_stream(self):
        remote = StubRemote(camstream_header(), None, staleness=2.5)
        src = self.source(remote)
        self.assertEqual(src.age(), 2.5)
        src.stop()
        self.assertTrue(remote.stopped)

    def test_describe_says_what_is_nearest_left_ahead_and_right(self):
        box = (1.0, 1.3, -0.15, 0.15, 0.0, 0.25)
        line = self.source(StubRemote(camstream_header(),
                                      self.packet(render(MOUNT, [box])))).describe()
        self.assertIn("points", line)
        self.assertIn("ahead 1.0", line)

    def test_describe_says_when_there_is_no_stream_yet_or_it_went_quiet(self):
        self.assertIn("waiting", self.source(StubRemote(camstream_header(), None,
                                                        staleness=math.inf)).describe())
        self.assertIn("2.0 s", self.source(StubRemote(camstream_header(), None,
                                                      staleness=2.0)).describe())

    def test_calibrate_reads_the_mount_from_the_stream(self):
        h, p = calibrate(StubRemote(camstream_header(), self.packet(render(MOUNT))),
                         timeout_s=0.1)
        self.assertAlmostEqual(h, MOUNT.height_m, delta=0.01)
        self.assertAlmostEqual(p, MOUNT.pitch_rad, delta=0.01)

    def test_calibrate_times_out_without_a_stream(self):
        with self.assertRaises(TimeoutError):
            calibrate(StubRemote(camstream_header(), None), timeout_s=0.05, poll_s=0.01)


@unittest.skipUnless(HAVE_NUMPY, "numpy not installed")
class TestCheck(unittest.TestCase):
    """The preflight a hardware test starts with: it must name the real failure,
    not just fail. Every case here has happened at least once."""

    def run_check(self, remote, mount=MOUNT):
        return check(CameraObstacleSource(remote, mount), seconds=0.03, poll_s=0.005)

    def packet(self, depth):
        return StubFrame(grid_message(depth), received_at=time.monotonic())

    def test_a_working_stream_is_ready_and_says_what_it_sees(self):
        box = (1.0, 1.3, -0.15, 0.15, 0.0, 0.25)
        ready, lines = self.run_check(
            StubRemote(camstream_header(), self.packet(render(MOUNT, [box]))))
        text = "\n".join(lines)
        self.assertTrue(ready, text)
        self.assertIn("READY", lines[-1])
        self.assertNotIn("!!", text)
        self.assertIn("obstacle points", text)
        self.assertIn("cells", text)

    def test_nothing_publishing_says_so_first(self):
        ready, lines = self.run_check(StubRemote(None, None))
        self.assertFalse(ready)
        self.assertIn("no header", lines[0])
        self.assertIn("depth publisher", lines[0])

    def test_a_detection_stream_is_named_as_the_wrong_publisher(self):
        header = {"type": "header", "frame_w": W, "frame_h": H,
                  "camera": "rdk", "detector": "yolo11n", "intrinsics": None}
        with self.assertLogs("retriever.perception.depth_obstacles", "WARNING"):
            ready, lines = self.run_check(StubRemote(header, self.packet(render(MOUNT))))
        text = "\n".join(lines)
        self.assertFalse(ready)
        self.assertIn("not the depth publisher", text)
        self.assertIn("NOT READY", lines[-1])

    def test_a_silent_publisher_is_distinguished_from_a_missing_one(self):
        ready, lines = self.run_check(StubRemote(camstream_header(), None))
        text = "\n".join(lines)
        self.assertFalse(ready)
        self.assertIn("reachable but silent", text)

    def test_an_empty_grid_is_called_out(self):
        ready, lines = self.run_check(
            StubRemote(camstream_header(), self.packet(np.zeros((H, W), np.float32))))
        self.assertFalse(ready)
        self.assertIn("almost no valid cells", "\n".join(lines))

    def test_a_placeholder_mount_is_flagged_but_does_not_block(self):
        ready, lines = self.run_check(
            StubRemote(camstream_header(), self.packet(render(DEFAULT_MOUNT))), DEFAULT_MOUNT)
        text = "\n".join(lines)
        self.assertTrue(ready, text)
        self.assertIn("still the placeholders", text)


if __name__ == "__main__":
    unittest.main()
