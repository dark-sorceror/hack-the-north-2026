#!/usr/bin/env python3
"""Find the goose in a camera stream and say where it is, in metres, on the floor.

    python3 scripts/floor_detector.py --url http://hao.local:8790/stream \
        --mount 0.15,0.20,12 --target goose --show

This is the real `Detector` for `scripts/central_pi.py` -- the thing `--fake-detect`
was standing in for. It runs on the Mac, pulls the robot's MJPEG stream, and turns a
detection into a robot-frame point.

NO DEPTH, AND IT DOES NOT NEED ANY. The object is on the floor, which is a stronger
constraint than a depth reading: the bottom edge of its box is where it touches the
ground, so the ray through that pixel hits a plane we already know the height of. That
removes the whole depth-alignment problem, and it is more accurate at range than a
stereo camera, whose error grows with distance while this one does not.

WHAT IT COSTS. It is only true for things ON THE FLOOR. Something on a table reads as
further away than it is (its ray hits the floor beyond it), which is the honest failure
for a robot whose arm works low down -- it will not drive confidently at a thing it
cannot reach anyway.

THE MOUNT IS THE WHOLE BALLGAME. `--mount HEIGHT,FORWARD,PITCH_DEG` is measured, not
guessed. Positive pitch means tilted DOWN. Check it before trusting a single number:
put the goose dead ahead and confirm y is about 0; move it left and confirm y goes
POSITIVE. The lidar's mount was wrong by 225 degrees for hours and every map it built
was scrambled -- it looked like a mapping bug for the whole afternoon.

All camera trigonometry is delegated to `perception/projection.py`, which is the only
file in the repo allowed to write the camera's sines and cosines.
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
import threading
import time
import urllib.request
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.perception.projection import CameraMount, Intrinsics, camera_to_base

LOG = logging.getLogger("detect")

# The D435i's colour stream, 69.4 x 42.5 degrees. Only used to derive fx/fy when the
# stream does not tell us; override with --hfov if you are on a different camera.
D435I_HFOV_DEG = 69.4

# What the goose plush actually comes back as. A COCO detector has no "goose" class, so
# accept the handful of labels it plausibly lands on and let the caller name the target.
GOOSE_ALIASES = ("bird", "teddy bear", "duck", "goose", "sheep", "cat", "dog")

# Open-vocabulary prompts. COCO has no goose, and a closed-set nano model calls this
# one "sports ball" at 0.03. Given the WORD, YOLO-World finds it. Order matters only
# in that every prompt becomes a class the model will happily fire on.
# Open-vocabulary scores sit an order of magnitude below a closed-set model's.
# The goose lying on its side scores 0.05 or less, so the threshold has to be low
# -- which is safe here only because two filters sit behind it: the floor solve
# throws out anything above the horizon, and MAX_BOX_FRACTION throws out anything
# too big to be a retrievable object.
DEFAULT_CONF = 0.01
MAX_BOX_FRACTION = 0.35

# Open-vocabulary scores sit an order of magnitude below a closed-set model's, so
# the threshold has to be low. That is safe only because two filters sit behind it:
# the floor solve discards anything above the horizon, and MAX_BOX_FRACTION discards
# anything too large to be a retrievable object.
DEFAULT_CONF = 0.01
MAX_BOX_FRACTION = 0.35

GOOSE_PROMPTS = (
    "toy on the floor",
    "stuffed animal",
    "plush toy",
    "soft toy",
    "goose",
    "goose plush toy",
    "plush bird",
)
# Generic prompts FIRST, and that ordering is a finding, not a style choice. Stood
# up the goose reads as "goose plush toy" at 0.06-0.6; lying on its side it stops
# looking like a goose at all and those prompts collapse to 0.018, while "toy on
# the floor" holds at 0.236 -- an order of magnitude better on the same frame.
# A fetch robot cares that a thing is a retrievable object on the ground, which is
# what the generic wording actually describes. The specific prompts stay because
# they win when the pose is favourable, and the floor solve throws out whatever is
# above the horizon regardless of what any of them scored.


@dataclass(frozen=True)
class Seen:
    """Matches scripts/central_pi.py's Seen: robot frame, +x forward, +y left."""
    x: float
    y: float
    age_s: float
    label: str = ""
    confidence: float | None = None


def intrinsics_for(width: int, height: int, hfov_deg: float = D435I_HFOV_DEG) -> Intrinsics:
    """Pinhole intrinsics from an image size and a horizontal field of view."""
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    return Intrinsics(fx=fx, fy=fx, cx=width / 2.0, cy=height / 2.0)


def floor_point(u: float, v: float, intr: Intrinsics, mount: CameraMount,
                right_m: float = 0.0) -> tuple[float, float] | None:
    """Where the ray through pixel (u, v) meets the floor, in the base frame.

    Solved rather than measured. A camera-frame ray at depth z is
    (x_right, y_down, z_fwd) = ((u-cx)/fx * z, (v-cy)/fy * z, z), and
    `camera_to_base` gives its height above the floor as

        up(z) = height_m - z * ( (v-cy)/fy * cos(pitch) + sin(pitch) )

    so the depth at which that reaches the floor falls straight out. A non-positive
    denominator means the ray is level or pointing UP: it never meets the floor, and
    the honest answer is None rather than a huge number.
    """
    ry = (v - intr.cy) / intr.fy
    cos_p, sin_p = math.cos(mount.pitch_rad), math.sin(mount.pitch_rad)
    denom = ry * cos_p + sin_p
    if denom <= 1e-6:
        return None
    z = mount.height_m / denom
    if not (0.05 < z < 12.0):            # nonsense, or so far the pixel noise dominates
        return None
    rx = (u - intr.cx) / intr.fx
    forward, left, _up = camera_to_base(rx * z, ry * z, z, mount)
    # `CameraMount` carries height and forward offset but no LATERAL one, and this
    # camera is bolted off to one side. A lens `right_m` to the right of the
    # centreline sees a thing dead ahead of itself as dead ahead -- but in the base
    # frame that thing is `right_m` to the right, i.e. that much less to the left.
    # Without this the robot aims a camera-offset beside the object, every time.
    return forward, left - right_m


class MjpegStream:
    """The newest frame from an MJPEG endpoint, decoded, in a background thread.

    Kept newest-only on purpose: a queue would hand the detector a stale frame and the
    whole point of `age_s` is that staleness is measured, not accumulated.
    """

    def __init__(self, url: str) -> None:
        import cv2                                   # noqa: F401  (fail early, loudly)
        self.url = url
        self._frame = None
        self._t = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> "MjpegStream":
        self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()

    def latest(self) -> tuple["object|None", float]:
        with self._lock:
            return self._frame, self._t

    def _run(self) -> None:
        import cv2
        import numpy as np
        while not self._stop.is_set():
            try:
                with urllib.request.urlopen(self.url, timeout=5) as r:
                    buf = b""
                    while not self._stop.is_set():
                        chunk = r.read(4096)
                        if not chunk:
                            break
                        buf += chunk
                        a, b = buf.find(b"\xff\xd8"), buf.find(b"\xff\xd9")
                        if a != -1 and b > a:
                            jpg, buf = buf[a:b + 2], buf[b + 2:]
                            img = cv2.imdecode(np.frombuffer(jpg, np.uint8),
                                               cv2.IMREAD_COLOR)
                            if img is not None:
                                with self._lock:
                                    self._frame, self._t = img, time.monotonic()
                        if len(buf) > 4_000_000:      # desync; resynchronise
                            buf = b""
            except Exception as exc:
                LOG.warning("stream: %s", exc)
                time.sleep(1.0)


class DepthStreamFrames:
    """Newest colour frame from the robot-camera stream on TCP :5577.

    The same stream navigation already reads, so the camera can sit on any board
    without this file caring which. One line of JSON per message; the first is a
    header. Only messages carrying `frame.jpeg_b64` are useful here -- the
    publisher only attaches one every `--jpeg-every` messages, and the rest are
    depth, which the floor solve does not need.

    AGE COMES FROM THE SENDER. Each message carries `age_s`, how old the frame was
    when it was sent on the PUBLISHER's clock. Adding our own time-since-receipt
    gives a true end-to-end age with no clock sync between the boards -- which is
    exactly what `/approach` wants.
    """

    def __init__(self, host: str, port: int = 5577) -> None:
        import cv2  # noqa: F401  (fail early, loudly)
        self.host, self.port = host, port
        self._frame = None
        self._age_at_rx = 0.0
        self._rx_t = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> "DepthStreamFrames":
        # Idempotent: one stream feeds both the detector and the voice turn's image,
        # and whichever wakes up second must not blow up on an already-running thread.
        if not self._thread.is_alive():
            self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()

    def latest(self) -> tuple["object|None", float]:
        """(frame, effective_capture_time) so callers can treat it like any stream."""
        with self._lock:
            if self._frame is None:
                return None, 0.0
            return self._frame, self._rx_t - self._age_at_rx

    def _run(self) -> None:
        import base64
        import json
        import socket
        import cv2
        import numpy as np
        while not self._stop.is_set():
            try:
                with socket.create_connection((self.host, self.port), timeout=5) as sk:
                    sk.settimeout(5)
                    buf = b""
                    while not self._stop.is_set():
                        chunk = sk.recv(65536)
                        if not chunk:
                            break
                        buf += chunk
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            if not line.strip():
                                continue
                            try:
                                msg = json.loads(line)
                            except Exception:
                                continue
                            fr = msg.get("frame")
                            if not fr or "jpeg_b64" not in fr:
                                continue
                            raw = base64.b64decode(fr["jpeg_b64"])
                            img = cv2.imdecode(np.frombuffer(raw, np.uint8),
                                               cv2.IMREAD_COLOR)
                            if img is not None:
                                with self._lock:
                                    self._frame = img
                                    self._age_at_rx = float(msg.get("age_s", 0.0))
                                    self._rx_t = time.monotonic()
            except Exception as exc:
                LOG.warning("stream %s:%s -- %s", self.host, self.port, exc)
                time.sleep(1.0)


class FloorDetector:
    """A `Detector` for central_pi.py: `locate(label) -> Seen | None`."""

    def __init__(self, source, mount: CameraMount, *, weights: str = "yolov8n.pt",
                 conf: float = DEFAULT_CONF, hfov_deg: float = D435I_HFOV_DEG,
                 aliases: tuple[str, ...] = GOOSE_ALIASES, show: bool = False,
                 right_m: float = 0.0, prompts: tuple[str, ...] | None = None) -> None:
        # A "-world" checkpoint is open-vocabulary: it takes the words we care about
        # instead of COCO's 80 fixed classes. That is the whole reason this works.
        self.prompts = tuple(prompts) if prompts else GOOSE_PROMPTS
        if "world" in str(weights).lower():
            from ultralytics import YOLOWorld
            self.model = YOLOWorld(weights)
            self.model.set_classes(list(self.prompts))
            self.open_vocab = True
        else:
            from ultralytics import YOLO
            self.model = YOLO(weights)
            self.open_vocab = False
        self.stream = source.start() if hasattr(source, 'start') else source
        self.mount = mount
        self.conf = conf
        self.hfov_deg = hfov_deg
        self.aliases = tuple(a.lower() for a in aliases)
        self.right_m = right_m
        self.show = show
        self._intr: Intrinsics | None = None

    def locate(self, label: str) -> Seen | None:
        frame, t = self.stream.latest()
        if frame is None:
            LOG.warning("no frame yet")
            return None
        age = time.monotonic() - t
        h, w = frame.shape[:2]
        if self._intr is None:
            self._intr = intrinsics_for(w, h, self.hfov_deg)
            LOG.info("intrinsics %dx%d fx=%.1f", w, h, self._intr.fx)

        want = label.lower().strip()
        cands = []
        for r in self.model.predict(frame, conf=self.conf, verbose=False):
            names = r.names
            for box in r.boxes:
                name = str(names[int(box.cls)]).lower()
                if not self.open_vocab and want not in name and name not in self.aliases:
                    continue
                cands.append((float(box.conf), name, [float(x) for x in box.xyxy[0]]))

        # Best box that ACTUALLY LANDS ON THE FLOOR, not merely the most confident.
        # Those differ, and the difference matters: the strongest hit is often a
        # look-alike up on a table, whose base sits above the horizon and therefore
        # cannot be on the ground at all. The floor constraint is a free
        # false-positive filter -- but only if we keep looking past the first reject.
        # A retrievable object occupies a small part of the view. An
        # open-vocabulary model asked for "toy on the floor" will happily return a
        # box covering the WHOLE frame at a low score, and that box passes the
        # floor test (its bottom edge is below the horizon), so it would yield a
        # confident, completely wrong position. Size is the cheap discriminator.
        frame_area = float(w * h)
        # A retrievable object occupies a small part of the view. Asked for "toy on
        # the floor" at a low threshold, an open-vocabulary model will happily return
        # a box covering the WHOLE frame -- and that box passes the floor test, since
        # its bottom edge is below the horizon. It would yield a confident, completely
        # wrong position. Size is the cheap discriminator.
        frame_area = float(w * h)
        best = None
        for conf, name, (x1, y1, x2, y2) in sorted(cands, key=lambda c: -c[0]):
            if (x2 - x1) * (y2 - y1) > MAX_BOX_FRACTION * frame_area:
                LOG.info("skipped %s (%.3f): box covers %.0f%% of the view",
                         name, conf, 100.0 * (x2 - x1) * (y2 - y1) / frame_area)
                continue
            if (x2 - x1) * (y2 - y1) > MAX_BOX_FRACTION * frame_area:
                LOG.info("skipped %s (%.3f): box covers %.0f%% of the view",
                         name, conf, 100.0 * (x2 - x1) * (y2 - y1) / frame_area)
                continue
            # The bottom edge is where it touches the floor; the middle of that edge
            # is the ray we solve. Not the box centre, which floats in mid-air.
            pt = floor_point((x1 + x2) / 2.0, y2, self._intr, self.mount, self.right_m)
            if pt is None:
                LOG.info("skipped %s (%.2f): base above the horizon", name, conf)
                continue
            best = (conf, name, (x1, y1, x2, y2), pt)
            break

        if best is None:
            return None
        conf, name, (x1, y1, x2, y2), (fwd, left) = best

        if self.show:
            self._draw(frame, (x1, y1, x2, y2), name, fwd, left)
        LOG.info("%s (%.2f) -> x=%.2f y=%.2f  age=%.2fs", name, conf, fwd, left, age)
        return Seen(x=fwd, y=left, age_s=age, label=name, confidence=conf)

    def _draw(self, frame, box, name, fwd, left) -> None:
        import cv2
        x1, y1, x2, y2 = (int(v) for v in box)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (60, 200, 90), 2)
        cv2.circle(frame, ((x1 + x2) // 2, y2), 5, (40, 120, 255), -1)
        cv2.putText(frame, f"{name}  {fwd:.2f}m fwd  {left:+.2f}m left",
                    (x1, max(18, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (60, 200, 90), 2)
        cv2.imshow("floor detector", frame)
        cv2.waitKey(1)

    def close(self) -> None:
        self.stream.close()


def parse_mount(s: str) -> CameraMount:
    """HEIGHT,FORWARD,PITCH_DEG in metres and degrees. Positive pitch = tilted down."""
    try:
        h, f, p = (float(v) for v in s.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(
            "--mount wants HEIGHT,FORWARD,PITCH_DEG e.g. 0.15,0.20,12") from None
    return CameraMount(height_m=h, forward_m=f, pitch_rad=math.radians(p))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stream", metavar="HOST[:PORT]", default="192.168.4.2:5577",
                    help="robot-camera stream (depth_publisher --jpeg-every N)")
    ap.add_argument("--url", default=None,
                    help="an MJPEG endpoint instead of --stream")
    ap.add_argument("--mount", type=parse_mount, required=True,
                    metavar="H,FWD,PITCH_DEG", help="MEASURE these, do not guess")
    ap.add_argument("--right", type=float, default=0.0, metavar="M",
                    help="metres the lens sits RIGHT of the robot centreline")
    ap.add_argument("--target", default="goose")
    ap.add_argument("--weights", default="yolov8n.pt")
    ap.add_argument("--conf", type=float, default=DEFAULT_CONF,
                    help="open-vocabulary scores run low; the floor and size filters "
                         "are what make a threshold this low safe")
    ap.add_argument("--prompts", default=",".join(GOOSE_PROMPTS),
                    help="comma-separated words for an open-vocabulary model")
    ap.add_argument("--hfov", type=float, default=D435I_HFOV_DEG)
    ap.add_argument("--show", action="store_true", help="draw a window while checking")
    ap.add_argument("--once", action="store_true")
    args = ap.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(name)s: %(message)s")
    if args.url:
        src = MjpegStream(args.url)
    else:
        h, _, pt = args.stream.partition(":")
        src = DepthStreamFrames(h, int(pt or 5577))
    det = FloorDetector(src, args.mount, weights=args.weights, conf=args.conf,
                        hfov_deg=args.hfov, show=args.show, right_m=args.right,
                        prompts=tuple(p.strip() for p in args.prompts.split(",") if p.strip()))
    try:
        while True:
            seen = det.locate(args.target)
            if seen is None:
                print("  (nothing)", flush=True)
            else:
                print(f"  {seen.label}: x={seen.x:+.2f} m  y={seen.y:+.2f} m  "
                      f"conf={seen.confidence:.2f}  age={seen.age_s:.2f} s", flush=True)
            if args.once:
                return 0 if seen else 1
            time.sleep(0.3)
    except KeyboardInterrupt:
        return 130
    finally:
        det.close()


if __name__ == "__main__":
    raise SystemExit(main())
