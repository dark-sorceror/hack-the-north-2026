"""The QNX supervisor's AI: person detection -> SAFE/STOP verdicts for lineguard.

Runs tflite-runtime (from oss.qnx.com, `apk add python3-tflite-runtime`) on
every frame and sends lineguard one datagram per frame:

    SAFE <seq> <capture_monotonic_s>
    STOP <capture_monotonic_s> <reason>

STOP when a person is closer than --stop-m (depth from the camera), or, with
no usable depth, when their box is at least --close-frac of the frame height.
SAFE otherwise. The detector is allowed to be
slow or to die: lineguard stops the robot when SAFE verdicts stop arriving,
so every failure in here (a crash, a hung model, a stalled camera, a frame
it cannot read) ends in STOP without this file having to get it right.

Frames come from camtap (a RealSense D435(i) through the QNX Sensor Framework,
colour + depth in /dev/shmem/camtap), or from a video file or an image glob
paced to --fps like a camera.

    python3 detector.py --model ssd_mobilenet_v1_quant.tflite --source camtap
    python3 detector.py --model ssd_mobilenet_v1_quant.tflite --source clip.avi --loop
"""
from __future__ import annotations

import argparse
import glob
import mmap
import socket
import struct
import sys
import time

import cv2
import numpy as np
from tflite_runtime.interpreter import Interpreter

PERSON = 0  # COCO class id in both SSD-MobileNet and EfficientDet-Lite label maps

# The D435's colour camera sees a narrower cone than its depth camera (about
# 69x42 vs 87x58 degrees, both 16:9), so a point in the colour image sits
# nearer the centre of the depth image. tan(half colour FOV) / tan(half depth
# FOV). Approximate: good enough to find a person's depth, not to measure them.
D435_COLOR_TO_DEPTH = (0.73, 0.70)  # (x, y)


class Detector:
    """A TFLite detector with the TFLite_Detection_PostProcess outputs:
    boxes [1,N,4] (ymin, xmin, ymax, xmax, normalised), classes [1,N],
    scores [1,N], count [1]."""

    def __init__(self, model: str, threads: int) -> None:
        self.it = Interpreter(model_path=model, num_threads=threads)
        self.it.allocate_tensors()
        self.inp = self.it.get_input_details()[0]
        _, self.h, self.w, _ = self.inp["shape"]
        self.outs = [d["index"] for d in self.it.get_output_details()]

    def people(self, bgr: np.ndarray, min_score: float) -> list[tuple[float, np.ndarray]]:
        """(score, box) for each person; box is (ymin, xmin, ymax, xmax), 0..1."""
        x = cv2.resize(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), (self.w, self.h))
        if self.inp["dtype"] == np.float32:
            x = (x.astype(np.float32) - 127.5) / 127.5
        self.it.set_tensor(self.inp["index"], np.expand_dims(x.astype(self.inp["dtype"]), 0))
        self.it.invoke()
        arrs = [self.it.get_tensor(i) for i in self.outs]
        boxes = next(a for a in arrs if a.ndim == 3)[0]
        a, b = (a[0] for a in arrs if a.ndim == 2)
        # Class ids are whole numbers; scores are not. Output order differs by model.
        classes, scores = (a, b) if np.all(a == np.round(a)) else (b, a)
        return [(float(s), np.clip(bx, 0.0, 1.0))
                for c, s, bx in zip(classes, scores, boxes)
                if int(c) == PERSON and s >= min_score]


def person_distance_m(box: np.ndarray, depth: np.ndarray) -> float | None:
    """Distance to a person from the depth frame: the median of the valid
    depths in the middle half of their box, mapped from colour to
    depth coordinates. None when the depth there is missing."""
    y0, x0, y1, x1 = box
    cy, cx, hh, hw = (y0 + y1) / 2, (x0 + x1) / 2, (y1 - y0) / 4, (x1 - x0) / 4
    sx, sy = D435_COLOR_TO_DEPTH
    H, W = depth.shape
    ys = [int(np.clip((0.5 + (v - 0.5) * sy) * H, 0, H - 1)) for v in (cy - hh, cy + hh)]
    xs = [int(np.clip((0.5 + (v - 0.5) * sx) * W, 0, W - 1)) for v in (cx - hw, cx + hw)]
    patch = depth[ys[0]:ys[1] + 1, xs[0]:xs[1] + 1]
    valid = patch[(patch > 0) & (patch < 65535)]  # 0 and 65535: no reading
    if valid.size < 20:
        return None
    return float(np.median(valid)) / 1000.0


class CamTap:
    """Reads the newest colour + depth pair that camtap.c publishes in
    /dev/shmem/camtap. Mirrors camtap_shm_t: a seqlock per stream, odd while
    camtap is copying, so a read that saw the counter move is retried."""

    COLOR_OFF, COLOR_MAX = 4096, 1920 * 1080 * 2
    DEPTH_OFF = 4096 + 1920 * 1080 * 2
    SIZE = DEPTH_OFF + 1280 * 720 * 2  # camtap.c SHM_SIZE; QNX won't map length 0
    SLOT = struct.Struct("<IIII d")  # seq, width, height, frames, mono_s
    HEAD = struct.Struct("<II")      # magic, version

    def __init__(self, path: str = "/dev/shmem/camtap") -> None:
        with open(path, "rb") as f:
            self.mm = mmap.mmap(f.fileno(), self.SIZE, prot=mmap.PROT_READ)
        magic, _ = self.HEAD.unpack_from(self.mm, 0)
        if magic != 0x43414D54:
            raise RuntimeError(f"{path} is not a camtap buffer (is camtap running?)")
        self.slot_off = (self.HEAD.size, self.HEAD.size + self.SLOT.size)

    def _read(self, which: int, off: int):
        for _ in range(50):
            seq, w, h, n, t = self.SLOT.unpack_from(self.mm, self.slot_off[which])
            if w == 0:
                return None             # this stream has not delivered a frame
            if seq % 2:
                continue                # mid-copy: a copy takes about a millisecond
            data = bytes(self.mm[off:off + w * h * 2])
            if self.SLOT.unpack_from(self.mm, self.slot_off[which])[0] == seq:
                return data, w, h, n, t
        return None

    def latest(self):
        """(bgr, depth_mm or None, frame number, capture time) or None."""
        c = self._read(0, self.COLOR_OFF)
        if c is None:
            return None
        data, w, h, n, t = c
        yuyv = np.frombuffer(data, np.uint8).reshape(h, w, 2)
        bgr = cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUYV)
        d = self._read(1, self.DEPTH_OFF)
        depth = None
        if d is not None and abs(d[4] - t) < 0.1:  # a depth frame from the same moment
            depth = np.frombuffer(d[0], np.uint16).reshape(d[2], d[1])
        return bgr, depth, n, t


def camtap_frames(stale_s: float = 0.5):
    """Yield (bgr, depth, capture time) for each new camtap frame. Raises if
    the camera stops delivering: lineguard stops the robot either way."""
    tap = CamTap()
    last_n, last_new = -1, time.monotonic()
    while True:
        got = tap.latest()
        if got is not None and got[2] != last_n:
            bgr, depth, last_n, t = got
            last_new = time.monotonic()
            yield bgr, depth, t
            continue
        if time.monotonic() - last_new > stale_s:
            raise RuntimeError(f"camera stalled: no new frame for {stale_s:.1f} s")
        time.sleep(0.002)


def frames(source: str, loop: bool):
    """Yield BGR frames from a video file or an image glob, forever if loop."""
    while True:
        paths = sorted(glob.glob(source))
        if len(paths) > 1 or (paths and not source.lower().endswith((".avi", ".mp4", ".mov", ".mkv"))):
            for p in paths:
                img = cv2.imread(p)
                if img is None:
                    raise RuntimeError(f"cannot read {p}")
                yield img
        else:
            cap = cv2.VideoCapture(source)
            if not cap.isOpened():
                raise RuntimeError(f"cannot open {source}")
            while True:
                ok, img = cap.read()
                if not ok:
                    break
                yield img
            cap.release()
        if not loop:
            return


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--model", required=True)
    ap.add_argument("--source", required=True, help="camtap, a video file, or an image glob")
    ap.add_argument("--loop", action="store_true", help="replay the source forever")
    ap.add_argument("--fps", type=float, default=15.0, help="pace frames like a camera")
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--min-score", type=float, default=0.5)
    ap.add_argument("--stop-m", type=float, default=1.0,
                    help="STOP when a person is closer than this (needs depth)")
    ap.add_argument("--close-frac", type=float, default=0.45,
                    help="without depth: STOP when a person's box is this tall a fraction of the frame")
    ap.add_argument("--sock", default="/tmp/lineguard.sock")
    ap.add_argument("--stats-s", type=float, default=2.0)
    args = ap.parse_args()

    det = Detector(args.model, args.threads)
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

    def send(msg: str) -> None:
        try:
            sock.sendto(msg.encode(), args.sock)
        except OSError as exc:  # lineguard not running: it would be STOP anyway
            if not getattr(send, "warned", False):
                print(f"warning: cannot reach lineguard at {args.sock}: {exc}", flush=True)
                send.warned = True

    if args.source == "camtap":
        source = camtap_frames()
    else:
        def paced():
            period, next_frame = 1.0 / args.fps, time.monotonic()
            for img in frames(args.source, args.loop):
                now = time.monotonic()
                if now < next_frame:
                    time.sleep(next_frame - now)
                next_frame = max(next_frame + period, time.monotonic())
                yield img, None, time.monotonic()
        source = paced()

    seq, lat, verdicts, last_verdict = 0, [], {"SAFE": 0, "STOP": 0}, None
    cpu0, wall0 = time.process_time(), time.monotonic()
    try:
        for img, depth, captured in source:
            t0 = time.monotonic()
            people = det.people(img, args.min_score)
            lat.append((time.monotonic() - t0) * 1e3)

            # Per person: distance from depth if there is any, else box height.
            why = []
            for s, box in people:
                d = person_distance_m(box, depth) if depth is not None else None
                if d is not None and d < args.stop_m:
                    why.append(f"person at {d:.2f} m (score {s:.2f})")
                elif d is None and box[2] - box[0] >= args.close_frac:
                    why.append(f"person box h={box[2] - box[0]:.2f}, no depth (score {s:.2f})")
            if why:
                send(f"STOP {captured:.6f} {why[0]}")
                verdict = "STOP"
            else:
                send(f"SAFE {seq} {captured:.6f}")
                verdict = "SAFE"
            verdicts[verdict] += 1
            seq += 1
            if verdict != last_verdict:
                dists = [person_distance_m(b, depth) if depth is not None else None for _, b in people]
                near = min((d for d in dists if d is not None), default=None)
                print(f"{time.strftime('%H:%M:%S')} {verdict}  people={len(people)}"
                      + (f" nearest={near:.2f} m" if near is not None else "")
                      + (f"  ({why[0]})" if why else ""), flush=True)
                last_verdict = verdict

            wall = time.monotonic() - wall0
            if wall >= args.stats_s:
                a = np.array(lat)
                cpu = (time.process_time() - cpu0) / wall
                print(f"  {len(a) / wall:4.1f} fps  infer p50={np.percentile(a, 50):.1f} ms "
                      f"p99={np.percentile(a, 99):.1f} ms  cpu={100 * cpu:.0f}% of a core  "
                      f"SAFE={verdicts['SAFE']} STOP={verdicts['STOP']}", flush=True)
                lat, cpu0, wall0 = [], time.process_time(), time.monotonic()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        # lineguard would stop on silence within its deadline; say why sooner.
        send(f"STOP {time.monotonic():.6f} detector error: {exc}")
        raise
    return 0


if __name__ == "__main__":
    sys.exit(main())
