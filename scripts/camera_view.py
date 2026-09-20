#!/usr/bin/env python3
"""The robot's camera, in a browser tab, with what the detector makes of it.

    python3 scripts/camera_view.py --mount 0.20,0.15,0 --right 0.08
    python3 scripts/camera_view.py --no-detect          # just the picture

Opens http://127.0.0.1:8792 on this machine. It reads the same robot-camera
stream on :5577 that navigation and `floor_detector.py` read, so it costs the
robot nothing extra -- that port takes any number of clients.

WHY IT DRAWS THE NUMBERS. A raw video feed tells you the camera works. This
tells you the GEOMETRY works, which is the part that silently ruins everything
downstream. Each detection gets the robot-frame metres printed on it, plus a
horizon line and a floor grid, so a wrong mount measurement is visible in one
glance instead of being discovered when the robot drives past the goose.

Read it like this: put an object dead ahead of the ROBOT (not the camera) and
`y` should read about 0.00. Move it to the robot's left and `y` must go POSITIVE.
If the sign is backwards, the mount's `--right` is wrong and every coordinate is
mirrored.
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.perception.floor import (D435I_HFOV_DEG, DepthStreamFrames, GOOSE_ALIASES,
                            floor_point, intrinsics_for, parse_mount)

LOG = logging.getLogger("view")
BOUNDARY = "gooseframe"

PAGE = b"""<!doctype html><meta charset=utf8><title>Robot camera</title>
<style>
 body{background:#14161a;color:#e8eaec;font:15px/1.5 system-ui,sans-serif;margin:0;
      display:flex;flex-direction:column;align-items:center;gap:14px;padding:22px}
 h1{font-size:17px;font-weight:600;margin:0;letter-spacing:.02em}
 img{max-width:100%;border:1px solid #2d343b;border-radius:8px;background:#000}
 p{color:#939ba4;font-size:13px;margin:0;max-width:60ch;text-align:center}
 code{background:#232724;padding:1px 5px;border-radius:4px}
</style>
<h1>Robot camera &mdash; floor projection</h1>
<img src="/stream">
<p>Numbers are metres in the <b>robot</b> frame: <code>x</code> forward,
<code>y</code> left. Put something dead ahead of the robot and <code>y</code>
should read about 0. Move it to the robot's left and <code>y</code> must go
positive.</p>
"""


class Annotator:
    """Draws detections and the floor solve onto each frame."""

    def __init__(self, mount, right_m, weights, conf, hfov, aliases, detect=True):
        self.mount, self.right_m, self.hfov = mount, right_m, hfov
        self.conf = conf
        self.aliases = tuple(a.lower() for a in aliases)
        self.model = None
        if detect:
            from ultralytics import YOLO
            self.model = YOLO(weights)

    def draw(self, frame):
        import cv2
        h, w = frame.shape[:2]
        intr = intrinsics_for(w, h, self.hfov)

        # The horizon: above it no ray ever meets the floor, so nothing there can
        # be placed. Drawing it makes "why is that object ignored" obvious.
        # denom = ry*cos(p) + sin(p) is zero at ry = -tan(p), i.e. this row.
        hy = int(round(intr.cy - intr.fy * math.tan(self.mount.pitch_rad)))
        if 0 <= hy < h:
            cv2.line(frame, (0, hy), (w, hy), (90, 90, 110), 1)
            cv2.putText(frame, "horizon", (8, max(12, hy - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (140, 140, 160), 1)

        # Range rings along the centre column, so distance has a visual reference.
        for metres in (0.5, 1.0, 2.0, 3.0):
            ry = self.mount.height_m / metres
            v = intr.cy + ry * intr.fy
            if hy < v < h:
                v = int(v)
                cv2.line(frame, (0, v), (w, v), (48, 56, 64), 1)
                cv2.putText(frame, f"{metres:g}m", (w - 42, v - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (110, 120, 132), 1)

        if self.model is None:
            return frame

        for r in self.model.predict(frame, conf=self.conf, verbose=False):
            for b in r.boxes:
                name = str(r.names[int(b.cls)]).lower()
                x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
                hit = name in self.aliases
                col = (90, 220, 130) if hit else (120, 130, 145)
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), col,
                              2 if hit else 1)
                pt = floor_point((x1 + x2) / 2.0, y2, intr, self.mount, self.right_m)
                if pt is None:
                    label = f"{name}  (above horizon)"
                else:
                    label = f"{name}  x={pt[0]:+.2f} y={pt[1]:+.2f}"
                    cv2.circle(frame, (int((x1 + x2) / 2), int(y2)), 4, (60, 120, 255), -1)
                cv2.putText(frame, label, (int(x1), max(14, int(y1) - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.46, col, 1)
        return frame


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stream", default="192.168.4.2:5577")
    ap.add_argument("--mount", type=parse_mount, default=parse_mount("0.20,0.15,0"))
    ap.add_argument("--right", type=float, default=0.08)
    ap.add_argument("--port", type=int, default=8792)
    ap.add_argument("--weights", default="yolov8n.pt")
    ap.add_argument("--conf", type=float, default=0.20)
    ap.add_argument("--hfov", type=float, default=D435I_HFOV_DEG)
    ap.add_argument("--no-detect", action="store_true")
    args = ap.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(name)s: %(message)s")
    host, _, port = args.stream.partition(":")
    src = DepthStreamFrames(host, int(port or 5577)).start()
    ann = Annotator(args.mount, args.right, args.weights, args.conf, args.hfov,
                    GOOSE_ALIASES, detect=not args.no_detect)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            import cv2
            if self.path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(PAGE)))
                self.end_headers()
                self.wfile.write(PAGE)
                return
            if self.path != "/stream":
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type",
                             f"multipart/x-mixed-replace; boundary={BOUNDARY}")
            self.end_headers()
            last = 0.0
            try:
                while True:
                    frame, t = src.latest()
                    if frame is None or t == last:
                        time.sleep(0.03)
                        continue
                    last = t
                    ok, jpg = cv2.imencode(".jpg", ann.draw(frame.copy()),
                                           [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if not ok:
                        continue
                    b = jpg.tobytes()
                    self.wfile.write(f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
                                     f"Content-Length: {len(b)}\r\n\r\n".encode())
                    self.wfile.write(b)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

    srv = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    srv.daemon_threads = True
    print(f"  camera view on http://127.0.0.1:{args.port}", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        return 130
    finally:
        src.close()
        srv.server_close()


if __name__ == "__main__":
    raise SystemExit(main())
