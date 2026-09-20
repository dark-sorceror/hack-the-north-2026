#!/usr/bin/env python3
"""MJPEG of the robot's camera, for the teleop dashboard.

    python3 scripts/camera_stream.py                      # /dev/video4, port 8790
    python3 scripts/camera_stream.py --device /dev/video4 --size 640x480 --fps 10

Eyes on the robot, nothing more: the dashboard shows the map, the route and the
bubble, but none of that says whether the thing in front of it is a chair leg
or a person's foot. This is a view, not a perception path -- obstacle points
still come from the lidar and from perception's CameraObstacles.

WHY FFMPEG. Encoding JPEG in pure Python is out, and the Pi side takes no
dependencies. ffmpeg is already on the Pi, reads V4L2 directly and writes a
stream of JPEGs to a pipe, so this process only has to find the frame
boundaries (SOI ffd8 ... EOI ffd9) and fan them out over HTTP. Stdlib only.

ONE READER, MANY VIEWERS. A camera node can be opened once, so ffmpeg is
started on demand and shared: the newest frame is kept and every connected
browser is served from it. Nobody watching for idle_s and ffmpeg is stopped,
which matters because holding /dev/videoN keeps that stream from anything else
(the RealSense SDK included).

THE BRIDGE IS UNTOUCHED. This is a separate process on its own port, so a
crash here cannot affect driving, the watchdog or the bubble.
"""

from __future__ import annotations

import argparse
import logging
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

LOG = logging.getLogger("cam")
SOI, EOI = b"\xff\xd8", b"\xff\xd9"
BOUNDARY = "retrieverframe"


class Camera:
    """ffmpeg on demand; keeps the newest JPEG for whoever is watching."""

    def __init__(self, device: str, size: str, fps: int, quality: int,
                 input_format: str | None, rotate: int = 0, idle_s: float = 10.0) -> None:
        self.device, self.size, self.fps = device, size, fps
        self.quality, self.input_format, self.idle_s = quality, input_format, idle_s
        self.rotate = rotate % 360
        self.frame: bytes | None = None
        self.frame_at = 0.0
        self.error: str | None = None
        self.viewers = 0
        self._proc: subprocess.Popen | None = None
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._new = threading.Condition(self._lock)

    def command(self) -> list[str]:
        args = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "v4l2"]
        if self.input_format:
            args += ["-input_format", self.input_format]
        args += ["-video_size", self.size, "-framerate", str(self.fps), "-i", self.device]
        # Mounted upside down, the picture comes out upside down. Rotating here
        # costs nothing (the frame is being re-encoded anyway) and keeps every
        # viewer consistent, unlike a CSS transform on one page.
        vf = {90: "transpose=1", 180: "hflip,vflip", 270: "transpose=2"}.get(self.rotate)
        if vf:
            args += ["-vf", vf]
        args += ["-f", "mjpeg", "-q:v", str(self.quality), "-"]
        return args

    def watch(self) -> None:
        with self._lock:
            self.viewers += 1
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, name="ffmpeg", daemon=True)
                self._thread.start()

    def unwatch(self) -> None:
        with self._lock:
            self.viewers = max(0, self.viewers - 1)

    def _run(self) -> None:
        try:
            self._proc = subprocess.Popen(self.command(), stdout=subprocess.PIPE,
                                          stderr=subprocess.PIPE, bufsize=0)
        except OSError as exc:
            with self._new:
                self.error = f"can't start ffmpeg: {exc}"
                self._new.notify_all()
            return
        LOG.info("ffmpeg reading %s at %s", self.device, self.size)
        buf = b""
        idle_since = time.monotonic()
        try:
            while True:
                chunk = self._proc.stdout.read(65536)        # type: ignore[union-attr]
                if not chunk:
                    err = (self._proc.stderr.read() or b"").decode(errors="replace").strip()
                    with self._new:
                        self.error = err.splitlines()[-1] if err else "the camera stopped"
                        self._new.notify_all()
                    LOG.warning("ffmpeg ended: %s", self.error)
                    return
                buf += chunk
                while True:                                  # whole frames only
                    start = buf.find(SOI)
                    end = buf.find(EOI, start + 2) if start >= 0 else -1
                    if start < 0 or end < 0:
                        break
                    with self._new:
                        self.frame = buf[start:end + 2]
                        self.frame_at = time.monotonic()
                        self.error = None
                        self._new.notify_all()
                    buf = buf[end + 2:]
                if len(buf) > 4_000_000:                     # never grow without bound
                    buf = b""
                with self._lock:
                    watching = self.viewers
                now = time.monotonic()
                if watching:
                    idle_since = now
                elif now - idle_since > self.idle_s:
                    LOG.info("nobody watching: releasing %s", self.device)
                    return
        finally:
            p, self._proc = self._proc, None
            if p is not None:
                p.terminate()
                try:
                    p.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    p.kill()
            with self._lock:
                self._thread = None
                self.frame = None

    def next_frame(self, last_at: float, timeout: float = 5.0) -> tuple[bytes | None, float]:
        with self._new:
            if not self._new.wait_for(lambda: (self.frame is not None and self.frame_at > last_at)
                                      or self.error is not None, timeout):
                return None, last_at
            if self.error is not None and self.frame is None:
                return None, last_at
            return self.frame, self.frame_at


def make_handler(cam: Camera):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):                   # quiet: one line per connect
            LOG.debug(fmt, *args)

        def do_GET(self) -> None:
            if self.path.startswith("/snapshot"):
                return self._snapshot()
            if self.path == "/" or self.path.startswith("/stream"):
                return self._stream()
            self.send_error(404)

        def _cors(self) -> None:
            # The dashboard is served from the laptop; this stream is on the Pi.
            self.send_header("Access-Control-Allow-Origin", "*")

        def _snapshot(self) -> None:
            cam.watch()
            try:
                frame, _ = cam.next_frame(0.0, timeout=6.0)
            finally:
                cam.unwatch()
            if frame is None:
                self.send_error(503, cam.error or "no frame")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(frame)))
            self.send_header("Cache-Control", "no-store")
            self._cors()
            self.end_headers()
            self.wfile.write(frame)

        def _stream(self) -> None:
            cam.watch()
            self.send_response(200)
            self.send_header("Content-Type",
                             f"multipart/x-mixed-replace; boundary={BOUNDARY}")
            self.send_header("Cache-Control", "no-store")
            self._cors()
            self.end_headers()
            last = 0.0
            try:
                while True:
                    frame, last = cam.next_frame(last, timeout=10.0)
                    if frame is None:
                        if cam.error:
                            return
                        continue
                    self.wfile.write(f"--{BOUNDARY}\r\n".encode())
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass                                          # the tab was closed
            finally:
                cam.unwatch()

    return Handler


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--device", default="/dev/video4",
                    help="V4L2 node (default /dev/video4, the D435i's colour stream; "
                         "`v4l2-ctl --list-devices` lists them)")
    ap.add_argument("--size", default="640x480")
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--quality", type=int, default=7, help="JPEG q:v, 2 best .. 31 worst")
    ap.add_argument("--input-format", default="yuyv422",
                    help="V4L2 pixel format (the D435i's colour is yuyv422; a webcam "
                         "that offers mjpeg is cheaper: pass mjpeg)")
    ap.add_argument("--rotate", type=int, choices=[0, 90, 180, 270], default=180,
                    help="degrees to rotate the picture (default 180: the camera is "
                         "mounted inverted)")
    ap.add_argument("--port", type=int, default=8790)
    ap.add_argument("--host", default="0.0.0.0")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s",
                        datefmt="%H:%M:%S")
    cam = Camera(args.device, args.size, args.fps, args.quality, args.input_format,
                 rotate=args.rotate)
    srv = ThreadingHTTPServer((args.host, args.port), make_handler(cam))
    srv.daemon_threads = True
    print(f"camera on http://{args.host}:{args.port}/stream.mjpg  ({args.device}, "
          f"{args.size} @ {args.fps} fps)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
