#!/usr/bin/env python3
"""
Stream one or more V4L2 cameras over WebSocket as JPEG frames.

Runs on the RDK S100 with system Python 3.10 (which already has cv2 and
websockets). Clients - the Raspberry Pi, a phone, a laptop browser - connect
to the board, so this side needs no knowledge of their addresses.

    python3 camera_ws_server.py \
        --camera birdseye=/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_...-video-index0 \
        --camera claw=/dev/v4l/by-id/usb-hiwonder-...-video-index0 \
        --camera bottom=/dev/v4l/by-id/usb-Intel_R__RealSense_...-video-index2

    ws://<board>:8765/<name>   binary JPEG frames, one per message
    http://<board>:8766        viewer: all cameras, click to enlarge
    http://<board>:8766/list   JSON list of camera names

Per-camera overrides:  --camera name=/dev/path:1280x720@15

Cameras are opened lazily and released once the last viewer disconnects, so
nothing holds /dev/video* while inference or recording is running. Still stop
the service outright before a LeRobot recording session if you want certainty
that nothing can race for the device.
"""

import argparse
import asyncio
import contextlib
import json
import logging
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

try:  # websockets >= 14 moved the asyncio server
    from websockets.asyncio.server import serve
except ImportError:  # pragma: no cover - older websockets
    from websockets import serve  # type: ignore

log = logging.getLogger("camstream")

VIEWER_HTML = """<!doctype html>
<html><head><meta charset="utf-8"><title>Board cameras</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
  :root { color-scheme: dark; }
  body { margin:0; background:#111; color:#eee; font:14px system-ui,sans-serif; padding:12px; }
  h1 { font-size:15px; font-weight:600; margin:0 0 12px; opacity:.7; }
  .grid { display:grid; gap:12px; grid-template-columns:repeat(auto-fit,minmax(320px,1fr)); }
  figure { margin:0; background:#000; border-radius:8px; overflow:hidden; }
  img { width:100%; display:block; background:#000; aspect-ratio:4/3; object-fit:contain; }
  figcaption { padding:6px 10px; font-size:13px; display:flex; justify-content:space-between; opacity:.8; }
  .fps { font-variant-numeric:tabular-nums; }
</style></head><body>
<h1>Board cameras &mdash; __NAMES__</h1>
<div class="grid" id="g"></div>
<script>
const names = __NAMES_JSON__, port = __WSPORT__, g = document.getElementById('g');
for (const name of names) {
  const fig = document.createElement('figure');
  fig.innerHTML = `<img alt="${name}"><figcaption><span>${name}</span><span class="fps">…</span></figcaption>`;
  g.appendChild(fig);
  const img = fig.querySelector('img'), fps = fig.querySelector('.fps');
  const ws = new WebSocket(`ws://${location.hostname}:${port}/${name}`);
  ws.binaryType = 'blob';
  let n = 0, t0 = performance.now(), url = null;
  ws.onmessage = e => {
    if (url) URL.revokeObjectURL(url);
    url = URL.createObjectURL(e.data);
    img.src = url;
    if (++n % 15 === 0) { const t = performance.now(); fps.textContent = (15000/(t-t0)).toFixed(1)+' fps'; t0 = t; }
  };
  ws.onclose = () => fps.textContent = 'disconnected';
  ws.onerror = () => fps.textContent = 'error';
}
</script></body></html>
"""


class Camera:
    """One capture device, opened only while it has subscribers.

    Keeps just the newest frame: a slow client must never make the stream fall
    behind real time. Dropping is always better than queueing.
    """

    def __init__(self, name, path, width, height, fps, quality, linger=3.0):
        self.name, self.path = name, path
        self.width, self.height, self.fps = width, height, fps
        self.quality, self.linger = quality, linger

        self.subscribers: set = set()
        self._lock = threading.Lock()
        self._jpeg = None
        self._seq = 0
        self._thread = None
        self._stop = threading.Event()

    # -- lifecycle ---------------------------------------------------------
    def acquire(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True, name=f"cap-{self.name}")
        self._thread.start()

    def release(self) -> None:
        self._stop.set()

    @property
    def running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    # -- capture -----------------------------------------------------------
    def _open(self):
        cap = cv2.VideoCapture(self.path, cv2.CAP_V4L2)
        if not cap.isOpened():
            log.error("[%s] cannot open %s (in use, or wrong node?)", self.name, self.path)
            return None
        # Ask for MJPEG so the camera encodes in hardware. Raw YUYV at
        # 640x480x30 is ~460KB/frame; three of those exceeds what USB 2 can
        # actually deliver, and these share a hub with the servo adapter.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        log.info(
            "[%s] opened %s -> %dx%d",
            self.name,
            self.path,
            int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        )
        return cap

    def _loop(self) -> None:
        cap = self._open()
        if cap is None:
            return
        params = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        idle_since = None
        misses = 0
        try:
            while not self._stop.is_set():
                if not self.subscribers:
                    idle_since = idle_since or time.time()
                    if time.time() - idle_since > self.linger:
                        log.info("[%s] no viewers; releasing device", self.name)
                        return
                    time.sleep(0.2)
                    continue
                idle_since = None

                ok, frame = cap.read()
                if not ok:
                    misses += 1
                    if misses % 30 == 1:
                        log.warning("[%s] frame grab failed (%d)", self.name, misses)
                    time.sleep(0.05)
                    continue
                misses = 0
                ok, buf = cv2.imencode(".jpg", frame, params)
                if not ok:
                    continue
                with self._lock:
                    self._jpeg = buf.tobytes()
                    self._seq += 1
        finally:
            cap.release()
            with self._lock:
                self._jpeg, self._seq = None, 0
            log.info("[%s] released", self.name)

    def latest(self):
        with self._lock:
            return self._jpeg, self._seq


def parse_camera(spec: str, d_w: int, d_h: int, d_fps: int):
    """name=/dev/path  or  name=/dev/path:1280x720@15"""
    if "=" not in spec:
        raise argparse.ArgumentTypeError(f"expected name=/dev/path, got {spec!r}")
    name, _, rest = spec.partition("=")
    w, h, fps = d_w, d_h, d_fps
    # Split only on the last ':' so /dev paths containing ':' still work.
    if ":" in rest and "x" in rest.rsplit(":", 1)[-1]:
        rest, _, tail = rest.rpartition(":")
        dims, _, f = tail.partition("@")
        w, _, h = dims.partition("x")
        w, h = int(w), int(h)
        if f:
            fps = int(f)
    return name.strip(), rest.strip(), w, h, fps


def start_viewer_server(http_port: int, ws_port: int, names: list[str]) -> ThreadingHTTPServer:
    page = (
        VIEWER_HTML.replace("__WSPORT__", str(ws_port))
        .replace("__NAMES_JSON__", json.dumps(names))
        .replace("__NAMES__", ", ".join(names))
    ).encode()
    listing = json.dumps({"cameras": names, "ws_port": ws_port}).encode()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):  # noqa: N802
            body, ctype = (listing, "application/json") if self.path.startswith("/list") else (page, "text/html; charset=utf-8")
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *_a):
            pass

    srv = ThreadingHTTPServer(("0.0.0.0", http_port), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


async def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", action="append", default=[], metavar="NAME=PATH[:WxH@FPS]")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--send-fps", type=float, default=15.0)
    ap.add_argument("--quality", type=int, default=75)
    ap.add_argument("--linger", type=float, default=3.0, help="seconds to hold a device after the last viewer leaves")
    ap.add_argument("--ws-port", type=int, default=8765)
    ap.add_argument("--http-port", type=int, default=8766)
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    specs = args.camera or ["cam0=/dev/video0"]
    cams = {}
    for spec in specs:
        name, path, w, h, fps = parse_camera(spec, args.width, args.height, args.fps)
        cams[name] = Camera(name, path, w, h, fps, args.quality, args.linger)
    log.info("cameras: %s", ", ".join(f"{n} -> {c.path}" for n, c in cams.items()))

    def ws_path(ws) -> str:
        req = getattr(ws, "request", None)  # websockets >= 14
        raw = getattr(req, "path", None) or getattr(ws, "path", "/")
        return raw.strip("/").split("?")[0]

    async def handler(ws):
        name = ws_path(ws)
        if not name and len(cams) == 1:
            name = next(iter(cams))
        cam = cams.get(name)
        if cam is None:
            await ws.close(code=1008, reason=f"unknown camera {name!r}; have {list(cams)}")
            return
        cam.subscribers.add(ws)
        cam.acquire()
        log.info("[%s] viewer joined (%d)", name, len(cam.subscribers))
        try:
            await ws.wait_closed()
        finally:
            cam.subscribers.discard(ws)
            log.info("[%s] viewer left (%d)", name, len(cam.subscribers))

    if args.http_port:
        start_viewer_server(args.http_port, args.ws_port, list(cams))
        log.info("viewer page on http://0.0.0.0:%d", args.http_port)

    async with serve(handler, "0.0.0.0", args.ws_port, max_queue=1):
        log.info("websocket stream on ws://0.0.0.0:%d/<name>", args.ws_port)
        interval = 1.0 / args.send_fps
        last = {n: -1 for n in cams}
        try:
            while True:
                await asyncio.sleep(interval)
                for name, cam in cams.items():
                    if not cam.subscribers:
                        continue
                    jpeg, seq = cam.latest()
                    if jpeg is None or seq == last[name]:
                        continue  # nothing new; never resend a stale frame
                    last[name] = seq
                    for ws in list(cam.subscribers):
                        with contextlib.suppress(Exception):
                            asyncio.create_task(ws.send(jpeg))
        finally:
            for cam in cams.values():
                cam.release()


if __name__ == "__main__":
    with contextlib.suppress(KeyboardInterrupt):
        asyncio.run(main())
