#!/usr/bin/env python3
"""
Stream a V4L2 camera over WebSocket as JPEG frames.

Runs on the RDK S100 with the system Python 3.10 (which already has cv2 and
websockets). Clients - the Raspberry Pi, a phone, a laptop browser - connect
to the board, so this side needs to know nothing about them.

    python3 camera_ws_server.py                     # defaults below
    python3 camera_ws_server.py --width 1280 --height 720 --fps 15

    ws://<board-ip>:8765      binary JPEG frames, one per message
    http://<board-ip>:8766    a test viewer page

NOTE: this holds /dev/video0 exclusively. LeRobot recording cannot open the
same camera while this is running - stop it first (Ctrl-C, or kill the pid).
That is fine for the intended design, where nav and manipulation are separate
phases, but it is not a limitation you can work around without v4l2loopback.
"""

import argparse
import asyncio
import contextlib
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

VIEWER_HTML = b"""<!doctype html>
<html><head><meta charset="utf-8"><title>Board camera</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
  :root { color-scheme: dark; }
  body { margin:0; background:#111; color:#eee; font:14px system-ui,sans-serif;
         display:flex; flex-direction:column; align-items:center; gap:8px; padding:12px; }
  img { max-width:100%; border-radius:8px; background:#000; }
  #s { font-variant-numeric: tabular-nums; opacity:.8; }
</style></head><body>
<img id="v" alt="camera stream">
<div id="s">connecting...</div>
<script>
  const img = document.getElementById('v'), st = document.getElementById('s');
  const ws = new WebSocket(`ws://${location.hostname}:__WSPORT__`);
  ws.binaryType = 'blob';
  let n = 0, t0 = performance.now(), url = null;
  ws.onmessage = e => {
    if (url) URL.revokeObjectURL(url);
    url = URL.createObjectURL(e.data);
    img.src = url;
    if (++n % 15 === 0) {
      const now = performance.now();
      st.textContent = `${(15000/(now-t0)).toFixed(1)} fps`;
      t0 = now;
    }
  };
  ws.onopen  = () => st.textContent = 'connected';
  ws.onclose = () => st.textContent = 'disconnected';
</script></body></html>
"""


class Camera:
    """Grabs frames in a background thread and keeps only the newest one.

    Holding just the latest frame is deliberate: a slow client must never make
    the stream fall behind real time. Dropping is always better than queueing.
    """

    def __init__(self, device: str, width: int, height: int, fps: int, quality: int):
        self._quality = quality
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._seq = 0
        self._stop = threading.Event()

        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise SystemExit(f"could not open {device} - is it in use? (lsof {device})")
        # Ask the camera for MJPEG rather than raw YUYV. The C920 encodes in
        # hardware, which is the difference between ~460KB and ~30KB per frame
        # over USB, and is what makes 720p viable at all on a shared bus.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual = (
            int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            self.cap.get(cv2.CAP_PROP_FPS),
        )
        log.info("camera %s -> %dx%d @ %.0f fps requested", device, *actual)

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        params = [int(cv2.IMWRITE_JPEG_QUALITY), self._quality]
        misses = 0
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if not ok:
                misses += 1
                if misses % 30 == 1:
                    log.warning("frame grab failed (%d)", misses)
                time.sleep(0.05)
                continue
            misses = 0
            ok, buf = cv2.imencode(".jpg", frame, params)
            if not ok:
                continue
            with self._lock:
                self._jpeg = buf.tobytes()
                self._seq += 1

    def latest(self) -> tuple[bytes | None, int]:
        with self._lock:
            return self._jpeg, self._seq

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)
        self.cap.release()


def start_viewer_server(http_port: int, ws_port: int) -> ThreadingHTTPServer:
    page = VIEWER_HTML.replace(b"__WSPORT__", str(ws_port).encode())

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):  # noqa: N802
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(page)))
            self.end_headers()
            self.wfile.write(page)

        def log_message(self, *_args):  # keep the console readable
            pass

    srv = ThreadingHTTPServer(("0.0.0.0", http_port), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


async def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30, help="capture rate requested from the camera")
    ap.add_argument("--send-fps", type=float, default=15.0, help="rate frames are pushed to clients")
    ap.add_argument("--quality", type=int, default=75, help="JPEG quality, 1-100")
    ap.add_argument("--ws-port", type=int, default=8765)
    ap.add_argument("--http-port", type=int, default=8766, help="0 disables the test viewer page")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    cam = Camera(args.device, args.width, args.height, args.fps, args.quality)
    clients: set = set()

    async def handler(ws):
        peer = getattr(ws, "remote_address", ("?", 0))
        clients.add(ws)
        log.info("client connected: %s (total %d)", peer[0], len(clients))
        try:
            await ws.wait_closed()
        finally:
            clients.discard(ws)
            log.info("client gone: %s (total %d)", peer[0], len(clients))

    if args.http_port:
        start_viewer_server(args.http_port, args.ws_port)
        log.info("viewer page on http://0.0.0.0:%d", args.http_port)

    async with serve(handler, "0.0.0.0", args.ws_port, max_queue=1):
        log.info("websocket stream on ws://0.0.0.0:%d", args.ws_port)
        interval = 1.0 / args.send_fps
        last_seq = -1
        try:
            while True:
                await asyncio.sleep(interval)
                if not clients:
                    continue
                jpeg, seq = cam.latest()
                if jpeg is None or seq == last_seq:
                    continue  # nothing new; don't resend a stale frame
                last_seq = seq
                # Fire and forget. A client that cannot keep up simply misses
                # frames rather than backing the whole stream up.
                for ws in list(clients):
                    with contextlib.suppress(Exception):
                        asyncio.create_task(ws.send(jpeg))
        finally:
            cam.close()


if __name__ == "__main__":
    with contextlib.suppress(KeyboardInterrupt):
        asyncio.run(main())
