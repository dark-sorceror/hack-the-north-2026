#!/usr/bin/env python3
"""Live lidar radar in the browser.

    <env-with-rplidar>/bin/python scripts/lidar_view.py [PORT]    ->  http://localhost:8790

A bench tool for SEEING the sensor, nothing more. It deliberately uses the
third-party `rplidar-roboticia` library rather than the robot's own lidar
driver, so when the two disagree you know which one to suspect. The server is
stdlib only: Server-Sent Events push each finished sweep to the page, which
needs no websocket library and reconnects by itself.

Angles follow the RPLIDAR convention: degrees, increasing CLOCKWISE seen from
above, 0 deg at the lidar's own front mark. The page draws 0 deg at the top.
Ctrl-C stops the motor and closes the port.
"""

from __future__ import annotations

import argparse
import glob
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

STATE: dict = {"seq": 0, "t": 0.0, "hz": 0.0, "points": [], "health": None,
               "error": None, "port": None}
COND = threading.Condition()
STOP = threading.Event()


def find_port() -> str:
    c = sorted(glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.SLAB_USBtoUART*")
               + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyAMA0"))
    if not c:
        sys.exit("no lidar serial port found; pass it explicitly")
    return c[0]


def read_forever(port: str, baud: int) -> None:
    """Scan until STOP. A glitch in the byte stream (the library raises on a bad
    descriptor) restarts the scan rather than killing the page."""
    from rplidar import RPLidar

    while not STOP.is_set():
        lidar = None
        try:
            lidar = RPLidar(port, baudrate=baud, timeout=2)
            health = lidar.get_health()
            lidar.start_motor()
            time.sleep(1.5)
            lidar.clean_input()
            last = time.time()
            with COND:
                STATE.update(error=None, health=list(health), port=port)
            for scan in lidar.iter_scans(max_buf_meas=5000, min_len=20):
                if STOP.is_set():
                    break
                now = time.time()
                pts = [[round(a, 2), int(d), int(q)] for q, a, d in scan if d > 0]
                with COND:
                    STATE.update(seq=STATE["seq"] + 1, t=now,
                                 hz=round(1.0 / max(1e-3, now - last), 1), points=pts)
                    COND.notify_all()
                last = now
        except Exception as exc:  # noqa: BLE001 -- keep the page alive, show the reason
            with COND:
                STATE["error"] = f"{type(exc).__name__}: {exc}"
                COND.notify_all()
            time.sleep(1.0)
        finally:
            if lidar is not None:
                for fn in (lidar.stop, lidar.stop_motor, lidar.disconnect):
                    try:
                        fn()
                    except Exception:  # noqa: BLE001
                        pass


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_a) -> None:  # quiet
        pass

    def do_GET(self) -> None:
        if self.path in ("/", "/index.html"):
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            seen = -1
            try:
                while not STOP.is_set():
                    with COND:
                        COND.wait_for(lambda: STATE["seq"] != seen or STOP.is_set(), timeout=1.0)
                        snap = dict(STATE, age=round(time.time() - STATE["t"], 2) if STATE["t"] else None)
                        seen = STATE["seq"]
                    self.wfile.write(f"data: {json.dumps(snap)}\n\n".encode())
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                return
        else:
            self.send_error(404)


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Lidar · live</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans+Condensed:wght@700&display=swap">
<style>
  :root { --ground:#E9EAEC; --surface:#FCFCFD; --ink:#14171C; --ink-soft:#757C86; --rule:#CACDD3;
          --hot:#B45C09; --near:#A11B1B; --pt:#17395F; --ok:#196B4A;
          --mono:"IBM Plex Mono",ui-monospace,Menlo,monospace; --display:"IBM Plex Sans Condensed",sans-serif; }
  @media (prefers-color-scheme: dark) { :root { --ground:#15171B; --surface:#1C1F24; --ink:#E9EBEE;
          --ink-soft:#79808B; --rule:#31373F; --hot:#F0A03C; --near:#E8756A; --pt:#8FB4DC; --ok:#4FBF8B; } }
  * { box-sizing:border-box; } html,body { height:100%; margin:0; }
  body { background:var(--ground); color:var(--ink); font-family:var(--mono); display:flex; flex-direction:column; }
  header { display:flex; flex-wrap:wrap; gap:8px 22px; align-items:baseline; padding:12px 18px;
           background:var(--surface); border-bottom:1px solid var(--rule); font-size:12.5px; }
  h1 { font-family:var(--display); font-size:19px; margin:0 8px 0 0; }
  .k { color:var(--ink-soft); } .v { font-variant-numeric:tabular-nums; }
  .dot { display:inline-block; width:8px; height:8px; border-radius:50%; background:var(--near); margin-right:6px; }
  .dot.on { background:var(--ok); }
  #near { color:var(--hot); font-weight:500; }
  main { flex:1; position:relative; min-height:0; }
  canvas { position:absolute; inset:0; width:100%; height:100%; display:block; }
  .hint { position:absolute; left:14px; bottom:10px; font-size:11px; color:var(--ink-soft); }
  .err { position:absolute; left:50%; top:14px; transform:translateX(-50%); background:var(--surface);
         border:1px solid var(--near); color:var(--near); padding:6px 12px; font-size:12px; max-width:90%; }
</style></head><body>
<header>
  <h1>Lidar</h1>
  <span><span class="dot" id="dot"></span><span id="link">connecting</span></span>
  <span><span class="k">rate</span> <span class="v" id="hz">–</span></span>
  <span><span class="k">points</span> <span class="v" id="n">–</span></span>
  <span><span class="k">health</span> <span class="v" id="health">–</span></span>
  <span><span class="k">nearest</span> <span class="v" id="near">–</span></span>
  <span><span class="k">range</span> <span class="v" id="range">–</span></span>
</header>
<main>
  <canvas id="c"></canvas>
  <div class="err" id="err" hidden></div>
  <div class="hint">scroll to zoom · space to freeze · 0° = the lidar's front mark, angles clockwise</div>
</main>
<script>
const cv = document.getElementById('c'), ctx = cv.getContext('2d');
const css = getComputedStyle(document.documentElement), C = n => css.getPropertyValue(n).trim();
const $ = id => document.getElementById(id);
let S = null, maxR = 3.0, frozen = false;

function fit() { const r = cv.getBoundingClientRect(), d = devicePixelRatio || 1;
  cv.width = r.width * d; cv.height = r.height * d; ctx.setTransform(d, 0, 0, d, 0, 0); draw(); }
addEventListener('resize', fit);
cv.addEventListener('wheel', e => { e.preventDefault();
  maxR = Math.min(12, Math.max(0.5, maxR * (e.deltaY > 0 ? 1.15 : 1 / 1.15))); draw(); }, { passive:false });
addEventListener('keydown', e => { if (e.code === 'Space') { frozen = !frozen; e.preventDefault(); } });

function draw() {
  const r = cv.getBoundingClientRect(), W = r.width, H = r.height, cx = W / 2, cy = H / 2;
  const R = Math.min(W, H) / 2 - 24, s = R / maxR;
  ctx.clearRect(0, 0, W, H);
  // range rings, one per metre (or half-metre when zoomed in)
  const step = maxR <= 2 ? 0.5 : 1;
  ctx.strokeStyle = C('--rule'); ctx.fillStyle = C('--ink-soft'); ctx.lineWidth = 1;
  ctx.font = '11px "IBM Plex Mono", monospace'; ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
  for (let m = step; m <= maxR + 1e-6; m += step) {
    ctx.beginPath(); ctx.arc(cx, cy, m * s, 0, 7); ctx.stroke();
    ctx.fillText(m.toFixed(step < 1 ? 1 : 0) + ' m', cx + 4, cy - m * s - 2);
  }
  // spokes every 45 degrees, 0 at the top
  for (let a = 0; a < 360; a += 45) {
    const t = a * Math.PI / 180;
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + Math.sin(t) * R, cy - Math.cos(t) * R); ctx.stroke();
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(a + '°', cx + Math.sin(t) * (R + 12), cy - Math.cos(t) * (R + 12));
  }
  // front marker
  ctx.fillStyle = C('--hot'); ctx.beginPath();
  ctx.moveTo(cx, cy - 16); ctx.lineTo(cx - 7, cy - 2); ctx.lineTo(cx + 7, cy - 2); ctx.closePath(); ctx.fill();
  if (!S || !S.points.length) return;
  let near = null;
  ctx.fillStyle = C('--pt');
  for (const [a, d] of S.points) {
    const m = d / 1000; if (m > maxR) continue;
    const t = a * Math.PI / 180;
    ctx.beginPath(); ctx.arc(cx + Math.sin(t) * m * s, cy - Math.cos(t) * m * s, 2.2, 0, 7); ctx.fill();
    if (!near || d < near[1]) near = [a, d];
  }
  if (near) {
    const t = near[0] * Math.PI / 180, m = near[1] / 1000;
    const x = cx + Math.sin(t) * m * s, y = cy - Math.cos(t) * m * s;
    ctx.strokeStyle = C('--near'); ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(x, y); ctx.stroke();
    ctx.fillStyle = C('--near'); ctx.beginPath(); ctx.arc(x, y, 5, 0, 7); ctx.fill();
  }
}

function panel(s) {
  $('hz').textContent = s.hz ? s.hz.toFixed(1) + ' Hz' : '–';
  $('n').textContent = s.points.length;
  $('health').textContent = s.health ? s.health[0] + (s.health[1] ? ' (' + s.health[1] + ')' : '') : '–';
  $('range').textContent = maxR.toFixed(1) + ' m';
  const pts = s.points.filter(p => p[1] > 0);
  if (pts.length) { const n = pts.reduce((a, b) => b[1] < a[1] ? b : a);
    $('near').textContent = (n[1] / 1000).toFixed(2) + ' m @ ' + n[0].toFixed(0) + '°'; }
  const stale = s.age !== null && s.age > 1.0;
  $('dot').classList.toggle('on', !s.error && !stale);
  $('link').textContent = s.error ? 'error' : stale ? 'stale ' + s.age + ' s' : (s.port || 'live');
  $('err').hidden = !s.error; $('err').textContent = s.error ? s.error + ' — retrying' : '';
}

const es = new EventSource('/stream');
es.onmessage = e => { const s = JSON.parse(e.data); panel(s); if (!frozen) { S = s; draw(); } };
es.onerror = () => { $('dot').classList.remove('on'); $('link').textContent = 'reconnecting'; };
fit();
</script></body></html>
"""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?")
    ap.add_argument("--baud", type=int, default=256000, help="A2M12: 256000 (verified)")
    ap.add_argument("--http", type=int, default=8790)
    args = ap.parse_args()
    port = args.port or find_port()

    threading.Thread(target=read_forever, args=(port, args.baud), daemon=True).start()
    srv = ThreadingHTTPServer(("127.0.0.1", args.http), Handler)
    srv.daemon_threads = True
    print(f"lidar on {port} @ {args.baud}  ->  http://localhost:{args.http}   (Ctrl-C to stop)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        STOP.set()
        with COND:
            COND.notify_all()
        srv.server_close()
        time.sleep(0.5)  # let the reader stop the motor
        print("motor stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
