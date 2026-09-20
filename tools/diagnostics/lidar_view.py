#!/usr/bin/env python3
"""Live lidar radar in the browser, with the avoidance planner drawn on top.

    <env-with-rplidar>/bin/python tools/diagnostics/lidar_view.py [PORT]    ->  http://localhost:8790
    .venv/bin/python tools/diagnostics/lidar_view.py --sim                  (no lidar: a simulated room)

A bench tool for SEEING the sensor. It deliberately uses the third-party
`rplidar-roboticia` library rather than the robot's own lidar driver, so when
the two disagree you know which one to suspect. The server is stdlib only:
Server-Sent Events push each finished sweep to the page, which needs no
websocket library and reconnects by itself.

PLANNER OVERLAY. Click anywhere on the radar to put a goal there; right-click
or Esc clears it. Every sweep is then run through navigation/avoid.py exactly
as the robot runs it (same ScanTracker merging the last 0.5 s, same
LocalPlanner), and the page draws the inflated blocked sectors (red), the
unseen ones (grey), the chosen heading (arrow, length = speed) and the spoken
reason. That is how to check the planner against real noise and dropouts
before the robot ever moves. The overlay assumes the robot IS the lidar: its
centre at the lidar, its front at the 0 mark. No goal set: the page is the
plain radar it always was.

Angles follow the RPLIDAR convention: degrees, increasing CLOCKWISE seen from
above, 0 deg at the lidar's own front mark. The page draws 0 deg at the top.
Ctrl-C stops the motor and closes the port.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from retriever.navigation.avoid import (  # noqa: E402  (stdlib only, runs in the lidar env)
    AvoidConfig,
    LocalPlanner,
    Mount,
    ScanTracker,
    scan_from_rplidar,
)
from retriever.types import Pose  # noqa: E402

STATE: dict = {"seq": 0, "t": 0.0, "hz": 0.0, "points": [], "health": None,
               "error": None, "port": None, "plan": None}
COND = threading.Condition()
STOP = threading.Event()

# Standard scan mode: 252 us per sample (measured on our unit), returns or not.
# iter_scans hands over only the returns, so the number of rays in a revolution
# has to come from the revolution rate instead of from the list.
SAMPLES_PER_S = 1e6 / 252


class Overlay:
    """The planner, fed every sweep; all of it under COND."""

    def __init__(self, config: AvoidConfig) -> None:
        self.config = config
        self.planner = LocalPlanner(config)
        self.latest = None
        self.tracker = ScanTracker(lambda: self.latest, config.memory_s, config.stale_s)
        self.goal: tuple[float, float] | None = None   # (forward, left) metres from the lidar

    def add_sweep(self, pts: list, now: float, hz: float, rays: int | None = None) -> None:
        if rays is None:
            rays = round(SAMPLES_PER_S / hz) if 3.0 < hz < 20.0 else 360
        self.latest = scan_from_rplidar(now, [(a, d / 1000.0) for a, d, _ in pts], Mount(),
                                        n_rays=rays)
        self.tracker.update(now, Pose())

    def plan(self, now: float) -> dict | None:
        if self.goal is None:
            return None
        fwd, left = self.goal
        goal_rad, goal_m = math.atan2(left, fwd), math.hypot(fwd, left)
        scan = self.tracker.current(now, Pose())
        p = (self.planner.plan(scan, goal_rad, goal_m) if scan is not None
             else self.planner.no_scan(goal_rad, goal_m, "no fresh lidar sweep"))
        c = self.config
        return {**p.as_dict(), "goal": [round(fwd, 3), round(left, 3)], "radius_m": c.radius_m,
                "inflated_m": c.radius_m + c.margin_m, "lookahead_m": c.lookahead_m}


OVERLAY = Overlay(AvoidConfig())


def publish(pts: list, now: float, hz: float, rays: int | None = None) -> None:
    """One finished sweep (returns only, [angle_cw_deg, range_mm, quality]) to the page."""
    with COND:
        OVERLAY.add_sweep(pts, now, hz, rays)
        STATE.update(seq=STATE["seq"] + 1, t=now, hz=hz, points=pts, plan=OVERLAY.plan(now))
        COND.notify_all()


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
                publish(pts, now, round(1.0 / max(1e-3, now - last), 1))
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


def simulate_forever(hz: float = 10.0) -> None:
    """--sim: a room with a chair, a box and a person, seen through the same
    60% dropouts as the real unit. Exercises the page and the planner with no
    lidar attached."""
    from retriever.navigation import simscan as sim

    world = (sim.room(-2.5, -2.0, 4.0, 2.0) + sim.chair(1.2, -0.15) + sim.box(1.6, 1.1, 0.5, 0.4)
             + sim.person(0.9, -1.2))
    lidar = sim.SimLidar(world, Pose, time.time, seed=1)
    with COND:
        STATE.update(port="sim (no lidar)", health=["Good", 0], error=None)
    while not STOP.is_set():
        raw = lidar.sweep(Pose())
        pts = [[round(a, 2), int(r * 1000), 15] for a, r in raw if r > 0]
        publish(pts, time.time(), hz, rays=lidar.rays)
        STOP.wait(1.0 / hz)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_a) -> None:  # quiet
        pass

    def do_POST(self) -> None:
        """/goal {"fwd": m, "left": m} sets the planner's goal; {"clear": true} clears it."""
        if self.path != "/goal":
            self.send_error(404)
            return
        try:
            body = json.loads(self.rfile.read(int(self.headers.get("Content-Length") or 0)) or b"{}")
            goal = None if body.get("clear") else (float(body["fwd"]), float(body["left"]))
        except (ValueError, KeyError, TypeError, AttributeError):
            self.send_error(400, "want {fwd, left} in metres, or {clear: true}")
            return
        with COND:
            OVERLAY.goal = goal
            OVERLAY.planner.reset()
            STATE.update(seq=STATE["seq"] + 1, plan=OVERLAY.plan(time.time()))
            COND.notify_all()
        self.send_response(204)
        self.end_headers()

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
          --blocked:rgba(161,27,27,.16); --unknown:rgba(117,124,134,.20); --goal:#6A3FA0;
          --mono:"IBM Plex Mono",ui-monospace,Menlo,monospace; --display:"IBM Plex Sans Condensed",sans-serif; }
  @media (prefers-color-scheme: dark) { :root { --ground:#15171B; --surface:#1C1F24; --ink:#E9EBEE;
          --ink-soft:#79808B; --rule:#31373F; --hot:#F0A03C; --near:#E8756A; --pt:#8FB4DC; --ok:#4FBF8B;
          --blocked:rgba(232,117,106,.20); --unknown:rgba(121,128,139,.24); --goal:#C3A3EE; } }
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
  .foot { position:absolute; left:14px; right:14px; bottom:10px; display:flex; flex-direction:column;
          align-items:flex-start; gap:6px; pointer-events:none; }
  .hint { font-size:11px; color:var(--ink-soft); }
  .err { position:absolute; left:50%; top:14px; transform:translateX(-50%); background:var(--surface);
         border:1px solid var(--near); color:var(--near); padding:6px 12px; font-size:12px; max-width:90%; }
  .plan { font-size:12.5px; max-width:640px; background:var(--surface); border:1px solid var(--rule);
          padding:8px 12px; }
  .plan b { font-weight:500; text-transform:uppercase; letter-spacing:.04em; margin-right:8px; }
  .plan .clear { color:var(--ok); } .plan .steering { color:var(--hot); } .plan .blocked { color:var(--near); }
  canvas { cursor:crosshair; }
</style></head><body>
<header>
  <h1>Lidar</h1>
  <span><span class="dot" id="dot"></span><span id="link">connecting</span></span>
  <span><span class="k">rate</span> <span class="v" id="hz">–</span></span>
  <span><span class="k">points</span> <span class="v" id="n">–</span></span>
  <span><span class="k">health</span> <span class="v" id="health">–</span></span>
  <span><span class="k">nearest</span> <span class="v" id="near">–</span></span>
  <span><span class="k">range</span> <span class="v" id="range">–</span></span>
  <span><span class="k">planner</span> <span class="v" id="pstat">click to set a goal</span></span>
</header>
<main>
  <canvas id="c"></canvas>
  <div class="err" id="err" hidden></div>
  <div class="foot">
    <div class="plan" id="plan" hidden><b id="pst"></b><span id="preason"></span></div>
    <div class="hint">scroll to zoom · space to freeze · click: goal · right-click / Esc: clear ·
      0° = the lidar's front mark, angles clockwise</div>
  </div>
</main>
<script>
const cv = document.getElementById('c'), ctx = cv.getContext('2d');
const css = getComputedStyle(document.documentElement), C = n => css.getPropertyValue(n).trim();
const $ = id => document.getElementById(id);
let S = null, maxR = 3.0, frozen = false;

function geom() { const r = cv.getBoundingClientRect(), W = r.width, H = r.height;
  const R = Math.min(W, H) / 2 - 24; return { W, H, cx: W / 2, cy: H / 2, R, s: R / maxR }; }
function setGoal(body) { fetch('/goal', { method: 'POST', headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify(body) }).catch(() => {}); }
cv.addEventListener('click', e => { const g = geom(), r = cv.getBoundingClientRect();
  const x = e.clientX - r.left, y = e.clientY - r.top;
  setGoal({ fwd: (g.cy - y) / g.s, left: (g.cx - x) / g.s }); });   // robot frame: x forward, y left
cv.addEventListener('contextmenu', e => { e.preventDefault(); setGoal({ clear: true }); });
addEventListener('keydown', e => { if (e.code === 'Escape') setGoal({ clear: true }); });

function fit() { const r = cv.getBoundingClientRect(), d = devicePixelRatio || 1;
  cv.width = r.width * d; cv.height = r.height * d; ctx.setTransform(d, 0, 0, d, 0, 0); draw(); }
addEventListener('resize', fit);
cv.addEventListener('wheel', e => { e.preventDefault();
  maxR = Math.min(12, Math.max(0.5, maxR * (e.deltaY > 0 ? 1.15 : 1 / 1.15))); draw(); }, { passive:false });
addEventListener('keydown', e => { if (e.code === 'Space') { frozen = !frozen; e.preventDefault(); } });

// The planner works in the robot frame: bearing counter-clockwise from the front,
// + = left. On this page the front is up and left is left, so bearing b draws
// at (cx - sin b, cy - cos b), and canvas arc angles are -b - 90 degrees.
function drawPlan(P, g) {
  const rad = d => d * Math.PI / 180, w = P.sector_deg;
  const rin = P.radius_m * g.s, rout = Math.min(P.lookahead_m, maxR) * g.s;
  for (let i = 0; i < P.sectors.length; i++) {
    const c = P.sectors[i]; if (c === 'f') continue;
    const b = i * w;
    ctx.fillStyle = C(c === 'b' ? '--blocked' : '--unknown');
    ctx.beginPath();
    ctx.arc(g.cx, g.cy, rout, rad(-(b + w / 2) - 90), rad(-(b - w / 2) - 90));
    ctx.arc(g.cx, g.cy, rin, rad(-(b - w / 2) - 90), rad(-(b + w / 2) - 90), true);
    ctx.closePath(); ctx.fill();
  }
  ctx.strokeStyle = C('--ink-soft'); ctx.lineWidth = 1;
  ctx.beginPath(); ctx.arc(g.cx, g.cy, rin, 0, 7); ctx.stroke();                 // body
  ctx.setLineDash([4, 4]);
  ctx.beginPath(); ctx.arc(g.cx, g.cy, P.inflated_m * g.s, 0, 7); ctx.stroke();  // body + margin
  // goal: dashed line and a cross
  const gx = g.cx - P.goal[1] * g.s, gy = g.cy - P.goal[0] * g.s;
  ctx.strokeStyle = C('--goal');
  ctx.beginPath(); ctx.moveTo(g.cx, g.cy); ctx.lineTo(gx, gy); ctx.stroke(); ctx.setLineDash([]);
  ctx.lineWidth = 2.5; ctx.beginPath();
  ctx.moveTo(gx - 7, gy - 7); ctx.lineTo(gx + 7, gy + 7); ctx.moveTo(gx + 7, gy - 7); ctx.lineTo(gx - 7, gy + 7);
  ctx.stroke();
  // chosen heading: arrow whose length is the speed
  const col = C(P.status === 'blocked' ? '--near' : P.status === 'steering' ? '--hot' : '--ok');
  ctx.strokeStyle = col; ctx.fillStyle = col; ctx.lineWidth = 3;
  if (P.status === 'blocked') {
    ctx.beginPath(); ctx.arc(g.cx, g.cy, 10, 0, 7); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(g.cx - 7, g.cy + 7); ctx.lineTo(g.cx + 7, g.cy - 7); ctx.stroke();
  } else {
    const b = rad(P.steer_deg), L = rin + (rout - rin) * Math.max(0.15, P.speed);
    const ex = g.cx - Math.sin(b) * L, ey = g.cy - Math.cos(b) * L;
    ctx.beginPath(); ctx.moveTo(g.cx, g.cy); ctx.lineTo(ex, ey); ctx.stroke();
    const a = Math.atan2(ey - g.cy, ex - g.cx);
    ctx.beginPath(); ctx.moveTo(ex, ey);
    ctx.lineTo(ex - 12 * Math.cos(a - 0.4), ey - 12 * Math.sin(a - 0.4));
    ctx.lineTo(ex - 12 * Math.cos(a + 0.4), ey - 12 * Math.sin(a + 0.4)); ctx.closePath(); ctx.fill();
  }
  ctx.lineWidth = 1;
}

function draw() {
  const g = geom(), W = g.W, H = g.H, cx = g.cx, cy = g.cy, R = g.R, s = g.s;
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
  if (S && S.plan) drawPlan(S.plan, g);
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
  const P = s.plan;
  $('plan').hidden = !P;
  $('pstat').textContent = P ? P.status + (P.status === 'blocked' ? '' : ' · ' + Math.round(P.speed * 100) + '%')
                             : 'click to set a goal';
  if (P) { $('pst').textContent = P.status; $('pst').className = P.status; $('preason').textContent = P.reason; }
}

const es = new EventSource('/stream');
es.onmessage = e => { const s = JSON.parse(e.data); panel(s); if (!frozen) { S = s; draw(); } };
es.onerror = () => { $('dot').classList.remove('on'); $('link').textContent = 'reconnecting'; };
fit();
</script></body></html>
"""


def main() -> int:
    global OVERLAY
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?")
    ap.add_argument("--baud", type=int, default=256000, help="A2M12: 256000 (verified)")
    ap.add_argument("--http", type=int, default=8790)
    ap.add_argument("--sim", action="store_true", help="no lidar: a simulated room with a chair")
    d = AvoidConfig()
    ap.add_argument("--radius", type=float, default=d.radius_m,
                    help="planner: robot radius to the corners, m")
    ap.add_argument("--margin", type=float, default=d.margin_m, help="planner: clearance kept, m")
    ap.add_argument("--lookahead", type=float, default=d.lookahead_m,
                    help="planner: ignore returns farther than this, m")
    args = ap.parse_args()
    OVERLAY = Overlay(AvoidConfig(radius_m=args.radius, margin_m=args.margin,
                                  lookahead_m=args.lookahead))

    if args.sim:
        port = "sim"
        threading.Thread(target=simulate_forever, daemon=True).start()
    else:
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
