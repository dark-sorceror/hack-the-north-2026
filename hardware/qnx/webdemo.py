"""Live test page for the QNX supervisor's sensing: colour, depth and the AI.

    python3 webdemo.py            then open http://<pi>:8080

Reads colour + depth from camtap, runs detector.py's person model, tracks one
colour, and lights the LED on GPIO17 when the chosen rule fires (by default:
an object of that colour closer than the threshold, so colour and depth have
to agree about the same thing). A bench tool: it drives the LED directly, so
the LED means "detected" here, not the safety heartbeat. Don't run it
alongside lineguard; both want GPIO17.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

import detector as D

# OpenCV HSV: H is 0..179. Red wraps around 0, so it has two ranges.
COLOURS = {
    "red":    [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (179, 255, 255))],
    "orange": [((11, 120, 90), (22, 255, 255))],
    "yellow": [((23, 100, 100), (35, 255, 255))],
    "green":  [((40, 80, 50), (85, 255, 255))],
    "blue":   [((95, 120, 50), (130, 255, 255))],
}
MODES = {
    "colour_close": "colour object closer than the threshold",
    "either": "anything close, or the colour seen",
    "close": "anything close",
    "colour": "the colour seen",
}
MIN_BLOB = 0.004  # a colour blob counts from 0.4% of the frame


class Led:
    """GPIO17 through the rpi_gpio resource manager's text interface."""

    def __init__(self, gpio: int = 17) -> None:
        self.fd = os.open(f"/dev/gpio/{gpio}", os.O_WRONLY)
        os.write(self.fd, b"out")
        self.on = None
        self.set(False)

    def set(self, on: bool) -> None:
        if on != self.on:
            os.write(self.fd, b"on" if on else b"off")
            self.on = on

    def close(self) -> None:
        self.set(False)
        os.write(self.fd, b"in")
        os.close(self.fd)


def nearest_ahead_m(depth: np.ndarray) -> float | None:
    """The nearest thing in the middle of the view (centre half across, 60%
    down): the 2nd percentile of valid depths, so a few noisy pixels don't count."""
    H, W = depth.shape
    patch = depth[int(H * 0.2):int(H * 0.8), int(W * 0.25):int(W * 0.75)]
    valid = patch[(patch > 0) & (patch < 65535)]
    if valid.size < 200:
        return None
    return float(np.percentile(valid, 2)) / 1000.0


def colour_blob(bgr: np.ndarray, name: str):
    """(box as (ymin, xmin, ymax, xmax) 0..1, area fraction) of the largest
    blob of the colour, or None."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in COLOURS[name]:
        mask |= cv2.inRange(hsv, lo, hi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    H, W = mask.shape
    area = cv2.contourArea(c) / (H * W)
    if area < MIN_BLOB:
        return None
    x, y, w, h = cv2.boundingRect(c)
    return np.array([y / H, x / W, (y + h) / H, (x + w) / W]), area


class Pipeline(threading.Thread):
    def __init__(self, model: str, threads: int, min_score: float) -> None:
        super().__init__(daemon=True)
        self.det = D.Detector(model, threads)
        self.min_score = min_score
        self.led = Led()
        self.settings = {"colour": "red", "near_m": 0.5, "mode": "colour_close"}
        self.lock = threading.Lock()
        self.new_frame = threading.Condition(self.lock)
        self.jpeg: bytes | None = None
        self.status: dict = {"state": "starting"}
        self.seq = 0

    def run(self) -> None:
        while True:
            try:
                tap = D.CamTap()
                break
            except (OSError, RuntimeError) as exc:
                with self.lock:
                    self.status = {"state": f"waiting for camtap: {exc}"}
                time.sleep(1)
        last_n, last_new, times = -1, time.monotonic(), []
        while True:
            got = tap.latest()
            if got is None or got[2] == last_n:
                if time.monotonic() - last_new > 0.5:
                    self.led.set(False)
                    with self.lock:
                        self.status = {"state": "camera stalled: is camtap running?"}
                time.sleep(0.003)
                continue
            bgr, depth, last_n, _ = got
            last_new = t0 = time.monotonic()
            with self.lock:
                s = dict(self.settings)
            self.step(bgr, depth, s)
            times.append(time.monotonic() - t0)
            times = times[-30:]
            with self.lock:
                self.status["fps"] = round(len(times) / max(sum(times), 1e-6), 1)

    def step(self, bgr: np.ndarray, depth: np.ndarray | None, s: dict) -> None:
        people = [(sc, box, D.person_distance_m(box, depth) if depth is not None else None)
                  for sc, box in self.det.people(bgr, self.min_score)]
        near = nearest_ahead_m(depth) if depth is not None else None
        blob = colour_blob(bgr, s["colour"])
        blob_m = D.person_distance_m(blob[0], depth) if blob is not None and depth is not None else None

        close = near is not None and near < s["near_m"]
        seen = blob is not None
        colour_close = blob_m is not None and blob_m < s["near_m"]
        led = {"colour_close": colour_close, "either": close or seen,
               "close": close, "colour": seen}[s["mode"]]
        self.led.set(led)

        view = self.draw(bgr, depth, people, blob, blob_m, s)
        ok, jpg = cv2.imencode(".jpg", view, [cv2.IMWRITE_JPEG_QUALITY, 70])
        status = {
            "state": "ok",
            "led": led,
            "near_m": near, "close": close,
            "colour": s["colour"], "colour_seen": seen,
            "colour_area": round(blob[1] * 100, 1) if blob is not None else None,
            "colour_m": blob_m, "colour_close": colour_close,
            "people": [{"score": round(sc, 2), "m": m} for sc, _, m in people],
            "depth": depth is not None,
            "settings": s,
        }
        with self.new_frame:
            self.jpeg, self.seq = jpg.tobytes(), self.seq + 1
            self.status = {**status, "fps": self.status.get("fps")}
            self.new_frame.notify_all()

    @staticmethod
    def draw(bgr, depth, people, blob, blob_m, s):
        img = bgr.copy()
        H, W = img.shape[:2]

        def rect(box, colour, label):
            y0, x0, y1, x1 = box
            p0, p1 = (int(x0 * W), int(y0 * H)), (int(x1 * W), int(y1 * H))
            cv2.rectangle(img, p0, p1, colour, 2)
            cv2.putText(img, label, (p0[0] + 3, max(p0[1] - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)

        for sc, box, m in people:
            rect(box, (255, 200, 0), f"person {sc:.2f}" + (f"  {m:.2f} m" if m else ""))
        if blob is not None:
            near = blob_m is not None and blob_m < s["near_m"]
            rect(blob[0], (0, 0, 255) if near else (0, 255, 255),
                 f"{s['colour']}" + (f"  {blob_m:.2f} m" if blob_m else ""))
        if depth is None:
            dview = np.zeros_like(img)
            cv2.putText(dview, "no depth", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 200, 200), 2)
        else:
            d8 = cv2.convertScaleAbs(np.where(depth == 65535, 0, depth), alpha=255 / 4000)
            dview = cv2.applyColorMap(d8, cv2.COLORMAP_TURBO)
            dview[depth == 0] = 0
            dview = cv2.resize(dview, (W, H))
            h, w = dview.shape[:2]  # the "ahead" window nearest_ahead_m looks at
            cv2.rectangle(dview, (int(w * 0.25), int(h * 0.2)), (int(w * 0.75), int(h * 0.8)),
                          (255, 255, 255), 1)
        return np.hstack([img, dview])


PAGE = """<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>QNX sensing test</title>
<style>
:root{--bg:#0f1115;--card:#181b22;--fg:#e8eaf0;--mute:#8a90a0;--on:#ff4d4f;--ok:#3ecf8e;--line:#2a2f3a}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.4 system-ui,sans-serif}
header{padding:14px 18px;border-bottom:1px solid var(--line);display:flex;gap:12px;align-items:center;flex-wrap:wrap}
header b{font-size:17px}header span{color:var(--mute)}
main{padding:16px;display:grid;gap:16px;grid-template-columns:minmax(0,1fr) 300px}
@media(max-width:900px){main{grid-template-columns:1fr}}
img{width:100%;border-radius:10px;background:#000;display:block}
.cards{display:grid;gap:10px;align-content:start}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px 14px}
.card h3{margin:0 0 4px;font-size:12px;letter-spacing:.06em;text-transform:uppercase;color:var(--mute)}
.big{font-size:22px;font-weight:600}.hit{color:var(--on)}.clear{color:var(--ok)}
#led{display:flex;align-items:center;gap:12px}
#dot{width:34px;height:34px;border-radius:50%;background:#333;box-shadow:inset 0 0 6px #000}
#dot.on{background:var(--on);box-shadow:0 0 18px var(--on)}
label{display:block;margin:8px 0 4px;color:var(--mute);font-size:13px}
select,input{width:100%;background:#0c0e12;color:var(--fg);border:1px solid var(--line);border-radius:8px;padding:7px}
</style></head><body>
<header><b>QNX sensing test</b><span>RealSense D435i &rarr; camtap &rarr; TFLite on QNX &rarr; LED on GPIO17</span>
<span id="fps"></span></header>
<main><div><img src="/stream.mjpg" alt="colour | depth"></div>
<div class="cards">
 <div class="card" id="led"><div id="dot"></div><div><h3>LED</h3><div class="big" id="ledtxt">off</div></div></div>
 <div class="card"><h3>Nearest thing ahead (depth)</h3><div class="big" id="near">-</div></div>
 <div class="card"><h3>Colour (<span id="cname">red</span>)</h3><div class="big" id="col">-</div></div>
 <div class="card"><h3>People (AI)</h3><div class="big" id="ppl">-</div></div>
 <div class="card"><h3>Settings</h3>
  <label>Colour to track</label><select id="colour"></select>
  <label>Close means nearer than <b id="nv"></b> m</label>
  <input id="near_m" type="range" min="0.2" max="3" step="0.05">
  <label>LED lights when</label><select id="mode"></select>
 </div>
 <div class="card" id="state" style="color:var(--mute)"></div>
</div></main>
<script>
const $=id=>document.getElementById(id), m=v=>v==null?"no reading":v.toFixed(2)+" m";
const COLOURS=__COLOURS__, MODES=__MODES__;
for(const c of COLOURS)$("colour").add(new Option(c,c));
for(const [k,v] of Object.entries(MODES))$("mode").add(new Option(v,k));
let loaded=false;
function send(){fetch("/settings",{method:"POST",body:JSON.stringify({colour:$("colour").value,
  near_m:+$("near_m").value,mode:$("mode").value})});$("nv").textContent=(+$("near_m").value).toFixed(2)}
for(const id of ["colour","near_m","mode"])$(id).addEventListener("input",send);
async function tick(){
 try{const s=await (await fetch("/status")).json();
  if(!loaded&&s.settings){$("colour").value=s.settings.colour;$("near_m").value=s.settings.near_m;
   $("mode").value=s.settings.mode;$("nv").textContent=s.settings.near_m.toFixed(2);loaded=true}
  $("state").textContent=s.state==="ok"?(s.depth?"colour + depth streaming":"colour only: no depth"):s.state;
  $("fps").textContent=s.fps?s.fps+" fps":"";
  $("dot").className=s.led?"on":"";$("ledtxt").textContent=s.led?"ON":"off";$("ledtxt").className="big "+(s.led?"hit":"");
  $("near").textContent=m(s.near_m);$("near").className="big "+(s.close?"hit":"clear");
  $("cname").textContent=s.colour||"";
  $("col").textContent=s.colour_seen?(s.colour_area+"% of view, "+m(s.colour_m)):"not seen";
  $("col").className="big "+(s.colour_close?"hit":s.colour_seen?"":"clear");
  $("ppl").textContent=s.people&&s.people.length?s.people.map(p=>m(p.m)).join(", "):"none";
 }catch(e){$("state").textContent="page lost the Pi: "+e}
 setTimeout(tick,200)}
tick();
</script></body></html>"""


def serve(pipe: Pipeline, port: int) -> None:
    page = (PAGE.replace("__COLOURS__", json.dumps(list(COLOURS)))
                .replace("__MODES__", json.dumps(MODES)).encode())

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send_body(self, body: bytes, ctype: str) -> None:
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                self.send_body(page, "text/html; charset=utf-8")
            elif self.path == "/status":
                with pipe.lock:
                    body = json.dumps(pipe.status).encode()
                self.send_body(body, "application/json")
            elif self.path == "/stream.mjpg":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                seen = -1
                try:
                    while True:
                        with pipe.new_frame:
                            pipe.new_frame.wait_for(lambda: pipe.seq != seen, timeout=2)
                            jpg, seen = pipe.jpeg, pipe.seq
                        if jpg is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404)

        def do_POST(self):
            if self.path != "/settings":
                return self.send_error(404)
            new = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
            with pipe.lock:
                if new.get("colour") in COLOURS:
                    pipe.settings["colour"] = new["colour"]
                if isinstance(new.get("near_m"), (int, float)):
                    pipe.settings["near_m"] = float(min(max(new["near_m"], 0.1), 5.0))
                if new.get("mode") in MODES:
                    pipe.settings["mode"] = new["mode"]
            self.send_body(b"{}", "application/json")

    ThreadingHTTPServer(("0.0.0.0", port), Handler).serve_forever()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--model", default="ssd_mobilenet_v1_quant.tflite")
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--min-score", type=float, default=0.4)
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()
    pipe = Pipeline(args.model, args.threads, args.min_score)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    pipe.start()
    print(f"webdemo: http://0.0.0.0:{args.port}  (LED on GPIO17)", flush=True)
    try:
        serve(pipe, args.port)
    except KeyboardInterrupt:
        pass
    finally:
        pipe.led.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
