"""Drive the robot by hand from a browser: hold W/A/S/D.

    .venv/bin/python scripts/teleop.py --sim                   # no robot: an in-process fake Pi
    .venv/bin/python scripts/teleop.py --bridge PI_IP:7777     # the real robot

Rung one of the navigation milestone. Before the robot can be told where to go
it has to go at all, and this is the smallest thing that proves the whole chain
Mac -> network -> Pi bridge -> DDSM115 wheels -> encoders -> odometry, with a
human closing the loop instead of a planner. It is also the tool the next rungs
need: driving round the room by hand while the lidar maps it (`--record`).

HOLD TO DRIVE. The page sends which keys are held about 15 times a second, and
nothing at all while none are. This side turns that into an Action 20 times a
second, ramped (a lunge tips an arm-topped robot), and falls back to zero when
the page goes quiet. Every way the human can disappear stops the robot:

    let go of the keys        the page sends one zero, the ramp brings it to rest
    the tab loses focus       the page clears its keys and sends that zero
    the page or WiFi dies     nothing arrives for hold_s -> zero
    this process dies         the Pi's watchdog (300 ms) stops the motors
    the Pi's link drops       the Pi stops on the socket closing; this reconnects

The last word stays on the Pi, as everywhere else: the watchdog, the e-stop
latch, and the lidar bubble if it has a lidar. Teleop is just another client
of the bridge, so it cannot drive into something the bubble can see.

A (turn left) always turns the nose left, reversing or not: a tank, not a car.

Stdlib only, like the Pi side, so it also runs on a Pi or any bare python3.
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable

from retriever.types import Action, Pose

PAGE_PATH = Path(__file__).with_name("teleop.html")

# (forward m/s, turn rad/s) per gear. Gear 1 is for the first time the real
# wheels touch the floor; the Pi caps every wheel at its max_wheel_mps anyway.
GEARS: tuple[tuple[float, float], ...] = ((0.15, 0.6), (0.30, 1.0), (0.50, 1.5))


def parse_gears(text: str) -> tuple[tuple[float, float], ...]:
    """'0.1:0.5,0.2:0.9' -> ((0.1, 0.5), (0.2, 0.9)). m/s:rad/s per gear."""
    out = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        try:
            v, w = (float(x) for x in part.split(":"))
        except ValueError:
            raise ValueError(f"bad gear {part!r}; expected M_PER_S:RAD_PER_S") from None
        if not (0 < v <= 1.5 and 0 < w <= 4.0):
            raise ValueError(f"gear {part!r} is out of range (0-1.5 m/s, 0-4 rad/s)")
        out.append((v, w))
    if not out:
        raise ValueError("no gears given")
    return tuple(out)


@dataclass(frozen=True)
class TeleopConfig:
    gears: tuple[tuple[float, float], ...] = GEARS
    hold_s: float = 0.30          # no word from the page for this long -> target zero
    rate_hz: float = 20.0         # acts sent to the Pi
    accel_mps2: float = 0.8       # speeding up
    decel_mps2: float = 2.0       # slowing down: quicker, but still no lurch
    ang_accel: float = 4.0        # rad/s^2
    ang_decel: float = 8.0
    arc_turn_scale: float = 0.6   # W+A turns this much of the gear's spin rate


def _approach(current: float, target: float, up: float, down: float, dt: float) -> float:
    """Move current toward target, at most `up` per second while |speed| grows
    (same sign, bigger) and `down` per second otherwise (slowing, reversing)."""
    growing = target * current >= 0 and abs(target) > abs(current)
    step = (up if growing else down) * dt
    if abs(target - current) <= step:
        return target
    return current + math.copysign(step, target - current)


class TeleopCore:
    """Held keys in, Action out. No threads and no clock of its own: every call
    takes `now`, so the ramps and the deadman are testable to the millisecond."""

    def __init__(self, config: TeleopConfig = TeleopConfig()) -> None:
        self.config = config
        self.gear = 0
        self.fwd = 0.0
        self.turn = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self._heard: float | None = None
        self._last_step: float | None = None

    def command(self, fwd: float, turn: float, now: float, gear: int | None = None) -> None:
        """What the page says is held: fwd, turn in [-1, 1] (+turn = left).
        Anything else (NaN, a string) raises ValueError and changes nothing."""
        f, t = float(fwd), float(turn)
        if not (math.isfinite(f) and math.isfinite(t)):
            raise ValueError("fwd and turn must be finite numbers")
        if gear is not None:
            g = int(gear)
            self.gear = min(max(g, 0), len(self.config.gears) - 1)
        self.fwd = min(max(f, -1.0), 1.0)
        self.turn = min(max(t, -1.0), 1.0)
        self._heard = now

    def halt(self) -> None:
        """Stop NOW, no ramp: the space bar, a lost link, an e-stop."""
        self.fwd = self.turn = self.vx = self.wz = 0.0
        self._heard = None

    def target(self, now: float) -> tuple[float, float]:
        """(vx, wz) the held keys ask for, or zero once the page has gone quiet."""
        c = self.config
        if self._heard is None or now - self._heard > c.hold_s:
            return 0.0, 0.0
        v, w = c.gears[self.gear]
        vx = self.fwd * v
        wz = self.turn * w * (c.arc_turn_scale if self.fwd else 1.0)
        return vx, wz

    def step(self, now: float) -> Action:
        c = self.config
        dt = 0.0 if self._last_step is None else min(max(0.0, now - self._last_step), 0.2)
        self._last_step = now
        tvx, twz = self.target(now)
        self.vx = _approach(self.vx, tvx, c.accel_mps2, c.decel_mps2, dt)
        self.wz = _approach(self.wz, twz, c.ang_accel, c.ang_decel, dt)
        return Action(base_vx=self.vx, base_wz=self.wz)

    @property
    def moving(self) -> bool:
        return abs(self.vx) > 1e-6 or abs(self.wz) > 1e-6


# ------------------------------------------------------------------ recording


class DriveRecorder:
    """Every state and scan of a drive, as JSON lines, for the mapping rung:
    wheel ticks and odometry at the Pi's state rate, the base-frame scan each
    time a new one lands, and what the keys asked for. One file per drive."""

    def __init__(self, path: Path | str) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._f = self.path.open("a", buffering=1)
        self._lock = threading.Lock()
        self.lines = 0

    def write(self, kind: str, **fields: Any) -> None:
        line = json.dumps({"k": kind, "wall": round(time.time(), 3), **fields},
                          separators=(",", ":"))
        with self._lock:
            if self._f.closed:
                return
            self._f.write(line + "\n")
            self.lines += 1

    def close(self) -> None:
        with self._lock:
            self._f.close()


# ------------------------------------------------------------------ session


@dataclass
class _Seen:
    """What the observe thread learned, for the page."""

    t: float | None = None
    rx: float = 0.0                       # time.monotonic() of the last state
    pose: Pose = field(default_factory=Pose)
    vx: float = 0.0                       # measured, from odometry
    wz: float = 0.0
    odometer_m: float = 0.0
    states: int = 0
    rate_hz: float = 0.0
    scan_t: float | None = None


class TeleopSession:
    """TeleopCore driven at rate_hz against a backend, plus what the page shows.

    `connect` makes a backend (a BridgeRobot); it is called again whenever the
    link drops, once a second, so a WiFi blip costs a reconnect, not a restart.
    Three threads: link (connects), control (acts at rate_hz), observe (reads
    every state for odometry, the trail and the recording)."""

    def __init__(
        self,
        connect: Callable[[], Any],
        config: TeleopConfig = TeleopConfig(),
        recorder: DriveRecorder | None = None,
        clock: Callable[[], float] = time.monotonic,
        footprint: tuple[float, float, float] = (0.25, 0.25, 0.20),
    ) -> None:
        self.connect = connect
        self.core = TeleopCore(config)
        self.recorder = recorder
        self.clock = clock
        self.footprint = footprint
        self.robot: Any = None
        self.link = "connecting"
        self.link_detail = ""
        self._lock = threading.RLock()
        self._closed = threading.Event()
        self._wake = threading.Event()
        self._seen = _Seen()
        self._trail: deque[tuple[float, float]] = deque(maxlen=3000)
        self._window: deque[tuple[float, Pose]] = deque()
        self._last_cmd = Action()
        self._threads: list[threading.Thread] = []

    # -- lifecycle --------------------------------------------------------

    def start(self) -> TeleopSession:
        for name, fn in (("link", self._link_loop), ("control", self._control_loop),
                         ("observe", self._observe_loop)):
            t = threading.Thread(target=fn, name=f"teleop-{name}", daemon=True)
            t.start()
            self._threads.append(t)
        return self

    def close(self) -> None:
        self._closed.set()
        self._wake.set()
        with self._lock:
            robot, self.robot = self.robot, None
        if robot is not None:
            try:
                robot.act(Action())
            except Exception:
                pass
            robot.close()   # the Pi stops the motors when the socket closes
        for t in self._threads:
            t.join(2.0)
        if self.recorder is not None:
            self.recorder.close()

    def wait_connected(self, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.link == "up":
                return True
            time.sleep(0.02)
        return False

    # -- from the page ----------------------------------------------------

    def drive(self, fwd: float, turn: float, gear: int | None = None) -> None:
        with self._lock:
            self.core.command(fwd, turn, self.clock(), gear)

    def halt(self) -> None:
        with self._lock:
            self.core.halt()
        self._send(Action())

    def estop(self) -> None:
        self.halt()
        robot = self.robot
        if robot is not None:
            robot.estop("teleop page")

    def clear_estop(self) -> None:
        robot = self.robot
        if robot is not None:
            robot.clear_estop()

    def reset_pose(self) -> None:
        robot = self.robot
        if robot is not None:
            robot.reset_pose(Pose())
        with self._lock:
            self._trail.clear()
            self._window.clear()
            self._seen.odometer_m = 0.0
            self._seen.pose = Pose()

    # -- threads ----------------------------------------------------------

    def _link_loop(self) -> None:
        while not self._closed.is_set():
            if self.robot is None:
                try:
                    robot = self.connect()
                except Exception as exc:
                    self.link, self.link_detail = "down", str(exc)
                    self._closed.wait(1.0)
                    continue
                with self._lock:
                    self.core.halt()        # a key held across the outage waits for a fresh press
                    self.robot = robot
                    self.link, self.link_detail = "up", ""
                    self._window.clear()
                self._record_meta(robot)
            self._wake.wait(0.5)
            self._wake.clear()

    def _drop(self, robot: Any, why: str) -> None:
        with self._lock:
            if self.robot is not robot:
                return
            self.robot = None
            self.core.halt()
            self.link, self.link_detail = "down", why
        try:
            robot.close()
        except Exception:
            pass
        self._wake.set()

    def _send(self, action: Action) -> None:
        robot = self.robot
        if robot is None:
            return
        try:
            robot.act(action)
            self._last_cmd = action
        except Exception as exc:
            self._drop(robot, str(exc))

    def _control_loop(self) -> None:
        period = 1.0 / self.core.config.rate_hz
        next_t = time.monotonic()
        while not self._closed.is_set():
            with self._lock:
                action = self.core.step(self.clock())
            self._send(action)
            next_t += period
            delay = next_t - time.monotonic()
            if delay < 0:
                next_t = time.monotonic()
                delay = 0.0
            self._closed.wait(delay)

    def _observe_loop(self) -> None:
        while not self._closed.is_set():
            robot = self.robot
            if robot is None:
                self._closed.wait(0.05)
                continue
            try:
                obs = robot.observe()
            except Exception as exc:
                self._drop(robot, str(exc))
                continue
            self._note(robot, obs)

    def _note(self, robot: Any, obs: Any) -> None:
        now = time.monotonic()
        with self._lock:
            s = self._seen
            if s.t is not None and obs.t == s.t:
                return                              # no new state yet
            if s.states and now > s.rx:
                s.rate_hz = 0.9 * s.rate_hz + 0.1 / (now - s.rx) if s.rate_hz else 1.0 / (now - s.rx)
            s.t, s.rx, s.states = obs.t, now, s.states + 1
            p = obs.base
            s.odometer_m += math.hypot(p.x - s.pose.x, p.y - s.pose.y) if s.states > 1 else 0.0
            s.pose = p
            if not self._trail or math.hypot(p.x - self._trail[-1][0],
                                             p.y - self._trail[-1][1]) >= 0.02:
                self._trail.append((p.x, p.y))
            self._window.append((obs.t, p))
            while len(self._window) > 2 and obs.t - self._window[0][0] > 0.3:
                self._window.popleft()
            t0, p0 = self._window[0]
            if obs.t - t0 > 0.05:
                dt = obs.t - t0
                dx, dy = p.x - p0.x, p.y - p0.y
                s.vx = (dx * math.cos(p0.theta) + dy * math.sin(p0.theta)) / dt
                s.wz = math.remainder(p.theta - p0.theta, math.tau) / dt
            cmd = self._last_cmd
        if self.recorder is None:
            return
        state = getattr(robot, "state", None)
        self.recorder.write(
            "state", t=round(obs.t, 4),
            lt=getattr(state, "left_ticks", None), rt=getattr(state, "right_ticks", None),
            x=round(p.x, 4), y=round(p.y, 4), th=round(p.theta, 5),
            cvx=round(cmd.base_vx, 3), cwz=round(cmd.base_wz, 3),
            estop=bool(getattr(state, "estop", False)))
        scan = getattr(robot, "scan", None)
        if scan is not None and scan.t != self._seen.scan_t:
            self._seen.scan_t = scan.t
            self.recorder.write("scan", t=round(scan.t, 4),
                                pts=[[round(x, 3), round(y, 3)] for x, y, *_ in scan.points])

    def _record_meta(self, robot: Any) -> None:
        if self.recorder is None:
            return
        hello = getattr(robot, "hello", None)
        self.recorder.write("meta", hello=None if hello is None else {
            k: getattr(hello, k) for k in ("counts_per_rev", "wheel_radius_m", "track_width_m",
                                           "scrub_factor", "state_hz")},
            footprint=list(self.footprint))

    # -- for the page -----------------------------------------------------

    def snapshot(self) -> dict[str, Any]:
        robot = self.robot
        with self._lock:
            s = self._seen
            core = self.core
            gear_v, gear_w = core.config.gears[core.gear]
            snap: dict[str, Any] = {
                "link": self.link,
                "link_detail": self.link_detail,
                "state_age_ms": None if s.t is None else round((time.monotonic() - s.rx) * 1000),
                "state_hz": round(s.rate_hz, 1),
                "gear": core.gear,
                "gears": [list(g) for g in core.config.gears],
                "gear_speed": [gear_v, gear_w],
                "held": [core.fwd, core.turn],
                "cmd": [round(core.vx, 3), round(core.wz, 3)],
                "meas": [round(s.vx, 3), round(s.wz, 3)],
                "pose": [round(s.pose.x, 3), round(s.pose.y, 3), round(s.pose.theta, 4)],
                "odometer_m": round(s.odometer_m, 2),
                "trail": [[round(x, 3), round(y, 3)] for x, y in list(self._trail)[-800:]],
                "footprint": list(self.footprint),
                "recording": None if self.recorder is None else {
                    "path": str(self.recorder.path), "lines": self.recorder.lines},
            }
        snap["estop"] = bool(robot is not None and robot.estopped)
        snap["watchdog"] = bool(robot is not None and robot.watchdog_tripped)
        bubble = getattr(robot, "bubble", None) if robot is not None else None
        snap["bubble"] = None if bubble is None else {
            "state": bubble.state, "reason": bubble.reason,
            "nearest_m": bubble.nearest_m, "stop_m": bubble.stop_m}
        scan = getattr(robot, "scan", None) if robot is not None else None
        snap["scan"] = None if scan is None else {
            "age_s": round(scan.age(), 2),
            "pts": [[round(x, 3), round(y, 3), int(bool(near))] for x, y, near in scan.points]}
        return snap


# ------------------------------------------------------------------ http


def make_handler(session: TeleopSession, page: str | None = None) -> type[BaseHTTPRequestHandler]:
    body_page = (page if page is not None else PAGE_PATH.read_text()).encode()

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *_a: Any) -> None:  # quiet
            pass

        def _reply(self, code: int, body: bytes = b"", ctype: str = "application/json") -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            if body:
                self.wfile.write(body)

        def do_POST(self) -> None:
            try:
                n = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(n) or b"{}") if n else {}
                if not isinstance(body, dict):
                    raise ValueError("want a JSON object")
                if self.path == "/drive":
                    session.drive(body.get("fwd", 0.0), body.get("turn", 0.0), body.get("gear"))
                elif self.path == "/halt":
                    session.halt()
                elif self.path == "/estop":
                    session.estop()
                elif self.path == "/clear":
                    session.clear_estop()
                elif self.path == "/reset":
                    session.reset_pose()
                else:
                    self._reply(404, b'{"error":"no such command"}')
                    return
            except (ValueError, TypeError) as exc:
                self._reply(400, json.dumps({"error": str(exc)}).encode())
                return
            except Exception as exc:  # the robot refused (e.g. link down mid-call)
                self._reply(503, json.dumps({"error": str(exc)}).encode())
                return
            self._reply(204)

        def do_GET(self) -> None:
            if self.path in ("/", "/index.html"):
                self._reply(200, body_page, "text/html; charset=utf-8")
            elif self.path == "/state":
                self._reply(200, json.dumps(session.snapshot()).encode())
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "close")
                self.end_headers()
                self.close_connection = True
                try:
                    while not session._closed.is_set():
                        data = json.dumps(session.snapshot(), separators=(",", ":"))
                        self.wfile.write(f"data: {data}\n\n".encode())
                        self.wfile.flush()
                        session._closed.wait(0.1)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    return
            else:
                self._reply(404, b'{"error":"not found"}')

    return Handler


def serve(session: TeleopSession, host: str = "127.0.0.1", port: int = 8791) -> ThreadingHTTPServer:
    """Start the page's HTTP server in a daemon thread; returns it (.server_port)."""
    httpd = ThreadingHTTPServer((host, port), make_handler(session))
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, name="teleop-http", daemon=True).start()
    return httpd
