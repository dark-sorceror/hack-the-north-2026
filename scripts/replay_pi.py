#!/usr/bin/env python3
"""Play a recorded drive back into the dashboard, as if the robot were driving it again.

    python3 scripts/replay_pi.py data/drives/drive-20260920-033710.jsonl --list
    python3 scripts/replay_pi.py data/drives/drive-20260920-033710.jsonl --segment 4 \\
        --page http://127.0.0.1:8795
    .venv/bin/python scripts/teleop.py --bridge 127.0.0.1:17777 --port 8795 --camera off \\
        --footprint 0.42,0.22,0.185

This stands in for the Pi: a TCP server on port 17777 (not 7777, so a real bridge can
keep that) speaking protocol v1, and the dashboard connects to it exactly as it connects
to the robot. The dashboard is not modified: its own odometry, map and page do all the
work, from what the robot really saw.

What a `teleop.py --record` file holds, and so what goes back out:

  - The pose the laptop computed, at every state it noticed. The file keeps the wheel
    ticks too, but not the gyro heading they were combined with, and its ticks can be a
    state newer than its pose. So each `state` carries ticks and a `yaw` MADE so that
    the dashboard's unmodified odometry (TankOdometry, heading from the gyro and
    distance from the wheels) retraces the recorded pose, to a few millimetres.
  - Every lidar revolution, in the 2-degree bins the Pi sent, at its original time.
    The dashboard puts each one in its map at the pose it was taken from.
  - Its timing, faster with --speed. Pauses longer than --max-gap (link hiccups) are
    cut, so the dashboard's 1 s stale-state check never fires.

What it cannot send, because it was never recorded: the Pi's safety bubble, the `near`
flag on scan bins, and the camera (run teleop with --camera off). Routes the
click-to-go planner drew are in the file, but the dashboard only draws a route its own
planner is driving, so they are counted and not shown. Commands from the page (keys,
clicks, e-stop) are read and dropped: nothing here moves.

A recording is split into segments where the pose jumps. A segment starts either when
the laptop (re)connected, which the replay repeats by hanging up so the dashboard
reconnects, or at the page's Reset (pose and map cleared). With --page the replay
presses that Reset on the dashboard at the same moment; without it, it hangs up
instead, which zeroes the pose but leaves the old map in place. --list shows them.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import threading
import time
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.bridge.protocol import (  # noqa: E402
    CLIENT_MESSAGES,
    MAX_LINE_BYTES,
    Act,
    ClearEstop,
    Error,
    Estop,
    Hello,
    ProtocolError,
    Scan,
    State,
    Subscribe,
    decode,
    encode,
)
from retriever.navigation.odometry import integrate_twist  # noqa: E402
from retriever.types import Pose  # noqa: E402

DEFAULT_REPLAY_PORT = 17777
STEP_DEG = 2.0            # the Pi's scan bins (BridgeCore.scan_message); checked per file
TIMEOUT_MS, MOTION_TIMEOUT_MS = 300, 500   # the bridge's defaults; not in the recording
MAX_SEND_HZ = 60.0        # above --speed 1, states are thinned to about this rate
HOLD_HZ = 50.0            # standing still: the first or last pose, resent so the link lives
STILL_M, STILL_RAD = 0.01, math.radians(1.0)
IDLE_AFTER_S = 1.0        # this long without moving and --idle-speed kicks in


def clock_text(s: float) -> str:
    s = max(0.0, s)
    return f"{int(s // 60):d}:{s % 60:04.1f}"


def parse_clock(text: str) -> float:
    """Seconds from "95", "95.5", "1:35" or "0:01:35"."""
    total = 0.0
    for part in text.strip().split(":"):
        total = total * 60.0 + float(part)
    return total


# ------------------------------------------------------------------ the file


@dataclass
class Segment:
    """A stretch of a recording with no jump in the pose. It starts when the laptop
    connected (odometry from zero) or at the page's Reset (odometry and map
    cleared), and runs until the next of either."""

    n: int
    starts: str               # "connect" | "reset"
    offset: int               # file offset of its first state line
    start_s: float            # seconds into the recording, by its wall clock
    hello: dict
    footprint: list | None
    end_s: float = 0.0
    states: int = 0
    scans: int = 0
    routes: int = 0
    path_m: float = 0.0
    turned_rad: float = 0.0
    moving_s: float = 0.0
    jumps: int = 0            # steps too big to be driven (encoder glitches); replayed as they are
    box: list = field(default_factory=lambda: [math.inf, math.inf, -math.inf, -math.inf])

    @property
    def span_m(self) -> tuple[float, float]:
        if self.box[0] > self.box[2]:
            return 0.0, 0.0
        return self.box[2] - self.box[0], self.box[3] - self.box[1]


def is_reset(prev: dict, cur: dict) -> bool:
    """The page's Reset: the pose lands on the origin from somewhere it could not
    have driven from in one state."""
    at_origin = abs(cur["x"]) < 0.02 and abs(cur["y"]) < 0.02 and abs(cur["th"]) < 0.05
    dth = abs(math.remainder(cur["th"] - prev["th"], math.tau))
    return at_origin and (math.hypot(cur["x"] - prev["x"], cur["y"] - prev["y"]) > 0.05
                          or dth > 0.2)


class Recording:
    """One `teleop.py --record` file, indexed into segments in a single pass."""

    def __init__(self, path: Path | str) -> None:
        self.path = Path(path)
        self.segments: list[Segment] = []
        self.wall0: float | None = None
        self.end_s = 0.0
        self.kinds: dict[str, int] = {}
        self.unreadable = 0
        self.on_centre = self.off_centre = 0      # scan points on / off STEP_DEG bin centres
        self._index()

    def _index(self) -> None:
        hello = footprint = prev = seg = None
        new_link = True
        sampled = 0
        with self.path.open("rb") as f:
            while True:
                off = f.tell()
                raw = f.readline()
                if not raw:
                    break
                if raw.startswith(b'{"k":"scan"'):      # the bulk of the file: parse a sample
                    self.kinds["scan"] = self.kinds.get("scan", 0) + 1
                    if seg is not None:
                        seg.scans += 1
                    if sampled < 300:
                        sampled += 1
                        self._check_bins(raw)
                    continue
                try:
                    o = json.loads(raw)
                    k = o["k"]
                except (ValueError, KeyError, TypeError):
                    self.unreadable += 1
                    continue
                self.kinds[k] = self.kinds.get(k, 0) + 1
                if self.wall0 is None:
                    self.wall0 = o.get("wall", 0.0)
                if k == "meta":
                    hello, footprint, prev, new_link = o.get("hello"), o.get("footprint"), None, True
                    continue
                if k == "route":
                    if seg is not None:
                        seg.routes += 1
                    continue
                if k != "state" or hello is None:
                    continue
                wall_s = o["wall"] - self.wall0
                starts = "connect" if new_link else "reset" if is_reset(prev, o) else None
                if starts:
                    seg = Segment(len(self.segments), starts, off, wall_s, hello, footprint)
                    self.segments.append(seg)
                    new_link = False
                else:
                    self._step(seg, prev, o)
                seg.end_s = self.end_s = wall_s
                seg.states += 1
                seg.box[:] = [min(seg.box[0], o["x"]), min(seg.box[1], o["y"]),
                              max(seg.box[2], o["x"]), max(seg.box[3], o["y"])]
                prev = o

    @staticmethod
    def _step(seg: Segment, prev: dict, o: dict) -> None:
        d = math.hypot(o["x"] - prev["x"], o["y"] - prev["y"])
        dth = abs(math.remainder(o["th"] - prev["th"], math.tau))
        dt = o["t"] - prev["t"]
        if d > 0.3 or dth > 1.0:
            seg.jumps += 1
            return
        seg.path_m += d
        seg.turned_rad += dth
        if (d > 0.002 or dth > 0.003) and 0 < dt < 0.5:
            seg.moving_s += dt

    def _check_bins(self, raw: bytes) -> None:
        try:
            pts = json.loads(raw)["pts"]
        except (ValueError, KeyError, TypeError):
            return
        for x, y in pts:
            b = math.degrees(math.atan2(y, x)) % STEP_DEG
            if abs(b - STEP_DEG / 2) < 0.25:
                self.on_centre += 1
            else:
                self.off_centre += 1

    def segment_at(self, s: float) -> Segment:
        """The segment playing `s` seconds into the recording."""
        found = self.segments[0]
        for seg in self.segments:
            if seg.start_s <= s:
                found = seg
        return found

    def events(self, first: Segment, start_s: float, end_s: float):
        """("begin" | "connect" | "reset", Segment) and ("state" | "scan" | "route", line),
        in file order, from the first state at or after start_s to end_s."""
        starts = {s.offset: s for s in self.segments}
        begun = False
        with self.path.open("rb") as f:
            f.seek(first.offset)
            while True:
                off = f.tell()
                raw = f.readline()
                if not raw:
                    return
                try:
                    o = json.loads(raw)
                    k = o["k"]
                except (ValueError, KeyError, TypeError):
                    continue
                if k == "state":
                    if o["wall"] - self.wall0 > end_s:
                        return
                    seg = starts.get(off)
                    if not begun:
                        if o["wall"] - self.wall0 < start_s:
                            if seg is not None:
                                first = seg
                            continue
                        begun = True
                        yield "begin", seg or first
                    elif seg is not None:
                        yield seg.starts, seg
                    yield "state", o
                elif begun and k in ("scan", "route"):
                    yield k, o

    def describe(self) -> str:
        size = self.path.stat().st_size / 1e6
        kinds = ", ".join(f"{v} {k}" for k, v in sorted(self.kinds.items()))
        lines = [f"{self.path}  {size:.1f} MB  {clock_text(self.end_s)} long  ({kinds})"]
        pts = self.on_centre + self.off_centre
        if pts:
            lines.append(f"  scan points on {STEP_DEG:g} deg bin centres: "
                         f"{100.0 * self.on_centre / pts:.1f}%")
        lines.append("  seg  starts    at        long     path    turned  moving  "
                     "scans  routes  spans       jumps")
        for s in self.segments:
            w, h = s.span_m
            lines.append(
                f"  {s.n:3d}  {s.starts:8s}  {clock_text(s.start_s):>8s}  "
                f"{clock_text(s.end_s - s.start_s):>7s}  {s.path_m:5.1f} m  "
                f"{math.degrees(s.turned_rad):6.0f}d  {s.moving_s:5.0f} s  {s.scans:5d}  "
                f"{s.routes:6d}  {w:4.1f}x{h:<4.1f} m  {s.jumps:5d}")
        return "\n".join(lines)


# ------------------------------------------------------------------ the wire


class PoseToWire:
    """Ticks and a gyro heading that make BridgeRobot's own odometry retrace
    recorded poses. Heading: the step's turn goes out as a change in `yaw`, which
    TankOdometry takes instead of the wheels'. Distance: both wheels move by the
    arc that lands the pose where the recording has it next. What a skid steer
    cannot do in one step (slide sideways) is left over, and made up on later
    steps, because each one aims from where the laptop really is.

    The DIFFERENCE between the wheels is the recorded one, so the page's "wheels
    vs gyro" slip is the real robot's; it moves no pose (the heading is the gyro's)."""

    GLITCH_M = 0.05           # recorded wheels disagreeing with the heading by more: a glitch

    def __init__(self, hello: dict) -> None:
        self.cpr = int(hello["counts_per_rev"])
        self.m_per_tick = math.tau * float(hello["wheel_radius_m"]) / self.cpr
        self.half_track = float(hello["track_width_m"]) * float(hello["scrub_factor"]) / 2.0
        self.max_step = 0.25 * self.cpr * self.m_per_tick   # well inside unwrap's half turn
        self.left = self.right = 0.0          # unwrapped tick counts
        self.yaw = 0.0
        self.pose: Pose | None = None         # the laptop's pose, as it will integrate it
        self.ticks: tuple[int, int] | None = None   # the recording's, at that pose
        self.error_m = 0.0                    # how far the laptop is from the recording, now
        self.worst_m = 0.0

    def restart(self) -> None:
        """The laptop's odometry was reset: its next state only records the ticks."""
        self.pose = self.ticks = None

    def wire(self) -> tuple[int, int, float]:
        return round(self.left) % self.cpr, round(self.right) % self.cpr, self.yaw

    def _unwrap(self, new: int, old: int) -> int:
        d = (new - old) % self.cpr
        return d - self.cpr if 2 * d >= self.cpr else d

    def to(self, x: float, y: float, th: float,
           ticks: tuple[int, int] | None = None) -> list[tuple[int, int, float]]:
        """The state(s) that take the laptop to this recorded pose: one, unless the
        step is too big for the encoders to wrap safely (a glitch in the file)."""
        last_ticks, self.ticks = self.ticks, ticks
        if self.pose is None:
            self.pose = Pose(x, y, th)
            return [self.wire()]
        p = self.pose
        dth = math.remainder(th - p.theta, math.tau)
        mid = p.theta + dth / 2.0
        chord = 1.0 if abs(dth) < 1e-9 else 2.0 * math.sin(dth / 2.0) / dth
        s = ((x - p.x) * math.cos(mid) + (y - p.y) * math.sin(mid)) / chord
        apart = 2.0 * dth * self.half_track            # right minus left, if the wheels agreed
        if ticks is not None and last_ticks is not None:
            recorded = (self._unwrap(ticks[1], last_ticks[1])
                        - self._unwrap(ticks[0], last_ticks[0])) * self.m_per_tick
            if abs(recorded - apart) < self.GLITCH_M:
                apart = recorded
        dl, dr = s - apart / 2.0, s + apart / 2.0
        n = max(1, math.ceil(max(abs(dl), abs(dr)) / self.max_step))
        out = []
        l0, r0, yaw0 = self.left, self.right, self.yaw
        for i in range(1, n + 1):
            old_l, old_r, old_yaw = round(self.left), round(self.right), self.yaw
            self.left = l0 + dl / self.m_per_tick * i / n
            self.right = r0 + dr / self.m_per_tick * i / n
            self.yaw = yaw0 + dth * i / n
            # Exactly what TankOdometry.update will do with this state.
            fwd = (round(self.left) - old_l + round(self.right) - old_r) / 2.0 * self.m_per_tick
            turn = math.remainder(self.yaw - old_yaw, math.tau)
            self.pose = integrate_twist(self.pose, fwd, 0.0, turn, 1.0)
            out.append(self.wire())
        self.error_m = math.hypot(self.pose.x - x, self.pose.y - y)
        self.worst_m = max(self.worst_m, self.error_m)
        return out


def scan_bins(pts: list, step_deg: float = STEP_DEG) -> list[int]:
    """Base-frame points (as the recorder wrote them, from bin centres) -> the Pi's
    bins: the nearest return in cm per step_deg of bearing, 0 for none."""
    n = round(360.0 / step_deg)
    step = math.radians(step_deg)
    bins = [0] * n
    for x, y in pts:
        cm = round(math.hypot(x, y) * 100.0)
        if not 0 < cm <= 65535:
            continue
        i = int((math.atan2(y, x) % math.tau) / step) % n
        if bins[i] == 0 or cm < bins[i]:
            bins[i] = cm
    return bins


class LinkLost(Exception):
    """The dashboard hung up."""


class Link:
    """One dashboard connection: hello out, then its commands read and dropped."""

    def __init__(self, sock: socket.socket, hello: dict) -> None:
        self.sock = sock
        self.hello = hello
        sock.settimeout(None)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.scan = False          # it subscribed to the lidar extensions
        self.seq = 0               # the last act it sent; echoed back like the Pi does
        self.acts = self.moves = 0
        self.closed = threading.Event()
        self._lock = threading.Lock()
        self.send(Hello(
            counts_per_rev=int(hello["counts_per_rev"]),
            wheel_radius_m=float(hello["wheel_radius_m"]),
            track_width_m=float(hello["track_width_m"]),
            scrub_factor=float(hello["scrub_factor"]),
            state_hz=float(hello.get("state_hz", 50.0)),
            timeout_ms=TIMEOUT_MS, motion_timeout_ms=MOTION_TIMEOUT_MS))
        threading.Thread(target=self._read, name="replay-reader", daemon=True).start()

    def send(self, msg) -> None:
        if self.closed.is_set():
            raise LinkLost("closed")
        try:
            with self._lock:
                self.sock.sendall(encode(msg))
        except OSError as exc:
            self.closed.set()
            raise LinkLost(str(exc)) from None

    def _read(self) -> None:
        try:
            for line in self.sock.makefile("rb"):
                if len(line) > MAX_LINE_BYTES + 2:
                    continue
                try:
                    msg = decode(line, allowed=CLIENT_MESSAGES)
                except ProtocolError as exc:
                    try:
                        self.send(Error(reason=str(exc)))
                    except LinkLost:
                        return
                    continue
                if isinstance(msg, Subscribe):
                    self.scan = msg.scan
                elif isinstance(msg, Act):
                    self.seq, self.acts = msg.seq, self.acts + 1
                    if msg.base_vx or msg.base_wz:
                        self.moves += 1
                elif isinstance(msg, (Estop, ClearEstop)):
                    print(f"  the page sent {msg.TYPE}: a replay has nothing to stop; ignored")
        except OSError:
            pass
        finally:
            self.closed.set()

    def close(self) -> None:
        self.closed.set()
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()


# ------------------------------------------------------------------ playback


class Player:
    """Plays [start_s, end_s] of a recording to whichever dashboard connects."""

    def __init__(self, rec: Recording, args: argparse.Namespace,
                 first: Segment, start_s: float, end_s: float) -> None:
        self.rec, self.args = rec, args
        self.first, self.start_s, self.end_s = first, start_s, end_s
        self.page = args.page.rstrip("/") if args.page else None
        # The replay's bridge clock: what `state.t` says. It starts high, and higher
        # on every run (faster than any --speed can use up), because a dashboard left
        # running keeps its pose history across reconnects: a clock that went back
        # would leave every new scan older than that history, and out of the map.
        self.t = 100.0 * (time.time() - 1.7e9)
        self.t_sent = 0.0             # ... on the last state that went out
        self.link: Link | None = None
        self.passes = 0
        self.warned_reset = False

    # -- the dashboard ---------------------------------------------------

    def _accept(self, srv: socket.socket, hello: dict) -> Link:
        if self.link is not None:
            self.link.close()
        print(f"  waiting for the dashboard on {self.args.host}:{self.args.port} ...")
        while True:
            try:
                sock, peer = srv.accept()
            except TimeoutError:
                continue
            try:
                self.link = Link(sock, hello)
            except LinkLost:
                continue
            print(f"  dashboard connected from {peer[0]}:{peer[1]}")
            return self.link

    def _press_reset(self) -> None:
        req = urllib.request.Request(self.page + "/reset", data=b"{}", method="POST",
                                     headers={"Content-Type": "application/json"})
        try:
            urllib.request.urlopen(req, timeout=3.0).close()
        except OSError as exc:
            print(f"  could not press Reset on {self.page} ({exc}); its map keeps what it had")

    def _emit(self, wire: tuple[int, int, float], estop: bool) -> None:
        """One state, stamped with the replay's clock as it stands."""
        left, right, yaw = wire
        self.link.send(State(seq=self.link.seq, t=round(self.t, 4), left_ticks=left,
                             right_ticks=right, estop=estop, yaw=yaw))
        self.t_sent = self.t

    def _drive(self, synth: PoseToWire, o: dict) -> None:
        """Take the dashboard to recorded state `o`, which self.t already stands at."""
        lt, rt = o.get("lt"), o.get("rt")
        ticks = (lt, rt) if isinstance(lt, int) and isinstance(rt, int) else None
        wires = synth.to(o["x"], o["y"], o["th"], ticks)
        t0, t1 = self.t_sent, self.t
        for i, w in enumerate(wires, 1):
            self.t = t0 + (t1 - t0) * i / len(wires)
            self._emit(w, bool(o.get("estop", False)))
        self.t = t1

    def _hold(self, synth: PoseToWire, estop: bool, seconds: float | None,
              until: threading.Event | None = None) -> None:
        """Stand still at the current pose, the link alive, for `seconds` (None: until
        `until` is set or the dashboard hangs up)."""
        period = 1.0 / HOLD_HZ
        end = None if seconds is None else time.monotonic() + seconds
        while (end is None or time.monotonic() < end) and not (until and until.is_set()):
            self.t += period
            self._emit(synth.wire(), estop)
            time.sleep(period)

    # -- one pass through the window -------------------------------------

    def run(self) -> None:
        srv = socket.create_server((self.args.host, self.args.port))
        srv.settimeout(0.5)
        try:
            while True:
                try:
                    self._pass(srv)
                    if not self.args.loop:
                        return
                except LinkLost as exc:
                    print(f"  the dashboard hung up ({exc}); starting over when it reconnects")
                    if self.link is not None:
                        self.link.close()
                        self.link = None
        finally:
            if self.link is not None:
                self.link.close()
            srv.close()

    def _pass(self, srv: socket.socket) -> None:
        a = self.args
        self.passes += 1
        synth: PoseToWire | None = None
        last_t = None                # the recording's clock at the last state on this link
        due = 0.0                     # when (monotonic) that state was due out
        last_sent = 0.0
        held = None                   # the last state thinned out, if none went since
        pending = None                # a scan waiting to go out just before the next state
        still_pose, still_since = None, None
        stats = dict(states=0, sent=0, scans=0, routes=0, cut_s=0.0, resets=0, links=0)
        last_print = time.monotonic()
        estop = False
        first_state = True
        for ev, o in self.rec.events(self.first, self.start_s, self.end_s):
            if ev in ("begin", "connect") or (ev == "reset" and not self.page):
                seg = o
                if ev == "reset" and not self.warned_reset:
                    print("  (a Reset in the original: without --page the replay hangs up "
                          "instead, which zeroes the pose but keeps the old map)")
                    self.warned_reset = True
                if ev != "begin":
                    print(f"  segment {seg.n} ({seg.starts} at {clock_text(seg.start_s)}): "
                          "the original link restarted here")
                    stats["links"] += 1
                # Starting over on the same link needs the page's Reset, or the
                # dashboard's odometry would read the fresh tick counts as a jump.
                reuse = (ev == "begin" and self.page and self.link is not None
                         and not self.link.closed.is_set() and self.link.hello == seg.hello)
                if reuse:
                    time.sleep(0.15)              # let the dashboard read what was sent
                else:
                    self._accept(srv, seg.hello)
                if ev == "begin" and self.page:
                    self._press_reset()           # a clean map and trail to start from
                synth, last_t = PoseToWire(seg.hello), None
                held = pending = None             # an old link's scan has no pose here
                first_state = ev == "begin"
                continue
            if ev == "reset":                     # with --page: press it, as the driver did
                held = pending = None             # the map it would go in is being cleared
                time.sleep(0.15)                  # let the dashboard read what was sent
                self._press_reset()
                synth.restart()
                last_t = None
                stats["resets"] += 1
                print(f"  segment {o.n}: Reset pressed at {clock_text(o.start_s)}, "
                      "as in the original")
                continue
            if ev == "route":
                stats["routes"] += 1
                continue
            if ev == "scan":
                if last_t is None or not self.link.scan:
                    continue
                scan = Scan(t=round(o["t"] - last_t + self.t, 4), step_deg=STEP_DEG,
                            ranges_cm=scan_bins(o["pts"]), near=[])
                if pending is not None:           # two scans, no state out between them:
                    self.link.send(pending)       # the page only notices the newest scan
                    if held is not None:          # each state brings, so send one between
                        self._drive(synth, held)
                        held, last_sent = None, due
                pending = scan
                stats["scans"] += 1
                continue
            # a state
            stats["states"] += 1
            estop = bool(o.get("estop", False))
            dt = 0.02 if last_t is None else o["t"] - last_t
            if dt > a.max_gap:
                stats["cut_s"] += dt - a.max_gap
                dt = a.max_gap
            dt = max(dt, 1e-3)
            last_t = o["t"]
            self.t += dt                          # the replay's clock keeps the recording's pace
            here = (o["x"], o["y"], o["th"])
            if still_pose is None or math.hypot(here[0] - still_pose[0], here[1] - still_pose[1]) \
                    > STILL_M or abs(math.remainder(here[2] - still_pose[2], math.tau)) > STILL_RAD:
                still_pose, still_since = here, o["wall"]
            rate = a.speed * (a.idle_speed if o["wall"] - still_since > IDLE_AFTER_S else 1.0)
            now = time.monotonic()
            due = now if first_state or now - due > 0.5 else due + dt / rate
            if not (first_state or synth.pose is None or due - last_sent >= 1.0 / MAX_SEND_HZ):
                held = o                          # thinned out; the next one covers it
                continue
            if due > now:
                time.sleep(due - now)
            if pending is not None:
                self.link.send(pending)
                pending = None
            self._drive(synth, o)
            held, last_sent = None, due
            stats["sent"] += 1
            if first_state:
                first_state = False
                self._start_hold(synth, estop)
                due = time.monotonic()
            if time.monotonic() - last_print >= 5.0:
                last_print = time.monotonic()
                s = o["wall"] - self.rec.wall0
                print(f"  {clock_text(s)}  ({100.0 * (s - self.start_s) / max(1e-9, self.end_s - self.start_s):3.0f}%)"
                      f"  pose {o['x']:+.2f}, {o['y']:+.2f} m {math.degrees(o['th']):+4.0f} deg"
                      f"  off by {1000 * synth.error_m:.0f} mm  scans {stats['scans']}"
                      f"  page moves ignored {self.link.moves}")
        if synth is None:
            print("  nothing to play in that window")
            return
        if pending is not None:                   # the hold's states bring it to the page
            self.link.send(pending)
        if held is not None:                      # end exactly where the recording does
            self._drive(synth, held)
            stats["sent"] += 1
        print(f"  end of the window: {stats['states']} states ({stats['sent']} sent), "
              f"{stats['scans']} scans, {stats['routes']} routes not shown, "
              f"{stats['cut_s']:.1f} s of pauses cut; pose error worst "
              f"{1000 * synth.worst_m:.0f} mm, at the end {1000 * synth.error_m:.0f} mm")
        if a.loop:
            self._hold(synth, estop, 2.0)
            return
        print("  holding the last pose; ctrl-c to stop")
        self._hold(synth, estop, None, self.link.closed)
        raise LinkLost("it hung up after the end")

    def _start_hold(self, synth: PoseToWire, estop: bool) -> None:
        if self.args.wait and sys.stdin.isatty():
            go = threading.Event()

            def ask() -> None:
                input("  at the first pose, map empty: press Enter to play ")
                go.set()

            threading.Thread(target=ask, daemon=True).start()
            self._hold(synth, estop, None, go)
        elif self.args.hold > 0:
            print(f"  holding the first pose for {self.args.hold:g} s")
            self._hold(synth, estop, self.args.hold)
        print("  playing")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("recording", nargs="+", help="a data/drives/*.jsonl file (several with --list)")
    ap.add_argument("--list", action="store_true", help="show the segments and exit")
    ap.add_argument("--segment", type=int, default=None, metavar="N",
                    help="play segment N from --list (then --start/--end count from its start)")
    ap.add_argument("--start", default=None, metavar="S", help='seconds or "m:ss" into the recording')
    ap.add_argument("--end", default=None, metavar="S", help='seconds or "m:ss" into the recording')
    ap.add_argument("--speed", type=float, default=1.0, help="2 plays twice as fast")
    ap.add_argument("--idle-speed", type=float, default=1.0, metavar="X",
                    help=f"standing still for over {IDLE_AFTER_S:g} s plays X times faster again "
                         "(nothing is dropped, still stretches just pass quicker)")
    ap.add_argument("--max-gap", type=float, default=0.5, metavar="S",
                    help="pauses in the recording longer than this are cut to it (default 0.5)")
    ap.add_argument("--hold", type=float, default=3.0, metavar="S",
                    help="stand still at the first pose this long once the dashboard connects")
    ap.add_argument("--wait", action="store_true", help="stand still at the first pose until Enter")
    ap.add_argument("--loop", action="store_true", help="start over at the end")
    ap.add_argument("--page", default=None, metavar="URL",
                    help="the dashboard, e.g. http://127.0.0.1:8795: press its Reset at the start "
                         "and wherever the driver pressed it")
    ap.add_argument("--host", default="127.0.0.1", help="address to listen on")
    ap.add_argument("--port", type=int, default=DEFAULT_REPLAY_PORT,
                    help=f"port to listen on (default {DEFAULT_REPLAY_PORT}; the real bridge's is 7777)")
    args = ap.parse_args()
    if args.speed <= 0 or args.idle_speed <= 0 or args.max_gap <= 0:
        ap.error("--speed, --idle-speed and --max-gap must be positive")

    if args.list:
        for path in args.recording:
            rec = Recording(path)
            print(rec.describe() if rec.segments else f"{path}: no states to replay")
            print()
        return 0
    if len(args.recording) != 1:
        ap.error("play one recording at a time")
    rec = Recording(args.recording[0])
    if not rec.segments:
        print(f"\n  {rec.path} has no states to replay\n", file=sys.stderr)
        return 1
    base, end = 0.0, rec.end_s
    if args.segment is not None:
        if not 0 <= args.segment < len(rec.segments):
            ap.error(f"--segment wants 0..{len(rec.segments) - 1}")
        seg = rec.segments[args.segment]
        base, end = seg.start_s, seg.end_s
    start_s = base + (parse_clock(args.start) if args.start else 0.0)
    end_s = min(end, base + parse_clock(args.end)) if args.end else end
    if end_s <= start_s:
        ap.error("the window is empty: --end must come after --start")
    first = rec.segment_at(start_s)
    fp = first.footprint or [0.25, 0.25, 0.20]
    print(f"\n  replaying {rec.path.name}  {clock_text(start_s)} to {clock_text(end_s)}"
          f"  ({clock_text((end_s - start_s) / args.speed)} at {args.speed:g}x)")
    print(f"  as the Pi on {args.host}:{args.port}; the dashboard:\n"
          f"    scripts/teleop.py --bridge 127.0.0.1:{args.port} --camera off "
          f"--footprint {','.join(f'{v:g}' for v in fp)}"
          + ("" if args.page else "   (add --page to this replay to clear its map)"))
    try:
        Player(rec, args, first, start_s, end_s).run()
    except KeyboardInterrupt:
        pass
    print("  stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
