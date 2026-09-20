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

CLICK TO GO. Click the map and the robot drives itself to that spot. With a
lidar, every scan (driven by hand or not) goes into a map of the room
(navigation/grid.py) and the click plans a route round what is on it and
follows it, re-planning as it sees more (navigation/navigator.py). Without the
map (no numpy) it steers round what the lidar sees right now (avoid.py), and
without a lidar it drives straight at the spot. The human
always wins: any drive key takes over, space or Esc stops it, and it only
drives itself while the page is open (close the tab and it stops). The goal is
in the odometry frame the page draws, so "there" means where the robot BELIEVES
that spot is: good for a few metres, drifting with every slipped turn until the
gyro and the map land. 1/2/3 set its top speed too.

Stdlib only, like the Pi side, so it also runs on a Pi or any bare python3;
the map needs numpy and is simply left out without it.
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

from retriever.navigation.avoid import AvoidConfig, bridge_scan_source
from retriever.navigation.drive import AvoidingGotoController, DifferentialGotoController, Limits
from retriever.navigation.pursuit import PathTracker, path_length, waypoint_curve
from retriever.types import Action, Pose

try:  # the map needs numpy; without it click-to-go falls back to avoid.py alone
    from retriever.navigation.navigator import FollowConfig, Mapper, PathFollower
    from retriever.navigation.pathplan import PlannerConfig
except ImportError:  # pragma: no cover - bare python3
    Mapper = PathFollower = FollowConfig = PlannerConfig = None  # type: ignore[assignment,misc]

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

    def step(self, now: float, target: tuple[float, float] | None = None) -> Action:
        """The next Action. `target` (vx, wz) replaces what the keys ask for:
        click-to-go drives through the same ramps."""
        c = self.config
        dt = 0.0 if self._last_step is None else min(max(0.0, now - self._last_step), 0.2)
        self._last_step = now
        tvx, twz = self.target(now) if target is None else target
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


ARRIVE_M = 0.08          # click-to-go is done this close to the spot (position only)
FACE_TOL_RAD = 0.05      # a leg that ends facing a heading is done within ~3 degrees
FACE_MIN_WZ = 0.25       # rad/s: slower than this and a skid-steer just sits there
MAX_GOAL_M = 15.0        # a click further than this is a mis-click
VIEWER_GRACE_S = 1.0     # page closed this long -> click-to-go stops
PICKUP_WAIT_S = 2.0      # a round trip waits this long at the far end
REVERSE_MAX_M = 1.5      # a click this close BEHIND the robot: back up to it, no spin
BEHIND_RAD = math.radians(100)   # "behind": the spot is more than this off the nose


@dataclass
class _Leg:
    """One stretch of a trip: drive to `goal`, and if `face`, turn to its heading.
    With a `path`, follow that curve there (pure pursuit), backwards if `reverse`."""

    goal: Pose
    face: bool
    label: str                # "there" | "home" | "path" | "back"
    path: list[tuple[float, float]] | None = None
    reverse: bool = False
    # A straight line to `goal`, built when the leg starts (from wherever the
    # robot is then): "rev" backs up it, "auto" picks forward or reverse by
    # the least turning (including the final turn, if `face`).
    straight: str | None = None


def least_turning_is_reverse(pose: Pose, goal: Pose, face: bool) -> bool:
    """Does backing up to `goal` need less turning than driving at it nose
    first (counting the turn to goal.theta at the end, if `face`)? The robot
    has an arm at the front, so only moves that don't end AT something use
    this; ties go forward."""
    bearing = math.atan2(goal.y - pose.y, goal.x - pose.x)
    turn = lambda a, b: abs(math.remainder(a - b, math.tau))   # noqa: E731
    fwd = turn(bearing, pose.theta) + (turn(goal.theta, bearing) if face else 0.0)
    rev = turn(bearing + math.pi, pose.theta) + (turn(goal.theta, bearing + math.pi) if face else 0.0)
    return rev < fwd - 0.1


@dataclass
class _Auto:
    """One click-to-go trip: one leg, or there-and-home."""

    legs: list[_Leg]          # legs[0] is the one being driven
    ctl: Any                  # PathFollower, AvoidingGotoController or DifferentialGotoController
    robot: Any                # the link it started on; a reconnect ends the trip
    started: float            # time.monotonic() the current leg started
    timeout_s: float
    avoiding: bool            # the route / steering uses the lidar
    wait_s: float = 0.0       # pause between legs (the pick-up)
    phase: str = "drive"      # drive | face | wait
    wait_until: float = 0.0
    dist: float | None = None

    @property
    def goal(self) -> Pose:
        return self.legs[0].goal


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
        mapping: bool = True,
        camera: Any = None,
        camera_url: str | None = None,
        map_min_range_m: float = 0.0,
    ) -> None:
        """mapping: build a map from the lidar's scans (needs numpy), and plan
        click-to-go routes over it. Off, or no numpy: avoid.py steering only.
        camera: a perception CameraObstacleSource (latest() / age(), see
        perception/depth_obstacles.py; perception/sim_camera.py stands in for
        it). Its points go into the map's camera layer, which blocks but never
        clears: routes go round what it sees, with or without a lidar."""
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
        self._history: deque[tuple[float, Pose]] = deque()   # ~2 s of (bridge t, pose)
        self.mapper = (Mapper(planner_config=PlannerConfig.for_footprint(*footprint))
                       if mapping and Mapper is not None else None)
        self.map_skipped = 0                   # scans left out: taken mid-turn
        self.camera = camera
        self._camera_t: float | None = None
        self.camera_url = camera_url      # an MJPEG stream for the page (cam_stream.py)
        # Returns this close to the lidar never reach the MAP. Brackets, the arm
        # and cable runs sit just outside the chassis and come back in nearly
        # every scan; mapped, they become a wall that rides along with the robot
        # -- which shows as clutter at its own edge and, worse, holds the
        # clearance-based speed limit down for an obstacle that is not there.
        # The safety bubble on the Pi is NOT filtered by this: it keeps full
        # close-range sight for stopping, with its own per-sector self-mask.
        self.map_min_range_m = float(map_min_range_m)
        self._last_cmd = Action()
        self._threads: list[threading.Thread] = []
        self._obs: Any = None                  # the latest Observation, for click-to-go
        self.auto: _Auto | None = None
        self.home = Pose()                     # where "go home" goes: the start, or set_home()
        self.turned = 0.0                      # rad, total rotation the odometry has seen (signed)
        self.calib: dict[str, float] | None = None    # a turn calibration in progress
        self.calib_result: dict[str, Any] | None = None
        self.auto_status = ""                  # driving | arrived | home | gave up | stopped
        self.auto_detail = ""
        # None: nobody counts viewers (tests, scripts). serve() sets 0, and from
        # then on click-to-go only drives while at least one page is watching.
        self.viewers: int | None = None
        self._viewers_gone: float | None = None

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
            if self.auto is not None and (self.core.fwd or self.core.turn):
                self._end_auto("stopped", "you took over")

    def halt(self) -> None:
        with self._lock:
            self.core.halt()
            self._end_auto("stopped", "stopped")
        self._send(Action())

    def estop(self) -> None:
        self.halt()
        robot = self.robot
        if robot is not None:
            robot.estop("teleop page")

    def goto(self, x: float, y: float, round_trip: bool = False,
             wait_s: float = PICKUP_WAIT_S) -> None:
        """Click-to-go: drive to (x, y) in the odometry frame. round_trip: then
        wait wait_s (the pick-up) and drive home, ending facing home's heading.
        Raises RuntimeError (said to the page as is) when it can't start."""
        gx, gy = float(x), float(y)
        if not (math.isfinite(gx) and math.isfinite(gy)):
            raise ValueError("x and y must be finite numbers")
        with self._lock:
            pose = self._seen.pose
            d = math.hypot(gx - pose.x, gy - pose.y)
            if d > MAX_GOAL_M:
                raise RuntimeError(f"that's {d:.0f} m away; click closer than {MAX_GOAL_M:.0f} m")
            bearing = math.atan2(gy - pose.y, gx - pose.x)
            behind = abs(math.remainder(bearing - pose.theta, math.tau)) > BEHIND_RAD
            if not round_trip and behind and d <= REVERSE_MAX_M:
                # a short hop to a spot behind: back up to it, no spin (nothing to face)
                legs = [_Leg(Pose(gx, gy, pose.theta), False, "there", straight="rev")]
            else:
                # nose first: an object is picked up with the arm, at the front.
                # Heading = the way it will be going: arriving needs no final turn.
                legs = [_Leg(Pose(gx, gy, bearing), False, "there")]
            if round_trip:
                # back the way it came, whichever way round turns least (usually reversing)
                legs.append(_Leg(self.home, True, "home", straight="auto"))
            self._start_trip(legs, wait_s if round_trip else 0.0)

    def follow_path(self, points: list[Any], come_back: bool = False,
                    reverse_back: bool = True, wait_s: float = PICKUP_WAIT_S) -> None:
        """Drive a smooth curve from here through the clicked waypoints (pure
        pursuit). come_back: then wait wait_s and retrace it to where it
        started, ending facing the way it started. reverse_back: retrace it
        backwards instead of turning round, which cancels the wheels' turn slip:
        every turn on the way back undoes one on the way out."""
        pts = []
        for q in points:
            x, y = float(q[0]), float(q[1])
            if not (math.isfinite(x) and math.isfinite(y)):
                raise ValueError("waypoints must be finite numbers")
            pts.append((x, y))
        if not pts:
            raise ValueError("no waypoints")
        if len(pts) > 50:
            raise ValueError("at most 50 waypoints")
        with self._lock:
            start = self._seen.pose
            curve = waypoint_curve([(start.x, start.y)] + pts)
            if len(curve) < 2 or path_length(curve) < 0.05:
                raise RuntimeError("the path is too short to drive")
            if path_length(curve) > 4 * MAX_GOAL_M:
                raise RuntimeError("that path is too long")
            end = curve[-1]
            legs = [_Leg(Pose(end[0], end[1], 0.0), False, "path", curve)]
            if come_back:
                legs.append(_Leg(start, True, "back", list(reversed(curve)), reverse_back))
            self._start_trip(legs, wait_s if come_back else 0.0)

    def go_home(self) -> None:
        """Drive back to home and turn to face the way it faced there: forwards
        or backwards, whichever needs less turning in all."""
        with self._lock:
            self._start_trip([_Leg(self.home, True, "home", straight="auto")], 0.0)

    def set_home(self) -> None:
        """Home is here, facing this way (it starts where teleop connected)."""
        with self._lock:
            self.home = self._seen.pose

    # -- turn calibration ---------------------------------------------------

    def calib_start(self) -> None:
        """Tape at the nose, then this; spin it N full turns by hand; calib_done(N)."""
        robot = self.robot
        if robot is None or self.link != "up":
            raise RuntimeError("not connected to the robot")
        odo = getattr(robot, "odometry", None)
        scrub = float(odo.geo.scrub_factor) if odo is not None else 1.0
        with self._lock:
            self._end_auto("stopped", "calibrating")
            self.calib = {"turned0": self.turned, "scrub": scrub}
            self.calib_result = None

    def calib_done(self, turns: float) -> dict[str, Any]:
        """The robot is back on the tape after `turns` full turns (either way).
        The odometry's count against the truth gives the scrub factor."""
        n = float(turns)
        if not (math.isfinite(n) and 0.5 <= n <= 10):
            raise ValueError("turns must be between 0.5 and 10")
        with self._lock:
            c = self.calib
            if c is None:
                raise RuntimeError("press Start first, with the robot lined up on the tape")
            odo = self.turned - c["turned0"]
            if abs(odo) < math.radians(90):
                raise RuntimeError("it has hardly turned: spin it the full turns first")
            truth = n * math.tau
            new = c["scrub"] * abs(odo) / truth
            self.calib = None
            self.calib_result = {
                "turns": n, "odometry_deg": round(math.degrees(abs(odo)), 1),
                "true_deg": round(math.degrees(truth), 1),
                "error_pct": round(100.0 * (abs(odo) - truth) / truth, 1),
                "scrub_was": round(c["scrub"], 4), "scrub_factor": round(new, 4),
            }
            return self.calib_result

    def calib_cancel(self) -> None:
        with self._lock:
            self.calib = None

    def _start_trip(self, legs: list[_Leg], wait_s: float) -> None:
        """Under self._lock."""
        robot = self.robot
        if robot is None or self.link != "up":
            raise RuntimeError("not connected to the robot")
        if getattr(robot, "estopped", False):
            raise RuntimeError("the e-stop is latched: clear it first")
        self.auto = _Auto(legs, None, robot, 0.0, 0.0, False, wait_s=wait_s)
        self._start_leg(self.auto)
        self.core.fwd = self.core.turn = 0.0     # no key target underneath it
        self.core._heard = None
        self.auto_status, self.auto_detail = "driving", ""

    def _start_leg(self, a: _Auto) -> None:
        """A fresh controller for a.legs[0], at the current gear's speed.
        Under self._lock."""
        robot = a.robot
        pose = self._seen.pose
        d = math.hypot(a.goal.x - pose.x, a.goal.y - pose.y)
        v, w = self.core.config.gears[self.core.gear]
        limits = Limits(v_max=v, w_max=w, pos_tol=ARRIVE_M)
        lidar = (getattr(robot, "scan", None) is not None
                 or getattr(robot, "bubble", None) is not None)
        leg = a.legs[0]
        if leg.straight is not None:
            here = (pose.x, pose.y)
            leg.reverse = (leg.straight == "rev" or
                           (leg.straight == "auto" and least_turning_is_reverse(pose, leg.goal, leg.face)))
            leg.path = [here, (leg.goal.x, leg.goal.y)]
            if (math.dist(here, leg.path[1]) > 0.05 and lidar and self.mapper is not None
                    and self.mapper.has_map() and not self._line_is_clear(here, leg.path[1])):
                leg.path, leg.reverse = None, False  # something in the way: the planner, nose first
        if leg.path is not None and math.dist(leg.path[0], leg.path[-1]) <= 0.05:
            leg.path = None                         # already there: straight to the final turn
            a.ctl = DifferentialGotoController(limits)
            d = 0.0
        elif leg.path is not None:                    # a curve or a straight line: follow it
            a.ctl = PathTracker(leg.path, limits, reverse=leg.reverse)
            d = path_length(leg.path)
        elif lidar and self.mapper is not None and self.mapper.has_map():
            a.ctl = PathFollower(self.mapper, limits, FollowConfig(),
                                 bubble=lambda: getattr(robot, "bubble", None))
        elif lidar:
            a.ctl = AvoidingGotoController(bridge_scan_source(robot), limits,
                                           AvoidConfig.for_footprint(*self.footprint))
        else:
            a.ctl = DifferentialGotoController(limits)
        a.avoiding = lidar and leg.path is None     # a drawn curve is followed as drawn
        a.phase, a.dist = "drive", d
        a.started = time.monotonic()
        a.timeout_s = max(20.0, 10.0 + 4.0 * d / v) + (10.0 if a.legs[0].face else 0.0)

    def cancel(self) -> None:
        with self._lock:
            self._end_auto("stopped", "cancelled")

    def viewer(self, joined: bool) -> None:
        """A page opened (True) or closed (False) its live stream."""
        with self._lock:
            n = max(0, (self.viewers or 0) + (1 if joined else -1))
            self.viewers = n
            self._viewers_gone = None if n else time.monotonic()

    def _end_auto(self, status: str, detail: str) -> None:
        """Under self._lock."""
        if self.auto is None:
            return
        self.auto = None
        self.auto_status, self.auto_detail = status, detail

    def _auto_target(self) -> tuple[float, float] | None:
        """This tick's (vx, wz) for click-to-go, or None: the keys drive.
        Under self._lock."""
        a = self.auto
        if a is None:
            return None
        now = time.monotonic()
        if self.robot is not a.robot:
            self._end_auto("stopped", "lost the link to the robot")
            return None
        if (self.viewers is not None and self._viewers_gone is not None
                and now - self._viewers_gone > VIEWER_GRACE_S):
            self._end_auto("stopped", "nobody is watching the page")
            return None
        if a.phase == "wait":
            if now < a.wait_until:
                return 0.0, 0.0
            self._start_leg(a)
        if now - a.started > a.timeout_s:
            self._end_auto("gave up", f"I didn't get there in {a.timeout_s:.0f} s")
            return None
        obs = self._obs
        if obs is None or now - self._seen.rx > 0.5:
            return 0.0, 0.0                             # no fresh state: hold still
        p = obs.base
        a.dist = math.hypot(a.goal.x - p.x, a.goal.y - p.y)
        if a.phase == "face":
            err = math.remainder(a.goal.theta - p.theta, math.tau)
            if abs(err) <= FACE_TOL_RAD:
                return self._leg_done(a, now, p)
            w = self.core.config.gears[self.core.gear][1]
            wz = max(-w, min(w, 2.2 * err))
            return 0.0, math.copysign(max(abs(wz), FACE_MIN_WZ), err)
        if a.dist <= ARRIVE_M and a.legs[0].path is None:
            return self._leg_arrived(a, now, p)
        step_observation = getattr(a.ctl, "step_observation", None)
        if step_observation is not None:
            action, done = step_observation(obs, a.goal)
        else:
            action, done = a.ctl.step(p, a.goal)
        failure = getattr(a.ctl, "failure", None)
        if failure is not None:
            self._end_auto("gave up", failure[0])
            return None
        if getattr(a.ctl, "remaining_m", None) is not None:
            a.dist = a.ctl.remaining_m                 # along the curve, not as the crow flies
        if done:
            return self._leg_arrived(a, now, p)
        return action.base_vx, action.base_wz

    def _line_is_clear(self, a: tuple[float, float], b: tuple[float, float]) -> bool:
        """No lethal cell of the map on the straight line a -> b."""
        from retriever.navigation.pathplan import _segment_cost

        try:
            return math.isfinite(_segment_cost(self.mapper.costmap(), a, b))
        except Exception:
            return False

    def _leg_arrived(self, a: _Auto, now: float, p: Pose) -> tuple[float, float] | None:
        """In position. Turn to face if the leg asks, else the leg is done."""
        leg = a.legs[0]
        if leg.face and abs(math.remainder(leg.goal.theta - p.theta, math.tau)) > FACE_TOL_RAD:
            a.phase = "face"
            return 0.0, 0.0
        return self._leg_done(a, now, p)

    def _leg_done(self, a: _Auto, now: float, p: Pose) -> tuple[float, float] | None:
        leg = a.legs.pop(0)
        cm = math.hypot(leg.goal.x - p.x, leg.goal.y - p.y) * 100
        deg = abs(math.degrees(math.remainder(leg.goal.theta - p.theta, math.tau)))
        if a.legs:                                   # the far end of a round trip: the pick-up
            a.phase, a.wait_until = "wait", now + a.wait_s
            self.auto_detail = f"{cm:.0f} cm from the spot"
            return 0.0, 0.0
        if leg.label in ("home", "back"):
            self._end_auto("home", f"{cm:.0f} cm and {deg:.0f}\u00b0 from where it started")
        else:
            self._end_auto("arrived", f"{cm:.0f} cm from the spot")
        return None

    def clear_estop(self) -> None:
        robot = self.robot
        if robot is not None:
            robot.clear_estop()

    def reset_pose(self) -> None:
        with self._lock:
            self._end_auto("stopped", "the pose was reset")
        robot = self.robot
        if robot is not None:
            robot.reset_pose(Pose())
        with self._lock:
            self._trail.clear()
            self._window.clear()
            self._history.clear()
            self._seen.odometer_m = 0.0
            self._seen.pose = Pose()
            self.home = Pose()
            if self.mapper is not None:
                self.mapper.reset()             # it was drawn in the frame just reset

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
            self._end_auto("stopped", "lost the link to the robot")
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
                try:
                    target = self._auto_target()
                except Exception as exc:   # a controller bug must not kill the control loop
                    self._end_auto("gave up", f"navigation error: {exc}")
                    target = None
                action = self.core.step(self.clock(), target)
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
            self._obs = obs
            p = obs.base
            s.odometer_m += math.hypot(p.x - s.pose.x, p.y - s.pose.y) if s.states > 1 else 0.0
            if s.states > 1:
                self.turned += math.remainder(p.theta - s.pose.theta, math.tau)
            s.pose = p
            if not self._trail or math.hypot(p.x - self._trail[-1][0],
                                             p.y - self._trail[-1][1]) >= 0.02:
                self._trail.append((p.x, p.y))
            self._window.append((obs.t, p))
            while len(self._window) > 2 and obs.t - self._window[0][0] > 0.3:
                self._window.popleft()
            self._history.append((obs.t, p, now))   # bridge clock, pose, laptop clock
            while len(self._history) > 2 and obs.t - self._history[0][0] > 2.0:
                self._history.popleft()
            t0, p0 = self._window[0]
            if obs.t - t0 > 0.05:
                dt = obs.t - t0
                dx, dy = p.x - p0.x, p.y - p0.y
                s.vx = (dx * math.cos(p0.theta) + dy * math.sin(p0.theta)) / dt
                s.wz = math.remainder(p.theta - p0.theta, math.tau) / dt
            cmd = self._last_cmd
        scan = getattr(robot, "scan", None)
        new_scan = scan is not None and scan.t != self._seen.scan_t
        if new_scan:
            self._seen.scan_t = scan.t
            self._map_scan(scan)
        self._map_camera()
        if self.recorder is None:
            return
        state = getattr(robot, "state", None)
        self.recorder.write(
            "state", t=round(obs.t, 4),
            lt=getattr(state, "left_ticks", None), rt=getattr(state, "right_ticks", None),
            x=round(p.x, 4), y=round(p.y, 4), th=round(p.theta, 5),
            cvx=round(cmd.base_vx, 3), cwz=round(cmd.base_wz, 3),
            estop=bool(getattr(state, "estop", False)))
        if new_scan:
            self.recorder.write("scan", t=round(scan.t, 4),
                                pts=[[round(x, 3), round(y, 3)] for x, y, *_ in scan.points])

    def _map_camera(self) -> None:
        """One packet of camera obstacle points into the map's camera layer, at
        the pose it was captured from. Their `t` is on THIS machine's clock."""
        cam, mapper = self.camera, self.mapper
        if cam is None or mapper is None:
            return
        try:
            packet = cam.latest()
        except Exception:                      # a broken stream must not stop odometry
            return
        if packet is None or packet.t == self._camera_t:
            return
        self._camera_t = packet.t
        at = self._pose_at(packet.t, laptop_clock=True)
        if at is None:
            return
        pose, wz = at
        if abs(wz) > 1.5:                      # mid-spin: too smeared to place
            return
        try:
            mapper.add_camera_points(pose, packet.points)
        except Exception:
            pass

    def _pose_at(self, t: float, laptop_clock: bool = False) -> tuple[Pose, float] | None:
        """Odometry at time t, interpolated, and the turn rate then. t is on the
        bridge's clock, or this machine's with laptop_clock. None if t is
        outside the ~2 s of history."""
        with self._lock:
            h = [(mono if laptop_clock else bt, pose) for bt, pose, mono in self._history]
        if len(h) < 2 or t < h[0][0] or t > h[-1][0] + 0.1:
            return None
        for (t0, a), (t1, b) in zip(h, h[1:]):
            if t0 <= t <= t1 or (t1 == h[-1][0] and t > t1):
                if t1 <= t0:
                    return b, 0.0
                s = min(1.0, (t - t0) / (t1 - t0))
                dth = math.remainder(b.theta - a.theta, math.tau)
                pose = Pose(a.x + s * (b.x - a.x), a.y + s * (b.y - a.y), a.theta + s * dth)
                return pose, dth / (t1 - t0)
        return None

    def _map_scan(self, scan: Any) -> None:
        """Put one scan in the map at the pose it was TAKEN from. A scan arrives
        up to ~0.2 s after it was taken; at 1 rad/s that is 11 degrees of smear
        if it went in at the current pose. Scans taken while spinning fast are
        left out: the Pi integrates a revolution over ~85 ms."""
        if self.mapper is None:
            return
        at = self._pose_at(scan.t)
        if at is None:
            return
        pose, wz = at
        if abs(wz) > 0.8 or abs(self._seen.wz) > 0.8:
            self.map_skipped += 1
            return
        try:
            near = self.map_min_range_m
            self.mapper.add_scan(pose, [(x, y) for x, y, *_ in scan.points
                                        if math.hypot(x, y) >= near])
        except Exception:   # a map bug must not take the observe loop (odometry) down
            self.map_skipped += 1

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
            a = self.auto
            plan = getattr(a.ctl, "plan", None) if a is not None else None
            route = getattr(a.ctl, "path", None) if a is not None else None
            snap["route"] = None if not route else [[round(x, 3), round(y, 3)] for x, y in route]
            snap["home"] = [round(self.home.x, 3), round(self.home.y, 3), round(self.home.theta, 4)]
            snap["calib"] = {
                "active": self.calib is not None,
                "turned_deg": None if self.calib is None else round(
                    math.degrees(self.turned - self.calib["turned0"]), 1),
                "result": self.calib_result,
            }
            snap["auto"] = {
                "status": "driving" if a is not None else self.auto_status,
                "detail": self.auto_detail,
                "leg": None if a is None else a.legs[0].label,
                "reverse": bool(a is not None and a.legs[0].reverse),
                "phase": None if a is None else a.phase,
                "then_home": bool(a is not None and len(a.legs) > 1),
                "goal": None if a is None else [round(a.goal.x, 3), round(a.goal.y, 3)],
                "dist": None if a is None or a.dist is None else round(a.dist, 2),
                "avoiding": bool(a is not None and a.avoiding),
                "plan": None if plan is None else {
                    k: plan.as_dict()[k] for k in ("status", "reason", "steer_deg", "speed")},
            }
        odo = getattr(robot, "odometry", None) if robot is not None else None
        hello = getattr(robot, "hello", None) if robot is not None else None
        snap["scrub"] = {"used": None if odo is None else odo.geo.scrub_factor,
                         "pi": None if hello is None else hello.scrub_factor}
        gyro_turn = getattr(odo, "turn_gyro_rad", 0.0) if odo is not None else 0.0
        snap["heading"] = {
            "source": getattr(odo, "heading_source", "wheels") if odo is not None else None,
            # how much more the wheels claim to turn than the gyro saw: the slip
            "slip_pct": (round(100.0 * (odo.turn_wheels_rad / gyro_turn - 1.0), 1)
                         if gyro_turn > math.radians(90) else None),
        }
        snap["camera_url"] = self.camera_url
        snap["estop"] = bool(robot is not None and robot.estopped)
        snap["watchdog"] = bool(robot is not None and robot.watchdog_tripped)
        bubble = getattr(robot, "bubble", None) if robot is not None else None
        snap["bubble"] = None if bubble is None else {
            "state": bubble.state, "reason": bubble.reason,
            "nearest_m": bubble.nearest_m, "stop_m": bubble.stop_m}
        scan = getattr(robot, "scan", None) if robot is not None else None
        mapper = self.mapper
        snap["map"] = None if mapper is None else mapper.page_view()
        snap["map_stats"] = None if mapper is None else {
            "scans": mapper.scans, "skipped": self.map_skipped,
            "camera_points": mapper.camera_points}
        cam = self.camera
        snap["camera"] = None if cam is None else {
            "age_s": None if not math.isfinite(cam.age()) else round(cam.age(), 2),
            "points": 0 if mapper is None else mapper.camera_points,
            "detail": cam.describe() if hasattr(cam, "describe") else ""}
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
                elif self.path == "/goto":
                    session.goto(body["x"], body["y"], round_trip=bool(body.get("round_trip")))
                elif self.path == "/path":
                    pts = body["points"]
                    if not isinstance(pts, list):
                        raise ValueError("points must be a list of [x, y]")
                    session.follow_path(pts, come_back=bool(body.get("come_back")),
                                        reverse_back=bool(body.get("reverse_back", True)))
                elif self.path == "/home":
                    session.go_home()
                elif self.path == "/sethome":
                    session.set_home()
                elif self.path == "/calib/start":
                    session.calib_start()
                elif self.path == "/calib/done":
                    session.calib_done(body.get("turns", 2))
                elif self.path == "/calib/cancel":
                    session.calib_cancel()
                elif self.path == "/cancel":
                    session.cancel()
                else:
                    self._reply(404, b'{"error":"no such command"}')
                    return
            except (ValueError, TypeError, KeyError) as exc:
                self._reply(400, json.dumps({"error": str(exc)}).encode())
                return
            except RuntimeError as exc:  # can't do that right now, and here's why
                self._reply(409, json.dumps({"error": str(exc)}).encode())
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
                session.viewer(True)
                try:
                    while not session._closed.is_set():
                        data = json.dumps(session.snapshot(), separators=(",", ":"))
                        self.wfile.write(f"data: {data}\n\n".encode())
                        self.wfile.flush()
                        session._closed.wait(0.1)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    return
                finally:
                    session.viewer(False)
            else:
                self._reply(404, b'{"error":"not found"}')

    return Handler


def serve(session: TeleopSession, host: str = "127.0.0.1", port: int = 8791) -> ThreadingHTTPServer:
    """Start the page's HTTP server in a daemon thread; returns it (.server_port)."""
    httpd = ThreadingHTTPServer((host, port), make_handler(session))
    httpd.daemon_threads = True
    with session._lock:                  # from now on, click-to-go needs a page watching
        if session.viewers is None:
            session.viewers = 0
            session._viewers_gone = time.monotonic()
    threading.Thread(target=httpd.serve_forever, name="teleop-http", daemon=True).start()
    return httpd
