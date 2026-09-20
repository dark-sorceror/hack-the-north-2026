#!/usr/bin/env python3
"""The central Pi's mission loop: a spoken sentence in, wheels and an arm out.

    python3 scripts/central_pi.py --dry-run                    # no hardware at all
    python3 scripts/central_pi.py --nav 10.0.0.50:8791 --fake-detect 1.6,0.2
    python3 scripts/central_pi.py --nav 10.0.0.50:8791 --arm sunrise@10.0.0.112

WHAT THIS IS. Voice works. Detection works. Driving works. The arm works. This
is the file that makes them one machine. It owns the sequencing, the dating of
detections, the narration, and every path by which the whole thing gives up.

WHAT IT IS NOT. It contains no perception, no control law and no model call. It
holds two SEAMS (`Voice` and `Detector` below) that the central Pi's own working
code plugs into, and it speaks to navigation and the arm over interfaces that
already exist. If you find yourself adding maths here, it belongs somewhere else.

THE FOUR MACHINES, and the only ways this process reaches them:

    central Pi (here) ──HTTP :8791──► Mac        /approach /state /home /halt
                      ──SSH────────► S100 arm    fetch.py, then arm_poses.py
                      ──(never)────► nav Pi      :7777 takes ONE client, the Mac

    The nav Pi is deliberately unreachable from here. If this process wants the
    robot to stop, it asks the Mac, and the Mac's own watchdogs are what make
    that true even when this process is dead.

STDLIB ONLY, on purpose: this runs on QNX, where the dependency you assumed is
present is the one that is not.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import re
import shlex
import subprocess
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Callable, Iterator, Protocol

LOG = logging.getLogger("central")

# Stop words are matched on the RAW transcript, before any model call, because
# "stop" must never depend on an API round trip. Same list as the robot app's.
STOP_WORDS = ("stop", "halt", "freeze", "stop stop", "emergency stop", "abort")
_STOP_RE = re.compile(r"\b(" + "|".join(re.escape(w) for w in STOP_WORDS) + r")\b", re.I)

# How far short of the object to stop. The claw already reaches ~0.20 m past the
# chassis, and the bubble stops the robot before that touches anything, so this
# is reach plus a margin -- not a guess at where the object is.
DEFAULT_STANDOFF_M = 0.35

# A detection older than this is refused by the Mac rather than placed wrong
# (it keeps ~2 s of pose history). Give up before it does, with a better message.
MAX_DETECTION_AGE_S = 1.5


def is_stop(text: str) -> bool:
    """True if a person asked the robot to stop. Checked first, always."""
    return bool(text) and bool(_STOP_RE.search(text))


# --------------------------------------------------------------- the two seams


@dataclass(frozen=True)
class Heard:
    """One turn from the person. `target` is what they asked for, if anything."""
    text: str
    target: str | None = None
    reply: str | None = None        # what the model already said back, if it did


@dataclass(frozen=True)
class Seen:
    """Where the object is, IN THE ROBOT'S FRAME: +x forward, +y left, metres.

    `age_s` is how old this fix was when it was handed over -- NOT a timestamp.
    Timestamps would need the two machines' clocks synced; an age needs nothing,
    and the Mac turns it back into a time on its own clock. This is exactly what
    the depth stream already does.
    """
    x: float
    y: float
    age_s: float
    label: str = ""
    confidence: float | None = None


class Voice(Protocol):
    """The central Pi's existing voice stack, whatever it is."""

    def listen(self) -> Heard | None: ...
    def speak(self, text: str) -> None: ...


class Detector(Protocol):
    """The central Pi's existing D435i detection, whatever it is.

    MUST return robot-frame metres, which means the camera's mounting is applied
    on this side. Sanity-check it before you trust it: put the goose dead ahead
    and confirm y is about 0; put it to the left and confirm y is positive. The
    lidar's mount was wrong by 225 degrees for several hours and every map it
    built was scrambled -- it looked like a mapping bug.
    """

    def locate(self, label: str) -> Seen | None: ...


# --------------------------------------------------------------- navigation


class NavError(RuntimeError):
    """The Mac said no, and said why. The text is meant to be read aloud."""


class Nav:
    """The Mac's teleop HTTP API. Every call is bounded; none of them block long."""

    def __init__(self, host: str, timeout_s: float = 5.0) -> None:
        self.base = host if "://" in host else f"http://{host}"
        self.timeout_s = timeout_s

    def _post(self, path: str, body: dict) -> dict:
        data = json.dumps(body).encode()
        req = urllib.request.Request(self.base + path, data=data, method="POST",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as r:
                raw = r.read()
                return json.loads(raw) if raw else {}
        except urllib.error.HTTPError as exc:
            detail = ""
            try:
                detail = json.loads(exc.read()).get("error", "")
            except Exception:
                pass
            # 409 is "can't right now, here's why" -- the message is a sentence.
            raise NavError(detail or f"the robot refused ({exc.code})") from None
        except urllib.error.URLError as exc:
            raise NavError(f"I can't reach the robot: {exc.reason}") from None

    def approach(self, seen: Seen, standoff_m: float = DEFAULT_STANDOFF_M) -> dict:
        """Send a robot-frame fix and let the Mac place it against its own pose."""
        return self._post("/approach", {"x": seen.x, "y": seen.y,
                                        "age_s": round(seen.age_s, 3),
                                        "standoff_m": standoff_m})

    def state(self) -> dict:
        req = urllib.request.Request(self.base + "/state")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as r:
                return json.loads(r.read())
        except Exception as exc:
            raise NavError(f"I can't reach the robot: {exc}") from None

    def home(self) -> None:
        self._post("/home", {})

    def halt(self) -> None:
        self._post("/halt", {})

    def cancel(self) -> None:
        self._post("/cancel", {})

    def watch(self, poll_hz: float = 3.0, timeout_s: float = 90.0) -> Iterator[dict]:
        """Yield the `auto` block each time it changes, until it settles or times out.

        Narrate from THIS, never from what you asked for: `auto.detail` is a
        sentence the navigation session writes to be read aloud, and it is the
        robot's account of what it is actually doing.
        """
        deadline = time.monotonic() + timeout_s
        last = None
        period = 1.0 / max(0.5, poll_hz)
        while time.monotonic() < deadline:
            auto = (self.state().get("auto") or {})
            status = auto.get("status")
            if auto != last:
                yield auto
                last = auto
            if status in ("arrived", "blocked", "gave up", "idle"):
                return
            time.sleep(period)
        raise NavError("it is taking too long; I've stopped it")


# --------------------------------------------------------------- the arm


class Arm:
    """The S100 over SSH. Two commands, and exit code is the answer.

    The board's own contract (SO-101/ARM_BOARD.md): pickup, then handoff, and
    only ONE may hold /dev/soarm_follower at a time. Never interrupt with
    Ctrl-C -- it orphans the serial port and can latch the gripper in overload.
    """

    def __init__(self, target: str | None, timeout_s: float = 60.0,
                 python: str = "./.venv/bin/python") -> None:
        self.target = target
        self.timeout_s = timeout_s
        self.python = python

    def _ssh(self, command: str, timeout_s: float) -> tuple[bool, str]:
        if self.target is None:
            LOG.info("[no arm] would run: %s", command)
            return True, ""          # let the caller's own wording stand
        argv = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
                self.target, command]
        LOG.info("arm: %s", command)
        try:
            p = subprocess.run(argv, capture_output=True, text=True, timeout=timeout_s)
        except subprocess.TimeoutExpired:
            # Do NOT kill it here. The board holds torque deliberately, and a
            # half-killed fetch.py is the orphaned-serial-port failure its own
            # docs warn about. Report and let a person look.
            return False, "the arm is taking too long; I've left it holding position"
        tail = (p.stderr or p.stdout or "").strip().splitlines()
        detail = tail[-1] if tail else ""
        return p.returncode == 0, detail

    def pick(self) -> tuple[bool, str]:
        ok, detail = self._ssh(f"{self.python} ~/fetch.py --no-handoff", self.timeout_s)
        if ok:
            return True, "Got it."
        return False, detail or "I couldn't pick it up."

    def handoff(self) -> tuple[bool, str]:
        ok, detail = self._ssh(
            "~/arm_poses.py goto handoff --hold-gripper --duration 6", 30.0)
        return ok, (detail or ("Here you go." if ok else "I couldn't hand it over."))


# --------------------------------------------------------------- the mission


def run_once(heard: Heard, *, voice: Voice, detector: Detector, nav: Nav, arm: Arm,
             standoff_m: float = DEFAULT_STANDOFF_M) -> bool:
    """One complete fetch. Returns True if the object came back.

    Every failure path ends the same way: say a sentence, stop the robot, return.
    Nothing here leaves the base moving.
    """
    say = voice.speak

    # 1. Stop words first, before anything that could be slow or clever.
    if is_stop(heard.text):
        nav.halt()
        say("Stopping.")
        return False

    target = heard.target or ""
    if not target:
        say("I didn't catch what you wanted.")
        return False

    if heard.reply:
        say(heard.reply)

    # 2. Where is it? The hint the person gave is already inside `target` --
    #    it is a prior over the search, not a coordinate, and the detector is
    #    what resolves it against the actual frame.
    seen = detector.locate(target)
    if seen is None:
        say(f"I can't see the {target} from here.")
        return False
    if seen.age_s > MAX_DETECTION_AGE_S:
        say("I saw it, but the picture was too old to trust. Let me look again.")
        return False

    LOG.info("located %s at x=%.2f y=%.2f (%.2f s old, conf=%s)",
             target, seen.x, seen.y, seen.age_s, seen.confidence)

    # 3. Hand the fix to navigation in the ROBOT's frame and let the Mac place
    #    it against the pose it actually had when the camera saw it.
    try:
        goal = nav.approach(seen, standoff_m=standoff_m)
    except NavError as exc:
        say(str(exc))
        return False
    LOG.info("goal %s", goal)
    say(f"I see the {target}. Going to get it.")

    # 4. Narrate what it IS doing, from /state -- not what we asked for.
    arrived = False
    try:
        for auto in nav.watch():
            detail = (auto.get("detail") or "").strip()
            if detail:
                say(detail)
            if auto.get("status") == "arrived":
                arrived = True
    except NavError as exc:
        nav.halt()
        say(str(exc))
        return False

    if not arrived:
        nav.halt()
        say("I couldn't get there.")
        return False

    # 5. The arm closes the loop on its OWN view. The coordinates that sent the
    #    robot here are stale by construction: odometry drifts, arrival is only
    #    good to about 0.15 m, and the camera at the end is the truth.
    ok, detail = arm.pick()
    say(detail)
    if not ok:
        return False

    # 6. Home, nose first, so the arm faces whoever is collecting.
    try:
        nav.home()
        for auto in nav.watch():
            pass
    except NavError as exc:
        say(str(exc))
        return False

    ok, detail = arm.handoff()
    say(detail)
    return ok


# --------------------------------------------------------------- stand-ins


class ConsoleVoice:
    """Types instead of listening. Real voice replaces this; the seam is the same."""

    def __init__(self, target: str = "goose", once: str | None = None) -> None:
        self.target, self.once, self.done = target, once, False

    def listen(self) -> Heard | None:
        if self.once is not None:
            if self.done:
                return None
            self.done = True
            text = self.once
        else:
            try:
                text = input("say something> ").strip()
            except EOFError:
                return None
        if not text:
            return None
        target = self.target if self.target.lower() in text.lower() else self.target
        return Heard(text=text, target=target,
                     reply=None if is_stop(text) else "Okay.")

    def speak(self, text: str) -> None:
        print(f"  robot: {text}", flush=True)


class FixedDetector:
    """Always reports the same place. For testing the drive half with no camera."""

    def __init__(self, x: float, y: float) -> None:
        self.x, self.y = x, y

    def locate(self, label: str) -> Seen | None:
        return Seen(x=self.x, y=self.y, age_s=0.05, label=label, confidence=1.0)


class NoDetector:
    def locate(self, label: str) -> Seen | None:
        return None


# --------------------------------------------------------------- entry point


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--nav", default="127.0.0.1:8791",
                    help="the Mac's teleop API (host:port)")
    ap.add_argument("--arm", default=None,
                    help="the S100 over ssh, e.g. sunrise@10.0.0.112; omit to skip the arm")
    ap.add_argument("--target", default="goose", help="what to fetch")
    ap.add_argument("--standoff", type=float, default=DEFAULT_STANDOFF_M,
                    help="stop this far short of it, metres")
    ap.add_argument("--fake-detect", metavar="X,Y",
                    help="skip the camera; pretend the object is at robot-frame X,Y")
    ap.add_argument("--once", metavar="TEXT",
                    help="run one mission with this as the spoken text, then exit")
    ap.add_argument("--dry-run", action="store_true",
                    help="touch no hardware: print the calls that would be made")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)

    logging.basicConfig(level=logging.INFO if args.verbose else logging.WARNING,
                        format="%(name)s: %(message)s")

    voice: Voice = ConsoleVoice(args.target, once=args.once)

    detector: Detector
    if args.fake_detect:
        try:
            xs, ys = args.fake_detect.split(",")
            detector = FixedDetector(float(xs), float(ys))
        except ValueError:
            ap.error("--fake-detect wants X,Y in metres, e.g. 1.6,0.2")
    elif args.dry_run:
        detector = FixedDetector(1.5, 0.0)
    else:
        # The central Pi's real detector plugs in here. It must satisfy
        # Detector.locate(label) -> Seen | None, in ROBOT-frame metres.
        print("no detector wired: pass --fake-detect X,Y, or plug the real one in "
              "at this line in scripts/central_pi.py", file=sys.stderr)
        return 2

    if args.dry_run:
        class _DryNav(Nav):
            def _post(self, path, body):
                print(f"  POST {path} {json.dumps(body)}")
                return {"x": 0.0, "y": 0.0, "standoff_m": body.get("standoff_m", 0.0)}

            def state(self):
                return {"auto": {"status": "arrived", "detail": "I'm there."}}
        nav: Nav = _DryNav(args.nav)
        arm = Arm(None)
    else:
        nav = Nav(args.nav)
        arm = Arm(args.arm)

    try:
        while True:
            heard = voice.listen()
            if heard is None:
                return 0
            try:
                run_once(heard, voice=voice, detector=detector, nav=nav, arm=arm,
                         standoff_m=args.standoff)
            except Exception as exc:               # never leave the base moving
                LOG.exception("mission failed")
                try:
                    nav.halt()
                except Exception:
                    pass
                voice.speak(f"Something went wrong: {exc}")
            if args.once is not None:
                return 0
    except KeyboardInterrupt:
        try:
            nav.halt()
        except Exception:
            pass
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
