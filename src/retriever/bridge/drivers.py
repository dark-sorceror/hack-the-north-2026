"""The real wheels, and one HardwareDriver assembled from the parts the robot has.

The bridge wants a single driver, but the robot is a base on one serial bus, arms on
another and maybe a vacuum on a GPIO pin, each written and tested on its own. This module
composes them. Routing is decided once, at construction: a joint name owned by two arms is
a wiring mistake and fails at startup, not when the arm first moves. The one rule that
matters is in `stop()`: the base stops FIRST, because it is the part that can hit someone,
and every part gets its stop even if an earlier one raised; the first error is re-raised
afterwards so the failure is still heard.

  DDSM115Driver    REAL. Four Waveshare DDSM115 hub motors on one RS485 bus
                   (USB-RS485 adapter), built on the frame/CRC/parse functions
                   of a teammate's known-working ddsm115.py (vendored as
                   bridge/ddsm115.py). Unit-tested against a byte-level fake
                   bus, and driven on the robot: the mirrored IDs and the
                   position sign are measured there. counts_per_rev and the
                   wheel radius are still unverified: scripts/wheel_check.py
                   checks them.

Import cost: this module stays stdlib-only at import time. The device libraries
(pyserial, the vendored ddsm115) are imported inside the methods that need them,
so `--driver fake` never touches them.
"""

from __future__ import annotations

import logging
import math
import os
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, Mapping, Protocol

from retriever.navigation.kinematics import TankGeometry
from retriever.navigation.odometry import unwrap_ticks

logger = logging.getLogger(__name__)


class BaseDriver(Protocol):
    """The drive base: two sides of wheels with encoders."""

    counts_per_rev: int

    def set_wheels(self, left_mps: float, right_mps: float) -> None:
        """Command left and right rim speeds in m/s."""
        ...

    def read_ticks(self) -> tuple[int, int]:
        """The (left, right) encoder counts."""
        ...

    def battery(self) -> float | None:
        """Charge in 0..1, or None if the base cannot tell."""
        ...

    def stop(self) -> None:
        """Wheels to zero now."""
        ...


class ArmDriver(Protocol):
    """One arm, optionally with a gripper."""

    joint_names: tuple[str, ...]
    has_gripper: bool

    def set_joints(self, targets: dict[str, float]) -> None:
        """Move these joints toward targets in radians."""
        ...

    def set_gripper(self, position: float) -> None:
        """Move the gripper toward `position`."""
        ...

    def read_joints(self) -> dict[str, float]:
        """Joint positions in radians, with "gripper" if the arm has one."""
        ...

    def gripper_load(self) -> float:
        """Normalised gripper servo current, 0..1."""
        ...

    def hold(self) -> None:
        """Stop moving and keep torque on, so the arm holds where it is."""
        ...


class VacuumDriver(Protocol):
    """A suction pump on a relay."""

    def set_vacuum(self, on: bool) -> None:
        """Switch the pump on or off."""
        ...


# ---------------------------------------------------------------- base: DDSM115 facts
#
# Waveshare DDSM115 over RS485, from the Waveshare wiki
# (https://www.waveshare.com/wiki/DDSM115) and the teammate's ddsm115.py:
#   115200 8N1, 10-byte frames, CRC-8/MAXIM over bytes 0..8, one request, one reply.
#   0x64 drive   [id, 0x64, v_hi, v_lo, 0, 0, accel, brake, 0, crc]; velocity mode
#                (the power-up default) takes rpm -330..330; brake byte 0xFF brakes.
#       reply    id, mode, current i16, rpm i16, POSITION u16, error, crc
#   0x74 query   same reply, but byte 6 = winding temperature C, byte 7 = u8 position
#   0xA0 mode    no CRC, no reply. 1 current, 2 velocity, 3 position.
#   error bits   0 sensor, 1 overcurrent, 2 phase overcurrent, 3 stall
# Not in the protocol at all: bus voltage, and ANY command timeout. A motor keeps
# its last setpoint until told otherwise, so only this driver's stop() -- driven
# by the bridge's watchdog, deadman and e-stop -- or cutting motor power stops it.

# The wiki gives position-loop commands as 0..32767 = 0..360 deg and shows the 0x64
# reply's position as 16 bits without restating its range; Waveshare's own library
# decodes it as (data[6] << 8) | data[7]. So one wheel turn = 32768 counts -- the
# encoder itself is 4096 counts/rev (wiki spec table), so it moves in steps of 8.
# CONFIRM ON HARDWARE: `scripts/wheel_check.py rev ID` turns one wheel exactly this
# many counts and asks whether the tape mark came back to its start. A position
# >= counts_per_rev is caught at runtime and refuses odometry.
DDSM115_COUNTS_PER_REV = 32768
# Measured on the robot, 2026-09-19: a DDSM115's position counter runs DOWN
# while it turns at +rpm (the wheel's own velocity sign). Every delta read off
# it is multiplied by this before the motor's mounting sign. Without it the
# wheels' odometry reads backwards: W moved the robot, odometry said it reversed.
DDSM115_POSITION_SIGN = -1
# Which motors are mirror-mounted (+rpm drives the robot BACKWARD), measured on
# the robot the same day: the teammate's FLIPPED = {3, 4} drove it back-end
# first and turned it right on "left". The vendored ddsm115.py keeps its own.
ROBOT_FLIPPED_IDS = (1, 2)
DDSM115_MAX_RPM = 330               # protocol limit; rated 115 rpm, ~200 rpm no-load
DDSM115_BAUD = 115200
WCH_USB_VID = 0x1A86                # WCH CH34x: the Waveshare USB-RS485 adapter
DDSM115_MODES = {1: "CURRENT", 2: "VELOCITY", 3: "POSITION"}
DDSM115_ERROR_BITS = ((0x01, "sensor"), (0x02, "overcurrent"),
                      (0x04, "phase overcurrent"), (0x08, "stall"))
# Replaces the vendored module's own ImportError, which names an offline wheel
# this repository does not ship.
PYSERIAL_MISSING = ("pyserial is not installed, and the DDSM115 wheels need it: on the Pi, "
                    "pi/setup.sh installs it (python3-serial); elsewhere, pip install pyserial")


def _ddsm115() -> Any:
    """The vendored teammate module. Imported late: only real wheels need it.
    Its frame/CRC/parse functions work without pyserial; its Bus does not."""
    from retriever.bridge import ddsm115

    return ddsm115


def mps_to_rpm(v_mps: float, wheel_radius_m: float) -> float:
    """Rim speed -> shaft rpm:  rpm = v / (2*pi*r) * 60."""
    return v_mps / (2.0 * math.pi * wheel_radius_m) * 60.0


def rpm_to_mps(rpm: float, wheel_radius_m: float) -> float:
    return rpm / 60.0 * 2.0 * math.pi * wheel_radius_m


def ddsm115_error_names(err: int) -> list[str]:
    names = [name for bit, name in DDSM115_ERROR_BITS if err & bit]
    if err & ~0x0F:
        names.append(f"unknown bits 0x{err & ~0x0F:02x}")
    return names


def parse_id_list(spec: str) -> tuple[int, ...]:
    """'1,2' or '1-4' or '' -> motor IDs, for command-line flags."""
    ids: list[int] = []
    for part in (p.strip() for p in spec.split(",")):
        if not part:
            continue
        lo, sep, hi = part.partition("-")
        try:
            ids.extend(range(int(lo), int(hi) + 1) if sep else [int(part)])
        except ValueError:
            raise ValueError(f"bad motor ID list {spec!r}; expected e.g. 1,2 or 1-4") from None
    return tuple(ids)


def find_wheel_port(env: Mapping[str, str] | None = None, comports: Iterable[Any] | None = None,
                    exclude: Iterable[str | None] = ()) -> str:
    """$DDSM115_PORT, else THE one WCH (VID 0x1A86) USB serial adapter.

    The vendored find_port() takes the first WCH device it sees. Another USB
    serial adapter on the robot can be a WCH chip too, so with two of them that
    guess could aim wheel frames at the wrong bus. This refuses to guess and
    names the candidates instead. `exclude` drops ports already taken.
    """
    env = os.environ if env is None else env
    if env.get("DDSM115_PORT"):
        return env["DDSM115_PORT"]
    if comports is None:
        try:
            comports = _ddsm115().list_ports.comports()
        except ImportError:
            raise ImportError(PYSERIAL_MISSING) from None
    taken = {os.path.realpath(p) for p in exclude if p}
    wch = [p for p in comports if getattr(p, "vid", None) == WCH_USB_VID
           and os.path.realpath(p.device) not in taken]
    if len(wch) == 1:
        return wch[0].device
    if not wch:
        raise ConnectionError(
            "no Waveshare USB-RS485 adapter (WCH, USB VID 1a86) found for the wheels. Plug "
            "it in, or pass --wheel-port (or set DDSM115_PORT). `ls /dev/serial/by-id/` "
            "lists what is there.")
    raise ConnectionError(
        "several WCH USB serial adapters, can't tell which one is the wheel bus: "
        + ", ".join(f"{p.device} ({getattr(p, 'description', '') or '?'})" for p in wch)
        + ". Pass --wheel-port /dev/serial/by-id/<the RS485 adapter>.")


# ---------------------------------------------------------------- base: the bus


class DDSM115Bus:
    """One DDSM115 RS485 bus: bounded request/reply, nothing else.

    The I/O is the teammate's Bus._xfer (flush input, write 10 bytes, read 10)
    with three changes for callers that must never hang or be fooled:
      - short timeouts. One transaction costs at most write_timeout_s +
        reply_timeout_s however dead the bus is (pyserial's read timeout is
        for the whole read, not per byte);
      - a reply only counts if its CRC is good, it names the motor we asked,
        and its mode byte is a real mode (1-3). An adapter that echoed our own
        frame back would otherwise pass: that echo has a valid CRC and our ID;
      - no sign flips: replies are the motor's own, raw view.

    `ser` is anything with write/read/reset_input_buffer/close and settable
    timeout/write_timeout: pyserial's Serial, or a test fake.
    """

    def __init__(self, ser: Any, *, reply_timeout_s: float = 0.010,
                 write_timeout_s: float = 0.010, port: str = "") -> None:
        self._dd = _ddsm115()
        self.ser = ser
        self.port = port or str(getattr(ser, "port", "") or "")
        self.reply_timeout_s = reply_timeout_s
        self.write_timeout_s = write_timeout_s
        ser.timeout = reply_timeout_s        # the whole bound rests on these two
        ser.write_timeout = write_timeout_s
        self._closed = False

    @classmethod
    def open(cls, port: str | None = None, *, reply_timeout_s: float = 0.010,
             write_timeout_s: float = 0.010, exclude: Iterable[str | None] = ()) -> DDSM115Bus:
        dd = _ddsm115()
        port = port or find_wheel_port(exclude=exclude)
        kwargs: dict[str, Any] = {"timeout": reply_timeout_s, "write_timeout": write_timeout_s}
        if os.name == "posix":
            # A second program on this port (wheel_check, the teammate's script)
            # is refused at open instead of becoming a second bus master.
            kwargs["exclusive"] = True
        try:
            ser = dd.serial.serial_for_url(port, DDSM115_BAUD, **kwargs)
        except ImportError:
            raise ImportError(PYSERIAL_MISSING) from None
        except Exception as exc:  # SerialException, FileNotFoundError, PermissionError, busy
            raise ConnectionError(f"can't open the wheel bus {port}: {exc}") from exc
        return cls(ser, reply_timeout_s=reply_timeout_s, write_timeout_s=write_timeout_s,
                   port=port)

    @property
    def transaction_bound_s(self) -> float:
        """Longest one transact() can block."""
        return self.write_timeout_s + self.reply_timeout_s

    def transact(self, mid: int, pkt: bytes) -> tuple[dict[str, Any] | None, str]:
        """One frame out, its reply back: (reply, "") or (None, why not).
        Serial exceptions propagate; the caller decides whether to carry on."""
        self.ser.reset_input_buffer()
        self.ser.write(pkt)
        raw = self.ser.read(10)
        r = self._dd.parse(raw)
        if r is None:
            if not raw:
                return None, "no reply"
            return None, f"bad reply ({len(raw)} bytes: {bytes(raw).hex(' ')})"
        if r["id"] != mid:
            return None, f"the reply came from ID {r['id']}"
        if r["mode"] not in DDSM115_MODES:
            return None, (f"not a motor reply (mode byte 0x{r['mode']:02x}; "
                          "is the adapter echoing our own frames?)")
        return r, ""

    def send(self, pkt: bytes) -> None:
        """A frame that gets no reply (the 0xA0 mode switch)."""
        self.ser.write(pkt)

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self.ser.close()


# ---------------------------------------------------------------- base: the driver


@dataclass
class _Motor:
    id: int
    side: str
    sign: int                   # -1 for a mirror-mounted motor: + rpm drives the robot back
    raw: int | None = None      # last u16 position from a 0x64 reply
    count: int = 0              # this motor's forward-positive counts since start
    misses: int = 0             # transactions in a row without a valid reply
    replied: bool = False       # in the latest sync
    last_ok: float = -math.inf
    why: str = ""               # why the latest transaction failed
    err: int = 0
    mode: int | None = None
    rpm: int = 0                # forward-positive
    current_a: float = 0.0
    temp_c: int | None = None


@dataclass
class _Side:
    name: str
    ids: tuple[int, ...]
    live: frozenset[int] = frozenset()   # motors odometry is using right now
    base: float = 0.0
    ref: dict[int, int] = field(default_factory=dict)
    snap_value: float = 0.0              # value at the last sync all live motors answered
    snap_counts: dict[int, int] = field(default_factory=dict)
    ever_live: bool = False


class DDSM115Driver:
    """Four Waveshare DDSM115 hub motors, skid-steer, on one RS485 bus.

    Speaks the motors' own protocol through a USB-RS485 adapter, with the frame,
    CRC and parse functions of the teammate's known-working ddsm115.py.

    Conventions (defaults UNVERIFIED on the robot; scripts/wheel_check.py sides):
      left_ids / right_ids   (1, 2) / (3, 4).
      flipped_ids            ROBOT_FLIPPED_IDS = (1, 2), measured: mirror-mounted
                             motors, so they get -rpm and +rpm is robot-forward
                             on every ID. The flip follows the motor ID, not the
                             side, because it is about how that motor is mounted.
      counts_per_rev         32768 (see DDSM115_COUNTS_PER_REV).
      wheel radius           geo.wheel_radius_m. The TankGeometry default is a
                             placeholder: measure the tyre.

    set_wheels(l, r)   m/s -> rpm = v / (2*pi*r) * 60, rounded, clamped to
                       +-max_rpm, then one velocity frame per motor (sides
                       interleaved, so they change speed ~one frame apart).
    read_ticks()       (left, right), each rising when that side rolls forward
                       and wrapping at counts_per_rev. Every 0x64 frame is
                       answered with position, so re-sending the current
                       setpoints IS the position read. Each motor's raw u16 is
                       unwrapped on its own (sign-flipped for mirrored motors,
                       which the vendored Bus does not do for position); a side
                       is the mean of its live motors' counts.
    stop()             BRAKE (0x64, brake byte 0xFF) to every motor; tries all
                       even if some raise, then raises if any did not confirm.
    battery()          None: the protocol has no voltage reading.

    Missing replies are never read as "not moving":
      - a single miss only delays that motor's count to its next reply;
      - `miss_limit` misses in a row drop the motor from odometry, logged, and
        the side carries on from its other motor with no distance lost (it is
        re-anchored from the last sync where both answered);
      - a side with NO motor answering is blind: read_ticks() and any non-zero
        set_wheels() raise, so the bridge stops streaming state and stops the
        base instead of reporting a frozen count. Zero and stop() always go out.
    Latched faults, which need a bridge restart: a motor reporting a mode other
    than VELOCITY (something else is on the bus; it gets read-only queries from
    then on, because in POSITION mode a drive frame is an angle target), and a
    position >= counts_per_rev (the constant is wrong, so every distance is).
    diagnostics() has the per-motor detail; the log has every transition.

    Blocking: the bridge calls this from its asyncio loop, so a slow bus delays
    the watchdog. Every public call does at most ONE round of one transaction
    per motor, so it blocks for at most
        len(motors) * (write_timeout_s + reply_timeout_s) = 4 * 20 ms = 80 ms
    (worst_case_call_s), and ~4 * 10 ms when the motors are silent but the port
    is fine. A healthy round is ~4 * 2 ms. read_ticks() reuses a round younger
    than reuse_s instead of starting another.

    No I/O thread, on purpose. The loop is the only thing touching the port, so
    there is no lock-step between a writer thread and stop() to get wrong, and
    the fakes test it deterministically. A thread would not make the robot
    safer: the motors hold their last setpoint whether or not anyone re-sends
    it, so a hung loop leaves the base driving either way. That case needs
    motor power cut (or a systemd watchdog restart: the constructor brakes).
    """

    def __init__(
        self,
        port: str | None = None,
        geo: TankGeometry = TankGeometry(),
        *,
        left_ids: Iterable[int] = (1, 2),
        right_ids: Iterable[int] = (3, 4),
        flipped_ids: Iterable[int] | None = None,
        counts_per_rev: int = DDSM115_COUNTS_PER_REV,
        accel: int = 0,
        max_rpm: int = DDSM115_MAX_RPM,
        reply_timeout_s: float = 0.010,
        miss_limit: int = 3,
        reuse_s: float = 0.015,
        bus: DDSM115Bus | None = None,
        exclude_ports: Iterable[str | None] = (),
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._dd = _ddsm115()
        left, right = tuple(int(i) for i in left_ids), tuple(int(i) for i in right_ids)
        flipped = set(ROBOT_FLIPPED_IDS if flipped_ids is None
                      else (int(i) for i in flipped_ids))
        every = left + right
        if not left or not right:
            raise ValueError("each side needs at least one motor ID")
        if len(set(every)) != len(every):
            raise ValueError(f"a motor ID appears twice: left {left}, right {right}")
        if any(not 1 <= i <= 253 for i in every):
            raise ValueError(f"motor IDs must be 1..253, got {every}")
        if not flipped <= set(every):
            raise ValueError(f"flipped IDs {sorted(flipped - set(every))} are on neither side")
        if not 0 <= accel <= 255:
            raise ValueError("accel is one byte, 0..255")
        if counts_per_rev < 2:
            raise ValueError("counts_per_rev must be at least 2")
        self.geo = geo
        self.counts_per_rev = counts_per_rev
        self.accel = accel
        self.max_rpm = min(abs(int(max_rpm)), DDSM115_MAX_RPM)
        self.miss_limit = max(1, miss_limit)
        self.reuse_s = reuse_s
        self.clock = clock
        self._sleep = sleep
        self.left_ids, self.right_ids, self.flipped_ids = left, right, frozenset(flipped)

        # Interleaved L, R, L, R: the two sides change speed one frame apart.
        order = [i for pair in zip(left, right) for i in pair]
        order += [i for i in every if i not in order]
        self._motors = {
            i: _Motor(i, "left" if i in left else "right", -1 if i in flipped else 1)
            for i in every}
        self._order = [self._motors[i] for i in order]
        self._sides = {"left": _Side("left", left), "right": _Side("right", right)}
        self._rpm = {"left": 0, "right": 0}
        self._braking = True
        self._mode_faults: set[int] = set()
        self._fault: str | None = None
        self._last_sync = -math.inf
        self._closed = False

        self.bus = bus if bus is not None else DDSM115Bus.open(
            port, reply_timeout_s=reply_timeout_s, exclude=exclude_ports)
        self.port = self.bus.port
        try:
            self._start()
        except BaseException:
            self.bus.close()
            raise

    # -- start-up ----------------------------------------------------------

    def _start(self) -> None:
        dd = self._dd
        # VELOCITY first: a drive frame's value is only an rpm in velocity mode.
        for m in self._order:
            self.bus.send(dd.set_mode(m.id, dd.VELOCITY))
            self._sleep(0.05)       # the teammate's Bus.mode() waits this long after each
        # Then a READ-ONLY check (0x74), before any drive frame: in POSITION mode a
        # drive frame of 0 means "turn to 0 degrees".
        pending = list(self._order)
        for _ in range(3):
            for m in pending:
                r, m.why = self._transact(m, dd.query(m.id))
                if r is not None:
                    m.mode, m.err, m.temp_c = r["mode"], r["err"], r["b6"]
            pending = [m for m in pending if m.mode is None]
            if not pending:
                break
        if pending:
            raise ConnectionError(self._missing_message(pending))
        wrong = [m for m in self._order if m.mode != dd.VELOCITY]
        if wrong:
            raise ConnectionError(
                "wheel motor(s) " + ", ".join(
                    f"{m.id} ({DDSM115_MODES.get(m.mode, m.mode)})" for m in wrong)
                + " did not switch to VELOCITY mode. Not sending them drive frames. Is "
                "something else talking on this bus? Power-cycle the motors (velocity is "
                "their power-up default) and restart.")
        for m in self._order:
            if m.err:
                logger.warning("wheel motor %d reports %s at start (error byte 0x%02x)",
                               m.id, ", ".join(ddsm115_error_names(m.err)), m.err)
        # First drive round: BRAKE, which is also every motor's first position.
        for _ in range(3):
            self._sync()
            if all(m.replied for m in self._order):
                break
        silent = [m for m in self._order if not m.replied]
        if silent:
            raise ConnectionError(self._missing_message(silent))
        self._refuse_if_unsafe(f"wheel bus {self.port} not usable")
        logger.info("DDSM115 wheels on %s: left %s, right %s, flipped %s, %d counts/rev, wheel "
                    "radius %.4f m (IDs, direction, counts/rev and radius: confirm with "
                    "scripts/wheel_check.py)", self.port, list(self.left_ids),
                    list(self.right_ids), sorted(self.flipped_ids), self.counts_per_rev,
                    self.geo.wheel_radius_m)

    def _missing_message(self, silent: list[_Motor]) -> str:
        who = ", ".join(f"{m.id} ({m.why})" for m in silent)
        return (f"no reply from wheel motor(s) {who} on {self.port}. Check the motors' "
                "power, the RS485 A/B wiring, and the IDs with `scripts/wheel_check.py "
                "scan`; set --left-ids/--right-ids to match.")

    # -- the one round of I/O ---------------------------------------------

    def _transact(self, m: _Motor, pkt: bytes) -> tuple[dict[str, Any] | None, str]:
        try:
            return self.bus.transact(m.id, pkt)
        except Exception as exc:  # SerialException, OSError: this motor, this round
            return None, f"{type(exc).__name__}: {exc}"

    def _sync(self) -> list[tuple[int, str]]:
        """One transaction per motor: its current setpoint out, feedback back.
        Returns [(id, why)] for motors whose transaction RAISED (already counted).
        A Ctrl-C mid-round finishes the round first -- a brake must reach every
        motor -- and is re-raised after it."""
        dd = self._dd
        raised: list[tuple[int, str]] = []
        interrupted: BaseException | None = None
        for m in self._order:
            if m.id in self._mode_faults:
                pkt, kind = dd.query(m.id), "query"          # read-only from now on
            elif self._braking:
                pkt, kind = dd.drive(m.id, 0, brake=True), "drive"
            else:
                pkt, kind = dd.drive(m.id, m.sign * self._rpm[m.side], self.accel), "drive"
            try:
                r, why = self.bus.transact(m.id, pkt)
            except Exception as exc:
                r, why = None, f"{type(exc).__name__}: {exc}"
                raised.append((m.id, why))
            except BaseException as exc:  # KeyboardInterrupt: finish the round first
                r, why = None, "interrupted"
                interrupted = interrupted or exc
            self._record(m, r, why, kind)
        self._last_sync = self.clock()
        for side in self._sides.values():
            self._update_side(side)
        if interrupted is not None:
            raise interrupted
        return raised

    def _record(self, m: _Motor, r: dict[str, Any] | None, why: str, kind: str) -> None:
        m.replied = r is not None
        if r is None:
            m.misses += 1
            m.why = why
            if m.misses == self.miss_limit:
                logger.warning("wheel motor %d (%s): %d replies missed in a row (%s)",
                               m.id, m.side, m.misses, why)
            return
        rejoined = m.misses >= self.miss_limit
        if rejoined:
            logger.info("wheel motor %d (%s) answering again", m.id, m.side)
        m.misses, m.why, m.last_ok = 0, "", self.clock()
        m.mode, m.rpm, m.current_a = r["mode"], m.sign * r["rpm"], m.sign * r["current_A"]
        if r["err"] != m.err:
            if r["err"]:
                logger.warning("wheel motor %d reports %s (error byte 0x%02x)",
                               m.id, ", ".join(ddsm115_error_names(r["err"])), r["err"])
            else:
                logger.info("wheel motor %d: error flags cleared", m.id)
            m.err = r["err"]
        if m.mode != self._dd.VELOCITY:
            if m.id not in self._mode_faults:
                self._mode_faults.add(m.id)
                logger.error("wheel motor %d is in %s mode, not VELOCITY: something else is "
                             "on the bus. Sending it read-only queries from now on; driving "
                             "is refused until the bridge restarts.",
                             m.id, DDSM115_MODES.get(m.mode, m.mode))
            return
        if kind == "query":
            m.temp_c = r["b6"]      # byte 7 is only a u8 position: not for odometry
            return
        pos = (r["b6"] << 8) | r["b7"]
        if pos >= self.counts_per_rev:
            if self._fault is None:
                self._fault = (
                    f"wheel motor {m.id} reported position {pos}, but counts_per_rev is "
                    f"{self.counts_per_rev}, so every odometry distance would be wrong. "
                    f"Measure it with `scripts/wheel_check.py rev {m.id} --yes` and pass "
                    "--wheel-counts-per-rev.")
                logger.error("%s", self._fault)
            return
        if m.raw is None or rejoined:
            m.raw = pos             # (re)joining: no delta across a gap we did not see
        else:
            m.count += (m.sign * DDSM115_POSITION_SIGN
                        * unwrap_ticks(pos, m.raw, self.counts_per_rev))
            m.raw = pos

    def _usable(self, m: _Motor) -> bool:
        return (m.raw is not None and m.misses < self.miss_limit
                and m.id not in self._mode_faults)

    def _side_value(self, side: _Side) -> float:
        if not side.live:
            return side.base
        return side.base + sum(self._motors[i].count - side.ref[i]
                               for i in side.live) / len(side.live)

    def _update_side(self, side: _Side) -> None:
        live = frozenset(i for i in side.ids if self._usable(self._motors[i]))
        if live != side.live:
            # Re-anchor so the side's value is continuous: from the last sync
            # where every live motor answered, plus what the survivors did since.
            common = [i for i in live if i in side.snap_counts]
            value = side.snap_value
            if common:
                value += sum(self._motors[i].count - side.snap_counts[i]
                             for i in common) / len(common)
            lost, back = side.live - live, live - side.live
            if not live:
                logger.error("%s wheels: no motor answering (%s): odometry for this side is "
                             "blind, driving refused", side.name,
                             ", ".join(f"{i}: {self._motors[i].why or 'fault'}"
                                       for i in side.ids))
            elif lost:
                logger.warning("%s wheels: odometry from motor(s) %s only; %s not usable (%s)",
                               side.name, sorted(live), sorted(lost),
                               ", ".join(self._motors[i].why or "fault" for i in sorted(lost)))
            elif back and side.ever_live:
                logger.info("%s wheels: motor(s) %s back in odometry", side.name, sorted(back))
            side.base, side.live = value, live
            side.ref = {i: self._motors[i].count for i in live}
            side.ever_live = side.ever_live or bool(live)
        if live and all(self._motors[i].replied for i in live):
            side.snap_value = self._side_value(side)
            side.snap_counts = {i: self._motors[i].count for i in live}

    def _problems(self) -> list[str]:
        out = [self._fault] if self._fault else []
        if self._mode_faults:
            out.append(f"motor(s) {sorted(self._mode_faults)} not in VELOCITY mode (restart "
                       "the bridge)")
        for side in self._sides.values():
            if not side.live:
                out.append(f"{side.name} side: no motor answering ("
                           + ", ".join(f"{i}: {self._motors[i].why or 'fault'}"
                                       for i in side.ids) + ")")
        return out

    # -- BaseDriver --------------------------------------------------------

    @property
    def worst_case_call_s(self) -> float:
        """The longest any public call can block: one round, every motor timing out."""
        return len(self._order) * self.bus.transaction_bound_s

    def _to_rpm(self, v_mps: float) -> int:
        rpm = round(mps_to_rpm(v_mps, self.geo.wheel_radius_m))
        return max(-self.max_rpm, min(self.max_rpm, rpm))

    def set_wheels(self, left_mps: float, right_mps: float) -> None:
        """Rim speeds in m/s, already clamped by the server. Refuses to MOVE
        (raises, nothing sent) with a blind side or a latched fault; zero always
        goes out. Raises after sending if a side went blind during this round,
        so the server stops the base."""
        if not (math.isfinite(left_mps) and math.isfinite(right_mps)):
            raise ValueError(f"non-finite wheel speed ({left_mps}, {right_mps})")
        rpm = {"left": self._to_rpm(left_mps), "right": self._to_rpm(right_mps)}
        moving = any(rpm.values())
        if moving:
            self._refuse_if_unsafe("refusing to drive")
        self._rpm, self._braking = rpm, False
        self._sync()
        if moving:
            self._refuse_if_unsafe("wheels lost while driving")

    def _refuse_if_unsafe(self, what: str) -> None:
        problems = self._problems()
        if problems:
            raise ConnectionError(f"{what}: " + "; ".join(problems))

    def read_ticks(self) -> tuple[int, int]:
        """(left, right) counts, rising as each side rolls forward, wrapping at
        counts_per_rev. Raises rather than report a side it cannot see."""
        if self.clock() - self._last_sync >= self.reuse_s:
            self._sync()
        problems = self._problems()
        if problems:
            raise ConnectionError("wheel odometry unavailable: " + "; ".join(problems))
        return (int(math.floor(self._side_value(self._sides["left"]))) % self.counts_per_rev,
                int(math.floor(self._side_value(self._sides["right"]))) % self.counts_per_rev)

    def battery(self) -> float | None:
        return None     # the DDSM115 protocol reports no voltage

    def stop(self) -> None:
        """BRAKE every motor, trying all of them even if some raise. Raises
        afterwards, naming them, if any motor did not confirm."""
        self._rpm, self._braking = {"left": 0, "right": 0}, True
        self._sync()
        bad = [f"{m.id}: not in VELOCITY mode, no brake frame sent" if m.id in self._mode_faults
               else f"{m.id}: {m.why}"
               for m in self._order if not m.replied or m.id in self._mode_faults]
        if bad:
            raise ConnectionError("brake not confirmed by wheel motor(s) " + "; ".join(bad))

    def close(self) -> None:
        """Brake, then release the port. Idempotent."""
        if self._closed:
            return
        self._closed = True
        try:
            self.stop()
        except Exception as exc:
            logger.warning("closing the wheels: %s", exc)
        finally:
            self.bus.close()

    def diagnostics(self) -> dict[str, Any]:
        """Per-motor and per-side health, for logs, scripts and tests."""
        now = self.clock()
        return {
            "port": self.port,
            "counts_per_rev": self.counts_per_rev,
            "fault": self._fault,
            "problems": self._problems(),
            "sides": {s.name: {"ids": list(s.ids), "live": sorted(s.live),
                               "blind": not s.live,
                               "degraded": bool(s.live) and len(s.live) < len(s.ids)}
                      for s in self._sides.values()},
            "motors": {m.id: {
                "side": m.side, "flipped": m.sign < 0, "ok": m.replied and m.misses == 0,
                "misses": m.misses, "why": m.why, "err": m.err,
                "errors": ddsm115_error_names(m.err),
                "mode": DDSM115_MODES.get(m.mode, m.mode) if m.mode is not None else None,
                "mode_fault": m.id in self._mode_faults, "rpm": m.rpm,
                "current_A": round(m.current_a, 3), "raw_position": m.raw,
                "age_s": now - m.last_ok if m.last_ok > -math.inf else None,
            } for m in self._order},
        }


# ---------------------------------------------------------------- composition


class CompositeDriver:
    """One HardwareDriver from a base, any number of arms and an optional vacuum."""

    def __init__(
        self,
        base: BaseDriver,
        arms: dict[str, ArmDriver] | None = None,
        vacuum: VacuumDriver | None = None,
    ) -> None:
        self._base = base
        self._arms = dict(arms or {})
        self._vacuum = vacuum
        self.counts_per_rev = base.counts_per_rev
        self._owner: dict[str, str] = {}  # joint name -> name of the arm that has it
        for arm_name, arm in self._arms.items():
            for joint in arm.joint_names:
                if joint in self._owner:
                    raise ValueError(
                        f"joint {joint!r} belongs to both arm {self._owner[joint]!r} and arm "
                        f"{arm_name!r}; joint names must be unique across arms"
                    )
                self._owner[joint] = arm_name
        self._gripper_arm = next((arm for arm in self._arms.values() if arm.has_gripper), None)

    def set_wheels(self, left_mps: float, right_mps: float) -> None:
        """Command the base's rim speeds."""
        self._base.set_wheels(left_mps, right_mps)

    def set_joints(self, targets: dict[str, float]) -> None:
        """Send each joint target to the arm that owns it; an unknown name moves nothing."""
        unknown = sorted(set(targets) - set(self._owner))
        if unknown:
            raise ValueError(f"unknown joints {unknown}; the arms have {sorted(self._owner)}")
        per_arm: dict[str, dict[str, float]] = {}
        for joint, angle in targets.items():
            per_arm.setdefault(self._owner[joint], {})[joint] = angle
        for arm_name, arm_targets in per_arm.items():
            self._arms[arm_name].set_joints(arm_targets)

    def set_gripper(self, position: float) -> None:
        """Move the gripper of the first arm that has one."""
        if self._gripper_arm is None:
            raise ValueError("no arm has a gripper")
        self._gripper_arm.set_gripper(position)

    def set_vacuum(self, on: bool) -> None:
        """Switch the vacuum; raises if none is attached."""
        if self._vacuum is None:
            raise ValueError("no vacuum is attached")
        self._vacuum.set_vacuum(on)

    def read_state(self) -> dict[str, Any]:
        """Ticks and battery from the base; joints, gripper and load from the arms."""
        left_ticks, right_ticks = self._base.read_ticks()
        joints: dict[str, float] = {}
        for arm in self._arms.values():
            reading = arm.read_joints()
            joints.update((name, angle) for name, angle in reading.items() if name != "gripper")
            if arm is self._gripper_arm:
                joints["gripper"] = reading["gripper"]
        load = 0.0 if self._gripper_arm is None else self._gripper_arm.gripper_load()
        battery = self._base.battery()
        return {
            "left_ticks": left_ticks,
            "right_ticks": right_ticks,
            "joints": joints,
            "gripper_load": load,
            "battery": 1.0 if battery is None else battery,
        }

    def stop(self) -> None:
        """Base first, then every arm holds; each part is stopped even if one raises."""
        stops = [("the base", self._base.stop)]
        stops += [(f"arm {name!r}", arm.hold) for name, arm in self._arms.items()]
        first_error: Exception | None = None
        for part, stop in stops:
            try:
                stop()
            except Exception as exc:
                logger.exception("stopping %s failed", part)
                if first_error is None:
                    first_error = exc
        if first_error is not None:
            raise first_error

    def close(self) -> None:
        """Release the hardware on shutdown. The base's close() brakes before
        it lets go of its port; a part with no close() is left as it is. The
        vacuum is its owner's to close."""
        for part in [self._base, *self._arms.values()]:
            close = getattr(part, "close", None)
            if close is None:
                continue
            try:
                close()
            except Exception as exc:
                logger.warning("closing %s: %s", type(part).__name__, exc)


def build_real_driver(
    wheel_port: str | None,
    geo: TankGeometry = TankGeometry(),
    vacuum: VacuumDriver | None = None,
    *,
    left_ids: Iterable[int] = (1, 2),
    right_ids: Iterable[int] = (3, 4),
    flipped_ids: Iterable[int] | None = None,
    wheel_counts_per_rev: int = DDSM115_COUNTS_PER_REV,
    wheel_reply_timeout_s: float = 0.010,
) -> CompositeDriver:
    """What scripts/fake_pi.py --driver real constructs.

    wheel_port None: $DDSM115_PORT, else the one WCH USB adapter
    (find_wheel_port refuses to guess between two). left_ids, right_ids and
    flipped_ids (None: ROBOT_FLIPPED_IDS, measured) are unverified until
    `scripts/wheel_check.py sides` says otherwise."""
    base = DDSM115Driver(wheel_port, geo=geo, left_ids=left_ids, right_ids=right_ids,
                         flipped_ids=flipped_ids, counts_per_rev=wheel_counts_per_rev,
                         reply_timeout_s=wheel_reply_timeout_s)
    return CompositeDriver(base, vacuum=vacuum)
