"""Wire protocol between the laptop and the Pi bridge: one JSON object per line, over TCP.

Why JSON lines: at 3 am, typing {"v":1,"type":"estop"} into `nc pi.local 7777` beats every
byte a binary format would save, and ~300 bytes of state at 50 Hz is nothing for Wi-Fi.

Why strict: these lines move a machine that can hurt someone, so decode() refuses whatever it
does not fully understand instead of guessing. An unknown field is an error, because ignoring
a field is exactly how a sideways `base_vy` gets silently dropped by a tank base that cannot
strafe. A non-finite number is an error, because NaN sails through every `if speed > limit`
clamp. encode() applies the same rules, so a bad command fails on the laptop, where the bug
is. decode() raises nothing but ProtocolError, so a garbled line costs an Error reply, never
the connection. Stdlib only, importing nothing else from `retriever`: the Pi runs this module
with nothing installed.

EXTENSIONS WITHOUT A VERSION BUMP (the lidar safety bubble). Strict decoding
means an old laptop drops any `state` carrying a field it does not know, so
new fields must never reach a client that did not ask for them:

  - A client that understands them sends `subscribe` once after `hello`. Only
    then does the server add the optional state fields (bubble, bubble_reason,
    nearest_m, stop_m) and stream `scan` messages. An old client never
    subscribes and gets exactly the bytes it always got.
  - The optional state fields are OMITTED from the line when None (they are
    never written as null), so a state without them is byte-identical to v1.
  - An old SERVER answers `subscribe` with an `error` ("unknown message type")
    and carries on; BridgeRobot recognises that and simply has no lidar data.
"""

from __future__ import annotations

import json
import math
from collections.abc import Callable
from dataclasses import MISSING, dataclass, field, fields
from typing import Any, ClassVar, TypeAlias

PROTOCOL_VERSION = 1
DEFAULT_PORT = 7777
MAX_LINE_BYTES = 64 * 1024  # per line, terminator excluded (as asyncio's readline limit counts)
MAX_JOINTS = 64
MAX_NAME_LEN = 64
MAX_TEXT_LEN = 500
MAX_SCAN_BINS = 720

# SafetyBubble states (bridge/safety.py), in the order it defines them.
BUBBLE_STATES = ("clear", "slowing", "blocked", "stale")

_MAX_SAFE_INT = 2**53  # beyond this, a peer that reads JSON numbers as doubles loses precision
_ENVELOPE = ("v", "type")
_STRAFE_FIELDS = ("base_vy", "vy")


class ProtocolError(ValueError):
    """A line or message that breaks the protocol: the only exception encode/decode raise.

    The message is clipped to MAX_TEXT_LEN, so it can always go back to the peer as an Error.
    """

    def __init__(self, message: str) -> None:
        super().__init__(_clip(message, MAX_TEXT_LEN))


class ProtocolVersionError(ProtocolError):
    """The line parsed but its "v" is missing or not ours: update the peer, do not retry."""


@dataclass(frozen=True)
class Act:
    """Laptop -> Pi: base velocity (m/s forward, rad/s CCW) and arm targets (radians).

    Joints left out hold position; a None gripper or vacuum is left as it is.
    """

    TYPE: ClassVar[str] = "act"
    seq: int
    base_vx: float = 0.0
    base_wz: float = 0.0
    joints: dict[str, float] = field(default_factory=dict)
    gripper: float | None = None
    vacuum: bool | None = None


@dataclass(frozen=True)
class Heartbeat:
    """Laptop -> Pi: still here. Feeds the watchdog; never keeps an old velocity alive."""

    TYPE: ClassVar[str] = "heartbeat"


@dataclass(frozen=True)
class Estop:
    """Laptop -> Pi: stop now and stay stopped (latching) until a ClearEstop."""

    TYPE: ClassVar[str] = "estop"
    reason: str = ""


@dataclass(frozen=True)
class ClearEstop:
    """Laptop -> Pi: release a latched estop. Nothing moves until the next act."""

    TYPE: ClassVar[str] = "clear_estop"


@dataclass(frozen=True)
class Subscribe:
    """"I understand the lidar extensions." Optional, sent once after hello.
    safety: add the bubble fields to every state. scan: stream `scan` too.
    Also counts as proof of life, like a heartbeat."""

    TYPE: ClassVar[str] = "subscribe"
    safety: bool = True
    scan: bool = True


@dataclass(frozen=True)
class Hello:
    """Pi -> laptop, first line on every connection: the constants and timeouts the Pi owns."""

    TYPE: ClassVar[str] = "hello"
    counts_per_rev: int
    wheel_radius_m: float
    track_width_m: float
    scrub_factor: float
    state_hz: float
    timeout_ms: int
    motion_timeout_ms: int


@dataclass(frozen=True)
class State:
    """Pi -> laptop at state_hz: raw wheel ticks, joints (with "gripper") and safety flags.

    `seq` is the last act the Pi applied; the laptop integrates odometry from the ticks.
    """

    TYPE: ClassVar[str] = "state"
    seq: int
    t: float
    left_ticks: int
    right_ticks: int
    joints: dict[str, float] = field(default_factory=dict)
    gripper_load: float = 0.0
    battery: float = 1.0
    estop: bool = False
    watchdog_tripped: bool = False
    # Lidar safety bubble. Only sent to a client that subscribed, only when the
    # Pi has a lidar, and left off the line entirely when None.
    bubble: str | None = None            # clear | slowing | blocked | stale
    bubble_reason: str | None = None     # why, in words; "" when clear
    nearest_m: float | None = None       # body -> nearest obstacle; None without a fresh scan
    stop_m: float | None = None          # body clearance the current command needs


@dataclass(frozen=True)
class Scan:
    """A downsampled lidar revolution, BASE frame, for the dashboard. Sent to
    subscribed clients at a few Hz. Bin i covers bearings [i, i+1) * step_deg,
    CCW from the base's forward axis, measured from the base origin, and holds
    the nearest return in cm (0 = nothing). `near` lists bins inside the
    current stop distance. `t` is on the same clock as State.t."""

    TYPE: ClassVar[str] = "scan"
    t: float
    step_deg: float
    ranges_cm: list[int] = field(default_factory=list)
    near: list[int] = field(default_factory=list)


@dataclass(frozen=True)
class Error:
    """Pi -> laptop: a line was malformed or a command was refused, and why."""

    TYPE: ClassVar[str] = "error"
    reason: str


Message: TypeAlias = (
    Act | Heartbeat | Estop | ClearEstop | Subscribe | Hello | State | Scan | Error
)

CLIENT_MESSAGES: tuple[type[Message], ...] = (Act, Heartbeat, Estop, ClearEstop, Subscribe)
SERVER_MESSAGES: tuple[type[Message], ...] = (Hello, State, Error, Scan)


def encode(msg: Message) -> bytes:
    """Serialise one message as a compact JSON line ending in "\\n", checking every field.

    A message the peer would reject never leaves: a NaN velocity or a "gripper" inside
    Act.joints raises ProtocolError here, on the laptop, where the bug is.
    """
    rules = _RULES.get(type(msg))
    if rules is None:
        raise ProtocolError(f"cannot encode {_describe(msg)}: not a protocol message")
    payload: dict[str, Any] = {"v": PROTOCOL_VERSION, "type": msg.TYPE}
    omit_if_none = _OMIT_IF_NONE.get(type(msg), ())
    for name, check in rules.items():
        value = getattr(msg, name)
        if value is None and name in omit_if_none:
            continue
        payload[name] = check(value, f"{msg.TYPE}.{name}")
    _check_across_fields(type(msg), payload)
    # ensure_ascii (the default) keeps every line plain ASCII, whatever the text fields hold.
    return json.dumps(payload, separators=(",", ":")).encode("ascii") + b"\n"


def decode(line: bytes | str, allowed: tuple[type[Message], ...] | None = None) -> Message:
    """Parse one line into a message, or raise ProtocolError saying exactly what is wrong.

    `allowed` limits the message classes accepted (CLIENT_MESSAGES on the Pi, SERVER_MESSAGES
    on the laptop). A trailing "\\n" or "\\r\\n" is optional.
    """
    try:
        obj = _parse_line(line)
        cls = _message_class(obj, allowed)
        return cls(**_checked_fields(cls, obj))
    except ProtocolError:
        raise
    except Exception as exc:  # a bug here must cost one line, not the reader that calls us
        raise ProtocolError(f"undecodable line ({type(exc).__name__}: {exc})") from exc


# --- field rules: each returns the value normalised for the wire, or raises ProtocolError --

_Check: TypeAlias = Callable[[Any, str], Any]


def _number(value: Any, where: str) -> float:
    # isfinite() is also what rejects JSON's non-standard NaN/Infinity literals (and 1e999),
    # all of which Python's json.loads happily turns into floats.
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        try:
            number = float(value)
        except OverflowError:  # an integer literal too large for a double
            number = math.inf
        if math.isfinite(number):
            return number
    raise _wrong(where, "a finite number", value)


def _positive(value: Any, where: str) -> float:
    number = _number(value, where)
    if number > 0:
        return number
    raise _wrong(where, "a finite number > 0", value)


def _is_safe_int(value: Any) -> bool:
    if isinstance(value, bool) or not isinstance(value, int):
        return False
    return abs(value) <= _MAX_SAFE_INT


def _integer(value: Any, where: str) -> int:
    if _is_safe_int(value):
        return int(value)
    raise _wrong(where, "an integer within +/-2**53", value)


def _count(value: Any, where: str) -> int:
    if _is_safe_int(value) and value >= 0:
        return int(value)
    raise _wrong(where, "an integer from 0 to 2**53", value)


def _positive_int(value: Any, where: str) -> int:
    if _is_safe_int(value) and value > 0:
        return int(value)
    raise _wrong(where, "an integer from 1 to 2**53", value)


def _boolean(value: Any, where: str) -> bool:
    if isinstance(value, bool):
        return value
    raise _wrong(where, "true or false", value)


def _text(value: Any, where: str) -> str:
    if isinstance(value, str) and len(value) <= MAX_TEXT_LEN:
        return str(value)
    raise _wrong(where, f"a string of at most {MAX_TEXT_LEN} characters", value)


def _joints(value: Any, where: str) -> dict[str, float]:
    if not isinstance(value, dict):
        raise _wrong(where, "an object of joint name -> radians", value)
    if len(value) > MAX_JOINTS:
        raise ProtocolError(f"{where} has {len(value)} joints; the limit is {MAX_JOINTS}")
    angles: dict[str, float] = {}
    for name, angle in value.items():
        if not isinstance(name, str) or not 1 <= len(name) <= MAX_NAME_LEN:
            raise ProtocolError(
                f"{where} has an invalid joint name ({_describe(name)}); "
                f"names are strings of 1 to {MAX_NAME_LEN} characters"
            )
        angles[name] = _number(angle, f"{where}[{name!r}]")
    return angles


def _arm_joints(value: Any, where: str) -> dict[str, float]:
    angles = _joints(value, where)
    if "gripper" in angles:
        raise ProtocolError(f"{where} must not contain 'gripper': it has its own act.gripper")
    return angles


def _nonneg(value: Any, where: str) -> float:
    number = _number(value, where)
    if number >= 0:
        return number
    raise _wrong(where, "a finite number >= 0", value)


def _bubble_state(value: Any, where: str) -> str:
    if isinstance(value, str) and value in BUBBLE_STATES:
        return str(value)
    raise _wrong(where, f"one of {', '.join(BUBBLE_STATES)}", value)


def _int_list(limit: int, lo: int, hi: int) -> _Check:
    """A JSON array of at most `limit` integers, each in lo..hi."""

    def check(value: Any, where: str) -> list[int]:
        if not isinstance(value, list):
            raise _wrong(where, "a list of integers", value)
        if len(value) > limit:
            raise ProtocolError(f"{where} has {len(value)} entries; the limit is {limit}")
        for x in value:
            if isinstance(x, bool) or not isinstance(x, int) or not lo <= x <= hi:
                raise _wrong(where, f"a list of integers in {lo}..{hi}", x)
        return list(value)

    return check


def _optional(check: _Check) -> _Check:
    """`check`, plus null (None), which means "leave this as it is"."""

    def check_or_none(value: Any, where: str) -> Any:
        return None if value is None else check(value, where)

    return check_or_none


# The schema, shared by encode and decode: every field of every message and its rule, in wire
# order. Which fields are required, and the defaults for the rest, come from the dataclasses.
_RULES: dict[type[Message], dict[str, _Check]] = {
    Act: {
        "seq": _count,
        "base_vx": _number,
        "base_wz": _number,
        "joints": _arm_joints,
        "gripper": _optional(_number),
        "vacuum": _optional(_boolean),
    },
    Heartbeat: {},
    Estop: {"reason": _text},
    ClearEstop: {},
    Subscribe: {"safety": _boolean, "scan": _boolean},
    Hello: {
        "counts_per_rev": _positive_int,
        "wheel_radius_m": _positive,
        "track_width_m": _positive,
        "scrub_factor": _positive,
        "state_hz": _positive,
        "timeout_ms": _positive_int,
        "motion_timeout_ms": _positive_int,
    },
    State: {
        "seq": _count,
        "t": _number,
        "left_ticks": _integer,
        "right_ticks": _integer,
        "joints": _joints,
        "gripper_load": _number,
        "battery": _number,
        "estop": _boolean,
        "watchdog_tripped": _boolean,
        "bubble": _optional(_bubble_state),
        "bubble_reason": _optional(_text),
        "nearest_m": _optional(_nonneg),
        "stop_m": _optional(_nonneg),
    },
    Scan: {
        "t": _number,
        "step_deg": _positive,
        "ranges_cm": _int_list(MAX_SCAN_BINS, 0, 65535),
        "near": _int_list(MAX_SCAN_BINS, 0, MAX_SCAN_BINS - 1),
    },
    Error: {"reason": _text},
}
# Fields left off the line when None, so a message without them is
# byte-identical to what a v1 peer sends and expects.
_OMIT_IF_NONE: dict[type[Message], tuple[str, ...]] = {
    State: ("bubble", "bubble_reason", "nearest_m", "stop_m"),
}
_BY_TYPE: dict[str, type[Message]] = {cls.TYPE: cls for cls in _RULES}
_REQUIRED: dict[type[Message], tuple[str, ...]] = {
    cls: tuple(f.name for f in fields(cls) if f.default is f.default_factory is MISSING)
    for cls in _RULES
}


# --- decoding, one layer at a time: line, envelope, fields -------------------------------


def _parse_line(line: bytes | str) -> dict[str, Any]:
    if isinstance(line, str):
        try:
            line = line.encode("utf-8")
        except UnicodeEncodeError:  # a lone surrogate
            raise ProtocolError("line is not valid UTF-8 text") from None
    elif not isinstance(line, (bytes, bytearray)):
        raise ProtocolError(f"a line must be bytes or str, not {type(line).__name__}")
    body = line.rstrip(b"\r\n")
    if len(body) > MAX_LINE_BYTES:
        raise ProtocolError(f"line is {len(body)} bytes; the limit is {MAX_LINE_BYTES}")
    if not body.strip():
        raise ProtocolError(
            'empty line; send one JSON object per line, like {"v":1,"type":"estop"}'
        )
    try:
        text = body.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ProtocolError(f"line is not valid UTF-8 (bad byte at {exc.start})") from None
    try:
        obj = json.loads(text, object_pairs_hook=_without_duplicate_keys)
    except ProtocolError:
        raise
    except json.JSONDecodeError as exc:
        raise ProtocolError(f"invalid JSON: {exc.msg} at column {exc.colno}") from None
    except (ValueError, RecursionError) as exc:  # a 5000-digit integer; [[[[... nested too deep
        raise ProtocolError(f"invalid JSON: {exc}") from None
    if not isinstance(obj, dict):
        raise ProtocolError(f"a message must be a JSON object, got {_describe(obj)}")
    return obj


def _without_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    # json.loads keeps the last duplicate; another parser may keep the first. Refuse to guess.
    obj: dict[str, Any] = {}
    for key, value in pairs:
        if key in obj:
            raise ProtocolError(f"duplicate key {_describe(key)}: which value is meant?")
        obj[key] = value
    return obj


def _message_class(
    obj: dict[str, Any], allowed: tuple[type[Message], ...] | None
) -> type[Message]:
    # The version comes first: a newer peer may well send types and fields we have never seen.
    if "v" not in obj:
        raise ProtocolVersionError(
            f'missing protocol version "v"; this end speaks v{PROTOCOL_VERSION}'
        )
    version = obj["v"]
    if type(version) is not int or version != PROTOCOL_VERSION:  # true == 1, but not a version
        raise ProtocolVersionError(
            f"protocol version {_describe(version)} is not supported; "
            f"this end speaks v{PROTOCOL_VERSION}"
        )
    if "type" not in obj:
        raise ProtocolError('missing message "type"')
    kind = obj["type"]
    if not isinstance(kind, str):
        raise ProtocolError(f'message "type" must be a string, got {_describe(kind)}')
    cls = _BY_TYPE.get(kind)
    if cls is None:
        raise ProtocolError(f"unknown message type {_describe(kind)}")
    if allowed is not None and cls not in allowed:
        raise ProtocolError(f"{kind!r} is not valid in this direction")
    return cls


def _checked_fields(cls: type[Message], obj: dict[str, Any]) -> dict[str, Any]:
    rules = _RULES[cls]
    given = {name: value for name, value in obj.items() if name not in _ENVELOPE}
    unknown = [name for name in given if name not in rules]
    if cls is Act:
        for name in unknown:
            if name in _STRAFE_FIELDS:
                raise ProtocolError(
                    f"act carries {name!r}, but a tank base cannot strafe: a skid-steer has no "
                    "sideways velocity, so drive with base_vx and base_wz only"
                )
    if unknown:
        raise ProtocolError(f"{cls.TYPE} has unknown field(s) {_names(unknown)}")
    missing = [name for name in _REQUIRED[cls] if name not in given]
    if missing:
        raise ProtocolError(f"{cls.TYPE} is missing required field(s) {_names(missing)}")
    checked = {name: rules[name](value, f"{cls.TYPE}.{name}") for name, value in given.items()}
    _check_across_fields(cls, checked)
    return checked


def _check_across_fields(cls: type[Message], values: dict[str, Any]) -> None:
    """Rules that tie one field to another, for encode and decode alike."""
    if cls is Scan:
        bins = len(values.get("ranges_cm", []))
        if any(i >= bins for i in values.get("near", [])):
            raise ProtocolError("scan.near refers to a bin past the end of ranges_cm")


# --- error text: short, specific, and never an error itself ------------------------------


def _wrong(where: str, wanted: str, value: Any) -> ProtocolError:
    return ProtocolError(f"{where} must be {wanted}, got {_describe(value)}")


def _describe(value: Any) -> str:
    """Render an offending value for an error message, in a few dozen characters at most."""
    if value is None or isinstance(value, bool):
        return json.dumps(value)  # null / true / false, as the peer wrote it
    if isinstance(value, str):
        if len(value) <= 40:
            return repr(value)
        return f"{value[:24]!r}... ({len(value)} characters)"
    if isinstance(value, int):  # str() of a huge int is slow, or refused outright
        return str(value) if value.bit_length() <= 64 else f"a {value.bit_length()}-bit integer"
    if isinstance(value, float):
        return repr(value)
    if isinstance(value, dict):
        return "an object"
    if isinstance(value, list):
        return "an array"
    return f"a value of type {type(value).__name__}"


def _names(names: list[str], shown: int = 5) -> str:
    listed = ", ".join(_describe(name) for name in names[:shown])
    hidden = len(names) - shown
    return f"{listed} and {hidden} more" if hidden > 0 else listed


def _clip(text: str, limit: int) -> str:
    return text if len(text) <= limit else text[: limit - 3] + "..."
