"""The laptop's handle on the robot, through the Pi bridge.

`BridgeRobot` is a RobotBackend, so nothing above backends/ can tell it from TankFakeRobot.
What the link adds is handled here rather than hidden. The Pi owns the robot's constants
and sends them in Hello, so the laptop's odometry cannot drift out of step with the robot
it drives. A reader thread integrates odometry from EVERY state, because skipping states
is how odometry misses an encoder rollover. A heartbeat thread keeps an idle-but-alive
laptop from tripping the Pi's watchdog, and a dead laptop takes its heartbeat with it.
observe() refuses stale state, because steering on a frozen picture is worse than
stopping, and act() refuses sideways velocity before it leaves the laptop.
"""

from __future__ import annotations

import logging
import socket
import threading
import time
from typing import Any

from retriever.bridge.protocol import (
    DEFAULT_PORT,
    MAX_LINE_BYTES,
    SERVER_MESSAGES,
    Act,
    ClearEstop,
    Error,
    Estop,
    Heartbeat,
    Hello,
    Message,
    ProtocolError,
    State,
    decode,
    encode,
)
from retriever.navigation.kinematics import TankGeometry, assert_no_strafe
from retriever.navigation.odometry import TankOdometry
from retriever.types import Action, Observation, Pose

logger = logging.getLogger(__name__)

_LOCALHOST = "127.0.0.1"


class BridgeError(ConnectionError):
    """The link to the Pi is not usable; the message says why, in words."""


def parse_address(addr: str, default_port: int = DEFAULT_PORT) -> tuple[str, int]:
    """Split "host", "host:port", ":port" (this machine), "[::1]:7777" or bare IPv6."""
    text = addr.strip()
    if not text:
        raise ValueError("empty address; expected host, host:port or :port")
    if text.startswith("["):
        host, bracket, rest = text[1:].partition("]")
        if not (bracket and host) or (rest and not rest.startswith(":")):
            raise ValueError(f"cannot parse address {addr!r}; write IPv6 as [::1]:7777")
        return host, (_port(rest[1:], addr) if rest else default_port)
    if text.count(":") > 1:  # a bare IPv6 address, like fe80::1%eth0
        return text, default_port
    host, colon, port = text.partition(":")
    return host or _LOCALHOST, (_port(port, addr) if colon else default_port)


class BridgeRobot:
    """A RobotBackend for the robot at the other end of the Pi bridge."""

    def __init__(
        self,
        host: str = _LOCALHOST,
        port: int = DEFAULT_PORT,
        *,
        connect_timeout_s: float = 3.0,
        observe_timeout_s: float = 0.1,
        stale_after_s: float = 1.0,
        heartbeat_hz: float = 10.0,
        geo: TankGeometry | None = None,
    ) -> None:
        if not heartbeat_hz > 0.0:
            raise ValueError(f"heartbeat_hz must be positive, got {heartbeat_hz!r}")
        self._where = _join(host, port)
        self._observe_timeout_s = observe_timeout_s
        self._stale_after_s = stale_after_s
        self._send_lock = threading.Lock()  # one writer at a time, and seq in wire order
        self._changed = threading.Condition()  # guards what the reader thread updates
        self._closed = threading.Event()
        self._seq = 0
        self._state: State | None = None
        self._state_received_at = 0.0
        self._states_received = 0
        self._link_down: str | None = None  # why the link is unusable, once it is
        self.last_error: str | None = None  # the Pi's latest Error reason
        self._sock = _connect(host, port, connect_timeout_s, self._where)
        self._lines = self._sock.makefile("rb")
        try:
            self._hello = self._read_hello(connect_timeout_s)
            # The Pi owns these numbers; `geo` exists only to override them on purpose.
            self._odometry = TankOdometry(
                geo if geo is not None else _geometry_of(self._hello),
                self._hello.counts_per_rev,
            )
        except BaseException:
            self._lines.close()
            self._sock.close()
            raise
        self._sock.settimeout(None)
        self._reader = threading.Thread(
            target=self._read_forever, name="bridge-reader", daemon=True
        )
        self._heartbeat = threading.Thread(
            target=self._beat_forever,
            args=(1.0 / heartbeat_hz,),
            name="bridge-heartbeat",
            daemon=True,
        )
        self._reader.start()
        self._heartbeat.start()
        logger.info("connected to the Pi bridge at %s", self._where)

    @classmethod
    def from_address(cls, addr: str, **kw: Any) -> BridgeRobot:
        """Connect to an address in any form `parse_address` accepts."""
        host, port = parse_address(addr)
        return cls(host, port, **kw)

    def observe(self) -> Observation:
        """The newest state, after waiting up to observe_timeout_s for a new one.

        Waiting paces the control loop to the stream. Raises BridgeError if the link is down
        or the newest state is older than stale_after_s.
        """
        with self._changed:
            seen = self._states_received
            self._changed.wait_for(
                lambda: self._link_down is not None or self._states_received != seen,
                timeout=self._observe_timeout_s,
            )
            down, state = self._link_down, self._state
            if down is not None:
                raise BridgeError(f"not connected to the Pi at {self._where}: {down}")
            if state is None:
                raise BridgeError(f"no state from the Pi at {self._where} yet")
            age = time.monotonic() - self._state_received_at
            if age > self._stale_after_s:
                raise BridgeError(
                    f"the newest state from the Pi is {age:.2f} s old; the link has stalled"
                )
            pose = self._odometry.pose
        return Observation(
            joints=dict(state.joints),
            base=pose,
            gripper_load=state.gripper_load,
            battery=state.battery,
            t=state.t,
        )

    def act(self, action: Action) -> None:
        """Send one command. Sideways velocity is refused before anything is sent."""
        assert_no_strafe(action.base_vy)
        joints = dict(action.joints)
        gripper = joints.pop("gripper", None)
        self._send_act(
            base_vx=action.base_vx, base_wz=action.base_wz, joints=joints, gripper=gripper
        )

    def close(self) -> None:
        """Hang up; the Pi stops the motors when the socket closes. Safe to call twice."""
        if self._closed.is_set():
            return
        self._closed.set()
        with self._changed:
            if self._link_down is None:
                self._link_down = "closed by this laptop"
            self._changed.notify_all()
        try:
            self._sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass  # already gone
        self._reader.join(timeout=1.0)
        self._heartbeat.join(timeout=1.0)
        self._lines.close()
        self._sock.close()

    def __enter__(self) -> BridgeRobot:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def estop(self, reason: str = "") -> None:
        """Latch the Pi's estop; it holds across reconnects until clear_estop()."""
        self._send(Estop(reason=reason))

    def clear_estop(self) -> None:
        """Release the Pi's estop; nothing moves until the next act."""
        self._send(ClearEstop())

    def set_vacuum(self, on: bool) -> None:
        """Switch the vacuum, sent as a zero-velocity act."""
        self._send_act(vacuum=on)

    def reset_pose(self, pose: Pose = Pose()) -> None:
        """Re-seed the odometry pose, e.g. from an AprilTag fix."""
        with self._changed:
            self._odometry.reset(pose)

    @property
    def state(self) -> State | None:
        """The newest State from the Pi, or None before the first."""
        with self._changed:
            return self._state

    @property
    def estopped(self) -> bool:
        """Whether the newest state says the estop is latched."""
        state = self.state
        return state is not None and state.estop

    @property
    def watchdog_tripped(self) -> bool:
        """Whether the newest state says the Pi's watchdog is tripped."""
        state = self.state
        return state is not None and state.watchdog_tripped

    @property
    def hello(self) -> Hello:
        """The Hello the Pi opened the connection with."""
        return self._hello

    @property
    def connected(self) -> bool:
        """Whether the link is still up."""
        with self._changed:
            return self._link_down is None

    def _read_hello(self, timeout_s: float) -> Hello:
        try:
            line = self._lines.readline(MAX_LINE_BYTES + 2)
        except OSError as exc:
            raise BridgeError(
                f"{self._where} accepted the connection but said nothing within {timeout_s} s"
            ) from exc
        if not line:
            raise BridgeError(f"{self._where} hung up before saying hello")
        try:
            msg = decode(line, allowed=SERVER_MESSAGES)
        except ProtocolError as exc:
            raise BridgeError(f"{self._where} did not say hello: {exc}") from exc
        if isinstance(msg, Error):
            raise BridgeError(f"the Pi at {self._where} refused this connection: {msg.reason}")
        if not isinstance(msg, Hello):
            raise BridgeError(f"{self._where} sent {msg.TYPE!r} before hello")
        return msg

    def _read_forever(self) -> None:
        why = "the Pi closed the connection"
        try:
            while True:
                line = self._lines.readline(MAX_LINE_BYTES + 2)
                if not line:
                    break
                self._on_line(line)
        except Exception as exc:  # a dead reader must read as a dead link, never a quiet Pi
            why = f"the link to the Pi failed: {exc}"
        with self._changed:
            if self._link_down is None:
                self._link_down = why
                logger.warning("bridge link to %s down: %s", self._where, why)
            self._changed.notify_all()

    def _on_line(self, line: bytes) -> None:
        try:
            msg = decode(line, allowed=SERVER_MESSAGES)
        except ProtocolError as exc:
            logger.warning("skipped an unreadable line from the Pi: %s", exc)
            return
        if isinstance(msg, State):
            self._on_state(msg)
        elif isinstance(msg, Error):
            logger.warning("the Pi says: %s", msg.reason)
            with self._changed:
                self.last_error = msg.reason
        else:
            logger.warning("ignored an unexpected %r from the Pi", msg.TYPE)

    def _on_state(self, state: State) -> None:
        with self._changed:
            previous = self._state
            if previous is None:
                self._odometry.update(state.left_ticks, state.right_ticks, 0.0)  # records only
            elif state.t > previous.t:
                self._odometry.update(state.left_ticks, state.right_ticks, state.t - previous.t)
            self._state = state
            self._state_received_at = time.monotonic()
            self._states_received += 1
            self._changed.notify_all()

    def _beat_forever(self, period_s: float) -> None:
        while not self._closed.wait(period_s):
            try:
                self._send(Heartbeat())
            except BridgeError as exc:
                if not self._closed.is_set():
                    logger.warning("heartbeat stopped: %s", exc)
                return

    def _send(self, msg: Message) -> None:
        with self._send_lock:
            self._write(encode(msg))

    def _send_act(self, **fields: Any) -> None:
        with self._send_lock:
            act = Act(seq=self._seq + 1, **fields)
            self._write(encode(act))  # a NaN raises ProtocolError here, before seq moves
            self._seq = act.seq

    def _write(self, line: bytes) -> None:
        with self._changed:
            down = self._link_down
        if down is not None:
            raise BridgeError(f"not connected to the Pi at {self._where}: {down}")
        try:
            self._sock.sendall(line)
        except OSError as exc:
            raise BridgeError(f"cannot send to the Pi at {self._where}: {exc}") from exc


def _connect(host: str, port: int, timeout_s: float, where: str) -> socket.socket:
    try:
        sock = socket.create_connection((host, port), timeout=timeout_s)
    except ConnectionRefusedError as exc:
        raise BridgeError(
            f"connection refused by {where}: the machine is up but nothing is listening on "
            f"port {port}; is the bridge (scripts/fake_pi.py) running there?"
        ) from exc
    except TimeoutError as exc:
        raise BridgeError(
            f"no answer from {where} within {timeout_s} s; is it on this network?"
        ) from exc
    except OSError as exc:
        raise BridgeError(f"cannot reach the Pi bridge at {where}: {exc}") from exc
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # a 60-byte act must not wait
    return sock


def _geometry_of(hello: Hello) -> TankGeometry:
    return TankGeometry(hello.wheel_radius_m, hello.track_width_m, hello.scrub_factor)


def _port(text: str, addr: str) -> int:
    if text.isascii() and text.isdigit() and 0 < int(text) < 65536:
        return int(text)
    raise ValueError(f"bad port in address {addr!r}; expected 1 to 65535")


def _join(host: str, port: int) -> str:
    return f"[{host}]:{port}" if ":" in host else f"{host}:{port}"
