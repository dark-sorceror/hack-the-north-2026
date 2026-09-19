"""The Pi end of the bridge: apply the laptop's commands, stream state, stop when in doubt.

The Pi sits next to the motors, so every decision to STOP lives here. Three stops cover
three different failures. The watchdog catches a laptop gone silent: it crashed, the Wi-Fi
dropped, or the socket is half-open and will never error. The motion deadman catches a
laptop that is alive but whose control loop is not (an exception, a slow LLM call):
heartbeats keep the link up but never keep an old velocity alive. The estop is explicit
and latching, and survives reconnects, so a laptop that restarts cannot drive on where it
left off. The core starts tripped: nothing moves until a client proves it is alive.

All of it is in `BridgeCore`, which has no sockets and no clock (every method takes `now`),
so the safety logic is tested exactly and instantly. `BridgeServer` is the thin asyncio
shell around it. Its tick loop catches everything, because if it died the watchdog would
die with it, and if it ends anyway the server stops the motors and raises.
"""

from __future__ import annotations

import asyncio
import logging
import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Protocol, runtime_checkable

from retriever.bridge.protocol import (
    CLIENT_MESSAGES,
    DEFAULT_PORT,
    MAX_LINE_BYTES,
    MAX_TEXT_LEN,
    Act,
    ClearEstop,
    Error,
    Estop,
    Hello,
    Message,
    ProtocolError,
    State,
    decode,
    encode,
)
from retriever.navigation.kinematics import TankGeometry, tank_body_to_wheels

logger = logging.getLogger(__name__)

MAX_WRITE_BUFFER = 256 * 1024  # a client this far behind is dropped (and the robot stops)
ZERO_REFRESH_S = 0.5  # while stopped, re-send zero wheel speed this often


@runtime_checkable
class HardwareDriver(Protocol):
    """The motors and sensors as the bridge drives them, real or fake."""

    counts_per_rev: int

    def set_wheels(self, left_mps: float, right_mps: float) -> None:
        """Command left and right rim speeds in m/s."""
        ...

    def set_joints(self, targets: dict[str, float]) -> None:
        """Move the named arm joints toward targets in radians; unknown names raise."""
        ...

    def set_gripper(self, position: float) -> None:
        """Move the gripper toward `position`."""
        ...

    def set_vacuum(self, on: bool) -> None:
        """Switch the vacuum on or off."""
        ...

    def read_state(self) -> dict[str, Any]:
        """left_ticks, right_ticks, joints (with "gripper"), gripper_load and battery."""
        ...

    def stop(self) -> None:
        """Wheels to zero now; arms hold where they are, torque on; vacuum unchanged."""
        ...


class BridgeCore:
    """All of the bridge's safety logic, with no sockets and no clock.

    Every method takes `now` in monotonic seconds. The server calls it from one thread.
    """

    def __init__(
        self,
        driver: HardwareDriver,
        now: float,
        geo: TankGeometry = TankGeometry(),
        timeout_s: float = 0.3,
        motion_timeout_s: float = 0.5,
        max_wheel_mps: float = 0.8,
    ) -> None:
        for name, value in (
            ("timeout_s", timeout_s),
            ("motion_timeout_s", motion_timeout_s),
            ("max_wheel_mps", max_wheel_mps),
        ):
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"{name} must be positive and finite, got {value!r}")
        if not isinstance(driver, HardwareDriver):
            raise TypeError(f"{type(driver).__name__} does not implement HardwareDriver")
        self.driver = driver
        self.geo = geo
        self.timeout_s = timeout_s
        self.motion_timeout_s = motion_timeout_s
        self.max_wheel_mps = max_wheel_mps
        self.estop = False
        self.estop_reason = ""
        self.watchdog_tripped = True  # nothing is in charge until a client proves it is alive
        self.last_seq = 0
        self._linked = False
        self._last_heard_at = now
        self._last_act_at = now
        self._act_in_force = False  # an act is being obeyed, so the motion deadman is armed
        self._wheels = (0.0, 0.0)
        self._zero_sent_at = now
        self._halt(now)

    def connected(self, now: float) -> None:
        """A client connected. It counts as live for `timeout_s`, but nothing moves yet."""
        self._linked = True
        self._last_heard_at = now

    def disconnected(self, now: float) -> None:
        """No client any more: stop now and trip the watchdog."""
        if self._linked:
            logger.warning("client gone: base stopped, watchdog tripped")
        self._linked = False
        self.watchdog_tripped = True
        self._halt(now)

    def link_alive(self, now: float) -> bool:
        """Whether a client is connected and was heard from within `timeout_s`."""
        return self._linked and now - self._last_heard_at < self.timeout_s

    def handle(self, msg: Message, now: float) -> str | None:
        """Apply one valid client message; return why it was refused, or None."""
        if not isinstance(msg, CLIENT_MESSAGES):
            return f"{msg.TYPE!r} is not a command the bridge accepts"
        self._last_heard_at = now
        if self.watchdog_tripped:
            self.watchdog_tripped = False
            logger.info("laptop heard from: watchdog cleared")
        if isinstance(msg, Act):
            return self._apply(msg, now)
        if isinstance(msg, Estop):
            self.trigger_estop(msg.reason or "estop sent by the laptop", now)
        elif isinstance(msg, ClearEstop):
            self._clear_estop()
        return None  # a Heartbeat only feeds the watchdog

    def reject(self, reason: str, now: float) -> None:
        """An unparseable line arrived: the last velocity is no longer trustworthy, so stop."""
        logger.warning("malformed line from the laptop, base stopped: %s", reason)
        self._halt(now)

    def trigger_estop(self, reason: str, now: float) -> None:
        """Stop, and refuse every act until ClearEstop. Also the hook for a physical button."""
        if not self.estop:
            self.estop = True
            self.estop_reason = reason or "no reason given"
            logger.warning("ESTOP latched: %s", self.estop_reason)
        self._halt(now)

    def wheel_speeds(self, vx: float, wz: float) -> tuple[float, float]:
        """Rim speeds for (vx, wz); if either exceeds the limit, both scale by one factor.

        Clipping only the faster side would tighten an arc, or turn it into a spin.
        """
        if not (math.isfinite(vx) and math.isfinite(wz)):
            raise ValueError(f"base velocity must be finite, got vx={vx!r} wz={wz!r}")
        left, right = tank_body_to_wheels(vx, wz, self.geo)
        fastest = max(abs(left), abs(right))
        if fastest <= self.max_wheel_mps:
            return left, right
        scale = self.max_wheel_mps / fastest
        return left * scale, right * scale

    def tick(self, now: float) -> None:
        """Run the stops that time passing triggers. Call at the state rate."""
        if not self.watchdog_tripped and now - self._last_heard_at >= self.timeout_s:
            self.watchdog_tripped = True
            logger.warning(
                "nothing from the laptop for %.0f ms: base stopped (watchdog)",
                self.timeout_s * 1000,
            )
            self._halt(now)
        if self._act_in_force and now - self._last_act_at >= self.motion_timeout_s:
            logger.warning(
                "no act for %.0f ms: base stopped, arm holding (motion deadman)",
                self.motion_timeout_s * 1000,
            )
            self._halt(now)
        if self._wheels == (0.0, 0.0) and now - self._zero_sent_at >= ZERO_REFRESH_S:
            self._resend_zero_wheels(now)

    def snapshot(self, now: float) -> State:
        """The State to stream now; raises whatever the driver raises if it cannot be read."""
        reading = self.driver.read_state()
        return State(
            seq=self.last_seq,
            t=now,
            left_ticks=reading["left_ticks"],
            right_ticks=reading["right_ticks"],
            joints=dict(reading["joints"]),
            gripper_load=reading["gripper_load"],
            battery=reading["battery"],
            estop=self.estop,
            watchdog_tripped=self.watchdog_tripped,
        )

    def hello(self, state_hz: float) -> Hello:
        """The first line of every connection: the constants and timeouts the Pi owns."""
        return Hello(
            counts_per_rev=self.driver.counts_per_rev,
            wheel_radius_m=self.geo.wheel_radius_m,
            track_width_m=self.geo.track_width_m,
            scrub_factor=self.geo.scrub_factor,
            state_hz=state_hz,
            timeout_ms=_whole_ms(self.timeout_s),
            motion_timeout_ms=_whole_ms(self.motion_timeout_s),
        )

    def _apply(self, act: Act, now: float) -> str | None:
        """The one place an act's commands reach the hardware."""
        if self.estop:
            return f"estop is latched ({self.estop_reason}); send clear_estop before driving"
        left, right = self.wheel_speeds(act.base_vx, act.base_wz)
        try:
            self.driver.set_wheels(left, right)
            self._wheels = (left, right)
            if left == right == 0.0:
                self._zero_sent_at = now
            if act.joints:
                self.driver.set_joints(dict(act.joints))
            if act.gripper is not None:
                self.driver.set_gripper(act.gripper)
            if act.vacuum is not None:
                self.driver.set_vacuum(act.vacuum)
        except Exception as exc:
            logger.error("driver refused act %d, robot stopped: %s", act.seq, exc)
            self._halt(now)
            return f"driver refused the command: {exc}"
        self.last_seq = act.seq
        self._last_act_at = now
        self._act_in_force = True
        return None

    def _halt(self, now: float) -> None:
        """Wheels to zero and arm holding, whatever the cause. A failed stop is retried."""
        self._act_in_force = False
        self._wheels = (0.0, 0.0)
        self._zero_sent_at = now
        try:
            self.driver.stop()
        except Exception:
            logger.exception(
                "driver.stop() failed; zero wheel speed is re-sent every %.1f s", ZERO_REFRESH_S
            )

    def _resend_zero_wheels(self, now: float) -> None:
        # A motor controller that missed one stop (a dropped serial frame) still stops.
        self._zero_sent_at = now
        try:
            self.driver.set_wheels(0.0, 0.0)
        except Exception as exc:
            logger.error("re-sending zero wheel speed failed: %s", exc)

    def _clear_estop(self) -> None:
        if self.estop:
            logger.warning("estop cleared (was: %s); nothing moves until the next act",
                           self.estop_reason)
        self.estop = False
        self.estop_reason = ""


@dataclass(eq=False)
class _Client:
    peer: str
    reader: asyncio.StreamReader
    writer: asyncio.StreamWriter


class BridgeServer:
    """BridgeCore behind an asyncio TCP server, with one client in charge at a time."""

    def __init__(
        self,
        driver: HardwareDriver,
        host: str = "0.0.0.0",
        port: int = DEFAULT_PORT,
        *,
        geo: TankGeometry = TankGeometry(),
        timeout_ms: float = 300,
        motion_timeout_ms: float = 500,
        state_hz: float = 50.0,
        max_wheel_mps: float = 0.8,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not (math.isfinite(state_hz) and state_hz > 0.0):
            raise ValueError(f"state_hz must be positive and finite, got {state_hz!r}")
        self._host = host
        self._port = port
        self._state_hz = state_hz
        self._clock = clock
        self.core = BridgeCore(
            driver,
            clock(),
            geo,
            timeout_s=timeout_ms / 1000.0,
            motion_timeout_s=motion_timeout_ms / 1000.0,
            max_wheel_mps=max_wheel_mps,
        )
        self._server: asyncio.Server | None = None
        self._ticks: asyncio.Task[None] | None = None
        self._client: _Client | None = None
        self._closing = False
        self._tick_failing = False

    @property
    def driver(self) -> HardwareDriver:
        """The hardware the core drives."""
        return self.core.driver

    async def start(self) -> int:
        """Bind and start streaming; return the bound port (port 0 picks a free one)."""
        # host "" (or None) is asyncio's "every address, IPv4 and IPv6".
        self._server = await asyncio.start_server(
            self._on_connect, self._host or None, self._port, limit=MAX_LINE_BYTES
        )
        self._ticks = asyncio.get_running_loop().create_task(self._tick_forever())
        port: int = self._server.sockets[0].getsockname()[1]
        logger.debug("bridge listening on %r port %d", self._host, port)
        return port

    async def serve_forever(self) -> None:
        """Serve until close(). If the tick loop ever ends by itself, stop and raise."""
        if self._ticks is None:
            await self.start()
        ticks = self._ticks
        assert ticks is not None
        try:
            await asyncio.wait({ticks})
            died = not self._closing
        finally:
            await self.close()
        if died:
            raise RuntimeError("the bridge tick loop ended by itself; the motors were stopped")

    async def close(self) -> None:
        """Stop accepting, drop the client and stop the motors; safe to call twice."""
        if self._closing:
            return
        self._closing = True
        # Everything up to the stop is synchronous, so no tick can run in between.
        if self._ticks is not None:
            self._ticks.cancel()
        if self._server is not None:
            self._server.close()
        if self._client is not None:
            self._drop(self._client, "the bridge is shutting down")
        self.core.disconnected(self._clock())
        if self._ticks is not None:
            await asyncio.wait({self._ticks})
        logger.info("bridge closed; motors stopped")

    def trigger_estop(self, reason: str = "local estop") -> None:
        """Latch the estop from the Pi itself (a button, a signal). Call on the loop thread."""
        self.core.trigger_estop(reason, self._clock())

    async def _on_connect(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        now = self._clock()
        peer = _peer_name(writer)
        if self._closing:
            writer.close()
            return
        current = self._client
        if current is not None:
            if self.core.link_alive(now):
                logger.warning("refused %s: %s is in charge", peer, current.peer)
                refusal = (
                    f"another client ({current.peer}) is driving this robot; try again once "
                    f"it disconnects or is silent for {self.core.timeout_s * 1000:.0f} ms"
                )
                writer.write(encode(_error(refusal)))
                writer.close()
                return
            # Half-open after a Wi-Fi drop, or a crashed laptop: a restarted one must get in.
            self._drop(current, f"its link is stale and {peer} is connecting")
        client = _Client(peer, reader, writer)
        self._client = client
        self.core.connected(now)
        self._send(client, self.core.hello(self._state_hz))  # before any State: no await
        logger.info("%s connected; nothing moves until it sends a valid message", peer)
        try:
            await self._read_lines(client)
        except Exception:
            logger.exception("reading from %s failed; hanging up", peer)
        finally:
            self._forget(client)
            writer.close()

    async def _read_lines(self, client: _Client) -> None:
        while self._client is client:
            try:
                line = await client.reader.readline()
            except ValueError:
                # Over MAX_LINE_BYTES without a newline: the stream can no longer be split
                # into lines with any confidence, so stop, say why, and hang up.
                reason = f"line longer than {MAX_LINE_BYTES} bytes; hanging up"
                if self._client is client:
                    self.core.reject(reason, self._clock())
                    self._send(client, _error(reason))
                return
            except OSError as exc:
                logger.info("%s: connection lost (%s)", client.peer, exc)
                return
            if not line:
                logger.info("%s hung up", client.peer)
                return
            if self._client is client:  # a replaced client's buffered lines are ignored
                self._on_line(client, line)

    def _on_line(self, client: _Client, line: bytes) -> None:
        now = self._clock()
        try:
            msg = decode(line, allowed=CLIENT_MESSAGES)
        except ProtocolError as exc:
            self.core.reject(str(exc), now)
            self._send(client, _error(str(exc)))
            return
        refusal = self.core.handle(msg, now)
        if refusal is not None:
            self._send(client, _error(refusal))

    def _send(self, client: _Client, msg: Message) -> None:
        """Queue one line; a client too far behind to keep up is dropped."""
        transport = client.writer.transport
        if transport.is_closing():
            return
        if transport.get_write_buffer_size() > MAX_WRITE_BUFFER:
            self._drop(client, f"over {MAX_WRITE_BUFFER} bytes behind on the state stream")
            return
        client.writer.write(encode(msg))

    def _forget(self, client: _Client) -> None:
        """Stop listening to `client`; if it was in charge, the robot stops."""
        if self._client is client:
            self._client = None
            self.core.disconnected(self._clock())

    def _drop(self, client: _Client, why: str) -> None:
        """Hang up on `client` at once, discarding anything unsent."""
        logger.warning("dropping %s: %s", client.peer, why)
        self._forget(client)
        client.writer.transport.abort()

    async def _tick_forever(self) -> None:
        loop = asyncio.get_running_loop()
        period = 1.0 / self._state_hz
        next_at = loop.time()
        while True:
            self._tick_once()
            next_at += period
            delay = next_at - loop.time()
            if delay < 0.0:  # fell behind (a slow driver): skip ahead rather than burst
                next_at, delay = loop.time(), 0.0
            await asyncio.sleep(delay)

    def _tick_once(self) -> None:
        """One tick and one State; nothing escapes, and a failure is logged once per streak."""
        try:
            now = self._clock()
            self.core.tick(now)
            if self._client is not None:
                self._send(self._client, self.core.snapshot(now))
        except Exception:
            if not self._tick_failing:
                logger.exception("bridge tick failed; retrying every tick")
            self._tick_failing = True
        else:
            if self._tick_failing:
                logger.info("bridge tick recovered")
            self._tick_failing = False


class ServerThread:
    """A BridgeServer on its own event loop in a daemon thread, for tests and sync callers."""

    def __init__(self, server: BridgeServer) -> None:
        self.server = server
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._ready = threading.Event()
        self._startup_error: BaseException | None = None
        self._port = 0

    def start(self, timeout: float = 5.0) -> int:
        """Start serving and return the bound port."""
        self._thread = threading.Thread(target=self._run, name="bridge-server", daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout):
            raise TimeoutError(f"the bridge server did not start within {timeout} s")
        if self._startup_error is not None:
            raise self._startup_error
        return self._port

    def call(self, fn: Callable[[], Any], timeout: float = 2.0) -> Any:
        """Run `fn()` on the server's loop thread and return its result, or raise its error."""
        if self._loop is None:
            raise RuntimeError("the server thread has not been started")

        async def invoke() -> Any:
            return fn()

        return asyncio.run_coroutine_threadsafe(invoke(), self._loop).result(timeout)

    def stop(self, timeout: float = 5.0) -> None:
        """Close the server (the motors stop) and wait for the thread; safe to call twice."""
        if self._thread is None or not self._thread.is_alive() or self._loop is None:
            return
        asyncio.run_coroutine_threadsafe(self.server.close(), self._loop).result(timeout)
        self._thread.join(timeout)

    def __enter__(self) -> ServerThread:
        self.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self.stop()

    def _run(self) -> None:
        asyncio.run(self._main())

    async def _main(self) -> None:
        self._loop = asyncio.get_running_loop()
        try:
            self._port = await self.server.start()
        except Exception as exc:
            self._startup_error = exc
            return
        finally:
            self._ready.set()
        await self.server.serve_forever()


def _error(reason: str) -> Error:
    """An Error that always fits on the wire, however long the reason."""
    if len(reason) > MAX_TEXT_LEN:
        reason = reason[: MAX_TEXT_LEN - 3] + "..."
    return Error(reason=reason)


def _whole_ms(seconds: float) -> int:
    return max(1, round(seconds * 1000.0))


def _peer_name(writer: asyncio.StreamWriter) -> str:
    peer = writer.get_extra_info("peername")
    if isinstance(peer, tuple) and len(peer) >= 2:
        host, port = peer[0], peer[1]
        return f"[{host}]:{port}" if ":" in str(host) else f"{host}:{port}"
    return str(peer)
