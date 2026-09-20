"""The Pi bridge: every stop exactly on a fake clock, then the same promises over sockets."""

from __future__ import annotations

import logging
import math
import re
import signal
import socket
import subprocess
import sys
import time
import unittest
from collections.abc import Callable
from pathlib import Path
from typing import Any

from retriever.bridge.client import BridgeError, BridgeRobot, parse_address
from retriever.bridge.fake_driver import FakeTankDriver
from retriever.bridge.protocol import (
    MAX_LINE_BYTES,
    MAX_TEXT_LEN,
    SERVER_MESSAGES,
    Act,
    ClearEstop,
    Error,
    Estop,
    Heartbeat,
    Hello,
    Message,
    State,
    decode,
    encode,
)
from retriever.bridge.server import (
    ZERO_REFRESH_S,
    BridgeCore,
    BridgeServer,
    HardwareDriver,
    ServerThread,
)
from retriever.navigation.geometry import TAU
from retriever.navigation.kinematics import TankGeometry, tank_body_to_wheels
from retriever.types import Action

SRC = Path(__file__).resolve().parents[1] / "src"

# Binary fractions, so every `now - then >= timeout` below is exact.
TIMEOUT_S = 0.25
MOTION_TIMEOUT_S = 0.5
MAX_WHEEL_MPS = 0.8

_QUIET = logging.NullHandler()


def setUpModule() -> None:
    # Every stop logs a warning, by design; tests that care about a log line use assertLogs.
    logging.getLogger("retriever.bridge").addHandler(_QUIET)


def tearDownModule() -> None:
    logging.getLogger("retriever.bridge").removeHandler(_QUIET)


class RecordingDriver:
    """A HardwareDriver that records every call and can be told to fail one of them."""

    counts_per_rev = 4096

    def __init__(self) -> None:
        self.calls: list[tuple[Any, ...]] = []
        self.wheels = (0.0, 0.0)
        self.fail: set[str] = set()

    def _record(self, name: str, *args: Any) -> None:
        self.calls.append((name, *args))
        if name in self.fail:
            raise OSError(f"{name} failed: serial bus timeout")

    def set_wheels(self, left_mps: float, right_mps: float) -> None:
        self._record("set_wheels", left_mps, right_mps)
        self.wheels = (left_mps, right_mps)

    def set_joints(self, targets: dict[str, float]) -> None:
        self._record("set_joints", dict(targets))

    def set_gripper(self, position: float) -> None:
        self._record("set_gripper", position)

    def set_vacuum(self, on: bool) -> None:
        self._record("set_vacuum", on)

    def read_state(self) -> dict[str, Any]:
        self._record("read_state")
        return {
            "left_ticks": 4095,
            "right_ticks": 7,
            "joints": {"shoulder_pan": 0.25, "gripper": 0.9},
            "gripper_load": 0.6,
            "battery": 0.75,
        }

    def stop(self) -> None:
        self._record("stop")
        self.wheels = (0.0, 0.0)

    @property
    def stops(self) -> int:
        return sum(1 for call in self.calls if call[0] == "stop")

    @property
    def moving(self) -> bool:
        return self.wheels != (0.0, 0.0)


def make_core(**kw: Any) -> tuple[BridgeCore, RecordingDriver]:
    driver = RecordingDriver()
    options: dict[str, Any] = {
        "timeout_s": TIMEOUT_S,
        "motion_timeout_s": MOTION_TIMEOUT_S,
        "max_wheel_mps": MAX_WHEEL_MPS,
    }
    options.update(kw)
    core = BridgeCore(driver, now=0.0, **options)
    core.connected(0.0)
    return core, driver


def drive(core: BridgeCore, now: float, seq: int = 1, vx: float = 0.2, **kw: Any) -> None:
    refusal = core.handle(Act(seq=seq, base_vx=vx, **kw), now)
    assert refusal is None, refusal


class StartsTrippedTest(unittest.TestCase):
    def test_nothing_is_in_charge_until_a_client_proves_it_is_alive(self) -> None:
        driver = RecordingDriver()
        core = BridgeCore(driver, now=0.0)
        self.assertTrue(core.watchdog_tripped)
        self.assertEqual(driver.calls, [("stop",)])  # the motors are told to stop at boot
        self.assertFalse(core.link_alive(0.0))
        self.assertTrue(core.snapshot(0.0).watchdog_tripped)

    def test_connecting_alone_clears_nothing(self) -> None:
        core, _ = make_core()
        self.assertTrue(core.watchdog_tripped)
        self.assertTrue(core.link_alive(0.1))  # a grace period, so it is not replaced at once
        core.tick(TIMEOUT_S + 0.1)
        self.assertTrue(core.watchdog_tripped)

    def test_first_valid_message_clears_the_trip(self) -> None:
        core, _ = make_core()
        self.assertIsNone(core.handle(Heartbeat(), 0.1))
        self.assertFalse(core.watchdog_tripped)
        self.assertFalse(core.snapshot(0.1).watchdog_tripped)

    def test_rejects_a_driver_that_is_not_a_hardware_driver(self) -> None:
        with self.assertRaisesRegex(TypeError, "HardwareDriver"):
            BridgeCore(object(), now=0.0)  # type: ignore[arg-type]
        self.assertIsInstance(RecordingDriver(), HardwareDriver)

    def test_rejects_non_positive_limits(self) -> None:
        for bad in (
            {"timeout_s": 0.0},
            {"motion_timeout_s": -1.0},
            {"max_wheel_mps": math.nan},
        ):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                BridgeCore(RecordingDriver(), now=0.0, **bad)


class WatchdogTest(unittest.TestCase):
    def test_trips_at_timeout_and_stops_the_wheels(self) -> None:
        core, driver = make_core()
        drive(core, 1.0)
        core.tick(1.0 + TIMEOUT_S - 0.01)
        self.assertTrue(driver.moving)
        self.assertFalse(core.watchdog_tripped)
        core.tick(1.0 + TIMEOUT_S)
        self.assertFalse(driver.moving)
        self.assertTrue(core.watchdog_tripped)
        self.assertTrue(core.snapshot(1.0 + TIMEOUT_S).watchdog_tripped)

    def test_stops_once_per_trip(self) -> None:
        core, driver = make_core()
        drive(core, 1.0)
        stops = driver.stops
        for step in range(10):
            core.tick(1.0 + TIMEOUT_S + step * 0.02)
        self.assertEqual(driver.stops, stops + 1)

    def test_any_valid_message_feeds_it_and_clears_a_trip(self) -> None:
        core, driver = make_core()
        for now, msg in ((0.0, Heartbeat()), (0.125, ClearEstop()), (0.25, Heartbeat())):
            core.handle(msg, now)
            core.tick(now + TIMEOUT_S - 0.01)
            self.assertFalse(core.watchdog_tripped)
        core.tick(0.25 + TIMEOUT_S)
        self.assertTrue(core.watchdog_tripped)
        core.handle(Heartbeat(), 1.0)
        self.assertFalse(core.watchdog_tripped)
        self.assertFalse(driver.moving)  # clearing the trip moves nothing by itself

    def test_link_alive_tracks_the_last_valid_message(self) -> None:
        core, _ = make_core()
        core.handle(Heartbeat(), 1.0)
        self.assertTrue(core.link_alive(1.0 + TIMEOUT_S - 0.01))
        self.assertFalse(core.link_alive(1.0 + TIMEOUT_S))
        core.disconnected(1.1)
        self.assertFalse(core.link_alive(1.1))

    def test_disconnect_stops_at_once_and_trips(self) -> None:
        core, driver = make_core()
        drive(core, 1.0)
        core.disconnected(1.01)
        self.assertFalse(driver.moving)
        self.assertTrue(core.watchdog_tripped)


class MotionDeadmanTest(unittest.TestCase):
    def test_heartbeats_alone_do_not_keep_the_wheels_moving(self) -> None:
        core, driver = make_core()
        drive(core, 1.0)
        for now in (1.125, 1.25, 1.375, 1.49):
            core.handle(Heartbeat(), now)
            core.tick(now)
        self.assertTrue(driver.moving)
        core.handle(Heartbeat(), 1.0 + MOTION_TIMEOUT_S)
        core.tick(1.0 + MOTION_TIMEOUT_S)
        self.assertFalse(driver.moving)
        self.assertEqual(driver.calls[-1], ("stop",))  # stop(): the arm holds too
        self.assertFalse(core.watchdog_tripped)  # the link is fine; the control loop is not

    def test_fresh_acts_keep_it_driving(self) -> None:
        core, driver = make_core()
        for step in range(20):
            now = 1.0 + step * 0.125
            drive(core, now, seq=step + 1)
            core.tick(now + 0.1)
        self.assertTrue(driver.moving)

    def test_the_next_act_drives_again(self) -> None:
        core, driver = make_core()
        drive(core, 1.0)
        core.handle(Heartbeat(), 1.375)
        core.tick(1.0 + MOTION_TIMEOUT_S)
        self.assertFalse(driver.moving)
        drive(core, 1.6, seq=2)
        self.assertTrue(driver.moving)


class EstopTest(unittest.TestCase):
    def test_latches_across_disconnect_and_refuses_acts_without_advancing_seq(self) -> None:
        core, driver = make_core()
        drive(core, 1.0, seq=7)
        core.handle(Estop(reason="operator hit the red button"), 1.1)
        self.assertTrue(core.estop)
        self.assertFalse(driver.moving)
        core.disconnected(1.2)
        core.connected(1.3)
        refusal = core.handle(Act(seq=8, base_vx=0.3), 1.4)
        assert refusal is not None
        self.assertIn("estop is latched", refusal)
        self.assertIn("operator hit the red button", refusal)
        self.assertEqual(core.last_seq, 7)
        self.assertFalse(driver.moving)
        self.assertTrue(core.snapshot(1.4).estop)

    def test_refused_acts_touch_nothing(self) -> None:
        core, driver = make_core()
        core.handle(Estop(), 1.0)
        calls = len(driver.calls)
        core.handle(Act(seq=1, base_vx=0.3, joints={"elbow_flex": 1.0}, vacuum=True), 1.1)
        self.assertEqual(len(driver.calls), calls)

    def test_only_clear_estop_clears_it_and_nothing_moves_until_the_next_act(self) -> None:
        core, driver = make_core()
        drive(core, 1.0)
        core.trigger_estop("button on the robot", 1.1)
        self.assertEqual(core.estop_reason, "button on the robot")
        core.handle(Heartbeat(), 1.2)
        self.assertTrue(core.estop)
        self.assertIsNone(core.handle(ClearEstop(), 1.3))
        self.assertFalse(core.estop)
        self.assertEqual(core.estop_reason, "")
        self.assertFalse(driver.moving)
        drive(core, 1.4, seq=2)
        self.assertTrue(driver.moving)
        self.assertEqual(core.last_seq, 2)

    def test_the_first_reason_is_kept_while_latched(self) -> None:
        core, _ = make_core()
        core.handle(Estop(reason="first"), 1.0)
        core.handle(Estop(reason="second"), 1.1)
        self.assertEqual(core.estop_reason, "first")

    def test_an_estop_with_no_reason_still_says_where_it_came_from(self) -> None:
        core, _ = make_core()
        core.handle(Estop(), 1.0)
        self.assertIn("laptop", core.estop_reason)


class RejectTest(unittest.TestCase):
    def test_a_malformed_line_zeroes_the_wheels(self) -> None:
        core, driver = make_core()
        drive(core, 1.0)
        core.reject("invalid JSON", 1.05)
        self.assertFalse(driver.moving)

    def test_a_malformed_line_does_not_feed_the_watchdog(self) -> None:
        core, _ = make_core()
        drive(core, 1.0)
        core.reject("invalid JSON", 1.125)
        core.tick(1.0 + TIMEOUT_S)
        self.assertTrue(core.watchdog_tripped)


class WheelSpeedTest(unittest.TestCase):
    def test_within_the_limit_it_is_plain_tank_kinematics(self) -> None:
        core, _ = make_core()
        self.assertEqual(core.wheel_speeds(0.3, 0.5), tank_body_to_wheels(0.3, 0.5, core.geo))

    def test_an_over_fast_arc_keeps_its_curvature(self) -> None:
        core, _ = make_core(geo=TankGeometry(scrub_factor=1.5))
        vx, wz = 1.2, 2.0
        raw_left, raw_right = tank_body_to_wheels(vx, wz, core.geo)
        left, right = core.wheel_speeds(vx, wz)
        self.assertAlmostEqual(max(abs(left), abs(right)), MAX_WHEEL_MPS)
        self.assertAlmostEqual(left / right, raw_left / raw_right)  # same arc, just slower
        self.assertAlmostEqual(left / raw_left, right / raw_right)

    def test_an_over_fast_spin_stays_a_spin(self) -> None:
        core, _ = make_core()
        left, right = core.wheel_speeds(0.0, 20.0)
        self.assertAlmostEqual(right, MAX_WHEEL_MPS)
        self.assertAlmostEqual(left, -MAX_WHEEL_MPS)

    def test_an_act_is_clamped_on_the_way_to_the_driver(self) -> None:
        core, driver = make_core()
        drive(core, 1.0, vx=5.0)
        self.assertEqual(driver.wheels, (MAX_WHEEL_MPS, MAX_WHEEL_MPS))

    def test_non_finite_velocity_is_refused(self) -> None:
        core, _ = make_core()
        for vx, wz in ((math.nan, 0.0), (0.0, math.inf)):
            with self.subTest(vx=vx, wz=wz), self.assertRaises(ValueError):
                core.wheel_speeds(vx, wz)


class ActTest(unittest.TestCase):
    def test_applies_wheels_then_joints_gripper_vacuum_then_records_seq(self) -> None:
        core, driver = make_core()
        driver.calls.clear()
        act = Act(seq=3, base_vx=0.2, joints={"elbow_flex": 1.0}, gripper=0.1, vacuum=True)
        self.assertIsNone(core.handle(act, 1.0))
        self.assertEqual(
            driver.calls,
            [
                ("set_wheels", 0.2, 0.2),
                ("set_joints", {"elbow_flex": 1.0}),
                ("set_gripper", 0.1),
                ("set_vacuum", True),
            ],
        )
        self.assertEqual(core.last_seq, 3)

    def test_absent_parts_are_left_alone(self) -> None:
        core, driver = make_core()
        driver.calls.clear()
        core.handle(Act(seq=1), 1.0)
        self.assertEqual(driver.calls, [("set_wheels", 0.0, 0.0)])

    def test_a_driver_exception_is_refused_and_stops_the_robot(self) -> None:
        core, driver = make_core()
        drive(core, 1.0, seq=4)
        driver.fail.add("set_joints")
        refusal = core.handle(Act(seq=5, base_vx=0.3, joints={"elbow_flex": 1.0}), 1.1)
        self.assertEqual(
            refusal, "driver refused the command: set_joints failed: serial bus timeout"
        )
        self.assertFalse(driver.moving)
        self.assertEqual(driver.calls[-1], ("stop",))
        self.assertEqual(core.last_seq, 4)

    def test_a_failing_stop_is_survived_and_retried_by_the_zero_refresh(self) -> None:
        core, driver = make_core()
        drive(core, 1.0)
        driver.fail.add("stop")
        with self.assertLogs("retriever.bridge.server", "ERROR"):
            core.disconnected(1.25)
        self.assertTrue(core.watchdog_tripped)
        core.tick(1.25 + ZERO_REFRESH_S)
        self.assertEqual(driver.calls[-1], ("set_wheels", 0.0, 0.0))
        self.assertFalse(driver.moving)


class ZeroRefreshTest(unittest.TestCase):
    def test_while_stopped_zero_is_re_sent_every_refresh_period(self) -> None:
        core, driver = make_core()
        drive(core, 1.0)
        core.tick(1.25)  # the watchdog calls stop()
        driver.calls.clear()
        core.tick(1.25 + ZERO_REFRESH_S - 0.01)
        self.assertEqual(driver.calls, [])
        core.tick(1.25 + ZERO_REFRESH_S)
        core.tick(1.25 + 2 * ZERO_REFRESH_S)
        self.assertEqual(driver.calls, [("set_wheels", 0.0, 0.0)] * 2)

    def test_nothing_is_re_sent_while_driving(self) -> None:
        core, driver = make_core()
        for step in range(20):
            now = 1.0 + step * 0.125
            drive(core, now, seq=step + 1)
            core.tick(now)
        self.assertNotIn(("set_wheels", 0.0, 0.0), driver.calls)

    def test_a_zero_velocity_act_counts_as_a_fresh_zero(self) -> None:
        core, driver = make_core()
        drive(core, 1.0, vx=0.0)
        driver.calls.clear()
        for now in (1.125, 1.25, 1.375, 1.49):
            core.handle(Heartbeat(), now)
            core.tick(now)
        self.assertEqual(driver.calls, [])


class SnapshotAndHelloTest(unittest.TestCase):
    def test_snapshot_reports_the_driver_and_the_safety_flags(self) -> None:
        core, _ = make_core()
        drive(core, 1.0, seq=9)
        core.trigger_estop("test", 1.1)
        state = core.snapshot(1.2)
        self.assertEqual(
            (state.seq, state.t, state.left_ticks, state.right_ticks), (9, 1.2, 4095, 7)
        )
        self.assertEqual(state.joints, {"shoulder_pan": 0.25, "gripper": 0.9})
        self.assertEqual((state.gripper_load, state.battery), (0.6, 0.75))
        self.assertTrue(state.estop)
        self.assertFalse(state.watchdog_tripped)
        self.assertEqual(decode(encode(state)), state)

    def test_snapshot_raises_when_the_driver_cannot_be_read(self) -> None:
        core, driver = make_core()
        driver.fail.add("read_state")
        with self.assertRaises(OSError):
            core.snapshot(1.0)

    def test_hello_carries_the_pis_constants_as_whole_milliseconds(self) -> None:
        geo = TankGeometry(wheel_radius_m=0.05, track_width_m=0.32, scrub_factor=1.4)
        core, _ = make_core(geo=geo, timeout_s=0.2504, motion_timeout_s=0.5)
        hello = core.hello(50.0)
        self.assertEqual(
            hello,
            Hello(
                counts_per_rev=4096,
                wheel_radius_m=0.05,
                track_width_m=0.32,
                scrub_factor=1.4,
                state_hz=50.0,
                timeout_ms=250,
                motion_timeout_ms=500,
            ),
        )
        self.assertEqual(decode(encode(hello)), hello)

    def test_a_server_message_is_refused_and_does_not_feed_the_watchdog(self) -> None:
        core, _ = make_core()
        refusal = core.handle(core.snapshot(0.1), 0.1)
        self.assertIsNotNone(refusal)
        self.assertTrue(core.watchdog_tripped)


# --- real sockets: ServerThread + BridgeRobot against FakeTankDriver ----------------------

SOCKET_TIMEOUT_MS = 150  # the Pi's watchdog in these tests; BridgeRobot beats at 50 Hz
SOCKET_MOTION_TIMEOUT_MS = 300
WAIT_S = 2.0  # generous upper bound for anything that should happen within a few ticks


def wait_until(predicate: Callable[[], bool], timeout: float = WAIT_S) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.002)
    return predicate()


class ManualClock:
    def __init__(self) -> None:
        self.now = 50.0

    def __call__(self) -> float:
        return self.now


class RawClient:
    """A hand-rolled client, like `nc`: it sends exactly what it is told, or goes silent."""

    def __init__(self, port: int) -> None:
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=WAIT_S)
        self.lines = self.sock.makefile("rb")

    def send(self, msg: Message | bytes) -> None:
        self.sock.sendall(msg if isinstance(msg, bytes) else encode(msg))

    def read(self) -> Message | None:
        line = self.lines.readline()
        return decode(line, allowed=SERVER_MESSAGES) if line else None

    def read_until(self, cls: type, check: Callable[[Any], bool] = lambda msg: True) -> Any:
        deadline = time.monotonic() + WAIT_S
        while time.monotonic() < deadline:
            msg = self.read()
            if msg is None:
                raise AssertionError(f"connection closed while waiting for {cls.__name__}")
            if isinstance(msg, cls) and check(msg):
                return msg
        raise AssertionError(f"no {cls.__name__} within {WAIT_S} s")

    def at_eof(self) -> bool:
        """Read until the server hangs up; True if it did within WAIT_S."""
        deadline = time.monotonic() + WAIT_S
        while time.monotonic() < deadline:
            if not self.lines.readline():
                return True
        return False

    def close(self) -> None:
        self.lines.close()
        self.sock.close()


class UnreadableDriver(FakeTankDriver):
    """A FakeTankDriver whose sensors can be made to fail, like a flaky serial bus."""

    unreadable = False

    def read_state(self) -> dict[str, Any]:
        if self.unreadable:
            raise OSError("encoder bus timeout")
        return super().read_state()


class BridgeOverSocketsTest(unittest.TestCase):
    def serve(
        self, driver: FakeTankDriver | None = None, **kw: Any
    ) -> tuple[ServerThread, int]:
        server = BridgeServer(
            driver if driver is not None else FakeTankDriver(),
            "127.0.0.1",
            0,
            timeout_ms=SOCKET_TIMEOUT_MS,
            motion_timeout_ms=SOCKET_MOTION_TIMEOUT_MS,
            state_hz=100.0,
            **kw,
        )
        thread = ServerThread(server)
        port = thread.start()
        self.addCleanup(thread.stop)
        return thread, port

    def robot(self, port: int, **kw: Any) -> BridgeRobot:
        robot = BridgeRobot("127.0.0.1", port, heartbeat_hz=50.0, **kw)
        self.addCleanup(robot.close)
        return robot

    def raw(self, port: int) -> RawClient:
        client = RawClient(port)
        self.addCleanup(client.close)
        self.assertIsInstance(client.read(), Hello)  # always the first line
        return client

    def test_hello_carries_the_pis_constants_and_odometry_uses_them(self) -> None:
        geo = TankGeometry(wheel_radius_m=0.05, track_width_m=0.32, scrub_factor=1.3)
        clock = ManualClock()  # the wheels turn only when the test says so
        driver = FakeTankDriver(geo, counts_per_rev=2048, clock=clock)
        _, port = self.serve(driver, geo=geo)
        robot = self.robot(port)
        hello = robot.hello
        self.assertEqual(hello.counts_per_rev, 2048)
        self.assertEqual(
            (hello.wheel_radius_m, hello.track_width_m, hello.scrub_factor), (0.05, 0.32, 1.3)
        )
        self.assertEqual((hello.timeout_ms, hello.motion_timeout_ms), (150, 300))
        robot.observe()  # the first state is the odometry origin
        robot.act(Action(base_vx=0.1))
        self.assertTrue(wait_until(lambda: driver.wheel_speeds == (0.1, 0.1)))
        clock.now += 1.0  # 0.1 m of wheel travel, counted in the Pi's own ticks
        tick_m = TAU * 0.05 / 2048
        self.assertTrue(wait_until(lambda: abs(robot.observe().base.x - 0.1) < 2 * tick_m))
        self.assertAlmostEqual(robot.observe().base.theta, 0.0)

    def test_driving_forward_moves_the_pose_forward(self) -> None:
        _, port = self.serve()
        robot = self.robot(port)
        start = time.monotonic()
        while time.monotonic() - start < 0.3:
            robot.act(Action(base_vx=0.3))
            robot.observe()
        pose = robot.observe().base
        self.assertGreater(pose.x, 0.03)
        self.assertAlmostEqual(pose.y, 0.0, places=6)
        self.assertAlmostEqual(pose.theta, 0.0, places=6)
        self.assertFalse(robot.watchdog_tripped)
        self.assertGreater(robot.state.seq, 1)  # type: ignore[union-attr]

    def test_a_client_that_hangs_up_stops_the_wheels_at_once(self) -> None:
        driver = FakeTankDriver()
        thread, port = self.serve(driver)
        robot = self.robot(port)
        robot.act(Action(base_vx=0.2))
        self.assertTrue(wait_until(lambda: driver.wheel_speeds != (0.0, 0.0)))
        robot.close()
        self.assertTrue(wait_until(lambda: driver.wheel_speeds == (0.0, 0.0), timeout=0.1))
        self.assertTrue(thread.call(lambda: thread.server.core.watchdog_tripped))
        with self.assertRaisesRegex(BridgeError, "closed by this laptop"):
            robot.observe()

    def test_a_silent_client_is_stopped_by_the_watchdog_within_the_timeout(self) -> None:
        driver = FakeTankDriver()
        _, port = self.serve(driver)
        client = self.raw(port)
        client.send(Act(seq=1, base_vx=0.2))  # ...and then nothing, socket still open
        self.assertTrue(wait_until(lambda: driver.wheel_speeds != (0.0, 0.0)))
        moving_seen_at = time.monotonic()
        self.assertTrue(wait_until(lambda: driver.wheel_speeds == (0.0, 0.0)))
        stopped_after = time.monotonic() - moving_seen_at
        self.assertGreater(stopped_after, 0.5 * SOCKET_TIMEOUT_MS / 1000)
        self.assertLess(stopped_after, SOCKET_MOTION_TIMEOUT_MS / 1000)  # the watchdog, first
        client.read_until(State, lambda state: state.watchdog_tripped)

    def test_heartbeats_alone_do_not_keep_the_robot_driving(self) -> None:
        driver = FakeTankDriver()
        _, port = self.serve(driver)
        robot = self.robot(port)
        robot.act(Action(base_vx=0.2, joints={"elbow_flex": 1.0}))
        self.assertTrue(wait_until(lambda: driver.wheel_speeds != (0.0, 0.0)))
        self.assertTrue(wait_until(lambda: driver.wheel_speeds == (0.0, 0.0)))
        self.assertTrue(robot.connected)
        robot.observe()
        self.assertFalse(robot.watchdog_tripped)  # the link is fine; the control loop is not
        self.assertEqual(driver.stop_count, 2)  # at start-up, and the deadman (arm holds)

    def test_a_second_client_is_refused_while_the_first_is_live(self) -> None:
        driver = FakeTankDriver()
        _, port = self.serve(driver)
        first = self.robot(port)
        with self.assertRaisesRegex(BridgeError, "refused this connection: another client"):
            self.robot(port)
        first.act(Action(base_vx=0.2))
        self.assertTrue(wait_until(lambda: driver.wheel_speeds != (0.0, 0.0)))

    def test_a_stale_client_is_replaced_so_a_restarted_laptop_gets_back_in(self) -> None:
        driver = FakeTankDriver()
        thread, port = self.serve(driver)
        stale = self.raw(port)
        stale.send(Heartbeat())  # then silence: a half-open socket after a Wi-Fi drop
        core = thread.server.core

        def stale_link() -> bool:
            return not thread.call(lambda: core.link_alive(time.monotonic()))

        self.assertTrue(wait_until(stale_link))
        robot = self.robot(port)
        robot.act(Action(base_vx=0.2))
        self.assertTrue(wait_until(lambda: driver.wheel_speeds != (0.0, 0.0)))
        self.assertTrue(stale.at_eof())

    def test_a_strafe_act_is_refused_at_the_laptop_and_never_sent(self) -> None:
        driver = FakeTankDriver()
        _, port = self.serve(driver)
        robot = self.robot(port)
        with self.assertRaisesRegex(ValueError, "cannot move sideways"):
            robot.act(Action(base_vx=0.2, base_vy=0.1))
        robot.act(Action(joints={"gripper": 0.5}))
        self.assertTrue(wait_until(lambda: robot.observe().joints["gripper"] < 1.0))
        self.assertEqual(robot.state.seq, 1)  # type: ignore[union-attr]
        self.assertEqual(driver.wheel_speeds, (0.0, 0.0))

    def test_a_malformed_line_gets_an_error_and_stops_the_base(self) -> None:
        driver = FakeTankDriver()
        _, port = self.serve(driver)
        client = self.raw(port)
        client.send(Act(seq=1, base_vx=0.2))
        self.assertTrue(wait_until(lambda: driver.wheel_speeds != (0.0, 0.0)))
        client.send(b'{"v":1,"type":"act","seq":2,"base_vx":0.2,\n')
        error = client.read_until(Error)
        self.assertIn("invalid JSON", error.reason)
        self.assertEqual(driver.wheel_speeds, (0.0, 0.0))
        client.send(Act(seq=3, base_vx=0.2))  # the connection is still up
        self.assertTrue(wait_until(lambda: driver.wheel_speeds != (0.0, 0.0)))

    def test_an_over_long_line_stops_the_base_says_why_and_hangs_up(self) -> None:
        driver = FakeTankDriver()
        _, port = self.serve(driver)
        client = self.raw(port)
        client.send(Act(seq=1, base_vx=0.2))
        self.assertTrue(wait_until(lambda: driver.wheel_speeds != (0.0, 0.0)))
        client.send(b"x" * (MAX_LINE_BYTES + 100))
        self.assertIn("longer than", client.read_until(Error).reason)
        self.assertEqual(driver.wheel_speeds, (0.0, 0.0))
        self.assertTrue(client.at_eof())

    def test_estop_latches_across_reconnect_until_cleared(self) -> None:
        driver = FakeTankDriver()
        _, port = self.serve(driver)
        robot = self.robot(port)
        robot.act(Action(base_vx=0.2))
        self.assertTrue(wait_until(lambda: driver.wheel_speeds != (0.0, 0.0)))
        robot.estop("operator: " + "x" * 490)  # long enough that the refusal must be clipped
        self.assertTrue(wait_until(lambda: robot.estopped))
        self.assertEqual(driver.wheel_speeds, (0.0, 0.0))
        robot.act(Action(base_vx=0.2))
        self.assertTrue(wait_until(lambda: robot.last_error is not None))
        self.assertIn("estop is latched (operator: xxx", robot.last_error or "")
        self.assertLessEqual(len(robot.last_error or ""), MAX_TEXT_LEN)
        robot.close()

        again = self.robot(port)
        self.assertTrue(wait_until(lambda: again.estopped))
        again.clear_estop()
        self.assertTrue(wait_until(lambda: not again.estopped))
        self.assertEqual(driver.wheel_speeds, (0.0, 0.0))  # clearing moves nothing
        again.act(Action(base_vx=0.2))
        self.assertTrue(wait_until(lambda: driver.wheel_speeds != (0.0, 0.0)))

    def test_a_local_estop_from_the_pi_stops_the_robot(self) -> None:
        driver = FakeTankDriver()
        thread, port = self.serve(driver)
        robot = self.robot(port)
        robot.act(Action(base_vx=0.2))
        self.assertTrue(wait_until(lambda: driver.wheel_speeds != (0.0, 0.0)))
        thread.call(lambda: thread.server.trigger_estop("red button on the robot"))
        self.assertEqual(driver.wheel_speeds, (0.0, 0.0))
        self.assertTrue(wait_until(lambda: robot.estopped))

    def test_observe_refuses_state_that_has_stopped_arriving(self) -> None:
        driver = UnreadableDriver()
        _, port = self.serve(driver)
        robot = self.robot(port, stale_after_s=0.1)
        robot.observe()
        driver.unreadable = True  # the Pi is up, the socket is up, but no states come

        def stalled() -> bool:
            try:
                robot.observe()
            except BridgeError as exc:
                return "stalled" in str(exc)
            return False

        self.assertTrue(wait_until(stalled))

    def test_vacuum_is_sent_as_a_zero_velocity_act(self) -> None:
        driver = FakeTankDriver()
        _, port = self.serve(driver)
        robot = self.robot(port)
        robot.set_vacuum(True)
        self.assertTrue(wait_until(lambda: driver.vacuum))
        self.assertEqual(driver.wheel_speeds, (0.0, 0.0))

    def test_the_tick_loop_survives_a_driver_that_cannot_be_read(self) -> None:
        driver = UnreadableDriver()
        _, port = self.serve(driver)
        client = self.raw(port)
        with self.assertLogs("retriever.bridge.server", logging.ERROR) as logs:
            driver.unreadable = True
            client.send(Act(seq=1, base_vx=0.2))
            self.assertTrue(wait_until(lambda: driver.wheel_speeds != (0.0, 0.0)))
            # No states can be read, but the watchdog still runs and still stops the base.
            self.assertTrue(wait_until(lambda: driver.wheel_speeds == (0.0, 0.0)))
        self.assertEqual(sum("tick failed" in line for line in logs.output), 1)  # once
        driver.unreadable = False
        client.read_until(State)

    def test_closing_the_server_stops_the_motors(self) -> None:
        driver = FakeTankDriver()
        thread, port = self.serve(driver)
        robot = self.robot(port)
        robot.act(Action(base_vx=0.2))
        self.assertTrue(wait_until(lambda: driver.wheel_speeds != (0.0, 0.0)))
        thread.stop()
        self.assertEqual(driver.wheel_speeds, (0.0, 0.0))
        self.assertTrue(wait_until(lambda: not robot.connected))
        thread.stop()  # idempotent


class BridgeRobotConnectTest(unittest.TestCase):
    def test_nothing_listening_is_a_bridge_error_that_says_so(self) -> None:
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]  # a port that was free a moment ago, and is again
        with self.assertRaisesRegex(BridgeError, "refused.*fake_pi"):
            BridgeRobot("127.0.0.1", port, connect_timeout_s=1.0)

    def test_parse_address(self) -> None:
        cases = {
            "pi.local": ("pi.local", 7777),
            "pi.local:8000": ("pi.local", 8000),
            " 10.0.0.2:7000 ": ("10.0.0.2", 7000),
            ":9000": ("127.0.0.1", 9000),
            "[::1]:7778": ("::1", 7778),
            "[::1]": ("::1", 7777),
            "fe80::1%eth0": ("fe80::1%eth0", 7777),
        }
        for addr, expected in cases.items():
            with self.subTest(addr=addr):
                self.assertEqual(parse_address(addr), expected)
        self.assertEqual(parse_address("pi", default_port=1234), ("pi", 1234))
        for bad in ("", "pi:", "pi:http", "pi:0", "pi:65536", "[::1", "[::1]7777", "[]:1"):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                parse_address(bad)


class FakePiScriptTest(unittest.TestCase):
    SCRIPT = SRC.parent / "scripts" / "fake_pi.py"

    def test_starts_answers_with_hello_and_stops_cleanly_on_sigint(self) -> None:
        proc = subprocess.Popen(
            [sys.executable, str(self.SCRIPT), "--host", "127.0.0.1", "--port", "0"],
            stderr=subprocess.PIPE,
            text=True,
        )
        self.addCleanup(proc.kill)
        assert proc.stderr is not None
        self.addCleanup(proc.stderr.close)
        started, match = "", None
        for _ in range(10):                     # the gyro says what it found first
            line = proc.stderr.readline()
            if not line:
                break
            started += line
            match = re.search(r"listening on 127\.0\.0\.1 port (\d+);.*watchdog 300 ms", line)
            if match:
                break
        self.assertIsNotNone(match, started)
        assert match is not None
        client = RawClient(int(match.group(1)))
        self.addCleanup(client.close)
        self.assertIsInstance(client.read(), Hello)
        self.assertIsInstance(client.read(), State)
        proc.send_signal(signal.SIGINT)
        self.assertEqual(proc.wait(timeout=10), 0)
        self.assertIn("motors stopped", proc.stderr.read())

    def test_the_real_driver_without_its_hardware_exits_with_directions(self) -> None:
        port = "/dev/no-such-wheel-bus"
        out = subprocess.run(
            [sys.executable, str(self.SCRIPT), "--driver", "real", "--wheel-port", port],
            capture_output=True,
            text=True,
            timeout=30,
        )
        self.assertEqual(out.returncode, 2)
        # no pyserial: how to install it; with pyserial: names the port it could not open
        self.assertTrue("nav-pi/setup.sh" in out.stderr or port in out.stderr, out.stderr)


class ImportTest(unittest.TestCase):
    def test_everything_the_pi_imports_works_under_python_dash_s(self) -> None:
        modules = ("server", "client", "fake_driver", "drivers")
        code = (
            f"import sys; sys.path.insert(0, {str(SRC)!r}); "
            + "; ".join(f"import retriever.bridge.{name}" for name in modules)
        )
        out = subprocess.run(
            [sys.executable, "-S", "-c", code], capture_output=True, text=True, timeout=30
        )
        self.assertEqual(out.returncode, 0, out.stderr)


if __name__ == "__main__":
    unittest.main()
