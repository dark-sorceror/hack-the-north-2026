"""The Pi bridge: every stop exactly on a fake clock, then the same promises over sockets."""

from __future__ import annotations

import logging
import math
import unittest
from typing import Any

from retriever.bridge.protocol import (
    Act,
    ClearEstop,
    Estop,
    Heartbeat,
    Hello,
    decode,
    encode,
)
from retriever.bridge.server import ZERO_REFRESH_S, BridgeCore, HardwareDriver
from retriever.navigation.kinematics import TankGeometry, tank_body_to_wheels

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


if __name__ == "__main__":
    unittest.main()
