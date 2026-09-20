"""The Pi's drivers: the fake that keeps driving until told to stop, and driver composition."""

from __future__ import annotations

import logging
import math
import unittest
from unittest import mock

from retriever.backends.fake import MAX_JOINT_RATE
from retriever.backends.tank import COUNTS_PER_REV
from retriever.bridge.drivers import CompositeDriver, DDSM115Driver, build_real_driver
from retriever.bridge.fake_driver import ARM_JOINTS, FakeTankDriver
from retriever.bridge.server import HardwareDriver
from retriever.navigation.geometry import TAU
from retriever.navigation.kinematics import TankGeometry

GEO = TankGeometry()
TICKS_PER_M = COUNTS_PER_REV / (TAU * GEO.wheel_radius_m)


class ManualClock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


def fake_driver(**kw: object) -> tuple[FakeTankDriver, ManualClock]:
    clock = ManualClock()
    return FakeTankDriver(GEO, clock=clock, **kw), clock  # type: ignore[arg-type]


class FakeTankDriverTest(unittest.TestCase):
    def test_is_a_hardware_driver(self) -> None:
        self.assertIsInstance(FakeTankDriver(), HardwareDriver)
        self.assertNotIn("gripper", ARM_JOINTS)

    def test_wheels_keep_turning_until_something_stops_them(self) -> None:
        driver, clock = fake_driver()
        driver.set_wheels(0.05, -0.05)
        clock.now += 2.0  # nobody calls anything: a stalled link
        state = driver.read_state()
        left, right = state["left_ticks"], state["right_ticks"]
        expected = math.floor(0.1 * TICKS_PER_M)
        self.assertEqual(left, expected)
        self.assertEqual(right, COUNTS_PER_REV - expected - 1)  # wrapped, backwards
        driver.stop()
        clock.now += 2.0
        state = driver.read_state()
        self.assertEqual((state["left_ticks"], state["right_ticks"]), (left, right))
        self.assertEqual(driver.wheel_speeds, (0.0, 0.0))

    def test_encoders_wrap(self) -> None:
        driver, clock = fake_driver()
        driver.set_wheels(1.0, 1.0)
        clock.now += 10.0
        state = driver.read_state()
        self.assertEqual(state["left_ticks"], math.floor(10.0 * TICKS_PER_M) % COUNTS_PER_REV)
        self.assertLess(state["left_ticks"], COUNTS_PER_REV)

    def test_joints_slew_at_the_rate_limit(self) -> None:
        driver, clock = fake_driver()
        driver.set_joints({"elbow_flex": 1.0})
        clock.now += 0.2
        elbow = driver.read_state()["joints"]["elbow_flex"]
        self.assertAlmostEqual(elbow, 0.2 * MAX_JOINT_RATE)
        clock.now += 1.0
        self.assertEqual(driver.read_state()["joints"]["elbow_flex"], 1.0)

    def test_stop_freezes_the_arm_where_it_is_and_leaves_the_vacuum(self) -> None:
        driver, clock = fake_driver()
        driver.set_vacuum(True)
        driver.set_joints({"shoulder_lift": -1.0})
        driver.set_gripper(0.0)
        clock.now += 0.1
        driver.stop()
        frozen = driver.read_state()["joints"]
        clock.now += 1.0
        self.assertEqual(driver.read_state()["joints"], frozen)
        self.assertAlmostEqual(frozen["shoulder_lift"], -0.1 * MAX_JOINT_RATE)
        self.assertTrue(driver.vacuum)
        self.assertEqual(driver.stop_count, 1)

    def test_unknown_joints_raise_and_move_nothing(self) -> None:
        driver, clock = fake_driver()
        with self.assertRaisesRegex(ValueError, "tail"):
            driver.set_joints({"elbow_flex": 1.0, "tail": 0.5})
        with self.assertRaises(ValueError):
            driver.set_joints({"gripper": 0.0})  # the gripper has its own command
        clock.now += 1.0
        self.assertEqual(driver.read_state()["joints"]["elbow_flex"], 0.0)

    def test_closing_on_an_object_reports_load_and_closing_on_nothing_does_not(self) -> None:
        for present, load in ((True, 0.6), (False, 0.0)):
            with self.subTest(object_in_gripper=present):
                driver, clock = fake_driver()
                driver.object_in_gripper = present
                self.assertEqual(driver.read_state()["gripper_load"], 0.0)  # still open
                driver.set_gripper(0.0)
                clock.now += 1.0
                state = driver.read_state()
                self.assertEqual(state["joints"]["gripper"], 0.0)
                self.assertEqual(state["gripper_load"], load)

    def test_battery_drains_linearly_and_never_below_zero(self) -> None:
        driver, clock = fake_driver(battery_life_s=100.0)
        self.assertEqual(driver.read_state()["battery"], 1.0)
        clock.now += 25.0
        self.assertAlmostEqual(driver.read_state()["battery"], 0.75)
        clock.now += 1000.0
        self.assertEqual(driver.read_state()["battery"], 0.0)


class Log(list):
    """The order in which the parts were called, shared between them."""


class FakeBase:
    counts_per_rev = 1024

    def __init__(self, log: Log, battery: float | None = 0.5, fail_stop: bool = False) -> None:
        self.log, self._battery, self.fail_stop = log, battery, fail_stop

    def set_wheels(self, left_mps: float, right_mps: float) -> None:
        self.log.append(("wheels", left_mps, right_mps))

    def read_ticks(self) -> tuple[int, int]:
        return 10, 20

    def battery(self) -> float | None:
        return self._battery

    def stop(self) -> None:
        self.log.append(("stop", "base"))
        if self.fail_stop:
            raise OSError("wheel bus is gone")


class FakeArm:
    def __init__(self, log: Log, name: str, joints: tuple[str, ...], gripper: bool) -> None:
        self.log, self.name, self.joint_names, self.has_gripper = log, name, joints, gripper

    def set_joints(self, targets: dict[str, float]) -> None:
        self.log.append(("joints", self.name, dict(targets)))

    def set_gripper(self, position: float) -> None:
        self.log.append(("gripper", self.name, position))

    def read_joints(self) -> dict[str, float]:
        angles = {joint: 0.5 for joint in self.joint_names}
        if self.has_gripper:
            angles["gripper"] = 0.9
        return angles

    def gripper_load(self) -> float:
        return 0.3

    def hold(self) -> None:
        self.log.append(("hold", self.name))
        if self.name == "broken":
            raise OSError("arm bus is gone")


class FakeVacuum:
    def __init__(self, log: Log) -> None:
        self.log = log

    def set_vacuum(self, on: bool) -> None:
        self.log.append(("vacuum", on))


def two_arms(log: Log) -> dict[str, FakeArm]:
    return {
        "left": FakeArm(log, "left", ("l_pan", "l_lift"), gripper=False),
        "right": FakeArm(log, "right", ("r_pan", "r_lift"), gripper=True),
    }


class CompositeDriverTest(unittest.TestCase):
    def test_is_a_hardware_driver_with_the_bases_counts(self) -> None:
        driver = CompositeDriver(FakeBase(Log()))
        self.assertIsInstance(driver, HardwareDriver)
        self.assertEqual(driver.counts_per_rev, 1024)

    def test_routes_each_joint_to_the_arm_that_owns_it(self) -> None:
        log = Log()
        driver = CompositeDriver(FakeBase(log), two_arms(log))
        driver.set_joints({"l_pan": 1.0, "r_lift": 2.0, "l_lift": 3.0})
        self.assertCountEqual(
            log,
            [
                ("joints", "left", {"l_pan": 1.0, "l_lift": 3.0}),
                ("joints", "right", {"r_lift": 2.0}),
            ],
        )

    def test_an_unknown_joint_raises_before_any_arm_moves(self) -> None:
        log = Log()
        driver = CompositeDriver(FakeBase(log), two_arms(log))
        with self.assertRaisesRegex(ValueError, "tail"):
            driver.set_joints({"l_pan": 1.0, "tail": 2.0})
        self.assertEqual(log, [])

    def test_a_joint_owned_by_two_arms_fails_at_construction(self) -> None:
        log = Log()
        arms = {"a": FakeArm(log, "a", ("pan",), False), "b": FakeArm(log, "b", ("pan",), True)}
        with self.assertRaisesRegex(ValueError, "unique"):
            CompositeDriver(FakeBase(log), arms)

    def test_gripper_goes_to_the_first_arm_that_has_one(self) -> None:
        log = Log()
        CompositeDriver(FakeBase(log), two_arms(log)).set_gripper(0.1)
        self.assertEqual(log, [("gripper", "right", 0.1)])
        with self.assertRaisesRegex(ValueError, "gripper"):
            CompositeDriver(FakeBase(log)).set_gripper(0.1)

    def test_vacuum_needs_a_vacuum(self) -> None:
        log = Log()
        CompositeDriver(FakeBase(log), vacuum=FakeVacuum(log)).set_vacuum(True)
        self.assertEqual(log, [("vacuum", True)])
        with self.assertRaisesRegex(ValueError, "vacuum"):
            CompositeDriver(FakeBase(log)).set_vacuum(True)

    def test_read_state_merges_the_parts(self) -> None:
        log = Log()
        state = CompositeDriver(FakeBase(log), two_arms(log)).read_state()
        self.assertEqual(
            state,
            {
                "left_ticks": 10,
                "right_ticks": 20,
                "joints": {
                    "l_pan": 0.5, "l_lift": 0.5, "r_pan": 0.5, "r_lift": 0.5, "gripper": 0.9
                },
                "gripper_load": 0.3,
                "battery": 0.5,
            },
        )

    def test_a_base_that_cannot_tell_its_battery_reads_full(self) -> None:
        state = CompositeDriver(FakeBase(Log(), battery=None)).read_state()
        self.assertEqual(state["battery"], 1.0)
        self.assertEqual((state["gripper_load"], state["joints"]), (0.0, {}))

    def test_stop_stops_the_base_first_then_holds_every_arm(self) -> None:
        log = Log()
        CompositeDriver(FakeBase(log), two_arms(log)).stop()
        self.assertEqual(log, [("stop", "base"), ("hold", "left"), ("hold", "right")])

    def test_every_part_stops_even_if_one_fails_and_the_first_error_is_raised(self) -> None:
        log = Log()
        arms = {"broken": FakeArm(log, "broken", ("a",), False), **two_arms(log)}
        driver = CompositeDriver(FakeBase(log, fail_stop=True), arms)
        with self.assertLogs("retriever.bridge.drivers", logging.ERROR):
            with self.assertRaisesRegex(OSError, "wheel bus"):
                driver.stop()
        self.assertEqual(
            log,
            [("stop", "base"), ("hold", "broken"), ("hold", "left"), ("hold", "right")],
        )


class BuildRealDriverTest(unittest.TestCase):
    def test_builds_the_ddsm115_wheels_as_the_base(self) -> None:
        log = Log()
        with mock.patch("retriever.bridge.drivers.DDSM115Driver",
                        return_value=FakeBase(log)) as wheels:
            driver = build_real_driver("/dev/ttyUSB0")
        self.assertEqual(wheels.call_args.args, ("/dev/ttyUSB0",))
        driver.stop()
        self.assertEqual(log, [("stop", "base")])


class WheelsNeedTheirBusTest(unittest.TestCase):
    """DDSM115Driver is real now (its tests: tests/test_ddsm115.py). With no
    wheel bus it must refuse with directions, never half-start."""

    def test_ddsm115_without_a_bus_refuses_with_directions(self) -> None:
        port = "/dev/no-such-wheel-bus"
        for build in (lambda: DDSM115Driver(port), lambda: build_real_driver(port)):
            with self.assertRaises((ConnectionError, ImportError)) as ctx:
                build()
            # no pyserial: how to install it; with pyserial: which port failed
            self.assertTrue("nav-pi/setup.sh" in str(ctx.exception) or port in str(ctx.exception),
                            str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
