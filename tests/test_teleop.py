"""Teleop: held keys -> ramped Actions, the deadman, and the whole chain over a
real bridge server with the fake tank."""

from __future__ import annotations

import json
import logging
import math
import sys
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.bridge.client import BridgeRobot
from retriever.bridge.fake_driver import FakeTankDriver
from retriever.bridge.lidar import FakeLidar, FakeTankPose, FakeWorld
from retriever.bridge.server import BridgeServer, ServerThread
from retriever.types import Pose
from retriever.teleop import (
    DriveRecorder,
    TeleopConfig,
    TeleopCore,
    TeleopSession,
    parse_gears,
    serve,
)

_QUIET = logging.NullHandler()


def setUpModule():
    # The bridge logs a warning on every stop, by design; these tests stop it a lot.
    logging.getLogger("retriever.bridge").addHandler(_QUIET)


def tearDownModule():
    logging.getLogger("retriever.bridge").removeHandler(_QUIET)


def run(core: TeleopCore, t0: float, t1: float, dt: float = 0.05):
    t, a = t0, None
    while t <= t1 + 1e-9:
        a = core.step(t)
        t += dt
    return a


class TestCore(unittest.TestCase):
    def test_silent_page_means_zero(self):
        core = TeleopCore()
        a = run(core, 0.0, 1.0)
        self.assertEqual((a.base_vx, a.base_wz), (0.0, 0.0))

    def test_forward_ramps_up_then_holds_gear_speed(self):
        core = TeleopCore(TeleopConfig(gears=((0.3, 1.0),), accel_mps2=1.0))
        core.step(0.0)
        core.command(1, 0, 0.0)
        self.assertAlmostEqual(core.step(0.1).base_vx, 0.1)      # 1 m/s^2 for 0.1 s
        for i in range(2, 6):
            core.command(1, 0, i / 10)                            # the page keeps saying so
            a = core.step(i / 10)
        self.assertAlmostEqual(a.base_vx, 0.3)
        self.assertEqual(a.base_wz, 0.0)

    def test_page_going_quiet_stops_it(self):
        c = TeleopConfig(gears=((0.3, 1.0),), hold_s=0.3, decel_mps2=2.0)
        core = TeleopCore(c)
        core.step(0.0)
        core.command(1, 0, 0.0)
        run(core, 0.05, 0.30)                                     # still inside hold_s: moving
        self.assertGreater(core.vx, 0.2)
        a = run(core, 0.35, 0.60)                                 # silent past hold_s: ramped down
        self.assertEqual(a.base_vx, 0.0)
        self.assertFalse(core.moving)

    def test_halt_is_immediate(self):
        core = TeleopCore()
        core.step(0.0)
        core.command(1, 1, 0.0)
        run(core, 0.05, 0.25)
        core.halt()
        a = core.step(0.3)
        self.assertEqual((a.base_vx, a.base_wz), (0.0, 0.0))

    def test_a_turns_left_whichever_way_it_drives(self):
        for fwd in (0, 1, -1):
            core = TeleopCore(TeleopConfig(ang_accel=100.0, accel_mps2=100.0))
            core.step(0.0)
            core.command(fwd, 1, 0.0)
            a = core.step(0.1)
            self.assertGreater(a.base_wz, 0.0, f"fwd={fwd}")
            self.assertEqual(math.copysign(1, a.base_vx) if fwd else 0, fwd)

    def test_arcs_turn_gentler_than_spins(self):
        c = TeleopConfig(gears=((0.3, 1.0),), arc_turn_scale=0.5)
        core = TeleopCore(c)
        core.command(0, 1, 0.0)
        self.assertEqual(core.target(0.0), (0.0, 1.0))
        core.command(1, 1, 0.0)
        self.assertEqual(core.target(0.0), (0.3, 0.5))

    def test_gear_is_clamped_and_bad_input_changes_nothing(self):
        core = TeleopCore()
        core.command(1, 0, 0.0, gear=99)
        self.assertEqual(core.gear, len(core.config.gears) - 1)
        core.command(1, 0, 0.0, gear=-3)
        self.assertEqual(core.gear, 0)
        core.command(5, -5, 0.0)
        self.assertEqual((core.fwd, core.turn), (1.0, -1.0))
        with self.assertRaises(ValueError):
            core.command(float("nan"), 0, 0.1)
        with self.assertRaises(ValueError):
            core.command("fast", 0, 0.1)
        self.assertEqual((core.fwd, core.turn), (1.0, -1.0))

    def test_reversing_direction_uses_the_quick_ramp(self):
        c = TeleopConfig(gears=((0.3, 1.0),), accel_mps2=0.5, decel_mps2=3.0)
        core = TeleopCore(c)
        core.vx = 0.3
        core.step(0.0)
        core.command(-1, 0, 0.0)
        self.assertAlmostEqual(core.step(0.05).base_vx, 0.15)    # braking: 3 m/s^2

    def test_parse_gears(self):
        self.assertEqual(parse_gears("0.1:0.5, 0.2:0.9"), ((0.1, 0.5), (0.2, 0.9)))
        for bad in ("", "fast", "0.1", "9:1", "0.1:-1"):
            with self.assertRaises(ValueError, msg=bad):
                parse_gears(bad)


class BridgeCase(unittest.TestCase):
    def setUp(self):
        self.drv = FakeTankDriver()
        self.server = BridgeServer(self.drv, "127.0.0.1", 0, timeout_ms=300,
                                   motion_timeout_ms=500, state_hz=50.0)
        self.st = ServerThread(self.server)
        self.port = self.st.start()
        self.addCleanup(self.st.stop)

    def session(self, **kw):
        s = TeleopSession(lambda: BridgeRobot("127.0.0.1", self.port), **kw).start()
        self.addCleanup(s.close)
        self.assertTrue(s.wait_connected(3.0), s.link_detail)
        return s

    def hold(self, s, fwd, turn, seconds, gear=1):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            s.drive(fwd, turn, gear)
            time.sleep(0.05)

    def wheels(self):
        return self.st.call(lambda: self.drv.wheel_speeds)


class TestSession(BridgeCase):
    def test_holding_w_drives_forward_and_letting_go_stops(self):
        s = self.session()
        self.hold(s, 1, 0, 0.8)
        snap = s.snapshot()
        self.assertEqual(snap["link"], "up")
        self.assertGreater(snap["cmd"][0], 0.25)
        self.assertGreater(snap["meas"][0], 0.15)
        time.sleep(0.8)                                  # silent past hold_s, then ramped down
        self.assertEqual(self.wheels(), (0.0, 0.0))
        snap = s.snapshot()
        self.assertGreater(snap["pose"][0], 0.1)
        self.assertLess(abs(snap["pose"][1]), 0.01)
        self.assertGreater(snap["odometer_m"], 0.1)
        self.assertTrue(snap["trail"])

    def test_turning_left_turns_counter_clockwise(self):
        s = self.session()
        self.hold(s, 0, 1, 0.8)
        time.sleep(0.4)
        self.assertGreater(s.snapshot()["pose"][2], 0.3)

    def test_estop_latches_on_the_pi_until_cleared(self):
        s = self.session()
        s.estop()
        deadline = time.monotonic() + 2.0
        while not s.snapshot()["estop"] and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertTrue(s.snapshot()["estop"])
        self.hold(s, 1, 0, 0.4)
        self.assertEqual(self.wheels(), (0.0, 0.0))     # the Pi ignores it
        s.clear_estop()
        deadline = time.monotonic() + 2.0
        while s.snapshot()["estop"] and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertFalse(s.snapshot()["estop"])

    def test_closing_stops_the_motors(self):
        s = self.session()
        self.hold(s, 1, 0, 0.4)
        s.close()
        time.sleep(0.2)
        self.assertEqual(self.wheels(), (0.0, 0.0))

    def test_records_states_and_meta(self):
        import tempfile

        with tempfile.TemporaryDirectory() as d:
            rec = DriveRecorder(Path(d) / "drive.jsonl")
            s = self.session(recorder=rec)
            self.hold(s, 1, 0, 0.3)
            s.close()
            lines = [json.loads(x) for x in (Path(d) / "drive.jsonl").read_text().splitlines()]
        kinds = {x["k"] for x in lines}
        self.assertIn("meta", kinds)
        self.assertIn("state", kinds)
        st = [x for x in lines if x["k"] == "state"]
        self.assertTrue(all("lt" in x and "rt" in x for x in st))
        self.assertGreater(max(x["cvx"] for x in st), 0.0)

    def test_reconnects_after_the_link_drops(self):
        tries = {"n": 0}

        def connect():
            tries["n"] += 1
            if tries["n"] == 1:
                raise ConnectionError("no route to host")
            return BridgeRobot("127.0.0.1", self.port)

        s = TeleopSession(connect).start()
        self.addCleanup(s.close)
        self.assertTrue(s.wait_connected(4.0))
        robot = s.robot
        robot.close()                                   # the link dies under it
        deadline = time.monotonic() + 4.0
        while (s.robot is robot or s.link != "up") and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertEqual(s.link, "up")
        self.assertIsNot(s.robot, robot)


def wait_done(s, timeout=15.0):
    deadline = time.monotonic() + timeout
    while s.auto is not None and time.monotonic() < deadline:
        time.sleep(0.05)
    return s.snapshot()


class TestClickToGo(BridgeCase):
    def test_drives_to_a_clicked_spot_and_stops_there(self):
        s = self.session()
        s.core.gear = 2
        s.goto(0.6, 0.0)
        snap = wait_done(s)
        self.assertEqual(snap["auto"]["status"], "arrived", snap["auto"])
        x, y, _ = snap["pose"]
        self.assertAlmostEqual(x, 0.6, delta=0.1)
        self.assertAlmostEqual(y, 0.0, delta=0.05)
        time.sleep(0.5)                                   # ramped down, not coasting on
        self.assertEqual(self.wheels(), (0.0, 0.0))
        self.assertIsNone(s.snapshot()["auto"]["goal"])

    def test_turns_round_for_a_spot_behind_and_to_the_left(self):
        s = self.session()
        s.core.gear = 2
        s.goto(-0.3, 0.4)
        snap = wait_done(s)
        self.assertEqual(snap["auto"]["status"], "arrived", snap["auto"])
        self.assertLess(math.hypot(snap["pose"][0] + 0.3, snap["pose"][1] - 0.4), 0.12)

    def test_a_drive_key_takes_over_but_letting_go_does_not(self):
        s = self.session()
        s.goto(2.0, 0.0)
        time.sleep(0.3)
        s.drive(0, 0, 1)                                  # a release / gear change: keeps going
        self.assertIsNotNone(s.auto)
        s.drive(0, 1)                                     # a key: the human has it
        self.assertIsNone(s.auto)
        self.assertEqual(s.snapshot()["auto"]["detail"], "you took over")

    def test_space_and_cancel_stop_it(self):
        s = self.session()
        s.goto(2.0, 0.0)
        s.halt()
        self.assertIsNone(s.auto)
        s.goto(2.0, 0.0)
        s.cancel()
        self.assertIsNone(s.auto)

    def test_refuses_while_estopped_and_mis_clicks(self):
        s = self.session()
        with self.assertRaises(RuntimeError):
            s.goto(100.0, 0.0)
        s.estop()
        deadline = time.monotonic() + 2.0
        while not s.snapshot()["estop"] and time.monotonic() < deadline:
            time.sleep(0.02)
        with self.assertRaisesRegex(RuntimeError, "e-stop"):
            s.goto(1.0, 0.0)
        with self.assertRaises(ValueError):
            s.goto(float("nan"), 0.0)

    def test_stops_when_nobody_is_watching(self):
        s = self.session()
        s.viewers, s._viewers_gone = 0, time.monotonic()    # what serve() does
        s.goto(3.0, 0.0)
        snap = wait_done(s, 3.0)
        self.assertEqual(snap["auto"]["detail"], "nobody is watching the page")

    def test_goes_on_while_a_page_watches(self):
        s = self.session()
        s.viewers = 0
        s.viewer(True)
        s.goto(3.0, 0.0)
        time.sleep(1.5)
        self.assertIsNotNone(s.auto)
        s.viewer(False)
        snap = wait_done(s, 3.0)
        self.assertEqual(snap["auto"]["detail"], "nobody is watching the page")


class TestHome(BridgeCase):
    """No lidar, as on the first autonomous test: straight lines, odometry only."""

    def test_go_home_drives_back_and_faces_the_way_it_started(self):
        s = self.session()
        s.core.gear = 2
        s.goto(0.6, 0.4)
        self.assertEqual(wait_done(s)["auto"]["status"], "arrived")
        s.go_home()
        snap = wait_done(s)
        self.assertEqual(snap["auto"]["status"], "home", snap["auto"])
        x, y, th = snap["pose"]
        self.assertLess(math.hypot(x, y), 0.1)
        self.assertLess(abs(math.degrees(math.remainder(th, math.tau))), 5.0)

    def test_round_trip_goes_there_waits_and_comes_back(self):
        s = self.session()
        s.core.gear = 2
        s.goto(0.7, -0.3, round_trip=True, wait_s=0.4)
        far, waited = 0.0, False
        deadline = time.monotonic() + 30.0
        while s.auto is not None and time.monotonic() < deadline:
            snap = s.snapshot()
            far = max(far, snap["pose"][0])
            waited = waited or snap["auto"]["phase"] == "wait"
            time.sleep(0.05)
        snap = s.snapshot()
        self.assertTrue(waited)
        self.assertGreater(far, 0.6)                     # it really went out there
        self.assertEqual(snap["auto"]["status"], "home", snap["auto"])
        self.assertLess(math.hypot(snap["pose"][0], snap["pose"][1]), 0.1)
        self.assertLess(abs(math.degrees(math.remainder(snap["pose"][2], math.tau))), 5.0)

    def test_set_home_moves_home(self):
        s = self.session()
        s.core.gear = 2
        s.goto(0.5, 0.0)
        wait_done(s)
        s.set_home()
        home = s.snapshot()["home"]
        s.goto(0.5, 0.5)
        wait_done(s)
        s.go_home()
        snap = wait_done(s)
        self.assertEqual(snap["auto"]["status"], "home")
        self.assertLess(math.hypot(snap["pose"][0] - home[0], snap["pose"][1] - home[1]), 0.1)
        self.assertLess(abs(math.remainder(snap["pose"][2] - home[2], math.tau)), math.radians(5))

    def test_a_key_cancels_a_round_trip(self):
        s = self.session()
        s.goto(1.0, 0.0, round_trip=True)
        s.drive(1, 0)
        self.assertIsNone(s.auto)


class TestTurnCalibration(unittest.TestCase):
    """Spin N full turns by hand against a tape mark; the wheels' count vs the
    truth is the scrub factor. The arithmetic, on a stand-in robot."""

    def session(self, scrub=1.0):
        from types import SimpleNamespace

        s = TeleopSession(lambda: None)                  # never started: no threads
        s.robot = SimpleNamespace(odometry=SimpleNamespace(geo=SimpleNamespace(scrub_factor=scrub)),
                                  estopped=False)
        s.link = "up"
        return s

    def test_wheels_overcounting_a_turn_raises_the_factor(self):
        s = self.session(1.0)
        s.turned = 5.0                                   # wherever it was
        s.calib_start()
        s.turned = 5.0 + math.radians(810)               # the wheels say 810 for a true 720
        r = s.calib_done(2)
        self.assertAlmostEqual(r["scrub_factor"], 810 / 720, places=3)
        self.assertAlmostEqual(r["error_pct"], 12.5, places=1)
        self.assertIsNone(s.calib)

    def test_either_direction_and_an_existing_factor(self):
        s = self.session(1.2)
        s.calib_start()
        s.turned = -math.radians(700)                    # spun clockwise; wheels under-count
        r = s.calib_done(2)
        self.assertAlmostEqual(r["scrub_factor"], 1.2 * 700 / 720, places=3)

    def test_refuses_nonsense(self):
        s = self.session()
        with self.assertRaisesRegex(RuntimeError, "Start"):
            s.calib_done(2)
        s.calib_start()
        s.turned = math.radians(30)
        with self.assertRaisesRegex(RuntimeError, "hardly turned"):
            s.calib_done(2)
        with self.assertRaises(ValueError):
            s.calib_done(0)

    def test_the_session_counts_turns_past_a_wrap(self):
        s = self.session()
        s._seen.states = 2
        for k in range(1, 30):                           # 29 steps of 25 deg, wrapped to +-180
            th = math.remainder(math.radians(25 * k), math.tau)
            s._note(s.robot, SimpleObs(Pose(0.0, 0.0, th), t=k * 0.1))
        self.assertAlmostEqual(math.degrees(s.turned), 25 * 29, delta=1.0)


class SimpleObs:
    def __init__(self, base, t):
        self.base, self.t = base, t


class TestScrubOverride(BridgeCase):
    def test_the_flag_changes_odometry_only(self):
        bot = BridgeRobot("127.0.0.1", self.port, scrub_factor=1.3)
        self.addCleanup(bot.close)
        self.assertEqual(bot.odometry.geo.scrub_factor, 1.3)
        self.assertEqual(bot.hello.scrub_factor, 1.0)     # the Pi still says its own
        with self.assertRaises(ValueError):
            BridgeRobot("127.0.0.1", self.port, scrub_factor=-1)


class TestClickToGoRoundAPost(unittest.TestCase):
    """With a lidar on the Pi, the path bends round what is in the way."""

    POST = (0.8, 0.0, 0.08)

    def setUp(self):
        self.drv = FakeTankDriver()
        world = FakeWorld.room(posts=[self.POST])
        lidar = FakeLidar(world, pose_fn=FakeTankPose(self.drv), noise_m=0.005)
        self.server = BridgeServer(self.drv, "127.0.0.1", 0, timeout_ms=300,
                                   motion_timeout_ms=500, state_hz=50.0, lidar=lidar)
        self.st = ServerThread(self.server)
        self.port = self.st.start()
        self.addCleanup(self.st.stop)

    def test_steers_round_a_post_to_the_spot_behind_it(self):
        s = TeleopSession(lambda: BridgeRobot("127.0.0.1", self.port)).start()
        self.addCleanup(s.close)
        self.assertTrue(s.wait_connected(3.0))
        deadline = time.monotonic() + 3.0
        while s.robot.scan is None and time.monotonic() < deadline:
            time.sleep(0.05)
        s.core.gear = 2
        s.goto(1.7, 0.0)
        self.assertTrue(s.auto.avoiding)
        closest, deadline = 9.0, time.monotonic() + 30.0
        while s.auto is not None and time.monotonic() < deadline:
            x, y, _ = s.snapshot()["pose"]
            closest = min(closest, math.hypot(x - self.POST[0], y - self.POST[1]))
            time.sleep(0.05)
        snap = s.snapshot()
        self.assertEqual(snap["auto"]["status"], "arrived", snap["auto"])
        # the base centre stayed a half-width plus the post's radius away: no contact
        self.assertGreater(closest, 0.20 + self.POST[2])


class TestHttp(BridgeCase):
    def setUp(self):
        super().setUp()
        self.s = self.session()
        self.httpd = serve(self.s, "127.0.0.1", 0)
        self.addCleanup(self.httpd.shutdown)
        self.base = f"http://127.0.0.1:{self.httpd.server_port}"

    def post(self, path, body):
        req = urllib.request.Request(self.base + path, json.dumps(body).encode(),
                                     {"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=2) as r:
                return r.status
        except urllib.error.HTTPError as e:
            return e.code

    def test_page_and_state(self):
        with urllib.request.urlopen(self.base + "/", timeout=2) as r:
            page = r.read().decode()
        self.assertIn("KeyW", page)
        self.assertIn("/drive", page)
        with urllib.request.urlopen(self.base + "/state", timeout=2) as r:
            self.assertEqual(json.load(r)["link"], "up")

    def test_drive_and_bad_input(self):
        self.assertEqual(self.post("/drive", {"fwd": 1, "turn": 0, "gear": 0}), 204)
        self.assertEqual(self.post("/drive", {"fwd": "fast"}), 400)
        self.assertEqual(self.post("/nope", {}), 404)
        self.assertEqual(self.post("/halt", {}), 204)

    def test_goto_and_cancel(self):
        self.assertEqual(self.post("/goto", {"x": 0.5, "y": 0.0}), 204)
        self.assertIsNotNone(self.s.auto)
        self.assertEqual(self.post("/cancel", {}), 204)
        self.assertIsNone(self.s.auto)
        self.assertEqual(self.post("/goto", {"x": 0.5}), 400)
        self.assertEqual(self.post("/goto", {"x": 500, "y": 0}), 409)
        self.assertEqual(self.post("/goto", {"x": 0.5, "y": 0, "round_trip": True}), 204)
        self.assertEqual(len(self.s.auto.legs), 2)
        self.assertEqual(self.post("/sethome", {}), 204)
        self.assertEqual(self.post("/home", {}), 204)
        self.assertEqual(self.s.auto.legs[0].label, "home")


if __name__ == "__main__":
    unittest.main()
