"""The arm's stop path: hardware/so101/scripts/fetch.py under SIGTERM/SIGINT.

No hardware and no lerobot here. `fetch.py` imports torch and a *forked*
lerobot that only exists on the arm board (ARM_BOARD.md §2), so this module
puts stub modules in sys.modules, loads fetch.py by path against them, and
then replaces its robot/policy seams with fakes. Nothing opens a serial port.

What is actually being tested is the stop path, not the policy:

* the handler records the signal and touches nothing else
* the loop notices between steps -- the step in flight still finishes
* the clean shutdown runs either way, so robot.disconnect() is not skipped
  and /dev/soarm_follower is not orphaned
* the exit code says which of the three outcomes happened

    .venv/bin/python -m unittest tests.test_arm_fetch_stop -v
"""

from __future__ import annotations

import contextlib
import importlib.util
import io
import json
import os
import signal
import sys
import tempfile
import threading
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FETCH = ROOT / "hardware" / "so101" / "scripts" / "fetch.py"
ARM_BOARD = ROOT / "hardware" / "so101" / "ARM_BOARD.md"

# Every name fetch.py imports at module scope, and the one attribute it uses
# from each. Stubbing rather than installing lerobot: the board's copy is a
# Hiwonder fork with uncommitted patches and cannot be pip-installed anyway.
STUBS = {
    "torch": {"device": lambda spec: spec},
    "lerobot.cameras.opencv.configuration_opencv": {"OpenCVCameraConfig": None},
    "lerobot.datasets.feature_utils": {"build_dataset_frame": None,
                                       "combine_feature_dicts": None},
    "lerobot.datasets.pipeline_features": {"aggregate_pipeline_dataset_features": None,
                                           "create_initial_features": None},
    "lerobot.policies.act.modeling_act": {"ACTPolicy": None},
    "lerobot.policies.factory": {"make_pre_post_processors": None},
    "lerobot.policies.utils": {"make_robot_action": None},
    "lerobot.processor": {"make_default_processors": None},
    "lerobot.robots.so_follower": {"SO101Follower": None, "SO101FollowerConfig": None},
    "lerobot.utils.constants": {"OBS_STR": "observation"},
    "lerobot.utils.control_utils": {"predict_action": None},
}


def _load_fetch():
    """Import fetch.py by path with the hardware stack stubbed out."""
    saved = {}
    sentinel = object()
    for name, attrs in STUBS.items():
        parts = name.split(".")
        for i in range(1, len(parts) + 1):
            dotted = ".".join(parts[:i])
            if dotted not in saved:
                saved[dotted] = sys.modules.get(dotted, sentinel)
            # Always a fresh stub, never the real module if one is installed:
            # setattr on a real torch would leak out of this test.
            sys.modules[dotted] = types.ModuleType(dotted)
        mod = sys.modules[name]
        for attr, value in attrs.items():
            setattr(mod, attr, value if value is not None else object())
    try:
        spec = importlib.util.spec_from_file_location("arm_fetch_under_test", FETCH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        for dotted, previous in saved.items():
            if previous is sentinel:
                sys.modules.pop(dotted, None)
            else:
                sys.modules[dotted] = previous


fetch = _load_fetch()


class FakeBus:
    """Just the three calls fetch.py makes on robot.bus."""

    def __init__(self, on_sync_write=None):
        self.writes = []
        self.on_sync_write = on_sync_write

    def read(self, register, motor, normalize=True):
        if register == "Present_Position":
            return 0.0
        return 100  # a calm gripper current: never crosses a threshold

    def sync_write(self, register, goal):
        self.writes.append(dict(goal))
        if self.on_sync_write:
            self.on_sync_write(len(self.writes))


class FakeRobot:
    """A robot that answers, counts, and records that it was disconnected."""

    robot_type = "so101_follower"
    observation_features = {}
    action_features = {}

    def __init__(self, fail_reads_after=None, on_sync_write=None):
        self.bus = FakeBus(on_sync_write)
        self.connects = 0
        self.disconnects = 0
        self.actions = []
        self.observations = 0
        self.fail_reads_after = fail_reads_after

    def connect(self):
        self.connects += 1

    def get_observation(self):
        self.observations += 1
        if (self.fail_reads_after is not None
                and self.observations > self.fail_reads_after):
            raise ConnectionError("Missing motor IDs: 6")
        return {"gripper.pos": 3.5}

    def send_action(self, action):
        self.actions.append(action)

    def disconnect(self):
        self.disconnects += 1


class StopPathTest(unittest.TestCase):
    """Base: a clean stop flag per test, and the process's handlers restored."""

    def setUp(self):
        fetch._stop_signal = None
        self.addCleanup(setattr, fetch, "_stop_signal", None)
        if threading.current_thread() is threading.main_thread():
            for sig in (signal.SIGTERM, signal.SIGINT):
                previous = signal.getsignal(sig)
                if previous is not None:  # None = set outside Python
                    self.addCleanup(signal.signal, sig, previous)

    def run_main(self, argv, robot, predict=None):
        """Run fetch.main() against fakes; return (exit code, parsed events)."""
        policy = types.SimpleNamespace(reset=lambda: None)
        identity = types.SimpleNamespace(reset=lambda: None)
        calls = {"predict": 0}

        def fake_predict(**kwargs):
            calls["predict"] += 1
            if predict:
                predict(calls["predict"])
            return [0.0]

        patches = {
            "build_robot": lambda: robot,
            "load_policy": lambda ckpt, device: (policy, identity, identity),
            "make_default_processors": lambda: (None, lambda pair: pair[0],
                                                lambda obs: obs),
            "combine_feature_dicts": lambda *a, **k: {},
            "aggregate_pipeline_dataset_features": lambda **k: {},
            "create_initial_features": lambda **k: {},
            "build_dataset_frame": lambda *a, **k: {},
            "predict_action": fake_predict,
            "make_robot_action": lambda values, features: {"gripper.pos": values[0]},
        }
        originals = {name: getattr(fetch, name) for name in patches}
        saved_argv = sys.argv
        sys.argv = ["fetch.py", *argv]
        out = io.StringIO()
        try:
            for name, value in patches.items():
                setattr(fetch, name, value)
            with contextlib.redirect_stdout(out):
                code = fetch.main()
        finally:
            sys.argv = saved_argv
            for name, value in originals.items():
                setattr(fetch, name, value)
        events = [json.loads(line) for line in out.getvalue().splitlines() if line]
        return code, events

    @staticmethod
    def of(events, name):
        return [e for e in events if e["event"] == name]

    def done(self, events):
        done = self.of(events, "done")
        self.assertEqual(len(done), 1, events)
        return done[0]


class TestHandler(StopPathTest):
    def test_handler_only_sets_a_flag(self):
        self.assertFalse(fetch.stop_requested())
        self.assertIsNone(fetch.stop_reason())
        fetch._on_stop_signal(signal.SIGTERM, None)
        self.assertTrue(fetch.stop_requested())
        self.assertEqual(fetch.stop_reason(), "sigterm")

    def test_a_second_signal_does_not_change_the_reason(self):
        fetch._on_stop_signal(signal.SIGINT, None)
        fetch._on_stop_signal(signal.SIGTERM, None)
        self.assertEqual(fetch.stop_reason(), "sigint")

    @unittest.skipUnless(threading.current_thread() is threading.main_thread(),
                         "signal handlers can only be installed on the main thread")
    def test_install_routes_a_real_sigterm_into_the_flag(self):
        fetch.install_stop_handlers()
        # Assert the handler is in place BEFORE raising anything: an
        # uninstalled SIGTERM would kill the test runner outright.
        self.assertIs(signal.getsignal(signal.SIGTERM), fetch._on_stop_signal)
        self.assertIs(signal.getsignal(signal.SIGINT), fetch._on_stop_signal)
        os.kill(os.getpid(), signal.SIGTERM)
        self.assertTrue(fetch.stop_requested())
        self.assertEqual(fetch.stop_reason(), "sigterm")

    def test_main_installs_the_handlers_before_the_model_load(self):
        installed = []
        original = fetch.install_stop_handlers
        try:
            fetch.install_stop_handlers = lambda: installed.append("installed")
            fetch._stop_signal = signal.SIGTERM  # stop at once, take no steps
            code, events = self.run_main(["--timeout", "5"], FakeRobot())
        finally:
            fetch.install_stop_handlers = original
        self.assertEqual(installed, ["installed"])
        # ...and ahead of the ~7 s load: the "loading" line comes after it.
        self.assertEqual(events[0]["event"], "loading")
        self.assertEqual(code, fetch.EXIT_STOPPED)


class TestLoopStopsBetweenSteps(StopPathTest):
    def test_the_step_in_flight_finishes_then_the_loop_stops(self):
        robot = FakeRobot()

        def signal_during_inference(call):
            # Mid-step: the observation is already read, the action is not yet
            # sent. The loop must still send it and stop at the next boundary.
            if call == 3:
                fetch._on_stop_signal(signal.SIGTERM, None)

        code, events = self.run_main(["--timeout", "30", "--fps", "1000"],
                                     robot, predict=signal_during_inference)

        self.assertEqual(len(robot.actions), 3, "the interrupted step was dropped")
        steps = self.of(events, "step")
        self.assertEqual([s["n"] for s in steps], [1, 2, 3])
        stopping = self.of(events, "stopping")
        self.assertEqual(len(stopping), 1)
        self.assertEqual(stopping[0]["reason"], "sigterm")
        self.assertEqual(stopping[0]["n"], 3)
        self.assertEqual(code, fetch.EXIT_STOPPED)

    def test_stop_before_the_first_step_sends_no_action_at_all(self):
        # A signal that arrived during the ~7 s model load.
        fetch._stop_signal = signal.SIGTERM
        robot = FakeRobot()
        code, events = self.run_main(["--timeout", "30"], robot)
        self.assertEqual(robot.actions, [])
        self.assertEqual(self.of(events, "step"), [])
        self.assertEqual(self.of(events, "stopping")[0]["n"], 0)
        self.assertEqual(code, fetch.EXIT_STOPPED)
        self.assertEqual(robot.disconnects, 1)


class TestCleanShutdown(StopPathTest):
    def test_a_stop_still_disconnects_the_robot(self):
        robot = FakeRobot()
        code, events = self.run_main(
            ["--timeout", "30", "--fps", "1000"], robot,
            predict=lambda call: call == 2 and fetch._on_stop_signal(signal.SIGTERM, None))
        self.assertEqual(robot.disconnects, 1, "the serial port was orphaned")
        done = self.done(events)
        self.assertEqual(done["status"], "stopped")
        self.assertEqual(done["reason"], "sigterm")
        self.assertFalse(done["grasped"])
        self.assertEqual(code, fetch.EXIT_STOPPED)

    def test_the_done_line_is_last_and_reports_what_was_achieved(self):
        robot = FakeRobot()
        _, events = self.run_main(
            ["--timeout", "30", "--fps", "1000"], robot,
            predict=lambda call: call == 2 and fetch._on_stop_signal(signal.SIGTERM, None))
        self.assertEqual(events[-1]["event"], "done")
        self.assertEqual(events[-1]["steps"], 2)
        self.assertIn("grasped", events[-1])
        self.assertIn("handoff", events[-1])

    def test_the_gripper_keeps_holding_on_disconnect(self):
        """build_robot() must not grow a torque release; the object would drop."""
        seen = {}

        def config(**kwargs):
            seen.update(kwargs)
            return "cfg"

        originals = (fetch.SO101FollowerConfig, fetch.SO101Follower,
                     fetch.OpenCVCameraConfig)
        try:
            fetch.SO101FollowerConfig = config
            fetch.SO101Follower = lambda cfg: cfg
            fetch.OpenCVCameraConfig = lambda **kw: kw
            fetch.build_robot()
        finally:
            (fetch.SO101FollowerConfig, fetch.SO101Follower,
             fetch.OpenCVCameraConfig) = originals
        self.assertIs(seen["disable_torque_on_disconnect"], False)
        self.assertEqual({c["fourcc"] for c in seen["cameras"].values()}, {"MJPG"})


class TestHandoffStops(StopPathTest):
    def setUp(self):
        super().setUp()
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.poses = Path(tmp.name) / "arm_poses.json"
        self.poses.write_text(json.dumps({"handoff": {"shoulder_pan": 10.0,
                                                      "elbow_flex": -5.0}}))
        original = fetch.POSES
        fetch.POSES = self.poses
        self.addCleanup(setattr, fetch, "POSES", original)

    def test_a_stop_aborts_the_sweep_between_interpolation_steps(self):
        robot = FakeRobot(fail_reads_after=2,
                          on_sync_write=lambda n: (n == 4 and fetch._on_stop_signal(
                              signal.SIGTERM, None)))
        code, events = self.run_main(
            ["--timeout", "30", "--fps", "1000", "--settle-steps", "1",
             "--handoff-duration", "2"], robot)

        self.assertEqual(len(self.of(events, "grasped")), 1)
        stopped = self.of(events, "handoff_stopped")
        self.assertEqual(len(stopped), 1)
        self.assertEqual(stopped[0]["reason"], "sigterm")
        self.assertEqual(len(robot.bus.writes), 4, "the sweep ran to completion")
        done = self.done(events)
        self.assertEqual(done["status"], "stopped")
        self.assertTrue(done["grasped"], "the object is still held")
        self.assertFalse(done["handoff"])
        self.assertEqual(code, fetch.EXIT_STOPPED)
        self.assertEqual(robot.disconnects, 1)

    def test_an_untouched_handoff_still_runs_to_the_end(self):
        robot = FakeRobot(fail_reads_after=2)
        code, events = self.run_main(
            ["--timeout", "30", "--fps", "1000", "--settle-steps", "1",
             "--handoff-duration", "0.1"], robot)
        self.assertEqual(self.of(events, "handoff_stopped"), [])
        self.assertTrue(self.done(events)["handoff"])
        self.assertEqual(code, fetch.EXIT_GRASPED)


class TestExitCodesStayMeaningful(StopPathTest):
    def test_the_three_codes_are_distinct(self):
        codes = (fetch.EXIT_GRASPED, fetch.EXIT_ABORTED, fetch.EXIT_STOPPED)
        self.assertEqual(codes, (0, 1, 3))
        self.assertNotIn(2, codes, "2 is argparse's usage error")

    def test_a_grasp_still_exits_zero(self):
        robot = FakeRobot(fail_reads_after=2)
        code, events = self.run_main(
            ["--timeout", "30", "--fps", "1000", "--settle-steps", "1",
             "--no-handoff"], robot)
        self.assertEqual(code, fetch.EXIT_GRASPED)
        done = self.done(events)
        self.assertEqual(done["status"], "succeeded")
        self.assertEqual(done["reason"], "gripper_unresponsive")
        self.assertEqual(robot.disconnects, 1)

    def test_a_timeout_still_exits_one(self):
        robot = FakeRobot()
        code, events = self.run_main(["--timeout", "0.05", "--fps", "200"], robot)
        self.assertEqual(code, fetch.EXIT_ABORTED)
        done = self.done(events)
        self.assertEqual(done["status"], "aborted")
        self.assertEqual(done["reason"], "timeout")
        self.assertEqual(robot.disconnects, 1)

    def test_argparse_usage_errors_do_not_collide(self):
        saved = sys.argv
        sys.argv = ["fetch.py", "--not-a-flag"]
        try:
            with contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as raised:
                    fetch.main()
        finally:
            sys.argv = saved
        self.assertEqual(raised.exception.code, 2)
        self.assertNotEqual(raised.exception.code, fetch.EXIT_STOPPED)


class TestTheDocMatchesTheCode(StopPathTest):
    def test_arm_board_documents_the_signal_behaviour_and_exit_3(self):
        doc = ARM_BOARD.read_text()
        self.assertIn("Exit `3`", doc)
        self.assertIn("SIGTERM", doc)
        # The warnings that are still true must survive any rewrite of §5.
        for kept in ("--hold-gripper", "power-cycle", "MJPG", "run_goose.sh"):
            self.assertIn(kept, doc)


if __name__ == "__main__":
    unittest.main()
