"""Driving to a pose: the control law on its own, then the loop against the tank fake."""

from __future__ import annotations

import math
import re
import unittest

from retriever.backends.tank import TankFakeRobot
from retriever.navigation.drive import DifferentialGotoController, Limits, run_goto
from retriever.navigation.geometry import angle_diff, distance, pose_error
from retriever.types import Action, Observation, Pose, Target

DT = 0.02
ZERO = Action()


def drive(robot: TankFakeRobot, action: Action, seconds: float) -> None:
    for _ in range(round(seconds / DT)):
        robot.act(action)


class Recorder:
    """A backend wrapper that remembers every action it was asked to send."""

    def __init__(self, inner: TankFakeRobot) -> None:
        self.inner = inner
        self.actions: list[Action] = []

    def observe(self) -> Observation:
        return self.inner.observe()

    def act(self, action: Action) -> None:
        self.actions.append(action)
        self.inner.act(action)

    def close(self) -> None:
        self.inner.close()


class StuckRobot:
    """Wheels on a stand: the pose never changes, and each act advances the clock by `dt`."""

    def __init__(self, base: Pose = Pose(), dt: float = DT, observe_fails_at: int = -1) -> None:
        self.base = base
        self.dt = dt
        self.observe_fails_at = observe_fails_at
        self.observed = 0
        self.actions: list[Action] = []

    def observe(self) -> Observation:
        if self.observed == self.observe_fails_at:
            raise ConnectionError("the link to the Pi dropped")
        self.observed += 1
        return Observation(joints={}, base=self.base, t=len(self.actions) * self.dt)

    def act(self, action: Action) -> None:
        self.actions.append(action)

    def close(self) -> None:
        pass

    @property
    def motion(self) -> list[Action]:
        return self.actions[:-1]


class LimitsTest(unittest.TestCase):
    def test_defaults_are_conservative(self) -> None:
        self.assertEqual(Limits(), Limits(v_max=0.35, w_max=1.2, pos_tol=0.05, ang_tol=0.1))

    def test_refuses_limits_that_could_never_arrive_or_would_flip_a_clip(self) -> None:
        for bad in (
            {"v_max": 0.0}, {"w_max": -1.0}, {"pos_tol": math.nan}, {"ang_tol": math.inf}
        ):
            with self.subTest(bad), self.assertRaises(ValueError):
                Limits(**bad)

    def test_controller_refuses_non_positive_gains(self) -> None:
        with self.assertRaisesRegex(ValueError, "kp_ang"):
            DifferentialGotoController(kp_ang=0.0)


class ControllerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.ctl = DifferentialGotoController()
        self.lim = self.ctl.limits

    def test_straight_ahead_drives_forward_without_turning(self) -> None:
        for base in (Pose(), Pose(1.0, 1.0, math.pi / 2), Pose(-2.0, 0.5, -3.0)):
            with self.subTest(base=base):
                far = Pose(base.x + 2 * math.cos(base.theta), base.y + 2 * math.sin(base.theta))
                action, done = self.ctl.step(base, far)
                self.assertFalse(done)
                self.assertAlmostEqual(action.base_vx, self.lim.v_max)
                self.assertAlmostEqual(action.base_wz, 0.0)

    def test_slows_in_proportion_as_it_closes(self) -> None:
        action, _ = self.ctl.step(Pose(), Pose(0.2, 0.0))
        self.assertAlmostEqual(action.base_vx, 1.2 * 0.2)

    def test_goal_to_the_side_turns_toward_it_without_driving(self) -> None:
        left, _ = self.ctl.step(Pose(), Pose(0.0, 1.0))
        right, _ = self.ctl.step(Pose(), Pose(0.0, -1.0))
        self.assertAlmostEqual(left.base_vx, 0.0)
        self.assertAlmostEqual(right.base_vx, 0.0)
        self.assertEqual((left.base_wz, right.base_wz), (self.lim.w_max, -self.lim.w_max))

    def test_speed_fades_as_the_goal_swings_to_the_side(self) -> None:
        speeds = [
            self.ctl.step(Pose(), Pose(0.2 * math.cos(b), 0.2 * math.sin(b)))[0].base_vx
            for b in (0.0, 0.4, 0.8, 1.2, 1.5)
        ]
        self.assertEqual(speeds, sorted(speeds, reverse=True))
        self.assertGreater(speeds[0], 5 * speeds[-1])

    def test_goal_behind_turns_on_the_spot_and_never_reverses(self) -> None:
        for goal in (Pose(-1.0, 0.01), Pose(-1.0, -0.01), Pose(-0.3, 0.2), Pose(-5.0, -4.0)):
            with self.subTest(goal=goal):
                action, done = self.ctl.step(Pose(), goal)
                self.assertFalse(done)
                self.assertEqual(action.base_vx, 0.0)
                self.assertEqual(action.base_wz, math.copysign(self.lim.w_max, goal.y))

    def test_turns_to_the_goal_heading_once_in_position(self) -> None:
        goal = Pose(1.0, 2.0, 0.5)
        action, done = self.ctl.step(Pose(1.02, 2.0, 1.5), goal)
        self.assertFalse(done)
        self.assertEqual(action.base_vx, 0.0)
        self.assertEqual(action.base_wz, -self.lim.w_max)
        action, done = self.ctl.step(Pose(1.0, 2.0, 0.7), goal)
        self.assertFalse(done)
        self.assertEqual(action.base_vx, 0.0)
        self.assertAlmostEqual(action.base_wz, -2.2 * 0.2)

    def test_arrival_needs_both_position_and_heading(self) -> None:
        goal = Pose(1.0, 0.0, 0.0)
        self.assertFalse(self.ctl.step(Pose(0.9, 0.0, 0.0), goal)[1])
        self.assertFalse(self.ctl.step(Pose(1.0, 0.0, 0.2), goal)[1])
        self.assertEqual(self.ctl.step(Pose(0.97, 0.02, -0.05), goal), (ZERO, True))

    def test_every_command_is_within_limits_and_never_sideways(self) -> None:
        cases = [
            (DifferentialGotoController(), Limits()),
            (DifferentialGotoController(Limits(0.1, 0.3, 0.02, 0.05), 50.0, 50.0),
             Limits(0.1, 0.3, 0.02, 0.05)),
        ]
        headings = [i * math.pi / 4 for i in range(-4, 4)]
        spots = [(x, y) for x in (-3.0, -0.5, 0.0, 0.04, 0.5, 3.0) for y in (-2.0, 0.0, 0.3)]
        for ctl, lim in cases:
            for (x, y) in spots:
                for base_theta in headings:
                    for goal_theta in headings:
                        action, _ = ctl.step(Pose(0.0, 0.0, base_theta), Pose(x, y, goal_theta))
                        self.assertEqual(action.base_vy, 0.0)
                        self.assertGreaterEqual(action.base_vx, 0.0)
                        self.assertLessEqual(action.base_vx, lim.v_max)
                        self.assertLessEqual(abs(action.base_wz), lim.w_max)

    def test_step_depends_only_on_the_poses(self) -> None:
        first = self.ctl.step(Pose(0.1, 0.2, 0.3), Pose(1.0, -1.0, 2.0))
        self.ctl.step(Pose(5.0, 5.0, -2.0), Pose(-4.0, 1.0, 1.0))
        self.assertEqual(self.ctl.step(Pose(0.1, 0.2, 0.3), Pose(1.0, -1.0, 2.0)), first)

    def test_refuses_a_pose_that_is_not_finite(self) -> None:
        with self.assertRaisesRegex(ValueError, "base pose is not finite"):
            self.ctl.step(Pose(math.nan, 0.0, 0.0), Pose(1.0, 0.0, 0.0))


class RunGotoTest(unittest.TestCase):
    def test_arrives_within_tolerance_from_several_start_poses(self) -> None:
        starts = {
            "origin": [],
            "turned left": [(Action(base_wz=1.5), 1.0)],
            "out and facing back": [(Action(base_vx=0.3), 2.0), (Action(base_wz=-1.5), 2.0)],
            "off on an arc": [(Action(base_vx=0.2, base_wz=0.6), 3.0)],
        }
        goals = (Pose(1.2, -0.6, 2.5), Pose(-1.0, 0.8, -1.0), Pose(0.0, 0.0, math.pi))
        lim = Limits()
        for name, program in starts.items():
            for goal in goals:
                with self.subTest(start=name, goal=goal):
                    robot = TankFakeRobot()
                    for action, seconds in program:
                        drive(robot, action, seconds)
                    recorder = Recorder(robot)
                    result = run_goto(recorder, goal)
                    self.assertTrue(result.ok, result.detail)
                    self.assertEqual(result.detail, "Arrived.")
                    for pose in (robot.observe().base, robot.base):  # estimate, then truth
                        self.assertLessEqual(distance(pose, goal), lim.pos_tol + 1e-3)
                        self.assertLess(abs(angle_diff(goal.theta, pose.theta)), lim.ang_tol)
                    self.assertLessEqual(result.data["distance_m"], lim.pos_tol)
                    self.assertEqual(recorder.actions[-1], ZERO)
                    self.assertTrue(all(a.base_vy == 0.0 for a in recorder.actions))

    def test_goal_behind_turns_on_the_spot_before_driving_forward(self) -> None:
        robot = TankFakeRobot()
        goal = Pose(-1.0, 0.3, 0.0)
        seen: list[tuple[Observation, Action]] = []
        result = run_goto(robot, goal, on_tick=lambda obs, act, _: seen.append((obs, act)))
        self.assertTrue(result.ok, result.detail)
        behind = [pose_error(obs.base, goal)[0] < 0.0 for obs, _ in seen]
        turning = behind.index(False)  # ticks spent before the goal first came round in front
        self.assertGreater(turning, 20)
        for obs, _ in seen[:turning]:
            self.assertLess(distance(obs.base, Pose()), 1e-3)  # on the spot, not an arc
        for (_, act), is_behind in zip(seen, behind):
            self.assertGreaterEqual(act.base_vx, 0.0)  # never reverses
            if is_behind:
                self.assertEqual(act.base_vx, 0.0)

    def test_finishes_by_turning_to_the_goal_heading(self) -> None:
        robot = TankFakeRobot()
        goal = Pose(0.8, 0.0, math.pi / 2)
        sent: list[Action] = []
        result = run_goto(robot, goal, on_tick=lambda obs, act, _: sent.append(act))
        self.assertTrue(result.ok, result.detail)
        self.assertLess(abs(angle_diff(goal.theta, robot.base.theta)), Limits().ang_tol)
        final_turn = sent[-30:-1]
        self.assertTrue(all(a.base_vx == 0.0 and a.base_wz > 0.0 for a in final_turn))

    def test_respects_the_limits_it_is_given(self) -> None:
        lim = Limits(v_max=0.15, w_max=0.5)
        recorder = Recorder(TankFakeRobot())
        result = run_goto(recorder, Pose(-0.8, -0.8, 1.0), limits=lim, timeout_s=60.0)
        self.assertTrue(result.ok, result.detail)
        self.assertEqual(max(a.base_vx for a in recorder.actions), lim.v_max)
        self.assertEqual(max(abs(a.base_wz) for a in recorder.actions), lim.w_max)
        self.assertTrue(all(a.base_vy == 0.0 for a in recorder.actions))

    def test_already_there_sends_only_the_stop(self) -> None:
        robot = StuckRobot(base=Pose(0.3, 0.4, 1.0))
        result = run_goto(robot, Pose(0.31, 0.4, 1.05))
        self.assertTrue(result.ok)
        self.assertEqual((robot.actions, result.data["ticks"]), ([ZERO], 0))

    def test_timeout_says_how_far_off_it_stopped(self) -> None:
        robot = StuckRobot()
        result = run_goto(robot, Pose(0.4, 0.0, math.radians(30)), timeout_s=1.0)
        self.assertFalse(result.ok)
        self.assertEqual(result.confidence, 0.0)
        self.assertEqual(
            result.detail, "Ran out of time after 1 s. Stopped 0.40 m short, facing 30° off."
        )
        self.assertAlmostEqual(result.data["distance_m"], 0.4)
        self.assertAlmostEqual(result.data["heading_error_rad"], math.radians(30))
        self.assertEqual(result.data["stopped_by"], "timeout")
        self.assertGreaterEqual(result.data["elapsed_s"], 1.0)
        self.assertLess(result.data["elapsed_s"], 1.0 + 1.5 * DT)
        self.assertEqual(robot.actions[-1], ZERO)

    def test_timeout_on_the_tank_reports_the_real_residual(self) -> None:
        robot = TankFakeRobot()
        goal = Pose(-2.0, 0.0, math.pi)
        result = run_goto(robot, goal, timeout_s=0.5)
        self.assertFalse(result.ok)
        match = re.fullmatch(
            r"Ran out of time after 0\.5 s\. Stopped (\d+\.\d\d) m short, facing (\d+)° off\.",
            result.detail,
        )
        self.assertIsNotNone(match, result.detail)
        final = robot.observe().base
        self.assertAlmostEqual(result.data["distance_m"], distance(final, goal))
        self.assertAlmostEqual(float(match[1]), distance(final, goal), places=2)
        heading = angle_diff(goal.theta, final.theta)
        self.assertAlmostEqual(result.data["heading_error_rad"], heading)
        self.assertEqual(int(match[2]), round(math.degrees(abs(heading))))
        self.assertAlmostEqual(result.data["elapsed_s"], 0.5)  # robot time: 25 ticks of DT
        self.assertEqual(result.data["ticks"], 25)

    def test_timeout_runs_on_observation_time(self) -> None:
        robot = StuckRobot(dt=10.0)  # every tick is ten seconds of robot time
        result = run_goto(robot, Pose(5.0, 0.0), timeout_s=25.0)
        self.assertEqual(result.data["stopped_by"], "timeout")
        self.assertEqual(len(robot.motion), 3)
        self.assertEqual(result.data["elapsed_s"], 30.0)

    def test_max_ticks_stops_a_loop_whose_clock_never_moves(self) -> None:
        robot = StuckRobot(dt=0.0)
        result = run_goto(robot, Pose(5.0, 0.0), max_ticks=7)
        self.assertFalse(result.ok)
        self.assertEqual(result.data["stopped_by"], "max_ticks")
        self.assertEqual(result.data["ticks"], 7)
        self.assertEqual(len(robot.motion), 7)
        self.assertTrue(all(a.base_vx > 0.0 for a in robot.motion))
        self.assertEqual(robot.actions[-1], ZERO)
        self.assertEqual(
            result.detail, "Gave up after 7 control ticks. Stopped 5.00 m short, "
            "facing the right way."
        )

    def test_max_ticks_on_the_tank(self) -> None:
        recorder = Recorder(TankFakeRobot())
        result = run_goto(recorder, Pose(3.0, 0.0), max_ticks=10)
        self.assertEqual(result.data["stopped_by"], "max_ticks")
        self.assertEqual(len(recorder.actions), 11)
        self.assertEqual(recorder.actions[-1], ZERO)

    def test_on_tick_sees_every_observation_and_the_command_that_followed(self) -> None:
        robot = StuckRobot()
        calls: list[tuple[Observation, Action, Target | None]] = []
        run_goto(robot, Pose(1.0, 0.5), timeout_s=0.1, on_tick=lambda *a: calls.append(a))
        self.assertEqual([act for _, act, _ in calls], robot.actions)
        self.assertEqual([obs.t for obs, _, _ in calls], [i * DT for i in range(len(calls))])
        self.assertTrue(all(target is None for _, _, target in calls))

    def test_stops_the_base_when_on_tick_raises(self) -> None:
        recorder = Recorder(TankFakeRobot())
        calls = 0

        def explode(obs: Observation, act: Action, target: Target | None) -> None:
            nonlocal calls
            calls += 1
            if calls == 5:
                raise RuntimeError("the dashboard fell over")

        with self.assertRaisesRegex(RuntimeError, "dashboard"):
            run_goto(recorder, Pose(2.0, 0.0), on_tick=explode)
        self.assertEqual(len(recorder.actions), 5)
        self.assertGreater(recorder.actions[-2].base_vx, 0.0)
        self.assertEqual(recorder.actions[-1], ZERO)

    def test_stops_the_base_when_the_backend_fails(self) -> None:
        robot = StuckRobot(observe_fails_at=4)
        with self.assertRaises(ConnectionError):
            run_goto(robot, Pose(2.0, 0.0))
        self.assertEqual(len(robot.actions), 5)
        self.assertEqual(robot.actions[-1], ZERO)

    def test_stops_the_base_when_the_arguments_are_bad(self) -> None:
        robot = StuckRobot()
        with self.assertRaisesRegex(ValueError, "timeout_s"):
            run_goto(robot, Pose(1.0, 0.0), timeout_s=0.0)
        self.assertEqual(robot.actions, [ZERO])

    def test_a_strafing_controller_is_refused_and_the_base_still_stops(self) -> None:
        class Holonomic(DifferentialGotoController):
            def step(self, base: Pose, goal: Pose) -> tuple[Action, bool]:
                action, done = super().step(base, goal)
                return Action(base_vx=action.base_vx, base_vy=0.1), done

        robot = TankFakeRobot()
        recorder = Recorder(robot)
        with self.assertRaisesRegex(ValueError, "cannot move sideways"):
            run_goto(recorder, Pose(1.0, 0.0), controller=Holonomic())
        self.assertEqual(recorder.actions[-1], ZERO)
        self.assertEqual(robot.base, Pose())

    def test_a_failed_stop_does_not_hide_the_original_error(self) -> None:
        class Broken(StuckRobot):
            def act(self, action: Action) -> None:
                super().act(action)
                if action == ZERO:
                    raise OSError("motor controller unplugged")

        robot = Broken(observe_fails_at=2)
        with self.assertLogs("retriever.navigation.drive", "ERROR") as logs, self.assertRaises(
            ConnectionError
        ):
            run_goto(robot, Pose(2.0, 0.0))
        self.assertIn("may still be moving", logs.output[0])
        self.assertEqual(robot.actions[-1], ZERO)


if __name__ == "__main__":
    unittest.main()
