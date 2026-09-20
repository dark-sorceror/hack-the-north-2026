"""Skill library and built-in skills. FakeRobot only: no hardware, no network,
no wall-clock sleeping."""

import json
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.backends.fake import FakeRobot
from retriever.memory import Memory
from retriever.skills.builtin import (
    FakeClassifier,
    RobotContext,
    SimulatedPerception,
    build_library,
    robot_state,
)
from retriever.skills.library import Skill, SkillLibrary, result_to_json
from retriever.types import Pose, Result, Station, Target

KITCHEN = Station("kitchen", tag_id=0, pose=Pose(1.8, 0.9, math.radians(30)))
DESK = Station("desk", tag_id=1, pose=Pose(1.6, -1.0, math.radians(-40)))


def _lib_with(*skills):
    lib = SkillLibrary()
    for s in skills:
        lib.register(s)
    return lib


GOTO = Skill(
    "goto",
    "Drive to a station.",
    {"properties": {"station": {"type": "string"}, "speed": {"type": "number"},
                    "mode": {"type": "string", "enum": ["fast", "slow"]}},
     "required": ["station"]},
    lambda station, speed=0.2, mode="slow": Result(ok=True, detail=f"At {station}."),
)


class CountingRobot(FakeRobot):
    """FakeRobot that counts act() calls, to prove a skill did not move."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.acts = 0

    def act(self, action):
        self.acts += 1
        super().act(action)


def _world(objects, heights=None, instances=None, stations=(KITCHEN, DESK), **ctx_kw):
    bot = CountingRobot(objects_at=objects)
    mem = Memory()
    spoken = []
    ctx = RobotContext(
        backend=bot,
        stations=list(stations),
        memory=mem,
        perceive=SimulatedPerception(bot, heights=heights, instances=instances),
        say=spoken.append,
        classifier=FakeClassifier({"soda can": ("recycling", 0.9),
                                   "banana peel": ("compost", 0.85),
                                   "chip bag": ("landfill", 0.4),
                                   "wallet": ("belongings", 0.95)}),
        **ctx_kw,
    )
    return bot, mem, ctx, build_library(ctx), spoken


# ------------------------------------------------------------------ library


class TestSchemas(unittest.TestCase):
    def test_anthropic_format(self):
        (tool,) = _lib_with(GOTO).to_tool_schemas("anthropic")
        self.assertEqual(set(tool), {"name", "description", "input_schema"})
        self.assertEqual(tool["name"], "goto")
        self.assertEqual(tool["input_schema"]["type"], "object")
        self.assertEqual(tool["input_schema"]["required"], ["station"])

    def test_openai_format(self):
        (tool,) = _lib_with(GOTO).to_tool_schemas("openai")
        self.assertEqual(tool["type"], "function")
        fn = tool["function"]
        self.assertEqual(fn["name"], "goto")
        self.assertEqual(fn["parameters"]["properties"]["station"]["type"], "string")

    def test_empty_params_still_a_valid_object_schema(self):
        lib = _lib_with(Skill("look_around", "Spin.", {}, lambda: Result(ok=True)))
        (a,) = lib.to_tool_schemas("anthropic")
        self.assertEqual(a["input_schema"], {"type": "object", "properties": {}})

    def test_unknown_format_rejected(self):
        with self.assertRaises(ValueError):
            _lib_with(GOTO).to_tool_schemas("gemini")

    def test_builtin_schemas_serialise_in_both_formats(self):
        _, _, ctx, lib, _ = _world({})
        for fmt in ("anthropic", "openai"):
            tools = lib.to_tool_schemas(fmt)
            json.dumps(tools)
            self.assertEqual(len(tools), 10)
        names = set(lib.names)
        self.assertEqual(names, {"look_around", "goto", "approach", "pick", "deliver",
                                 "recall", "classify", "refuse", "ask_user", "say"})

    def test_pick_tool_enum_comes_from_the_robot(self):
        """A robot with only a claw must not offer suction to the model."""
        _, _, _, lib, _ = _world({}, tools=("claw",))
        (pick,) = [t for t in lib.to_tool_schemas("anthropic") if t["name"] == "pick"]
        self.assertEqual(pick["input_schema"]["properties"]["tool"]["enum"], ["claw"])

    def test_bad_skill_name_rejected_at_registration(self):
        with self.assertRaises(ValueError):
            SkillLibrary().register(Skill("go to", "x", {}, lambda: Result(ok=True)))


class TestValidation(unittest.TestCase):
    def setUp(self):
        self.lib = _lib_with(GOTO)

    def test_valid_call(self):
        r = self.lib.dispatch("goto", {"station": "desk"})
        self.assertTrue(r.ok)

    def test_missing_required(self):
        r = self.lib.dispatch("goto", {})
        self.assertFalse(r.ok)
        self.assertEqual(r.data["error"], "bad_args")
        self.assertIn("station", r.detail)

    def test_wrong_type(self):
        r = self.lib.dispatch("goto", {"station": 3})
        self.assertFalse(r.ok)
        self.assertIn("string", r.detail)

    def test_bool_is_not_a_number(self):
        r = self.lib.dispatch("goto", {"station": "desk", "speed": True})
        self.assertFalse(r.ok)

    def test_int_is_a_number(self):
        self.assertTrue(self.lib.dispatch("goto", {"station": "desk", "speed": 1}).ok)

    def test_enum(self):
        r = self.lib.dispatch("goto", {"station": "desk", "mode": "ludicrous"})
        self.assertFalse(r.ok)
        self.assertIn("fast", r.detail)

    def test_unknown_argument(self):
        r = self.lib.dispatch("goto", {"station": "desk", "teleport": True})
        self.assertFalse(r.ok)
        self.assertIn("teleport", r.detail)

    def test_json_string_arguments_accepted(self):
        """OpenAI-compatible endpoints pass arguments as a JSON string."""
        self.assertTrue(self.lib.dispatch("goto", '{"station": "desk"}').ok)

    def test_invalid_json_string_is_a_spoken_failure(self):
        r = self.lib.dispatch("goto", '{"station": "de')
        self.assertFalse(r.ok)
        self.assertEqual(r.data["error"], "bad_args")

    def test_none_args_mean_no_args(self):
        lib = _lib_with(Skill("look_around", "Spin.", {}, lambda: Result(ok=True)))
        self.assertTrue(lib.dispatch("look_around", None).ok)


class TestDispatchNeverRaises(unittest.TestCase):
    def test_unknown_skill(self):
        r = _lib_with(GOTO).dispatch("fly", {})
        self.assertFalse(r.ok)
        self.assertEqual(r.data["error"], "unknown_skill")
        self.assertIn("goto", r.detail)  # tells the listener what it CAN do

    def test_exception_becomes_failed_result(self):
        def boom():
            raise RuntimeError("servo 4 overheated")

        with self.assertLogs("retriever.skills", level="ERROR"):
            r = _lib_with(Skill("wave", "Wave.", {}, boom)).dispatch("wave", {})
        self.assertFalse(r.ok)
        self.assertEqual(r.data["error"], "exception")
        self.assertEqual(r.data["exception"], "RuntimeError")
        self.assertIn("servo 4 overheated", r.detail)

    def test_non_result_return(self):
        r = _lib_with(Skill("wave", "Wave.", {}, lambda: "done!")).dispatch("wave")
        self.assertFalse(r.ok)
        self.assertEqual(r.data["error"], "bad_return")

    def test_keyboard_interrupt_still_propagates(self):
        """ctrl-c must stop the robot, not be turned into a spoken failure."""
        def interrupted():
            raise KeyboardInterrupt

        with self.assertRaises(KeyboardInterrupt):
            _lib_with(Skill("wave", "Wave.", {}, interrupted)).dispatch("wave")


class TestTimingAndLogging(unittest.TestCase):
    def test_every_dispatch_recorded_with_elapsed(self):
        lib = _lib_with(GOTO)
        seen = []
        lib.on_dispatch = seen.append
        lib.dispatch("goto", {"station": "desk"})
        lib.dispatch("nope", {})
        self.assertEqual([r.name for r in lib.history], ["goto", "nope"])
        self.assertEqual(len(seen), 2)
        for rec in lib.history:
            self.assertGreaterEqual(rec.elapsed_s, 0.0)
            json.dumps(rec.to_json())

    def test_broken_hook_does_not_fail_the_skill(self):
        lib = _lib_with(GOTO)
        lib.on_dispatch = lambda rec: 1 / 0
        with self.assertLogs("retriever.skills", level="ERROR"):
            self.assertTrue(lib.dispatch("goto", {"station": "desk"}).ok)

    def test_logs_each_call(self):
        with self.assertLogs("retriever.skills", level="INFO") as cm:
            _lib_with(GOTO).dispatch("goto", {"station": "desk"})
        self.assertTrue(any("goto" in line for line in cm.output))


class TestResultJson(unittest.TestCase):
    def test_dataclasses_tuples_and_nan_flatten(self):
        r = Result(ok=False, detail="x", data={
            "pose": Pose(1.0, 2.0, 0.5),
            "target": Target("keys", 0.1, 0.5, bbox=(1, 2, 3, 4)),
            "residual": float("nan"),
            "blob": b"\xff\xd8",
        })
        j = result_to_json(r)
        text = json.dumps(j, allow_nan=False)
        self.assertEqual(j["data"]["pose"], {"x": 1.0, "y": 2.0, "theta": 0.5})
        self.assertEqual(j["data"]["target"]["bbox"], [1, 2, 3, 4])
        self.assertIsNone(j["data"]["residual"])
        self.assertIn("bytes", text)


# ------------------------------------------------------------------ built-ins


class TestRecall(unittest.TestCase):
    def test_answers_from_memory_without_moving(self):
        bot, mem, _, lib, _ = _world({"keys": Pose(2.3, 1.2)})
        mem.saw("keys", Pose(2.3, 1.2), 0.9, at=0)  # long ago is fine
        mem.saw("keys", Pose(2.1, 1.0), 0.8)
        before = bot.base
        r = lib.dispatch("recall", {"label": "my keys"})
        self.assertTrue(r.ok, r.detail)
        self.assertIn("kitchen", r.detail)
        self.assertEqual(r.data["near_station"], "kitchen")
        self.assertAlmostEqual(r.data["x"], 2.1)
        self.assertEqual(bot.acts, 0)
        self.assertEqual(bot.base, before)

    def test_never_seen_is_an_honest_answer(self):
        bot, _, _, lib, _ = _world({})
        r = lib.dispatch("recall", {"label": "glasses"})
        self.assertFalse(r.ok)
        self.assertFalse(r.data["found"])
        self.assertIn("haven't seen", r.detail)
        self.assertEqual(bot.acts, 0)

    def test_far_from_every_station_gives_coordinates(self):
        _, mem, _, lib, _ = _world({})
        mem.saw("keys", Pose(-3.0, 3.0), 0.9)
        r = lib.dispatch("recall", {"label": "keys"})
        self.assertNotIn("near the", r.detail)


class TestFetchSequence(unittest.TestCase):
    def test_goto_approach_pick_deliver(self):
        bot, mem, ctx, lib, _ = _world({"keys": Pose(2.3, 1.2)})
        self.assertTrue(lib.dispatch("goto", {"station": "kitchen"}).ok)
        r = lib.dispatch("approach", {"label": "keys"})
        self.assertTrue(r.ok, r.detail)
        r = lib.dispatch("pick", {"label": "keys", "tool": "claw"})
        self.assertTrue(r.ok, r.detail)
        self.assertTrue(r.data["holding"])
        self.assertGreater(r.data["gripper_load"], ctx.contact_load)
        self.assertEqual(bot.holding, "keys")

        r = lib.dispatch("deliver", {"destination": "user"})
        self.assertTrue(r.ok, r.detail)
        self.assertIsNone(bot.holding)
        self.assertLess(math.hypot(bot.objects["keys"].x, bot.objects["keys"].y), 0.1)
        # Memory follows the object to where it was put down.
        self.assertLess(math.hypot(mem.last_seen("keys").pose.x, mem.last_seen("keys").pose.y), 0.1)

    def test_pick_reports_empty_gripper_honestly(self):
        """Perception reports keys where there are none: the claw closes on air
        and the skill must say so rather than claim success."""
        bot, mem, ctx, lib, _ = _world({})
        ghost = Pose(0.6, 0.0)

        def perceive(obs):
            dx, dy = ghost.x - obs.base.x, ghost.y - obs.base.y
            return [Target("keys", math.atan2(dy, dx) - obs.base.theta, math.hypot(dx, dy), 0.9)]

        ctx.perceive = perceive
        r = lib.dispatch("pick", {"label": "keys"})
        self.assertFalse(r.ok)
        self.assertFalse(r.data["holding"])
        self.assertLess(r.data["gripper_load"], ctx.contact_load)
        self.assertIn("felt nothing", r.detail)

    def test_suction_miss_is_worded_for_suction(self):
        _, _, ctx, lib, _ = _world({})
        ctx.perceive = lambda obs: [Target("card", 0.0, max(0.0, 0.5 - obs.base.x), 0.9)]
        r = lib.dispatch("pick", {"label": "card", "tool": "suction"})
        self.assertFalse(r.ok)
        self.assertIn("suction", r.detail)

    def test_out_of_reach_is_refused_before_moving(self):
        bot, _, _, lib, _ = _world({"water bottle": Pose(0.5, 0.0)},
                                   heights={"water bottle": 0.75})
        r = lib.dispatch("pick", {"label": "water bottle"})
        self.assertFalse(r.ok)
        self.assertIn("75 cm", r.detail)
        self.assertEqual(bot.acts, 0)

    def test_approach_not_visible_points_at_memory(self):
        _, mem, _, lib, _ = _world({})
        mem.saw("inhaler", DESK.pose, 0.9)
        r = lib.dispatch("approach", {"label": "inhaler"})
        self.assertFalse(r.ok)
        self.assertFalse(r.data["visible"])
        self.assertIn("desk", r.detail)

    def test_two_lookalikes_are_ambiguous_not_guessed(self):
        bot, _, _, lib, _ = _world({"mug#1": Pose(0.6, 0.25), "mug#2": Pose(0.6, -0.25)})
        r = lib.dispatch("approach", {"label": "mug"})
        self.assertFalse(r.ok)
        self.assertTrue(r.data["ambiguous"])
        self.assertEqual(len(r.data["candidates"]), 2)
        self.assertEqual(bot.acts, 0)
        r = lib.dispatch("approach", {"label": "mug", "which": "left"})
        self.assertTrue(r.ok, r.detail)
        self.assertGreater(bot.base.y, 0.1)  # went to the left one

    def test_taught_instance_breaks_the_tie(self):
        _, _, _, lib, _ = _world({"mug#1": Pose(0.6, 0.25), "mug#2": Pose(0.6, -0.25)},
                                 instances={"mug#2": "obj-7"})
        self.assertTrue(lib.dispatch("approach", {"label": "mug"}).ok)

    def test_deliver_with_empty_gripper(self):
        bot, _, _, lib, _ = _world({})
        r = lib.dispatch("deliver", {"destination": "user"})
        self.assertFalse(r.ok)
        self.assertEqual(bot.acts, 0)

    def test_goto_unknown_station_lists_known(self):
        _, _, _, lib, _ = _world({})
        r = lib.dispatch("goto", {"station": "garage"})
        self.assertFalse(r.ok)
        self.assertIn("kitchen", r.detail)

    def test_goto_uses_differential_controller_by_default(self):
        """Tank base: heading must track the direction of travel (no strafing)."""
        bot, _, _, lib, _ = _world({})
        ys = []
        bot_act = bot.act

        def spy(action):
            ys.append(action.base_vy)
            bot_act(action)

        bot.act = spy
        self.assertTrue(lib.dispatch("goto", {"station": "desk"}).ok)
        self.assertTrue(all(vy == 0.0 for vy in ys))


class TestLookAround(unittest.TestCase):
    def test_records_sightings_in_world_frame(self):
        bot, mem, _, lib, _ = _world({"keys": Pose(1.0, 1.0), "inhaler": Pose(-1.0, -0.5)})
        r = lib.dispatch("look_around", {})
        self.assertTrue(r.ok, r.detail)
        self.assertEqual({s["label"] for s in r.data["seen"]}, {"keys", "inhaler"})
        s = mem.last_seen("inhaler")
        self.assertAlmostEqual(s.pose.x, -1.0, places=2)
        self.assertAlmostEqual(s.pose.y, -0.5, places=2)
        self.assertLess(math.hypot(bot.base.x, bot.base.y), 0.05)  # turned, didn't drive


class TestJudgementSkills(unittest.TestCase):
    def test_classify_confident(self):
        _, _, _, lib, _ = _world({})
        r = lib.dispatch("classify", {"label": "soda can"})
        self.assertTrue(r.ok)
        self.assertEqual(r.data["category"], "recycling")

    def test_classify_low_confidence_is_uncertain(self):
        _, _, _, lib, _ = _world({})
        r = lib.dispatch("classify", {"label": "chip bag"})
        self.assertEqual(r.data["category"], "uncertain")
        self.assertEqual(r.data["guess"], "landfill")

    def test_classify_unknown_is_uncertain(self):
        _, _, _, lib, _ = _world({})
        self.assertEqual(lib.dispatch("classify", {"label": "gizmo"}).data["category"],
                         "uncertain")

    def test_refuse_speaks(self):
        _, _, _, lib, spoken = _world({})
        r = lib.dispatch("refuse", {"label": "wallet", "reason": "it belongs to someone"})
        self.assertTrue(r.ok)
        self.assertEqual(len(spoken), 1)
        self.assertIn("wallet", spoken[0])

    def test_ask_user_without_a_listener(self):
        _, _, _, lib, _ = _world({})
        self.assertFalse(lib.dispatch("ask_user", {"question": "Which mug?"}).ok)

    def test_ask_user_with_a_listener(self):
        _, _, ctx, lib, _ = _world({})
        ctx.ask = lambda q: "the red one"
        r = lib.dispatch("ask_user", {"question": "Which mug?"})
        self.assertEqual(r.data["answer"], "the red one")

    def test_robot_state_summary(self):
        _, _, ctx, _, _ = _world({})
        text = robot_state(ctx)
        self.assertIn("kitchen", text)
        self.assertIn("Gripper: empty", text)


if __name__ == "__main__":
    unittest.main()
