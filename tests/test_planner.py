"""Planner loop, niches and provider message formats.

No network and no API keys: the scripted provider drives the loop, and the
real providers are exercised through their pure request builders and a fake
SDK client that records what would have been sent.
"""

import json
import math
import os
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

try:  # optional: the core suite must still run with no third-party packages
    import anthropic
    import httpx
    import httpx2
    import openai

    HAVE_SDKS = True
except ImportError:  # pragma: no cover
    HAVE_SDKS = False

from retriever.backends.fake import FakeRobot
from retriever.memory import Memory
from retriever.planner.loop import Planner, format_event
from retriever.planner.niches import BASE_PROMPT, FETCH, SORT, get_niche
from retriever.planner.providers import (
    DEFAULT_ANTHROPIC_MODEL,
    assistant_message,
    tool_results_message,
    user_message,
    FALLBACK_BETA,
    AnthropicProvider,
    OpenAICompatibleProvider,
    PlannerTurn,
    Provider,
    ScriptedProvider,
    ToolCall,
    accumulate_openai_stream,
    to_anthropic_messages,
    to_openai_messages,
)
from retriever.skills.builtin import (
    FakeClassifier,
    RobotContext,
    SimulatedPerception,
    build_library,
    robot_state,
)
from retriever.skills.library import Skill
from retriever.types import Pose, Station

STATIONS = [
    Station("kitchen", tag_id=0, pose=Pose(1.8, 0.9, math.radians(30))),
    Station("desk", tag_id=1, pose=Pose(1.6, -1.0, math.radians(-40))),
]

CLEAN_ENV = {k: "" for k in ("RETRIEVER_MODEL", "RETRIEVER_EFFORT", "RETRIEVER_FALLBACKS",
                             "PLANNER_BASE_URL", "PLANNER_API_KEY", "PLANNER_MODEL",
                             "PLANNER_STREAM", "OPENAI_API_KEY")}


def _world():
    bot = FakeRobot(objects_at={"keys": Pose(2.3, 1.2)})
    mem = Memory()
    mem.saw("keys", Pose(2.3, 1.2), 0.9)
    ctx = RobotContext(bot, list(STATIONS), mem, SimulatedPerception(bot),
                       classifier=FakeClassifier({"soda can": ("recycling", 0.9)}))
    return bot, mem, ctx, build_library(ctx)


def _planner(provider, niche=FETCH):
    bot, mem, ctx, lib = _world()
    events = []
    p = Planner(provider, lib, niche, memory=mem, state=lambda: robot_state(ctx),
                on_event=events.append)
    return p, bot, ctx, events


FETCH_SCRIPT = [
    {"tool_calls": [("recall", {"label": "keys"})]},
    {"text": "Heading to the kitchen.", "tool_calls": [("goto", {"station": "kitchen"})]},
    {"tool_calls": [("approach", {"label": "keys"})]},
    {"tool_calls": [("pick", {"label": "keys", "tool": "claw"})]},
    {"tool_calls": [("deliver", {"destination": "user"})]},
    "Here are your keys.",
]


# ------------------------------------------------------------------ loop


class TestPlannerLoop(unittest.TestCase):
    def test_scripted_multistep_goal_completes(self):
        prov = ScriptedProvider(FETCH_SCRIPT)
        p, bot, _, events = _planner(prov)
        r = p.run("get my keys")
        self.assertEqual(r.status, "done", r.text)
        self.assertEqual(r.text, "Here are your keys.")
        self.assertEqual(r.steps, 6)
        self.assertEqual([c["name"] for c in r.tool_calls],
                         ["recall", "goto", "approach", "pick", "deliver"])
        self.assertTrue(all(c["ok"] for c in r.tool_calls), r.tool_calls)
        # The world actually changed: the keys are with the user now.
        self.assertLess(math.hypot(bot.objects["keys"].x, bot.objects["keys"].y), 0.1)
        kinds = [e.kind for e in events]
        self.assertEqual(kinds[0], "goal")
        self.assertEqual(kinds[-1], "final")
        self.assertEqual(kinds.count("tool_result"), 5)
        json.dumps([e.data for e in events])
        json.loads(p.transcript_json())

    def test_results_are_fed_back_to_the_model(self):
        prov = ScriptedProvider(FETCH_SCRIPT[:2] + ["ok"])
        p, *_ = _planner(prov)
        p.run("get my keys")
        second = prov.requests[1]["messages"]
        self.assertEqual(second[-1]["role"], "tool")
        payload = json.loads(second[-1]["results"][0]["content"])
        self.assertTrue(payload["ok"])
        self.assertEqual(payload["data"]["near_station"], "kitchen")

    def test_generator_script_reacts_to_results(self):
        def script():
            r = yield ("recall", {"label": "keys"})
            r = yield ("goto", {"station": r["data"]["near_station"]})
            return f"Went there: {r['ok']}"

        p, bot, _, _ = _planner(ScriptedProvider.from_generator(script))
        r = p.run("where are my keys")
        self.assertEqual(r.text, "Went there: True")
        self.assertEqual(r.tool_calls[1]["args"], {"station": "kitchen"})

    def test_max_steps_cap(self):
        forever = lambda msgs: {"tool_calls": [("say", {"text": "still going"})]}  # noqa: E731
        prov = ScriptedProvider([forever] * 50)
        p, *_ = _planner(prov)
        r = p.run("do something forever", max_steps=3)
        self.assertEqual(r.status, "max_steps")
        self.assertEqual(r.steps, 3)
        self.assertEqual(len(prov.requests), 3)
        self.assertTrue(r.text)  # something honest to say

    def test_ask_user_pauses_then_resumes(self):
        asked = []
        prov = ScriptedProvider([
            {"tool_calls": [("ask_user", {"question": "The red mug or the blue one?"})]},
            lambda msgs: f"You said {json.loads(msgs[-1]['results'][0]['content'])['data']['answer']}.",
        ])
        p, _, ctx, events = _planner(prov)
        ctx.ask = asked.append  # must NOT be called from inside the loop
        r = p.run("get my mug")
        self.assertEqual(r.status, "needs_input")
        self.assertEqual(r.question, "The red mug or the blue one?")
        self.assertEqual(p.pending_question, r.question)
        self.assertEqual(asked, [])
        self.assertIn("needs_input", [e.kind for e in events])

        r = p.resume("the red one")
        self.assertEqual(r.status, "done")
        self.assertEqual(r.text, "You said the red one.")
        self.assertIsNone(p.pending_question)
        self.assertEqual(asked, [])

    def test_ask_user_mid_turn_keeps_every_result_in_one_message(self):
        prov = ScriptedProvider([
            {"tool_calls": [("say", {"text": "one moment"}),
                            ("ask_user", {"question": "Left or right?"}),
                            ("recall", {"label": "keys"})]},
            "Done.",
        ])
        p, *_ = _planner(prov)
        self.assertEqual(p.run("get my mug").status, "needs_input")
        self.assertEqual(p.resume("left").status, "done")
        tool_msg = prov.requests[1]["messages"][-1]
        self.assertEqual(tool_msg["role"], "tool")
        ids = [r["id"] for r in tool_msg["results"]]
        call_ids = [c.id for c in prov.requests[1]["messages"][-2]["tool_calls"]]
        self.assertEqual(ids, call_ids)
        names = [r["name"] for r in tool_msg["results"]]
        self.assertEqual(names, ["say", "ask_user", "recall"])

    def test_resume_without_a_question(self):
        p, *_ = _planner(ScriptedProvider(["hi"]))
        self.assertEqual(p.resume("yes").status, "error")

    def test_provider_exception_does_not_escape(self):
        class Down:
            name, model, tool_format = "down", "none", "anthropic"

            def complete(self, system, messages, tools):
                raise ConnectionError("wifi died")

        p, _, _, events = _planner(Down())
        with self.assertLogs("retriever.planner", level="ERROR"):
            r = p.run("get my keys")
        self.assertEqual(r.status, "error")
        self.assertIn("ConnectionError", r.text)
        self.assertEqual(events[-1].kind, "error")

    def test_skill_exception_and_unknown_tool_do_not_escape(self):
        prov = ScriptedProvider([
            {"tool_calls": [("explode", {}), ("teleport", {"to": "mars"})]},
            "I couldn't do that.",
        ])
        p, *_ = _planner(prov)

        def boom():
            raise RuntimeError("kaboom")

        p.library.register(Skill("explode", "Explodes.", {}, boom))
        with self.assertLogs("retriever.skills", level="ERROR"):
            r = p.run("do it")
        self.assertEqual(r.status, "done")
        self.assertEqual([c["ok"] for c in r.tool_calls], [False, False])
        results = prov.requests[1]["messages"][-1]["results"]
        self.assertTrue(all(res["is_error"] for res in results))

    def test_failed_skill_is_not_marked_as_a_tool_error(self):
        """'I missed' is information for the model, not a malformed call."""
        prov = ScriptedProvider([{"tool_calls": [("recall", {"label": "glasses"})]}, "Sorry."])
        p, *_ = _planner(prov)
        r = p.run("where are my glasses")
        self.assertFalse(r.tool_calls[0]["ok"])
        self.assertFalse(prov.requests[1]["messages"][-1]["results"][0]["is_error"])

    def test_latency_recorded(self):
        prov = ScriptedProvider(FETCH_SCRIPT[:2] + ["done"], latency_s=0.25)
        p, _, _, events = _planner(prov)
        r = p.run("get my keys")
        self.assertEqual(len(r.model_calls), 3)
        for m in r.model_calls:
            self.assertGreaterEqual(m["latency_s"], 0.25)
            self.assertEqual(m["provider"], "scripted")
        self.assertAlmostEqual(r.model_latency_s, 0.75, delta=0.05)
        self.assertGreaterEqual(r.skill_latency_s, 0.0)
        self.assertTrue(all("elapsed_s" in c for c in r.tool_calls))
        model_events = [e for e in events if e.kind == "model"]
        self.assertTrue(all(e.data["latency_s"] >= 0.25 for e in model_events))
        lines = [format_event(e) for e in events]
        self.assertTrue(any(line and "0.25s" in line for line in lines))

    def test_broken_event_hook_does_not_stop_the_plan(self):
        p, *_ = _planner(ScriptedProvider(FETCH_SCRIPT))
        p.on_event = lambda e: 1 / 0
        with self.assertLogs("retriever.planner", level="ERROR"):
            self.assertEqual(p.run("get my keys").status, "done")


# ------------------------------------------------------------------ niches


class TestNiches(unittest.TestCase):
    def test_swap_changes_only_prompt_and_config(self):
        prov = ScriptedProvider(["a", "b"])
        p, *_ = _planner(prov, niche=FETCH)
        names_before = list(p.library.names)
        p.run("get my keys")
        fetch_system = prov.requests[0]["system"]
        p.niche = get_niche("sort")
        p.run("clean up the desk")
        sort_system = prov.requests[1]["system"]

        self.assertEqual(prov.requests[0]["tools"], prov.requests[1]["tools"])
        self.assertEqual(p.library.names, names_before)
        self.assertNotEqual(fetch_system, sort_system)
        for text in (fetch_system, sort_system):
            self.assertTrue(text.startswith(BASE_PROMPT))
            self.assertIn("## Robot state", text)
            self.assertIn("keys: (2.30, 1.20)", text)  # Memory.summary() is in the prompt
        self.assertIn("landfill", sort_system)
        self.assertIn("refuse", sort_system)
        self.assertIn("ask_user", fetch_system)

    def test_niche_config(self):
        self.assertEqual(FETCH.destinations, ("user",))
        self.assertEqual(SORT.uncertain_destination, "landfill")
        self.assertEqual(SORT.bin_stations, ("recycling", "compost", "landfill"))
        self.assertIn("uncertain", SORT.categories)
        with self.assertRaises(ValueError):
            get_niche("juggle")


# ------------------------------------------------------------------ formats


CALLS = [ToolCall("t1", "goto", {"station": "desk"}), ToolCall("t2", "recall", {"label": "keys"})]
NEUTRAL = [
    {"role": "user", "content": "get my keys"},
    {"role": "assistant", "content": "On it.", "tool_calls": CALLS, "provider": "x", "raw": None},
    {"role": "tool", "results": [
        {"id": "t1", "name": "goto", "content": '{"ok": true}', "is_error": False},
        {"id": "t2", "name": "recall", "content": '{"ok": false}', "is_error": True},
    ]},
]


class TestAnthropicFormat(unittest.TestCase):
    def test_messages(self):
        out = to_anthropic_messages(NEUTRAL)
        self.assertEqual(out[0], {"role": "user", "content": "get my keys"})
        self.assertEqual(out[1]["role"], "assistant")
        self.assertEqual(out[1]["content"][0], {"type": "text", "text": "On it."})
        self.assertEqual(out[1]["content"][1],
                         {"type": "tool_use", "id": "t1", "name": "goto",
                          "input": {"station": "desk"}})
        # All results in ONE user message, is_error only where it is true.
        self.assertEqual(len(out), 3)
        self.assertEqual(out[2]["role"], "user")
        self.assertEqual(out[2]["content"][0],
                         {"type": "tool_result", "tool_use_id": "t1", "content": '{"ok": true}'})
        self.assertTrue(out[2]["content"][1]["is_error"])

    def test_own_raw_content_replayed_verbatim(self):
        """Thinking blocks must go back unchanged; rebuilding would drop them."""
        raw = [{"type": "thinking", "thinking": "", "signature": "sig"},
               {"type": "tool_use", "id": "t1", "name": "goto", "input": {"station": "desk"}}]
        msgs = [NEUTRAL[0], {**NEUTRAL[1], "provider": "anthropic", "raw": raw}]
        out = to_anthropic_messages(msgs)
        self.assertIs(out[1]["content"], raw)

    @mock.patch.dict(os.environ, CLEAN_ENV)
    def test_request_defaults(self):
        prov = AnthropicProvider()
        req = prov.build_request("SYS", NEUTRAL, [{"name": "goto"}])
        self.assertEqual(req["model"], DEFAULT_ANTHROPIC_MODEL)
        self.assertEqual(req["system"], "SYS")
        self.assertEqual(req["output_config"], {"effort": "low"})
        self.assertEqual(req["betas"], [FALLBACK_BETA])
        self.assertEqual(req["fallbacks"], "default")
        self.assertNotIn("thinking", req)  # adaptive by default; never disabled
        self.assertEqual(req["tools"], [{"name": "goto"}])
        json.dumps(req)

    @mock.patch.dict(os.environ, {**CLEAN_ENV, "RETRIEVER_MODEL": "claude-haiku-4-5"})
    def test_model_from_env_and_haiku_quirks(self):
        req = AnthropicProvider().build_request("S", NEUTRAL[:1], [])
        self.assertEqual(req["model"], "claude-haiku-4-5")
        self.assertNotIn("output_config", req)
        self.assertNotIn("fallbacks", req)
        self.assertNotIn("tools", req)

    @mock.patch.dict(os.environ, CLEAN_ENV)
    def test_complete_with_fake_client(self):
        sent = {}

        def create(**kw):
            sent.update(kw)
            return SimpleNamespace(
                stop_reason="tool_use", model="claude-opus-5",
                usage=SimpleNamespace(input_tokens=900, output_tokens=40),
                content=[SimpleNamespace(type="thinking", thinking="", signature="s"),
                         SimpleNamespace(type="text", text="Checking memory."),
                         SimpleNamespace(type="tool_use", id="tu_1", name="recall",
                                         input={"label": "keys"})],
            )

        client = SimpleNamespace(beta=SimpleNamespace(messages=SimpleNamespace(create=create)))
        prov = AnthropicProvider(client=client)
        self.assertIsInstance(prov, Provider)
        turn = prov.complete("SYS", NEUTRAL[:1], [])
        self.assertEqual(sent["fallbacks"], "default")
        self.assertEqual(turn.text, "Checking memory.")
        self.assertEqual(turn.tool_calls, [ToolCall("tu_1", "recall", {"label": "keys"})])
        self.assertEqual(turn.stop_reason, "tool_use")
        self.assertEqual(turn.usage["input_tokens"], 900)
        self.assertGreaterEqual(turn.latency_s, 0.0)
        self.assertEqual(len(turn.raw), 3)  # thinking block kept for replay

    def test_refusal_discards_tool_calls(self):
        prov = AnthropicProvider(client=object(), fallbacks=False)
        turn = prov.parse_response(SimpleNamespace(
            stop_reason="refusal", content=[SimpleNamespace(type="tool_use", id="x",
                                                           name="goto", input={})]), 0.1)
        self.assertEqual(turn.tool_calls, [])
        self.assertTrue(turn.text)

    @mock.patch.dict(os.environ, CLEAN_ENV)
    def test_planner_round_trip_replays_raw_and_pairs_results(self):
        replies = [
            SimpleNamespace(stop_reason="tool_use", model="m", usage=None, content=[
                SimpleNamespace(type="thinking", thinking="", signature="s"),
                SimpleNamespace(type="tool_use", id="tu_1", name="recall",
                                input={"label": "keys"})]),
            SimpleNamespace(stop_reason="end_turn", model="m", usage=None, content=[
                SimpleNamespace(type="text", text="They're in the kitchen.")]),
        ]
        requests = []

        def create(**kw):
            requests.append(kw)
            return replies[len(requests) - 1]

        client = SimpleNamespace(beta=SimpleNamespace(messages=SimpleNamespace(create=create)))
        p, *_ = _planner(AnthropicProvider(client=client))
        r = p.run("where are my keys?")
        self.assertEqual(r.text, "They're in the kitchen.")
        second = requests[1]["messages"]
        self.assertEqual(second[1]["content"], replies[0].content)  # verbatim replay
        self.assertEqual(second[2]["content"][0]["type"], "tool_result")
        self.assertEqual(second[2]["content"][0]["tool_use_id"], "tu_1")
        self.assertEqual(requests[0]["tools"][0].keys(), {"name", "description", "input_schema"})


class TestOpenAIFormat(unittest.TestCase):
    def test_messages(self):
        out = to_openai_messages("SYS", NEUTRAL)
        self.assertEqual(out[0], {"role": "system", "content": "SYS"})
        self.assertEqual(out[1], {"role": "user", "content": "get my keys"})
        a = out[2]
        self.assertEqual(a["role"], "assistant")
        self.assertEqual(a["tool_calls"][0]["type"], "function")
        self.assertEqual(json.loads(a["tool_calls"][0]["function"]["arguments"]),
                         {"station": "desk"})
        self.assertEqual(out[3], {"role": "tool", "tool_call_id": "t1", "content": '{"ok": true}'})
        self.assertEqual(out[4]["tool_call_id"], "t2")

    def test_anthropic_raw_turns_are_rebuilt_for_openai(self):
        msgs = [NEUTRAL[0], {**NEUTRAL[1], "provider": "anthropic", "raw": ["opaque"]}]
        out = to_openai_messages("", msgs)
        self.assertEqual(out[1]["tool_calls"][1]["function"]["name"], "recall")

    @mock.patch.dict(os.environ, {**CLEAN_ENV,
                                  "PLANNER_BASE_URL": "https://inference.baseten.co/v1",
                                  "PLANNER_API_KEY": "k", "PLANNER_MODEL": "qwen-test"})
    def test_env_config_and_request(self):
        prov = OpenAICompatibleProvider(client=object())
        self.assertEqual(prov.name, "baseten")
        self.assertEqual(prov.model, "qwen-test")
        self.assertEqual(prov.api_key, "k")
        tools = [{"type": "function", "function": {"name": "goto", "parameters": {}}}]
        req = prov.build_request("SYS", NEUTRAL, tools)
        self.assertEqual(req["model"], "qwen-test")
        self.assertEqual(req["tool_choice"], "auto")
        self.assertEqual(req["messages"][0]["role"], "system")
        self.assertNotIn("stream", req)
        self.assertEqual(req["max_tokens"], 1024)
        json.dumps(req)

    @mock.patch.dict(os.environ, {**CLEAN_ENV, "PLANNER_EXTRA_BODY": '{"modalities": ["text"]}'})
    def test_extra_body_from_env_and_official_openai_token_param(self):
        prov = OpenAICompatibleProvider(model="gpt-x", client=object())
        req = prov.build_request("S", NEUTRAL[:1], [])
        self.assertEqual(prov.name, "openai")
        self.assertEqual(req["extra_body"], {"modalities": ["text"]})
        self.assertIn("max_completion_tokens", req)
        self.assertNotIn("tools", req)

    @mock.patch.dict(os.environ, CLEAN_ENV)
    def test_model_is_required(self):
        with self.assertRaises(ValueError):
            OpenAICompatibleProvider()

    def test_parse_response_normalises_quirks(self):
        prov = OpenAICompatibleProvider(model="m", client=object(), name="relay")
        resp = SimpleNamespace(model="m", usage=None, choices=[SimpleNamespace(
            finish_reason="stop",  # some servers say stop even with tool calls
            message=SimpleNamespace(content=None, tool_calls=[
                SimpleNamespace(id=None, function=SimpleNamespace(
                    name="goto", arguments='{"station": "desk"}')),
                SimpleNamespace(id="c2", function=SimpleNamespace(
                    name="recall", arguments='{"label": ')),
            ]))])
        turn = prov.parse_response(resp, 0.4)
        self.assertEqual(turn.stop_reason, "tool_use")
        self.assertEqual(turn.tool_calls[0], ToolCall("call_0", "goto", {"station": "desk"}))
        self.assertEqual(turn.tool_calls[1].args, '{"label": ')  # library reports bad JSON
        self.assertEqual(turn.latency_s, 0.4)
        self.assertEqual(turn.provider, "relay")

    def test_stream_accumulation(self):
        chunks = [
            {"choices": [{"delta": {"content": "Let me "}}]},
            {"choices": [{"delta": {"content": "check."}}]},
            {"choices": [{"delta": {"tool_calls": [
                {"index": 0, "id": "c1", "function": {"name": "recall", "arguments": '{"la'}}]}}]},
            {"choices": [{"delta": {"tool_calls": [
                {"index": 0, "function": {"arguments": 'bel": "keys"}'}}]}}]},
            {"choices": [{"delta": {}, "finish_reason": "tool_calls"}]},
            {"choices": [], "usage": {"prompt_tokens": 10, "completion_tokens": 5}},
        ]
        acc = accumulate_openai_stream(chunks)
        self.assertEqual(acc["text"], "Let me check.")
        self.assertEqual(acc["tool_calls"],
                         [{"id": "c1", "name": "recall", "arguments": '{"label": "keys"}'}])
        self.assertEqual(acc["finish_reason"], "tool_calls")
        self.assertIsNotNone(acc["ttft_s"])

    def test_planner_round_trip_with_fake_client(self):
        replies = [
            SimpleNamespace(model="m", usage=None, choices=[SimpleNamespace(
                finish_reason="tool_calls", message=SimpleNamespace(content="", tool_calls=[
                    SimpleNamespace(id="c1", function=SimpleNamespace(
                        name="recall", arguments='{"label": "keys"}'))]))]),
            SimpleNamespace(model="m", usage=None, choices=[SimpleNamespace(
                finish_reason="stop",
                message=SimpleNamespace(content="Kitchen.", tool_calls=None))]),
        ]
        requests = []

        def create(**kw):
            requests.append(kw)
            return replies[len(requests) - 1]

        client = SimpleNamespace(chat=SimpleNamespace(completions=SimpleNamespace(create=create)))
        p, *_ = _planner(OpenAICompatibleProvider(model="m", client=client))
        r = p.run("where are my keys?")
        self.assertEqual(r.text, "Kitchen.")
        self.assertEqual(requests[0]["tools"][0]["type"], "function")
        last = requests[1]["messages"][-1]
        self.assertEqual(last["role"], "tool")
        self.assertEqual(last["tool_call_id"], "c1")
        self.assertTrue(json.loads(last["content"])["ok"])


RECALL_ANTHROPIC = [{"name": "recall", "description": "d", "input_schema": {
    "type": "object", "properties": {"label": {"type": "string"}}, "required": ["label"]}}]
RECALL_OPENAI = [{"type": "function", "function": {
    "name": "recall", "description": "d", "parameters": RECALL_ANTHROPIC[0]["input_schema"]}}]


@unittest.skipUnless(HAVE_SDKS, "anthropic/openai SDKs not installed")
class TestRealSdkSerialisation(unittest.TestCase):
    """The real SDK clients, with a mock HTTP transport in place of the network.

    Catches what the pure builders cannot: a kwarg the installed SDK rejects,
    or raw content blocks it cannot serialise back.
    """

    @mock.patch.dict(os.environ, CLEAN_ENV)
    def test_anthropic_wire_payload_and_thinking_replay(self):
        sent = []

        def handler(req):
            sent.append((req, json.loads(req.content)))
            return httpx2.Response(200, json={
                "id": "msg_1", "type": "message", "role": "assistant", "model": "claude-opus-5",
                "stop_reason": "tool_use", "stop_sequence": None,
                "usage": {"input_tokens": 10, "output_tokens": 5},
                "content": [
                    {"type": "thinking", "thinking": "", "signature": "sig"},
                    {"type": "tool_use", "id": "toolu_1", "name": "recall",
                     "input": {"label": "keys"}},
                ],
            })

        client = anthropic.Anthropic(
            api_key="test-key", max_retries=0,
            http_client=anthropic.DefaultHttpxClient(transport=httpx2.MockTransport(handler)))
        prov = AnthropicProvider(client=client)
        turn = prov.complete("SYS", [user_message("hi")], RECALL_ANTHROPIC)
        req, body = sent[0]
        self.assertTrue(str(req.url).endswith("/v1/messages?beta=true"))
        self.assertIn(FALLBACK_BETA, req.headers["anthropic-beta"])
        self.assertEqual(body["model"], "claude-opus-5")
        self.assertEqual(body["output_config"], {"effort": "low"})
        self.assertEqual(body["fallbacks"], "default")
        self.assertEqual(turn.tool_calls, [ToolCall("toolu_1", "recall", {"label": "keys"})])

        msgs = [user_message("hi"), assistant_message(turn), tool_results_message(
            [{"id": "toolu_1", "name": "recall", "content": '{"ok": true}', "is_error": False}])]
        prov.complete("SYS", msgs, RECALL_ANTHROPIC)
        _, body = sent[1]
        self.assertEqual(body["messages"][1]["content"][0],
                         {"type": "thinking", "thinking": "", "signature": "sig"})
        self.assertEqual(body["messages"][2]["content"][0]["tool_use_id"], "toolu_1")

    @mock.patch.dict(os.environ, CLEAN_ENV)
    def test_openai_wire_payload_plain_and_streaming(self):
        sent = []

        def handler(req):
            body = json.loads(req.content)
            sent.append((req, body))
            if body.get("stream"):
                chunks = [
                    {"choices": [{"index": 0, "delta": {"role": "assistant", "content": "Hm."}}]},
                    {"choices": [{"index": 0, "delta": {"tool_calls": [{
                        "index": 0, "id": "call_a", "type": "function",
                        "function": {"name": "recall", "arguments": '{"label":'}}]}}]},
                    {"choices": [{"index": 0, "finish_reason": "tool_calls", "delta": {
                        "tool_calls": [{"index": 0, "function": {"arguments": ' "keys"}'}}]}}]},
                    {"choices": [], "usage": {"prompt_tokens": 3, "completion_tokens": 4,
                                              "total_tokens": 7}},
                ]
                sse = "".join(
                    "data: " + json.dumps({"id": "c", "object": "chat.completion.chunk",
                                           "created": 0, "model": "m", **c}) + "\n\n"
                    for c in chunks) + "data: [DONE]\n\n"
                return httpx.Response(200, content=sse.encode(),
                                      headers={"content-type": "text/event-stream"})
            return httpx.Response(200, json={
                "id": "x", "object": "chat.completion", "created": 0, "model": "m",
                "choices": [{"index": 0, "finish_reason": "tool_calls", "message": {
                    "role": "assistant", "content": None, "tool_calls": [{
                        "id": "call_1", "type": "function",
                        "function": {"name": "recall", "arguments": '{"label": "keys"}'}}]}}],
            })

        base = "https://inference.baseten.co/v1"
        client = openai.OpenAI(api_key="k", base_url=base, max_retries=0,
                               http_client=httpx.Client(transport=httpx.MockTransport(handler)))
        prov = OpenAICompatibleProvider(model="m", base_url=base, client=client)
        turn = prov.complete("SYS", [user_message("hi")], RECALL_OPENAI)
        req, body = sent[0]
        self.assertEqual(str(req.url), base + "/chat/completions")
        self.assertEqual(body["messages"][0], {"role": "system", "content": "SYS"})
        self.assertEqual(turn.tool_calls, [ToolCall("call_1", "recall", {"label": "keys"})])

        msgs = [user_message("hi"), assistant_message(turn), tool_results_message(
            [{"id": "call_1", "name": "recall", "content": '{"ok": true}', "is_error": False}])]
        prov.complete("SYS", msgs, RECALL_OPENAI)
        _, body = sent[1]
        self.assertEqual(body["messages"][2]["tool_calls"][0]["id"], "call_1")
        self.assertEqual(body["messages"][3],
                         {"role": "tool", "tool_call_id": "call_1", "content": '{"ok": true}'})

        prov.stream = True
        turn = prov.complete("SYS", [user_message("hi")], RECALL_OPENAI)
        self.assertTrue(sent[2][1]["stream"])
        self.assertEqual(turn.text, "Hm.")
        self.assertEqual(turn.tool_calls, [ToolCall("call_a", "recall", {"label": "keys"})])
        self.assertIsNotNone(turn.ttft_s)
        self.assertEqual(turn.usage["completion_tokens"], 4)


class TestScriptedProvider(unittest.TestCase):
    def test_exhausted_script_ends_politely(self):
        prov = ScriptedProvider([])
        turn = prov.complete("", [], [])
        self.assertEqual(turn.tool_calls, [])
        self.assertTrue(turn.text)

    def test_planner_turn_passthrough(self):
        t = PlannerTurn(text="hi")
        self.assertIs(ScriptedProvider([t]).complete("", [], []), t)


if __name__ == "__main__":
    unittest.main()
