"""Model providers for the planner, behind one small Protocol.

The team will compare planner latency across Claude, OpenAI-compatible
endpoints (Baseten, a Qwen-Omni relay) and whatever else turns up at 2am, so
the planner loop must not know which API it is talking to. It speaks a neutral
message format; each provider converts to and from its own wire format.

Neutral messages (what the loop keeps):

    {"role": "user", "content": "get my keys"}
    {"role": "assistant", "content": "<text>", "tool_calls": [ToolCall, ...],
     "provider": "anthropic", "raw": <provider-native content or None>}
    {"role": "tool", "results": [{"id", "name", "content": "<json>", "is_error"}]}

`raw` exists for one reason: Claude's assistant turns can carry thinking
blocks that must be sent back unchanged on the next request. Rebuilding the
turn from text + tool calls would drop them, so a provider replays its own raw
content verbatim and only falls back to rebuilding for turns another provider
produced.

The request builders are pure functions of (system, messages, tools), so the
format conversion is unit-tested without a network or an API key.

Latency is measured around the HTTP call only (perf_counter), never including
tool execution, so numbers are comparable across providers.
"""

from __future__ import annotations

import inspect
import json
import os
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, NamedTuple, Protocol, runtime_checkable
from urllib.parse import urlparse

# Claude Opus 5. Overridable with RETRIEVER_MODEL; see AnthropicProvider for why
# effort defaults to "low" instead of switching to a smaller model.
DEFAULT_ANTHROPIC_MODEL = "claude-opus-5"
DEFAULT_EFFORT = "low"

# Server-side refusal fallback: if Claude declines, the API re-runs the request
# on Anthropic's recommended fallback model inside the same call.
FALLBACK_BETA = "server-side-fallback-2026-07-01"


class ToolCall(NamedTuple):
    """(id, name, args). A tuple so `for id, name, args in turn.tool_calls`
    works; named so the rest of the code can say `call.name`.

    `args` is a dict normally. If an OpenAI-compatible endpoint returned
    arguments that are not valid JSON, it is the raw string, and the skill
    library turns that into a spoken bad-arguments failure."""

    id: str
    name: str
    args: Any


@dataclass
class PlannerTurn:
    text: str = ""
    tool_calls: list[ToolCall] = field(default_factory=list)
    stop_reason: str = "end_turn"   # end_turn | tool_use | max_tokens | refusal | ...
    latency_s: float = 0.0
    provider: str = ""
    model: str = ""
    usage: dict[str, Any] = field(default_factory=dict)
    raw: Any = None
    ttft_s: float | None = None     # time to first token, streaming only


@runtime_checkable
class Provider(Protocol):
    name: str
    model: str
    tool_format: str  # "anthropic" | "openai": which SkillLibrary schema format it wants

    def complete(
        self, system: str, messages: list[dict[str, Any]], tools: list[dict[str, Any]]
    ) -> PlannerTurn: ...


# ------------------------------------------------------------------ neutral format


def user_message(text: str) -> dict[str, Any]:
    return {"role": "user", "content": text}


def assistant_message(turn: PlannerTurn) -> dict[str, Any]:
    return {
        "role": "assistant",
        "content": turn.text,
        "tool_calls": list(turn.tool_calls),
        "provider": turn.provider,
        "raw": turn.raw,
    }


def tool_results_message(results: list[dict[str, Any]]) -> dict[str, Any]:
    """results: [{"id", "name", "content": str, "is_error": bool}]"""
    return {"role": "tool", "results": list(results)}


# ------------------------------------------------------------------ Anthropic format


def to_anthropic_messages(
    messages: list[dict[str, Any]], replay_provider: str = "anthropic"
) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for m in messages:
        role = m["role"]
        if role == "user":
            out.append({"role": "user", "content": m["content"]})
        elif role == "assistant":
            if m.get("raw") is not None and m.get("provider") == replay_provider:
                # Verbatim, including thinking blocks. Do not rebuild.
                out.append({"role": "assistant", "content": m["raw"]})
                continue
            blocks: list[dict[str, Any]] = []
            if m.get("content"):
                blocks.append({"type": "text", "text": m["content"]})
            for c in m.get("tool_calls") or []:
                args = c.args if isinstance(c.args, dict) else {}
                blocks.append({"type": "tool_use", "id": c.id, "name": c.name, "input": args})
            if blocks:  # the API rejects an empty assistant turn
                out.append({"role": "assistant", "content": blocks})
        elif role == "tool":
            # ALL results for one assistant turn go in ONE user message.
            # Splitting them teaches the model to stop making parallel calls.
            blocks = []
            for r in m["results"]:
                b: dict[str, Any] = {
                    "type": "tool_result",
                    "tool_use_id": r["id"],
                    "content": r["content"],
                }
                if r.get("is_error"):
                    b["is_error"] = True
                blocks.append(b)
            out.append({"role": "user", "content": blocks})
        else:
            raise ValueError(f"unknown neutral role: {role!r}")
    return out


def _model_family(model: str) -> str:
    m = model.lower()
    for fam in ("opus-5", "fable-5", "mythos-5", "sonnet-5", "opus-4-8", "opus-4-7",
                "opus-4-6", "sonnet-4-6", "haiku"):
        if fam in m:
            return fam
    return "other"


def _env_flag(name: str) -> bool | None:
    v = os.environ.get(name)
    if v is None or v == "":
        return None
    return v.strip().lower() in ("1", "true", "yes", "on")


class AnthropicProvider:
    """Claude via the official `anthropic` SDK (Messages API, manual tool loop).

    Model: RETRIEVER_MODEL, else claude-opus-5.

    Latency: effort defaults to "low" (RETRIEVER_EFFORT to change; "none" omits
    it and gets the API default). A planner
    turn is a short routing decision, which is exactly the workload where low
    effort holds quality and cuts time-to-answer. Thinking is left at the
    model default (adaptive) rather than disabled: with thinking disabled,
    Opus 5 occasionally writes a tool call into plain text instead of a
    tool_use block, which in this loop would silently never run.

    Refusals: on models that support it, `fallbacks: "default"` is sent so a
    refused request is retried server-side instead of stalling the robot.
    RETRIEVER_FALLBACKS=0 turns it off (e.g. behind a proxy that rejects betas).
    """

    tool_format = "anthropic"

    def __init__(
        self,
        model: str | None = None,
        api_key: str | None = None,
        max_tokens: int = 16000,
        effort: str | None = None,
        fallbacks: bool | None = None,
        timeout_s: float = 45.0,
        max_retries: int = 2,
        client: Any = None,
        name: str = "anthropic",
    ) -> None:
        self.name = name
        self.model = model or os.environ.get("RETRIEVER_MODEL") or DEFAULT_ANTHROPIC_MODEL
        fam = _model_family(self.model)
        if effort is None:
            effort = os.environ.get("RETRIEVER_EFFORT") or DEFAULT_EFFORT
        self.effort = None if effort.lower() in ("none", "off", "default") else effort
        if fam == "haiku":
            self.effort = None  # Haiku 4.5 rejects the effort parameter
        if fallbacks is None:
            fallbacks = _env_flag("RETRIEVER_FALLBACKS")
        if fallbacks is None:
            fallbacks = fam in ("opus-5", "fable-5", "mythos-5")
        self.fallbacks = bool(fallbacks)
        self.max_tokens = max_tokens
        self.api_key = api_key
        self.timeout_s = timeout_s
        self.max_retries = max_retries
        self._client = client

    @property
    def client(self) -> Any:
        """Built on first use, so constructing a provider (in tests, or to
        print a config) never needs credentials or the network."""
        if self._client is None:
            import anthropic

            kwargs: dict[str, Any] = {"timeout": self.timeout_s, "max_retries": self.max_retries}
            if self.api_key:
                kwargs["api_key"] = self.api_key
            self._client = anthropic.Anthropic(**kwargs)
        return self._client

    def build_request(
        self, system: str, messages: list[dict[str, Any]], tools: list[dict[str, Any]]
    ) -> dict[str, Any]:
        req: dict[str, Any] = {
            "model": self.model,
            "max_tokens": self.max_tokens,
            "system": system,
            "messages": to_anthropic_messages(messages, self.name),
        }
        if tools:
            req["tools"] = tools
        if self.effort:
            req["output_config"] = {"effort": self.effort}
        if self.fallbacks:
            req["betas"] = [FALLBACK_BETA]
            req["fallbacks"] = "default"
        return req

    def complete(
        self, system: str, messages: list[dict[str, Any]], tools: list[dict[str, Any]]
    ) -> PlannerTurn:
        req = self.build_request(system, messages, tools)
        api = self.client.beta.messages if "betas" in req else self.client.messages
        t0 = time.perf_counter()
        resp = api.create(**req)
        return self.parse_response(resp, time.perf_counter() - t0)

    def parse_response(self, resp: Any, latency_s: float) -> PlannerTurn:
        stop = getattr(resp, "stop_reason", None) or "end_turn"
        content = list(getattr(resp, "content", None) or [])
        texts: list[str] = []
        calls: list[ToolCall] = []
        for b in content:
            kind = getattr(b, "type", None)
            if kind == "text":
                texts.append(b.text)
            elif kind == "tool_use":
                args = b.input if isinstance(b.input, dict) else dict(b.input or {})
                calls.append(ToolCall(b.id, b.name, args))
        text = "".join(texts).strip()
        raw: Any = content
        if stop == "refusal":
            # Declined after the fallback chain too. Discard partial output; the
            # loop ends on a turn with no tool calls.
            calls, raw = [], None
            text = "I can't help with that request."
        return PlannerTurn(
            text=text,
            tool_calls=calls,
            stop_reason=stop,
            latency_s=latency_s,
            provider=self.name,
            model=getattr(resp, "model", None) or self.model,
            usage=_usage(getattr(resp, "usage", None)),
            raw=raw,
        )


def _usage(u: Any) -> dict[str, Any]:
    if u is None:
        return {}
    if isinstance(u, dict):
        return {k: v for k, v in u.items() if isinstance(v, (int, float))}
    out = {}
    for k in ("input_tokens", "output_tokens", "cache_read_input_tokens",
              "cache_creation_input_tokens", "prompt_tokens", "completion_tokens",
              "total_tokens"):
        v = getattr(u, k, None)
        if isinstance(v, (int, float)):
            out[k] = v
    return out


# ------------------------------------------------------------------ OpenAI format


def to_openai_messages(system: str, messages: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = [{"role": "system", "content": system}] if system else []
    for m in messages:
        role = m["role"]
        if role == "user":
            out.append({"role": "user", "content": m["content"]})
        elif role == "assistant":
            calls = m.get("tool_calls") or []
            msg: dict[str, Any] = {"role": "assistant", "content": m.get("content") or ""}
            if calls:
                msg["content"] = m.get("content") or None
                msg["tool_calls"] = [
                    {
                        "id": c.id,
                        "type": "function",
                        "function": {
                            "name": c.name,
                            "arguments": c.args if isinstance(c.args, str)
                            else json.dumps(c.args or {}),
                        },
                    }
                    for c in calls
                ]
            out.append(msg)
        elif role == "tool":
            # One message per result; OpenAI has no is_error flag, but the
            # content JSON carries ok:false, which is what the model reads.
            for r in m["results"]:
                out.append({"role": "tool", "tool_call_id": r["id"], "content": r["content"]})
        else:
            raise ValueError(f"unknown neutral role: {role!r}")
    return out


_FINISH = {
    "stop": "end_turn",
    "tool_calls": "tool_use",
    "function_call": "tool_use",
    "length": "max_tokens",
    "content_filter": "refusal",
}


def _get(obj: Any, key: str, default: Any = None) -> Any:
    if obj is None:
        return default
    if isinstance(obj, dict):
        return obj.get(key, default)
    return getattr(obj, key, default)


def _parse_args(raw: Any) -> Any:
    if isinstance(raw, dict):
        return raw
    if raw is None or (isinstance(raw, str) and not raw.strip()):
        return {}
    try:
        v = json.loads(raw)
        return v if isinstance(v, dict) else raw
    except (TypeError, json.JSONDecodeError):
        return raw


def accumulate_openai_stream(chunks: Iterable[Any]) -> dict[str, Any]:
    """Fold chat.completions stream chunks into one message.

    Tool calls arrive as fragments keyed by `index`: the id and name in the
    first fragment, the JSON arguments spread across the rest. Some relays
    (Qwen-Omni among them) only offer streaming, so this is not optional.
    """
    t0 = time.perf_counter()
    ttft: float | None = None
    text: list[str] = []
    calls: dict[int, dict[str, Any]] = {}
    finish = None
    usage = None
    for ch in chunks:
        if _get(ch, "usage"):
            usage = _get(ch, "usage")
        for choice in _get(ch, "choices") or []:
            delta = _get(choice, "delta") or {}
            piece = _get(delta, "content")
            if piece:
                if ttft is None:
                    ttft = time.perf_counter() - t0
                text.append(piece)
            for tc in _get(delta, "tool_calls") or []:
                if ttft is None:
                    ttft = time.perf_counter() - t0
                idx = _get(tc, "index", len(calls)) or 0
                slot = calls.setdefault(idx, {"id": None, "name": "", "arguments": ""})
                if _get(tc, "id"):
                    slot["id"] = _get(tc, "id")
                fn = _get(tc, "function") or {}
                if _get(fn, "name"):
                    slot["name"] += _get(fn, "name")
                if _get(fn, "arguments"):
                    slot["arguments"] += _get(fn, "arguments")
            if _get(choice, "finish_reason"):
                finish = _get(choice, "finish_reason")
    return {
        "text": "".join(text),
        "tool_calls": [calls[i] for i in sorted(calls)],
        "finish_reason": finish,
        "usage": usage,
        "ttft_s": ttft,
    }


def _provider_name_from_url(base_url: str | None) -> str:
    if not base_url:
        return "openai"
    host = urlparse(base_url).hostname or base_url
    for known in ("baseten", "together", "groq", "fireworks", "openrouter", "dashscope",
                  "huawei", "myhuaweicloud", "localhost", "127.0.0.1"):
        if known in host:
            return {"myhuaweicloud": "huawei", "127.0.0.1": "localhost"}.get(known, known)
    return host


class OpenAICompatibleProvider:
    """Any Chat Completions endpoint: OpenAI, Baseten, vLLM, a Qwen relay.

    Config from args or env: PLANNER_BASE_URL (e.g. https://inference.baseten.co/v1),
    PLANNER_API_KEY (falls back to OPENAI_API_KEY), PLANNER_MODEL (required),
    PLANNER_STREAM=1 for endpoints that only stream, PLANNER_EXTRA_BODY='{...}'
    for vendor-specific request fields. `name` defaults to the
    endpoint's host so latency tables read "baseten" rather than "openai".
    """

    tool_format = "openai"

    def __init__(
        self,
        model: str | None = None,
        base_url: str | None = None,
        api_key: str | None = None,
        name: str | None = None,
        max_tokens: int = 1024,
        stream: bool | None = None,
        temperature: float | None = None,
        timeout_s: float = 45.0,
        max_retries: int = 1,
        extra_body: dict[str, Any] | None = None,
        client: Any = None,
    ) -> None:
        self.model = model or os.environ.get("PLANNER_MODEL") or ""
        if not self.model:
            raise ValueError(
                "OpenAI-compatible provider needs a model: pass model= or set PLANNER_MODEL."
            )
        self.base_url = base_url or os.environ.get("PLANNER_BASE_URL") or None
        self.api_key = (
            api_key or os.environ.get("PLANNER_API_KEY") or os.environ.get("OPENAI_API_KEY")
        )
        self.name = name or _provider_name_from_url(self.base_url)
        self.max_tokens = max_tokens
        if stream is None:
            stream = bool(_env_flag("PLANNER_STREAM"))
        self.stream = stream
        self.temperature = temperature
        self.timeout_s = timeout_s
        self.max_retries = max_retries
        if extra_body is None and os.environ.get("PLANNER_EXTRA_BODY"):
            # e.g. '{"modalities": ["text"]}' for an omni model behind a relay
            extra_body = json.loads(os.environ["PLANNER_EXTRA_BODY"])
        self.extra_body = extra_body
        # Official OpenAI reasoning models reject max_tokens; most compatible
        # servers (vLLM, Baseten) only know max_tokens.
        official = self.base_url is None or "api.openai.com" in self.base_url
        self.max_tokens_param = "max_completion_tokens" if official else "max_tokens"
        self._client = client

    @property
    def client(self) -> Any:
        if self._client is None:
            import openai

            self._client = openai.OpenAI(
                base_url=self.base_url,
                api_key=self.api_key or "missing-key",
                timeout=self.timeout_s,
                max_retries=self.max_retries,
            )
        return self._client

    def build_request(
        self, system: str, messages: list[dict[str, Any]], tools: list[dict[str, Any]]
    ) -> dict[str, Any]:
        req: dict[str, Any] = {
            "model": self.model,
            "messages": to_openai_messages(system, messages),
            self.max_tokens_param: self.max_tokens,
        }
        if tools:
            req["tools"] = tools
            req["tool_choice"] = "auto"
        if self.temperature is not None:
            req["temperature"] = self.temperature
        if self.stream:
            req["stream"] = True
            req["stream_options"] = {"include_usage": True}
        if self.extra_body:
            req["extra_body"] = dict(self.extra_body)
        return req

    def complete(
        self, system: str, messages: list[dict[str, Any]], tools: list[dict[str, Any]]
    ) -> PlannerTurn:
        req = self.build_request(system, messages, tools)
        t0 = time.perf_counter()
        resp = self.client.chat.completions.create(**req)
        if self.stream:
            acc = accumulate_openai_stream(resp)
            return self._turn(
                acc["text"], acc["tool_calls"], acc["finish_reason"], acc["usage"],
                time.perf_counter() - t0, ttft=acc["ttft_s"],
            )
        latency = time.perf_counter() - t0
        return self.parse_response(resp, latency)

    def parse_response(self, resp: Any, latency_s: float) -> PlannerTurn:
        choices = _get(resp, "choices") or []
        if not choices:
            return self._turn("", [], "end_turn", _get(resp, "usage"), latency_s)
        choice = choices[0]
        msg = _get(choice, "message") or {}
        raw_calls = [
            {
                "id": _get(tc, "id"),
                "name": _get(_get(tc, "function"), "name", ""),
                "arguments": _get(_get(tc, "function"), "arguments", ""),
            }
            for tc in (_get(msg, "tool_calls") or [])
        ]
        return self._turn(
            _get(msg, "content") or "", raw_calls, _get(choice, "finish_reason"),
            _get(resp, "usage"), latency_s, model=_get(resp, "model"),
        )

    def _turn(
        self, text: str, raw_calls: list[dict[str, Any]], finish: str | None, usage: Any,
        latency_s: float, ttft: float | None = None, model: str | None = None,
    ) -> PlannerTurn:
        calls = [
            # Some compatible servers omit ids; the loop needs one to pair results.
            ToolCall(c.get("id") or f"call_{i}", c.get("name") or "",
                     _parse_args(c.get("arguments")))
            for i, c in enumerate(raw_calls)
        ]
        stop = _FINISH.get(finish or "stop", finish or "end_turn")
        if calls:
            stop = "tool_use"  # several servers report "stop" alongside tool calls
        return PlannerTurn(
            text=(text or "").strip(),
            tool_calls=calls,
            stop_reason=stop,
            latency_s=latency_s,
            provider=self.name,
            model=model or self.model,
            usage=_usage(usage),
            raw=None,  # rebuilt from the neutral form; nothing to preserve
            ttft_s=ttft,
        )


# ------------------------------------------------------------------ scripted


TurnSpec = Any  # PlannerTurn | str | dict | Callable[[list[dict]], TurnSpec]


class ScriptedProvider:
    """Replays predefined turns. For tests, and for demos with no API key.

    Each entry of `turns` is one of:
      * str                           final answer, no tool calls
      * {"text": ..., "tool_calls": [("goto", {"station": "desk"}), ...]}
      * PlannerTurn                   used as-is
      * callable(messages) -> any of the above, for turns that depend on
        earlier results

    `latency_s` is REPORTED, not slept (unless sleep=True), so tests stay
    instant while still exercising the latency plumbing.
    """

    tool_format = "anthropic"

    def __init__(
        self,
        turns: Iterable[TurnSpec],
        name: str = "scripted",
        model: str = "script",
        latency_s: float = 0.0,
        sleep: bool = False,
    ) -> None:
        self.name = name
        self.model = model
        self.turns = list(turns)
        self.latency_s = latency_s
        self.sleep = sleep
        self.requests: list[dict[str, Any]] = []
        self._i = 0
        self._ids = 0
        self._step: Callable[[list[dict[str, Any]]], TurnSpec] | None = None

    @property
    def exhausted(self) -> bool:
        return self._step is None and self._i >= len(self.turns)

    def complete(
        self, system: str, messages: list[dict[str, Any]], tools: list[dict[str, Any]]
    ) -> PlannerTurn:
        t0 = time.perf_counter()
        self.requests.append({"system": system, "messages": list(messages), "tools": tools})
        if self._step is not None:
            spec: TurnSpec = self._step
        elif self.exhausted:
            spec = "I've reached the end of my script."
        else:
            spec = self.turns[self._i]
            self._i += 1
        turn = self._coerce(spec, messages)
        if self.sleep and self.latency_s:
            time.sleep(self.latency_s)
        turn.latency_s = (time.perf_counter() - t0) + (0.0 if self.sleep else self.latency_s)
        turn.provider = turn.provider or self.name
        turn.model = turn.model or self.model
        return turn

    def _coerce(self, spec: TurnSpec, messages: list[dict[str, Any]]) -> PlannerTurn:
        for _ in range(8):  # callables may return callables; don't loop forever
            if callable(spec) and not isinstance(spec, PlannerTurn):
                spec = spec(messages)
            else:
                break
        if isinstance(spec, PlannerTurn):
            return spec
        if isinstance(spec, str):
            return PlannerTurn(text=spec, stop_reason="end_turn")
        if isinstance(spec, dict):
            calls = [self._call(c) for c in spec.get("tool_calls", [])]
            return PlannerTurn(
                text=spec.get("text", ""),
                tool_calls=calls,
                stop_reason="tool_use" if calls else "end_turn",
            )
        raise TypeError(f"unusable scripted turn: {spec!r}")

    def _call(self, c: Any) -> ToolCall:
        self._ids += 1
        if isinstance(c, ToolCall):
            return c
        if isinstance(c, dict):
            return ToolCall(c.get("id") or f"call_{self._ids}", c["name"], c.get("args", {}))
        if len(c) == 3:
            return ToolCall(*c)
        name, args = c
        return ToolCall(f"call_{self._ids}", name, args)

    @classmethod
    def from_generator(cls, genfn: Callable[[], Any], **kw: Any) -> "ScriptedProvider":
        """A script that reacts to results, written as a generator:

            def fetch():
                r = yield ("recall", {"label": "keys"})
                if not r["ok"]:
                    return "I haven't seen them."
                ...

        Each `yield` is one model turn: a (name, args) tuple, a list of them,
        or a str final answer. The value sent back is that turn's tool result
        (parsed JSON), or a list of results if a list was yielded. The
        generator's return value is the final answer.
        """
        if not inspect.isgeneratorfunction(genfn):
            raise TypeError("from_generator needs a generator function")
        state: dict[str, Any] = {"gen": None, "last": None}

        def step(messages: list[dict[str, Any]]) -> TurnSpec:
            try:
                if state["gen"] is None:
                    state["gen"] = genfn()
                    out = next(state["gen"])
                else:
                    results = last_tool_results(messages)
                    sent = results if isinstance(state["last"], list) else (
                        results[0] if results else None
                    )
                    out = state["gen"].send(sent)
            except StopIteration as stop:
                return stop.value or "Done."
            state["last"] = out
            if isinstance(out, str):
                return out
            calls = out if isinstance(out, list) else [out]
            return {"tool_calls": calls}

        # One callable, consulted for every turn until the generator finishes.
        provider = cls([], **kw)
        provider._step = step
        return provider


def last_tool_results(messages: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """The parsed tool results the model is about to read (the last message),
    or [] if the last message is not a tool-results message."""
    if not messages or messages[-1].get("role") != "tool":
        return []
    out = []
    for r in messages[-1]["results"]:
        try:
            out.append(json.loads(r["content"]))
        except (TypeError, json.JSONDecodeError):
            out.append({"ok": False, "detail": str(r.get("content")), "data": {}})
    return out
