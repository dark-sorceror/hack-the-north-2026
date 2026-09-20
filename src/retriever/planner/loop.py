"""The planner loop: model decides the next skill, the library runs it, repeat.

The model decides WHAT; skills decide HOW. This loop is the only place the two
meet, and it is written so that nothing the model or a skill does can take it
down: provider errors, bad tool calls and skill exceptions all end in a
PlanResult with a sentence the robot can say.

ask_user is the one skill the loop does not execute. Blocking on stdin (or a
microphone) inside the loop would freeze everything upstream of it — voice,
dashboard, the supervisor's safety checks. Instead the loop stops and returns
`needs_input` with the question; the caller gets the answer however it likes
(push-to-talk, a dashboard button, a test) and calls `resume(answer)`.

Every model call and skill call is emitted as a PlanEvent, with latency, so
the dashboard log and the provider latency comparison read the same stream.
"""

from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass, field
from typing import Any, Callable

from retriever.planner.niches import FETCH, NicheConfig
from retriever.planner.providers import (
    PlannerTurn,
    Provider,
    ToolCall,
    assistant_message,
    tool_results_message,
    user_message,
)
from retriever.skills.library import DISPATCH_ERRORS, SkillLibrary, result_to_json, to_jsonable
from retriever.types import Result

log = logging.getLogger("retriever.planner")

INTERRUPTING = ("ask_user",)


@dataclass(frozen=True)
class PlanEvent:
    """kind: goal | model | tool_call | tool_result | needs_input | answer |
    final | error | max_steps. `data` is plain JSON."""

    kind: str
    data: dict[str, Any]
    t: float = field(default_factory=time.time)


@dataclass
class PlanResult:
    status: str               # done | needs_input | max_steps | error
    text: str                 # what to say: the answer, the question, or an honest give-up
    steps: int = 0            # model calls in this leg (run or resume)
    question: str | None = None
    tool_calls: list[dict[str, Any]] = field(default_factory=list)
    model_calls: list[dict[str, Any]] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.status == "done"

    @property
    def needs_input(self) -> bool:
        return self.status == "needs_input"

    @property
    def model_latency_s(self) -> float:
        return sum(m["latency_s"] for m in self.model_calls)

    @property
    def skill_latency_s(self) -> float:
        return sum(c["elapsed_s"] for c in self.tool_calls)


@dataclass
class _Pending:
    """An ask_user that is waiting for an answer, plus the rest of its turn."""

    call: ToolCall
    question: str
    done: list[dict[str, Any]]        # results for calls before the question
    rest: list[ToolCall]              # calls after it, not yet run


class Planner:
    def __init__(
        self,
        provider: Provider,
        library: SkillLibrary,
        niche: NicheConfig = FETCH,
        memory: Any = None,
        state: Callable[[], str] | None = None,
        on_event: Callable[[PlanEvent], None] | None = None,
        interrupting: tuple[str, ...] = INTERRUPTING,
    ) -> None:
        self.provider = provider
        self.library = library
        self.niche = niche
        self.memory = memory
        self.state = state
        self.on_event = on_event
        self.interrupting = interrupting

        self.messages: list[dict[str, Any]] = []   # neutral, with provider raw content
        self.transcript: list[dict[str, Any]] = []  # JSON-safe record of the same
        self.system: str = ""
        self._pending: _Pending | None = None
        self._max_steps = 12
        self._result: PlanResult | None = None

    # -- prompt ---------------------------------------------------------------

    def system_prompt(self) -> str:
        """Niche config + robot state + memory.

        Built once per goal, not per step: the history is append-only and the
        prompt stays byte-stable across the loop, which is what lets a provider
        cache it. Fresh state arrives through tool results instead.
        """
        parts = [self.niche.system_prompt()]
        if self.state is not None:
            try:
                parts += ["", "## Robot state (at the start of this request)", self.state()]
            except Exception as exc:
                parts += ["", "## Robot state", f"Unavailable ({type(exc).__name__})."]
        if self.memory is not None:
            try:
                parts += ["", "## Memory: where things were last seen", self.memory.summary()]
            except Exception as exc:
                parts += ["", "## Memory", f"Unavailable ({type(exc).__name__})."]
        return "\n".join(parts)

    # -- public API -----------------------------------------------------------

    @property
    def pending_question(self) -> str | None:
        return self._pending.question if self._pending else None

    def run(self, goal: str, max_steps: int = 12) -> PlanResult:
        """Start a new goal. Any unanswered question from a previous goal is dropped."""
        self._pending = None
        self._max_steps = max_steps
        self.system = self.system_prompt()
        self.messages = [user_message(goal)]
        self.transcript = [{"role": "user", "content": goal, "t": time.time()}]
        self._result = PlanResult(status="running", text="")
        self._emit("goal", {"goal": goal, "niche": self.niche.name,
                            "provider": self.provider.name, "model": self.provider.model})
        return self._loop(max_steps)

    def resume(self, answer: str, max_steps: int | None = None) -> PlanResult:
        """Answer the pending ask_user and carry on. Gets a fresh step budget:
        the answer is new information, not a continuation of a stuck plan."""
        p = self._pending
        if p is None:
            return PlanResult(status="error", text="I wasn't waiting for an answer.")
        self._pending = None
        self._result = PlanResult(status="running", text="")
        answer = (answer or "").strip()
        if answer:
            r = Result(ok=True, confidence=1.0, detail=f"They said: {answer}",
                       data={"question": p.question, "answer": answer})
        else:
            r = Result.failed("I asked, but nobody answered.", question=p.question)
        self._emit("answer", {"id": p.call.id, "question": p.question, "answer": answer})
        results = list(p.done)
        results.append(self._result_entry(p.call, r, elapsed_s=0.0))
        results += self._run_calls(p.rest)
        if self._pending is not None:  # a second question later in the same turn
            self._commit(results, partial=True)
            return self._needs_input_result(0)
        self._commit(results)
        return self._loop(max_steps or self._max_steps)

    def transcript_json(self) -> str:
        return json.dumps(self.transcript, indent=2)

    # -- the loop -------------------------------------------------------------

    def _loop(self, max_steps: int) -> PlanResult:
        tools = self.library.to_tool_schemas(self.provider.tool_format)
        for step in range(1, max_steps + 1):
            try:
                turn = self.provider.complete(self.system, self.messages, tools)
            except Exception as exc:
                log.exception("provider %s failed", self.provider.name)
                self._emit("error", {"step": step, "where": "provider",
                                     "error": f"{type(exc).__name__}: {exc}"[:300]})
                return self._finish(
                    "error",
                    f"I couldn't reach my planner ({type(exc).__name__}), so I've stopped "
                    "where I am.",
                    step,
                )
            self._record_turn(step, turn)

            if not turn.tool_calls:
                text = turn.text.strip() or _fallback_text(turn.stop_reason)
                self._emit("final", {"step": step, "text": text,
                                     "stop_reason": turn.stop_reason})
                return self._finish("done", text, step)

            results = self._run_calls(turn.tool_calls)
            if self._pending is not None:
                self._pending.done = results
                return self._needs_input_result(step)
            self._commit(results)

        self._emit("max_steps", {"steps": max_steps})
        return self._finish(
            "max_steps",
            "I've run out of steps before finishing, so I've stopped. "
            "Tell me what to try next.",
            max_steps,
        )

    def _run_calls(self, calls: list[ToolCall]) -> list[dict[str, Any]]:
        """Run calls in order. Stops at an interrupting call and parks it (with
        the calls after it) in self._pending; returns results for the rest."""
        results: list[dict[str, Any]] = []
        for i, call in enumerate(calls):
            args = call.args
            if call.name in self.interrupting and isinstance(args, dict) and args.get("question"):
                self._pending = _Pending(call, str(args["question"]), [], list(calls[i + 1:]))
                return results
            self._emit("tool_call", {"id": call.id, "name": call.name,
                                     "args": to_jsonable(args)})
            try:
                rec = self.library.run(call.name, args)
                r, elapsed = rec.result, rec.elapsed_s
            except Exception as exc:  # library.run never raises; belt and braces
                r = Result.failed(f"Something went wrong running {call.name}.",
                                  error="exception", exception=type(exc).__name__)
                elapsed = 0.0
            results.append(self._result_entry(call, r, elapsed))
        return results

    def _result_entry(self, call: ToolCall, r: Result, elapsed_s: float) -> dict[str, Any]:
        payload = result_to_json(r)
        is_error = (r.data or {}).get("error") in DISPATCH_ERRORS
        entry = {
            "id": call.id,
            "name": call.name,
            "content": json.dumps(payload),
            "is_error": is_error,
        }
        record = {
            "id": call.id, "name": call.name, "args": to_jsonable(call.args),
            "ok": r.ok, "confidence": r.confidence, "detail": r.detail,
            "data": payload["data"], "elapsed_s": round(elapsed_s, 4),
        }
        assert self._result is not None
        self._result.tool_calls.append(record)
        self._emit("tool_result", record)
        return entry

    def _commit(self, results: list[dict[str, Any]], partial: bool = False) -> None:
        if partial:
            # Results for a turn that is still waiting on a question: hold them
            # so the eventual tool message carries every id from that turn.
            assert self._pending is not None
            self._pending.done = results
            return
        self.messages.append(tool_results_message(results))
        self.transcript.append({
            "role": "tool",
            "results": [
                {"id": e["id"], "name": e["name"], "result": json.loads(e["content"]),
                 "is_error": e["is_error"]}
                for e in results
            ],
            "t": time.time(),
        })

    def _record_turn(self, step: int, turn: PlannerTurn) -> None:
        self.messages.append(assistant_message(turn))
        calls = [{"id": c.id, "name": c.name, "args": to_jsonable(c.args)}
                 for c in turn.tool_calls]
        self.transcript.append({
            "role": "assistant", "content": turn.text, "tool_calls": calls,
            "stop_reason": turn.stop_reason, "provider": turn.provider,
            "model": turn.model, "latency_s": round(turn.latency_s, 4), "t": time.time(),
        })
        info = {
            "step": step, "provider": turn.provider or self.provider.name,
            "model": turn.model or self.provider.model,
            "latency_s": round(turn.latency_s, 4), "ttft_s": turn.ttft_s,
            "stop_reason": turn.stop_reason, "usage": dict(turn.usage),
            "text": turn.text, "tool_calls": calls,
        }
        assert self._result is not None
        self._result.model_calls.append(info)
        self._emit("model", info)

    def _needs_input_result(self, step: int) -> PlanResult:
        assert self._pending is not None
        q = self._pending.question
        self._emit("needs_input", {"id": self._pending.call.id, "question": q})
        return self._finish("needs_input", q, step, question=q)

    def _finish(
        self, status: str, text: str, steps: int, question: str | None = None
    ) -> PlanResult:
        r = self._result or PlanResult(status=status, text=text)
        r.status, r.text, r.steps, r.question = status, text, steps, question
        if status != "needs_input":
            self.transcript.append({"role": "final", "status": status, "text": text,
                                    "t": time.time()})
        return r

    def _emit(self, kind: str, data: dict[str, Any]) -> None:
        if self.on_event is None:
            return
        try:
            self.on_event(PlanEvent(kind, data))
        except Exception:  # a broken dashboard must not stop the robot
            log.exception("on_event hook raised")


def _fallback_text(stop_reason: str) -> str:
    if stop_reason == "max_tokens":
        return "I lost my train of thought there. Could you ask me again?"
    if stop_reason == "refusal":
        return "I can't help with that request."
    return "Done."


def format_event(ev: PlanEvent) -> str | None:
    """One human line per event, for a terminal or the dashboard log."""
    d = ev.data
    if ev.kind == "goal":
        return f'Heard: "{d["goal"]}"  [{d["provider"]} / {d["model"]}, {d["niche"]}]'
    if ev.kind == "model":
        calls = ", ".join(f"{c['name']}({_args(c['args'])})" for c in d["tool_calls"])
        what = calls or "final answer"
        return f"[plan {d['step']}] {d['provider']} {d['latency_s']:.2f}s -> {what}"
    if ev.kind == "tool_result":
        mark = "ok" if d["ok"] else "FAILED"
        return f"  {d['name']} {mark} ({d['elapsed_s']:.2f}s): {d['detail']}"
    if ev.kind == "needs_input":
        return f"  ? {d['question']}"
    if ev.kind == "answer":
        return f"  > {d['answer'] or '(no answer)'}"
    if ev.kind == "final":
        return f"Robot: {d['text']}"
    if ev.kind == "error":
        return f"  ! {d['where']} error: {d['error']}"
    if ev.kind == "max_steps":
        return f"  ! stopped after {d['steps']} planner steps"
    return None


def _args(a: Any) -> str:
    if isinstance(a, dict):
        return ", ".join(f"{k}={v!r}" for k, v in a.items())
    return repr(a)
