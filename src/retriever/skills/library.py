"""The typed action interface between a language model and the robot.

The model sees exactly the skills registered here and nothing else. That
constraint is what stops it inventing capabilities: it cannot "grab the keys
from the drawer" if no skill opens drawers.

Three properties matter more than features:

  * `dispatch` never raises. An unknown skill, bad arguments, or a skill that
    throws all come back as `Result.failed` with a sentence the robot can say.
    A planner loop that dies on a KeyError mid-demo is worse than one that says
    "I don't know how to do that".
  * Every call is timed and logged. Planner latency and skill latency are the
    two numbers the team compares across providers, so they are recorded at
    the one place every call passes through.
  * Results serialise to plain JSON, because the model reads them back as tool
    results. Poses, Targets and tuples inside `Result.data` are flattened here
    so no caller has to think about it.

Argument validation is deliberately a small subset of JSON Schema (required,
types, enums, no unknown keys). It is enough to catch what models actually get
wrong, and it keeps this module stdlib-only.
"""

from __future__ import annotations

import dataclasses
import json
import logging
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Callable, Iterable

from retriever.types import Result

log = logging.getLogger("retriever.skills")

# JSON Schema "type" -> Python types that satisfy it. bool is excluded from the
# numeric types on purpose: isinstance(True, int) is True in Python, and a
# model passing `true` for a distance is a bug worth catching.
_TYPES: dict[str, tuple[type, ...]] = {
    "string": (str,),
    "number": (int, float),
    "integer": (int,),
    "boolean": (bool,),
    "array": (list, tuple),
    "object": (dict,),
}

# Dispatch-level failures, as opposed to the world saying no. The planner uses
# this to mark a tool result as an error ("you called it wrong") versus a
# normal failed Result ("I missed the grasp").
DISPATCH_ERRORS = frozenset({"unknown_skill", "bad_args", "exception", "bad_return"})


@dataclass(frozen=True)
class Skill:
    """One thing the robot can do, described well enough for a model to pick it.

    `params` is a JSON Schema object: {"type": "object", "properties": {...},
    "required": [...]}. `fn` receives the validated arguments as keyword
    arguments and must return a Result.
    """

    name: str
    description: str
    params: dict[str, Any]
    fn: Callable[..., Result]

    def schema(self) -> dict[str, Any]:
        """The parameter schema, normalised so both APIs accept it."""
        p = dict(self.params) if self.params else {}
        p.setdefault("type", "object")
        p.setdefault("properties", {})
        return p


@dataclass(frozen=True)
class DispatchRecord:
    """What happened when a skill was called. Kept for the dashboard and for
    latency comparison; `result` is what goes back to the model."""

    name: str
    args: dict[str, Any]
    result: Result
    elapsed_s: float
    started_at: float

    def to_json(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "args": to_jsonable(self.args),
            "result": result_to_json(self.result),
            "elapsed_s": round(self.elapsed_s, 4),
            "started_at": self.started_at,
        }


# ------------------------------------------------------------------ JSON


def to_jsonable(obj: Any, _depth: int = 0) -> Any:
    """Flatten anything a skill might put in Result.data into plain JSON.

    Dataclasses (Pose, Target, Sighting) become dicts, tuples become lists,
    non-finite floats become None (json.dumps would emit NaN, which is not
    JSON and which some providers reject), and anything unknown becomes its
    repr rather than an exception mid-loop.
    """
    if _depth > 12:
        return repr(obj)
    if obj is None or isinstance(obj, (bool, int, str)):
        return obj
    if isinstance(obj, float):
        return round(obj, 4) if math.isfinite(obj) else None
    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        return {
            f.name: to_jsonable(getattr(obj, f.name), _depth + 1)
            for f in dataclasses.fields(obj)
        }
    if isinstance(obj, dict):
        return {str(k): to_jsonable(v, _depth + 1) for k, v in obj.items()}
    if isinstance(obj, (list, tuple, set, frozenset, deque)):
        return [to_jsonable(v, _depth + 1) for v in obj]
    if isinstance(obj, (bytes, bytearray)):
        return f"<{len(obj)} bytes>"
    return repr(obj)


def result_to_json(r: Result) -> dict[str, Any]:
    return {
        "ok": r.ok,
        "confidence": round(r.confidence, 3),
        "detail": r.detail,
        "data": to_jsonable(r.data or {}),
    }


# ------------------------------------------------------------------ validation


def validate_args(schema: dict[str, Any], args: dict[str, Any]) -> str | None:
    """Return a spoken-style complaint about `args`, or None if they are fine."""
    props: dict[str, Any] = schema.get("properties", {}) or {}
    for key in schema.get("required", []) or []:
        if key not in args or args[key] is None:
            return f"I need to know the {key.replace('_', ' ')}."

    if schema.get("additionalProperties", False) is False:
        unknown = sorted(k for k in args if k not in props)
        if unknown:
            return f"I don't take {', '.join(repr(k) for k in unknown)} for this."

    for key, value in args.items():
        spec = props.get(key)
        if not spec or value is None:
            continue
        want = spec.get("type")
        if want in _TYPES:
            ok = isinstance(value, _TYPES[want])
            if want in ("number", "integer") and isinstance(value, bool):
                ok = False
            if want == "number" and isinstance(value, float) and not math.isfinite(value):
                ok = False
            if not ok:
                return f"The {key.replace('_', ' ')} should be a {want}, not {value!r}."
        if "enum" in spec and value not in spec["enum"]:
            options = " or ".join(repr(o) for o in spec["enum"])
            return f"The {key.replace('_', ' ')} has to be {options}, not {value!r}."
    return None


# ------------------------------------------------------------------ library


class SkillLibrary:
    """Registry plus the single choke point every skill call passes through."""

    def __init__(self, skills: Iterable[Skill] = (), history: int = 500) -> None:
        self._skills: dict[str, Skill] = {}
        self.history: deque[DispatchRecord] = deque(maxlen=history)
        self.on_dispatch: Callable[[DispatchRecord], None] | None = None
        for s in skills:
            self.register(s)

    # -- registration -------------------------------------------------------

    def register(self, skill: Skill) -> Skill:
        """Add a skill. Re-registering a name replaces it, which is how a
        backend-specific pick swaps in for the default one."""
        if not skill.name.isidentifier():
            # Both APIs restrict tool names to [a-zA-Z0-9_-]; catching it here
            # beats a 400 from the provider during the demo.
            raise ValueError(f"skill name must be an identifier: {skill.name!r}")
        self._skills[skill.name] = skill
        return skill

    def skill(self, name: str, description: str, params: dict[str, Any] | None = None):
        """Decorator form of register()."""

        def wrap(fn: Callable[..., Result]) -> Callable[..., Result]:
            self.register(Skill(name, description, params or {}, fn))
            return fn

        return wrap

    def get(self, name: str) -> Skill | None:
        return self._skills.get(name)

    @property
    def names(self) -> list[str]:
        return list(self._skills)

    def __contains__(self, name: object) -> bool:
        return name in self._skills

    def __len__(self) -> int:
        return len(self._skills)

    # -- schemas ------------------------------------------------------------

    def to_tool_schemas(self, format: str = "anthropic") -> list[dict[str, Any]]:
        """Tool definitions in the shape each API expects.

        anthropic: [{"name", "description", "input_schema"}]
        openai:    [{"type": "function", "function": {"name", "description", "parameters"}}]

        Order is registration order, and stable, so a provider-side prompt
        cache keyed on the tool list is not invalidated between calls.
        """
        if format == "anthropic":
            return [
                {"name": s.name, "description": s.description, "input_schema": s.schema()}
                for s in self._skills.values()
            ]
        if format == "openai":
            return [
                {
                    "type": "function",
                    "function": {
                        "name": s.name,
                        "description": s.description,
                        "parameters": s.schema(),
                    },
                }
                for s in self._skills.values()
            ]
        raise ValueError(f"unknown tool schema format: {format!r}")

    # -- dispatch -----------------------------------------------------------

    def run(self, name: str, args: Any = None) -> DispatchRecord:
        """Validate, execute, time and log one skill call. Never raises."""
        started = time.time()
        t0 = time.perf_counter()
        clean_args: dict[str, Any] = {}
        try:
            result, clean_args = self._execute(name, args)
        except BaseException as exc:  # noqa: BLE001 — see below
            # KeyboardInterrupt still propagates: ctrl-c must stop the robot.
            if isinstance(exc, (KeyboardInterrupt, SystemExit)):
                raise
            log.exception("skill %s raised", name)
            result = Result.failed(
                f"Something went wrong while I was trying to {name.replace('_', ' ')}: "
                f"{_short(exc)}",
                error="exception",
                exception=type(exc).__name__,
            )
        rec = DispatchRecord(
            name=name,
            args=clean_args,
            result=result,
            elapsed_s=time.perf_counter() - t0,
            started_at=started,
        )
        self.history.append(rec)
        log.info(
            "skill %s(%s) -> %s in %.3fs: %s",
            name, json.dumps(to_jsonable(clean_args)), "ok" if result.ok else "FAILED",
            rec.elapsed_s, result.detail,
        )
        if self.on_dispatch is not None:
            try:
                self.on_dispatch(rec)
            except Exception:  # a broken dashboard hook must not fail the skill
                log.exception("on_dispatch hook raised")
        return rec

    def dispatch(self, name: str, args: Any = None) -> Result:
        return self.run(name, args).result

    def _execute(self, name: str, args: Any) -> tuple[Result, dict[str, Any]]:
        skill = self._skills.get(name)
        if skill is None:
            known = ", ".join(self._skills) or "nothing yet"
            return (
                Result.failed(
                    f"I don't know how to {name.replace('_', ' ')}. I can: {known}.",
                    error="unknown_skill",
                ),
                {},
            )

        if args is None:
            args = {}
        if isinstance(args, str):
            # OpenAI-compatible endpoints hand arguments over as a JSON string,
            # and smaller models sometimes emit one that does not parse.
            try:
                args = json.loads(args) if args.strip() else {}
            except json.JSONDecodeError:
                return (
                    Result.failed(
                        f"I couldn't read the instructions for {name}: they weren't valid JSON.",
                        error="bad_args",
                    ),
                    {},
                )
        if not isinstance(args, dict):
            return (
                Result.failed(
                    f"The instructions for {name} should be named arguments.",
                    error="bad_args",
                ),
                {},
            )

        problem = validate_args(skill.schema(), args)
        if problem:
            return Result.failed(problem, error="bad_args"), dict(args)

        out = skill.fn(**args)
        if not isinstance(out, Result):
            return (
                Result.failed(
                    f"The {name} skill gave me an answer I couldn't read.",
                    error="bad_return",
                    returned=type(out).__name__,
                ),
                dict(args),
            )
        return out, dict(args)


def _short(exc: BaseException, limit: int = 120) -> str:
    text = str(exc) or type(exc).__name__
    return text if len(text) <= limit else text[: limit - 1] + "…"
