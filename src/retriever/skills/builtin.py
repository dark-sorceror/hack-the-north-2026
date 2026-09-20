"""The built-in skills, written once and shared by every product niche.

Fetching someone's keys and sorting litter into bins are the same robot doing
the same motions: look, drive, line up, grab, carry, let go. What differs is
which object is wanted and where it goes — and that lives in the planner's
niche config, not here. If a skill needs to know which niche it is in, the
abstraction is wrong.

Every skill:

  * returns a Result whose `detail` is a sentence the robot can say,
  * gives up on a timeout instead of hanging (the loops in navigation/drive.py
    carry the timeouts; the gripper loop here has its own),
  * reports what it SENSED, not what it commanded. `pick` reads gripper load
    after closing, so "I closed on nothing" is a real answer rather than a
    mimed success.

Everything the skills touch comes in through `RobotContext`, so the same code
runs on FakeRobot, the tank base, or the arm, and tests can hand in fakes.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from typing import Any, Callable, Protocol, runtime_checkable

from retriever.memory import Memory, Sighting
from retriever.navigation.avoid import AvoidConfig
from retriever.navigation.approach import AvoidingApproachController, run_approach
from retriever.navigation.drive import (
    AvoidingGotoController,
    DifferentialGotoController,
    Limits,
    run_goto,
)
from retriever.navigation.geometry import distance, target_offset, world_to_base
from retriever.navigation.localization import compose
from retriever.skills.library import Skill, SkillLibrary
from retriever.types import Action, Observation, Pose, Result, Station, Target

# ------------------------------------------------------------------ protocols

Perceive = Callable[[Observation], "list[Target]"]
"""Everything perception can see in this frame. One callable serves both
look_around (wants all of it) and approach (filters to one label), so the
detector is only wired up in one place."""


@runtime_checkable
class Classifier(Protocol):
    """Decides what kind of thing an object is.

    Returns (category, confidence). "uncertain" is a legitimate category, and
    the one to return when unsure: the sort niche has a safe default for it
    (landfill), whereas a confident wrong "recycling" contaminates the bin.
    """

    def classify(self, label: str, image: bytes | None = None) -> tuple[str, float]: ...


# ------------------------------------------------------------------ context


def _silent(_: str) -> None:
    pass


@dataclass
class RobotContext:
    """Everything a skill needs, passed in rather than imported.

    `ask` is for use OUTSIDE the planner (a CLI, a test). Inside the planner
    loop, ask_user never reaches this: the loop pauses and returns the question
    to its caller instead, because blocking on stdin inside a control loop
    freezes the dashboard and the voice pipeline with it.
    """

    backend: Any
    stations: list[Station]
    memory: Memory
    perceive: Perceive
    say: Callable[[str], None] = _silent
    ask: Callable[[str], "str | None"] | None = None
    classifier: Classifier | None = None

    user_pose: Pose = field(default_factory=Pose)   # where "deliver to user" drives
    limits: Limits = field(default_factory=Limits)
    # Tank base by default: it cannot strafe, so it steers instead.
    make_controller: Callable[[Limits], Any] = DifferentialGotoController
    on_tick: Callable[..., None] | None = None
    # Lidar obstacle avoidance (navigation/avoid.py). None: drive exactly as
    # before. A callable returning the latest base-frame Scan: goto, deliver and
    # approach steer round what the lidar sees (never round their own target).
    scan_source: Callable[[], Any] | None = None
    avoid_config: AvoidConfig = field(default_factory=AvoidConfig)

    approach_standoff_m: float = 0.25   # where approach stops: in view, not touching
    grasp_standoff_m: float = 0.04      # where pick stops: object between the jaws
    reach_height_m: float = 0.50        # anything higher is honestly out of reach
    goto_timeout_s: float = 25.0
    approach_timeout_s: float = 15.0
    gripper_timeout_s: float = 2.0
    look_steps: int = 8                 # 45 degrees per look

    tools: tuple[str, ...] = ("claw", "suction")
    gripper_joint: str = "gripper"
    gripper_open: float = 1.0
    gripper_closed: float = 0.0
    contact_load: float = 0.30          # normalised current above this = holding something
    classify_threshold: float = 0.60    # below this, classify reports "uncertain"

    held: str | None = None             # what we BELIEVE we hold; load is the truth

    # -- helpers shared by the skills ----------------------------------------

    def station(self, name: str) -> Station | None:
        key = _norm(name)
        for s in self.stations:
            if _norm(s.name) == key:
                return s
        return None

    def nearest_station(self, x: float, y: float) -> tuple[Station | None, float]:
        best, best_d = None, math.inf
        for s in self.stations:
            d = math.hypot(s.pose.x - x, s.pose.y - y)
            if d < best_d:
                best, best_d = s, d
        return best, best_d

    def see(self, obs: Observation) -> list[Target]:
        """Perception, with a crash treated as seeing nothing this frame.
        run_approach already coasts through missed frames; a raising detector
        should degrade the same way instead of aborting the skill."""
        try:
            return list(self.perceive(obs) or [])
        except Exception:
            return []

    def holding(self, obs: Observation) -> bool:
        return obs.gripper_load >= self.contact_load

    def goto_controller(self) -> Any:
        """make_controller as before, or the avoiding one when there is a lidar."""
        if self.scan_source is None:
            return self.make_controller(self.limits)
        return AvoidingGotoController(self.scan_source, self.limits, self.avoid_config)

    def approach_controller(self) -> Any:
        """None (run_approach's own controller, as before) without a lidar."""
        if self.scan_source is None:
            return None
        return AvoidingApproachController(self.scan_source, self.approach_standoff_m,
                                          self.limits, self.avoid_config)


# ------------------------------------------------------------------ helpers

_ARTICLES = re.compile(r"^(my|your|the|a|an|our|their|his|her|some)\s+")


def _norm(label: str) -> str:
    """'My Keys ' -> 'keys'. Users say 'my keys'; perception says 'keys'."""
    s = " ".join(str(label).lower().split())
    while True:
        t = _ARTICLES.sub("", s)
        if t == s:
            return s
        s = t


def _matches(query: str, label: str) -> bool:
    q, l = _norm(query), _norm(label)
    return bool(q) and (q == l or q.rstrip("s") == l.rstrip("s"))


def _ago(seconds: float) -> str:
    s = max(0.0, seconds)
    if s < 90:
        return "just now" if s < 20 else f"about {int(s)} seconds ago"
    if s < 90 * 60:
        return f"about {round(s / 60)} minutes ago"
    return f"about {round(s / 3600)} hours ago"


def _stop(ctx: RobotContext) -> None:
    """Command zero velocity. On FakeRobot this is a no-op tick; on a real base
    it is what stops the last velocity command persisting after a skill ends."""
    try:
        ctx.backend.act(Action())
    except Exception:
        pass


WHICH = ("nearest", "left", "right")

# Further than this from every station, "near the kitchen" stops being true
# and recall gives coordinates instead of a misleading landmark.
NEAR_M = 1.2


def _find(
    ctx: RobotContext, label: str, obs: Observation, which: str | None = None
) -> tuple[Target | None, list[Target]]:
    """The best visible match for `label`, plus every candidate.

    Returns (None, candidates) when there are several and nothing says which
    one is meant. A taught-instance match (instance_id set) on exactly one
    candidate settles it — that is the whole point of teaching the robot your
    keys. Otherwise the caller has to ask, not guess.
    """
    # A fused detection can carry two labels -- COCO says "remote", open-vocab
    # says "inhaler". Asking for either must find it.
    cands = [
        t for t in ctx.see(obs)
        if any(_matches(label, lab) for lab in (getattr(t, "labels", None) or (t.label,)))
    ]
    if not cands:
        return None, []
    if len(cands) == 1:
        return cands[0], cands
    if which == "left":
        return max(cands, key=lambda t: t.bearing_rad), cands
    if which == "right":
        return min(cands, key=lambda t: t.bearing_rad), cands
    if which == "nearest":
        # An estimated range (no depth) can be off by 2x; never let it beat a
        # measured one. Plain Targets count as measured.
        measured = [t for t in cands if getattr(t, "range_measured", True)]
        return min(measured or cands, key=lambda t: t.range_m), cands
    known = [t for t in cands if t.instance_id]
    if len(known) == 1:
        return known[0], cands
    return None, cands


_COUNT = {2: "two", 3: "three", 4: "four", 5: "five"}


def _ambiguous(label: str, cands: list[Target]) -> Result:
    """Describe lookalikes relative to EACH OTHER (left/right), because that is
    what `which` understands and what a person standing behind the robot sees."""
    ordered = sorted(cands, key=lambda t: -t.bearing_rad)   # leftmost first
    rel = ["on the left"] + ["in the middle"] * (len(ordered) - 2) + ["on the right"]
    name = _norm(label)
    plural = name if name.endswith("s") else name + "s"
    where = ", ".join(f"one {r}" for r in rel)
    return Result.failed(
        f"I can see {_COUNT.get(len(cands), len(cands))} {plural} — {where} — and I can't "
        "tell which one is yours.",
        ambiguous=True,
        candidates=[
            {"label": t.label, "position": r, "range_m": t.range_m,
             "bearing_rad": t.bearing_rad, "confidence": t.confidence,
             "instance_id": t.instance_id}
            for t, r in zip(ordered, rel)
        ],
    )


def _world(obs: Observation, t: Target) -> Pose:
    fwd, left = target_offset(t)
    p = compose(obs.base, Pose(fwd, left, 0.0))
    return Pose(p.x, p.y, 0.0)


def _set_gripper(ctx: RobotContext, value: float) -> Observation:
    """Drive the gripper toward `value` until it gets there, stalls on
    something, or times out. A stall is the expected outcome of a good grasp,
    so hitting the timeout while closing is not itself a failure."""
    obs = ctx.backend.observe()
    t0 = obs.t
    closing = value < obs.joints.get(ctx.gripper_joint, value)
    for _ in range(10_000):
        ctx.backend.act(Action(joints={ctx.gripper_joint: value}))
        obs = ctx.backend.observe()
        if abs(obs.joints.get(ctx.gripper_joint, value) - value) < 0.02:
            break
        if closing and ctx.holding(obs):
            break
        if obs.t - t0 > ctx.gripper_timeout_s:
            break
    # A few settle ticks so the load reading reflects the final squeeze.
    for _ in range(5):
        ctx.backend.act(Action(joints={ctx.gripper_joint: value}))
    return ctx.backend.observe()


def _sighting_json(s: Sighting, ctx: RobotContext) -> dict[str, Any]:
    near, d = ctx.nearest_station(s.pose.x, s.pose.y)
    return {
        "label": s.label,
        "x": s.pose.x,
        "y": s.pose.y,
        "near_station": near.name if near else None,
        "distance_to_station_m": d if near else None,
        "age_s": s.age_s(),
        "confidence": s.confidence,
    }


# ------------------------------------------------------------------ skills


def look_around(ctx: RobotContext) -> Result:
    """Turn a full circle on the spot, writing every sighting into Memory."""
    step = 2 * math.pi / max(1, ctx.look_steps)
    seen: dict[str, dict[str, Any]] = {}
    obs = ctx.backend.observe()
    turned_ok = True
    try:
        for i in range(max(1, ctx.look_steps)):
            for t in ctx.see(obs):
                w = _world(obs, t)
                ctx.memory.saw(t.label, w, t.confidence, object_id=t.instance_id)
                prev = seen.get(t.label)
                if prev is None or t.confidence > prev["confidence"]:
                    near, _ = ctx.nearest_station(w.x, w.y)
                    seen[t.label] = {
                        "label": t.label, "x": w.x, "y": w.y, "confidence": t.confidence,
                        "near_station": near.name if near else None,
                    }
            if i == ctx.look_steps - 1:
                break
            goal = Pose(obs.base.x, obs.base.y, obs.base.theta + step)
            r = run_goto(
                ctx.backend, goal, ctx.limits, timeout_s=ctx.goto_timeout_s,
                on_tick=ctx.on_tick, controller=ctx.make_controller(ctx.limits),
            )
            obs = ctx.backend.observe()
            if not r.ok:
                turned_ok = False
                break
    finally:
        _stop(ctx)

    labels = list(seen)
    if not labels:
        detail = "I looked all the way around and didn't see anything I recognise."
    elif len(labels) == 1:
        detail = f"I can see the {labels[0]}."
    else:
        detail = f"I can see the {', the '.join(labels[:-1])} and the {labels[-1]}."
    if not turned_ok:
        detail += " I couldn't finish turning all the way round, though."
    return Result(
        ok=turned_ok or bool(labels),
        confidence=1.0 if turned_ok else 0.5,
        detail=detail,
        data={"seen": list(seen.values()), "full_turn": turned_ok},
    )


def goto(ctx: RobotContext, station: str) -> Result:
    st = ctx.station(station)
    if st is None:
        known = ", ".join(s.name for s in ctx.stations) or "nowhere yet"
        return Result.failed(
            f"I don't know a place called {station}. I know: {known}.",
            known_stations=[s.name for s in ctx.stations],
        )
    try:
        r = run_goto(
            ctx.backend, st.pose, ctx.limits, timeout_s=ctx.goto_timeout_s,
            on_tick=ctx.on_tick, controller=ctx.goto_controller(),
        )
    finally:
        _stop(ctx)
    if r.ok:
        return Result(ok=True, confidence=r.confidence, detail=f"I'm at the {st.name}.",
                      data={"station": st.name, **r.data})
    if r.data.get("blocked"):  # only the lidar can say this; it says why
        return Result.failed(f"I couldn't get to the {st.name}. {r.detail}",
                             station=st.name, **r.data)
    residual = r.data.get("residual_m", 0.0)
    return Result.failed(
        f"I couldn't get to the {st.name} — I stopped about {residual:.1f} m short.",
        station=st.name, **r.data,
    )


def approach(ctx: RobotContext, label: str, which: str | None = None) -> Result:
    obs = ctx.backend.observe()
    best, cands = _find(ctx, label, obs, which)
    if best is None and cands:
        return _ambiguous(label, cands)
    if best is None:
        s = ctx.memory.last_seen(label) or ctx.memory.last_seen(_norm(label))
        hint = ""
        if s is not None:
            near, _ = ctx.nearest_station(s.pose.x, s.pose.y)
            if near is not None:
                hint = f" I last saw it near the {near.name}."
        return Result.failed(f"I can't see the {_norm(label)} from here.{hint}", visible=False)

    def track(o: Observation) -> Target | None:
        t, _ = _find(ctx, label, o, which or ("nearest" if len(cands) > 1 else None))
        return t

    try:
        r = run_approach(
            ctx.backend, track, standoff_m=ctx.approach_standoff_m, limits=ctx.limits,
            timeout_s=ctx.approach_timeout_s, on_tick=ctx.on_tick,
            controller=ctx.approach_controller(),
        )
    finally:
        _stop(ctx)
    if not r.ok:
        return Result(ok=False, confidence=0.0, detail=r.detail, data={"label": label, **r.data})
    obs = ctx.backend.observe()
    t = track(obs)
    if t is not None:
        ctx.memory.saw(t.label, _world(obs, t), t.confidence, object_id=t.instance_id)
    return Result(ok=True, confidence=r.confidence,
                  detail=f"I'm lined up on the {_norm(label)}.", data={"label": label, **r.data})


def pick(ctx: RobotContext, label: str, tool: str = "claw", which: str | None = None) -> Result:
    """Grab something that is already in front of the robot, and say honestly
    whether anything ended up in the gripper."""
    part = "suction cup" if tool == "suction" else "claw"
    name = _norm(label)
    obs = ctx.backend.observe()
    if ctx.holding(obs):
        return Result.failed(
            f"My {part} is already holding something"
            + (f" — the {ctx.held}" if ctx.held else "") + ". I need to put it down first.",
            holding=True, gripper_load=obs.gripper_load,
        )

    best, cands = _find(ctx, label, obs, which)
    if best is None and cands:
        return _ambiguous(label, cands)
    if best is None:
        return Result.failed(
            f"I can't see the {name} in front of me. I need to get closer first.",
            visible=False,
        )
    if best.height_m is not None and best.height_m > ctx.reach_height_m:
        return Result.failed(
            f"I can see the {name}, but it's about {best.height_m * 100:.0f} cm up — "
            "that's above what I can reach.",
            height_m=best.height_m, reachable=False,
        )

    chosen = which or ("nearest" if len(cands) > 1 else None)
    try:
        _set_gripper(ctx, ctx.gripper_open)
        creep = run_approach(
            ctx.backend, lambda o: _find(ctx, label, o, chosen)[0],
            standoff_m=ctx.grasp_standoff_m, limits=ctx.limits,
            timeout_s=ctx.approach_timeout_s, on_tick=ctx.on_tick,
        )
        if not creep.ok:
            return Result.failed(
                f"I couldn't get the {part} around the {name}: {creep.detail}",
                tool=tool, holding=False, **creep.data,
            )
        obs = _set_gripper(ctx, ctx.gripper_closed)
    finally:
        _stop(ctx)

    load = obs.gripper_load
    if ctx.holding(obs):
        ctx.held = name
        felt = ("the suction cup has a seal" if tool == "suction"
                else "I can feel the weight in the claw")
        return Result(
            ok=True,
            confidence=max(0.0, min(1.0, best.confidence)),
            detail=f"Got the {name} — {felt}.",
            data={"label": label, "tool": tool, "gripper_load": load, "holding": True},
        )

    # Missed. Open again so a retry starts from a known state, and say what the
    # sensor said rather than what we hoped.
    _set_gripper(ctx, ctx.gripper_open)
    ctx.held = None
    missed = (
        f"I tried the suction cup but it didn't seal on the {name}."
        if tool == "suction"
        else f"I closed the claw but felt nothing — I missed the {name}."
    )
    return Result.failed(missed, tool=tool, gripper_load=load, holding=False)


def deliver(ctx: RobotContext, destination: str) -> Result:
    """Carry what is in the gripper to a station (or to the user) and let go."""
    obs = ctx.backend.observe()
    item = ctx.held or "it"
    if not ctx.holding(obs):
        ctx.held = None
        return Result.failed("I'm not holding anything to deliver.", holding=False)

    to_user = _norm(destination) in ("user", "me", "you", "person", "owner")
    if to_user:
        goal, where = ctx.user_pose, "you"
    else:
        st = ctx.station(destination)
        if st is None:
            known = ", ".join(s.name for s in ctx.stations)
            return Result.failed(
                f"I don't know where {destination} is. I can deliver to you or to: {known}.",
                known_stations=[s.name for s in ctx.stations],
            )
        goal, where = st.pose, f"the {st.name}"

    try:
        r = run_goto(
            ctx.backend, goal, ctx.limits, timeout_s=ctx.goto_timeout_s,
            on_tick=ctx.on_tick, controller=ctx.goto_controller(),
        )
    finally:
        _stop(ctx)
    if not r.ok and r.data.get("blocked"):  # only the lidar can say this; it says why
        return Result.failed(f"I couldn't get to {where}. {r.detail} I'm still holding the {item}.",
                             holding=True, **r.data)
    if not r.ok:
        residual = r.data.get("residual_m", 0.0)
        return Result.failed(
            f"I couldn't get to {where} — I stopped about {residual:.1f} m short. "
            f"I'm still holding the {item}.",
            holding=True, **r.data,
        )

    obs = ctx.backend.observe()
    if not ctx.holding(obs):
        ctx.held = None
        return Result.failed(
            f"I got to {where}, but the {item} isn't in my gripper any more — "
            "I must have dropped it on the way.",
            holding=False,
        )

    obs = _set_gripper(ctx, ctx.gripper_open)
    for _ in range(3):
        _stop(ctx)
    released = not ctx.holding(obs)
    base = obs.base
    ctx.memory.saw(item, Pose(base.x, base.y, 0.0), 1.0)
    ctx.held = None if released else ctx.held
    if not released:
        return Result.failed(
            f"I opened up at {where} but the {item} seems stuck.",
            holding=True, gripper_load=obs.gripper_load,
        )
    detail = (
        f"Here you go — your {item}." if to_user else f"I put the {item} in {where}."
    )
    return Result(ok=True, confidence=1.0, detail=detail,
                  data={"item": item, "destination": "user" if to_user else destination})


def recall(ctx: RobotContext, label: str) -> Result:
    """Answer from memory. Never moves the robot — that is the point: 'where
    are my keys?' should be answered in a second, not after a lap of the room."""
    s = ctx.memory.last_seen(label) or ctx.memory.last_seen(_norm(label))
    name = _norm(label)
    if s is None:
        return Result.failed(f"I haven't seen the {name} yet.", found=False, label=label)
    info = _sighting_json(s, ctx)
    near, d = info["near_station"], info["distance_to_station_m"]
    if distance(s.pose, ctx.user_pose) <= NEAR_M / 2:
        where = "right where you are"
    elif near and d is not None and d <= NEAR_M:
        where = f"near the {near}"
    else:
        where = f"at ({s.pose.x:.1f}, {s.pose.y:.1f}), not near any station"
    return Result(
        ok=True,
        confidence=max(0.0, min(1.0, s.confidence)),
        detail=f"I last saw the {name} {where}, {_ago(info['age_s'])}.",
        data={"found": True, **info},
    )


def classify(ctx: RobotContext, label: str) -> Result:
    if ctx.classifier is None:
        return Result.failed("I don't have a way to tell what things are made of yet.")
    obs = ctx.backend.observe()
    category, conf = ctx.classifier.classify(_norm(label), obs.cam_front)
    conf = max(0.0, min(1.0, float(conf)))
    name = _norm(label)
    if category == "uncertain" or conf < ctx.classify_threshold:
        return Result(
            ok=True,
            confidence=conf,
            detail=f"I'm not sure what the {name} is.",
            data={"label": label, "category": "uncertain", "guess": category, "confidence": conf},
        )
    if category == "belongings":
        detail = f"The {name} looks like someone's belongings, not litter."
    else:
        detail = f"The {name} looks like {category}."
    return Result(ok=True, confidence=conf, detail=detail,
                  data={"label": label, "category": category, "confidence": conf})


def refuse(ctx: RobotContext, label: str, reason: str) -> Result:
    """Decline out loud. A robot that silently skips an object looks broken; one
    that says why looks like it has judgment."""
    text = f"I'm leaving the {_norm(label)} where it is — {reason.rstrip('.')}."
    ctx.say(text)
    return Result(ok=True, confidence=1.0, detail=text,
                  data={"refused": label, "reason": reason})


def ask_user(ctx: RobotContext, question: str) -> Result:
    if ctx.ask is None:
        return Result.failed("There's nobody for me to ask right now.", question=question)
    answer = ctx.ask(question)
    if not answer or not str(answer).strip():
        return Result.failed("I asked, but nobody answered.", question=question)
    return Result(ok=True, confidence=1.0, detail=f"They said: {answer}",
                  data={"question": question, "answer": answer})


def say(ctx: RobotContext, text: str) -> Result:
    ctx.say(text)
    return Result(ok=True, confidence=1.0, detail=text)


# ------------------------------------------------------------------ assembly


def build_library(ctx: RobotContext) -> SkillLibrary:
    """Every built-in skill bound to one context.

    Descriptions are written for the model: when to use the skill and what it
    will NOT do, since the second is what stops a planner from calling pick on
    something across the room.
    """
    s: dict[str, Any] = {"type": "string"}
    lib = SkillLibrary()
    lib.register(Skill(
        "look_around",
        "Turn a full circle on the spot and report every object seen. Updates memory. "
        "Use when you don't know where something is, or to survey an area after arriving.",
        {"properties": {}},
        lambda: look_around(ctx),
    ))
    lib.register(Skill(
        "goto",
        "Drive to a named station. Only the stations listed in the robot state exist. "
        "Gets you near things; use approach to line up on a specific object.",
        {"properties": {"station": {**s, "description": "Station name"}},
         "required": ["station"]},
        lambda station: goto(ctx, station),
    ))
    lib.register(Skill(
        "approach",
        "Drive up to an object that is visible right now and stop just in front of it. "
        "Fails if the object is not in view — goto its station or look_around first. "
        "If several match, it fails and lists them; then ask_user, or pass `which`.",
        {"properties": {
            "label": {**s, "description": "What to approach, e.g. 'keys'"},
            "which": {**s, "enum": list(WHICH),
                      "description": "Tie-break when several match"},
        }, "required": ["label"]},
        lambda label, which=None: approach(ctx, label, which),
    ))
    lib.register(Skill(
        "pick",
        "Grab the named object once you have approached it. Reports honestly whether "
        "anything is actually in the gripper afterwards (it reads the gripper load).",
        {"properties": {
            "label": {**s, "description": "What to pick up"},
            "tool": {**s, "enum": list(ctx.tools),
                     "description": "End effector to use"},
            "which": {**s, "enum": list(WHICH),
                      "description": "Tie-break when several match"},
        }, "required": ["label"]},
        lambda label, tool=ctx.tools[0], which=None: pick(ctx, label, tool, which),
    ))
    lib.register(Skill(
        "deliver",
        "Carry whatever is in the gripper to a destination and release it. "
        "destination is 'user' (hand it to the person) or a station name.",
        {"properties": {"destination": {**s, "description": "'user' or a station name"}},
         "required": ["destination"]},
        lambda destination: deliver(ctx, destination),
    ))
    lib.register(Skill(
        "recall",
        "Say where an object was last seen, from memory, WITHOUT moving. "
        "Use first when asked where something is or before searching for it.",
        {"properties": {"label": {**s, "description": "What to look up, e.g. 'keys'"}},
         "required": ["label"]},
        lambda label: recall(ctx, label),
    ))
    lib.register(Skill(
        "classify",
        "Decide what kind of thing an object is: recycling, compost, landfill, or "
        "belongings. Returns 'uncertain' when it can't tell.",
        {"properties": {"label": {**s, "description": "The object to classify"}},
         "required": ["label"]},
        lambda label: classify(ctx, label),
    ))
    lib.register(Skill(
        "refuse",
        "Decline to handle an object and say why, out loud. Use for anything that "
        "should not be moved (someone else's belongings, unsafe, out of scope).",
        {"properties": {"label": s, "reason": {**s, "description": "Short, spoken reason"}},
         "required": ["label", "reason"]},
        lambda label, reason: refuse(ctx, label, reason),
    ))
    lib.register(Skill(
        "ask_user",
        "Ask the person a short clarifying question and wait for the answer. Use when "
        "two candidates look alike or the request is ambiguous, instead of guessing.",
        {"properties": {"question": s}, "required": ["question"]},
        lambda question: ask_user(ctx, question),
    ))
    lib.register(Skill(
        "say",
        "Say something out loud, e.g. a progress update. Keep it to one sentence.",
        {"properties": {"text": s}, "required": ["text"]},
        lambda text: say(ctx, text),
    ))
    return lib


def robot_state(ctx: RobotContext) -> str:
    """Compact, factual state for the planner's system prompt."""
    try:
        obs = ctx.backend.observe()
    except Exception as exc:  # the prompt must still build if the robot is down
        return f"Robot not responding ({type(exc).__name__})."
    near, d = ctx.nearest_station(obs.base.x, obs.base.y)
    if near is None:
        place = "no stations are mapped"
    elif d <= NEAR_M:
        place = f"near the {near.name} ({d:.1f} m)"
    else:
        place = f"not at any station (nearest: {near.name}, {d:.1f} m)"
    grip = (f"holding the {ctx.held or 'something'}" if ctx.holding(obs) else "empty")
    lines = [
        f"- Position: ({obs.base.x:.2f}, {obs.base.y:.2f}), heading "
        f"{math.degrees(obs.base.theta):.0f} deg, {place}",
        f"- Gripper: {grip} (load {obs.gripper_load:.2f})",
        f"- Battery: {obs.battery * 100:.0f}%",
        f"- Tools: {', '.join(ctx.tools)}",
        "- Stations: " + (", ".join(
            f"{s.name} ({s.pose.x:.1f}, {s.pose.y:.1f})" for s in ctx.stations) or "none"),
        f"- The user is at ({ctx.user_pose.x:.1f}, {ctx.user_pose.y:.1f})",
    ]
    return "\n".join(lines)


# ------------------------------------------------------------------ fakes


class FakeClassifier:
    """A lookup table standing in for a vision classifier.

    Unknown labels come back "uncertain" with zero confidence, which is the
    behaviour the real one must have too: not knowing is an answer.
    """

    def __init__(
        self,
        table: dict[str, tuple[str, float]] | None = None,
        default: tuple[str, float] = ("uncertain", 0.0),
    ) -> None:
        self.table = {_norm(k): v for k, v in (table or {}).items()}
        self.default = default
        self.calls: list[str] = []

    def classify(self, label: str, image: bytes | None = None) -> tuple[str, float]:
        self.calls.append(label)
        return self.table.get(_norm(label), self.default)


class SimulatedPerception:
    """Stands in for the detector against FakeRobot: reports the fake world's
    objects that fall inside a camera-shaped wedge, as bearing/range Targets.

    Reads `backend.objects` live, so an object the robot has dropped somewhere
    new is seen there, and skips whatever the robot is holding.
    """

    def __init__(
        self,
        backend: Any,
        fov_rad: float = math.radians(100),
        max_range_m: float = 3.0,
        heights: dict[str, float] | None = None,
        instances: dict[str, str] | None = None,
        confidence: float = 0.9,
    ) -> None:
        self.backend = backend
        self.fov_rad = fov_rad
        self.max_range_m = max_range_m
        self.heights = dict(heights or {})
        self.instances = dict(instances or {})
        self.confidence = confidence

    def __call__(self, obs: Observation) -> list[Target]:
        out: list[Target] = []
        held = getattr(self.backend, "holding", None)
        for name, where in dict(getattr(self.backend, "objects", {})).items():
            if name == held:
                continue
            fwd, left = world_to_base(where.x - obs.base.x, where.y - obs.base.y, obs.base.theta)
            rng = math.hypot(fwd, left)
            bearing = math.atan2(left, fwd)
            if rng > self.max_range_m or abs(bearing) > self.fov_rad / 2:
                continue
            out.append(Target(
                label=_base_label(name),
                bearing_rad=bearing,
                range_m=rng,
                confidence=self.confidence,
                instance_id=self.instances.get(name),
                height_m=self.heights.get(name, 0.0),
            ))
        return out


def _base_label(name: str) -> str:
    """FakeRobot object keys must be unique, so two mugs are 'mug#1', 'mug#2';
    a detector would call both 'mug'."""
    return name.split("#", 1)[0]


__all__ = [
    "Classifier",
    "FakeClassifier",
    "Perceive",
    "RobotContext",
    "SimulatedPerception",
    "build_library",
    "robot_state",
]
