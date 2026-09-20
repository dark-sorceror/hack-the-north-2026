"""Drive the skid-steer base to a pose: a pure control law, and the loop that runs it.

The control law (`DifferentialGotoController.step`) is a pure function of where the base
is and where it should be, so it is tested pose by pose with no robot, clock or loop, and
a subclass can change the law without touching the loop. `run_goto` owns everything with
side effects: observing, acting, giving up and stopping. It sends a zero Action on every
exit, exceptions included, because a loop that dies must not leave its last velocity
running. Time comes from `Observation.t`, the robot's own clock, never the wall clock, so
a run against TankFakeRobot takes milliseconds yet honours the same timeout as the floor.
Forward speed is scaled by cos(bearing) and floored at zero: it fades as the goal swings
to the side and stays zero while the goal is behind, so the base turns on the spot instead
of driving a long arc round, and it never reverses blind. That one term replaces a
turn-then-drive state machine, which could stall between its states.
"""

from __future__ import annotations

import logging
import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from retriever.backends.base import RobotBackend
from retriever.navigation.avoid import (
    BLOCKED,
    AvoidConfig,
    LocalPlanner,
    Plan,
    Scan,
    ScanTracker,
)
from retriever.navigation.geometry import distance, pose_error
from retriever.types import Action, Observation, Pose, Result, Target

logger = logging.getLogger(__name__)

# Called once per observation with the command sent in response to it. The Target is what
# perception is tracking, for loops that chase one; driving to a fixed pose passes None.
Tick = Callable[[Observation, Action, "Target | None"], None]


@dataclass(frozen=True)
class Limits:
    """Speed caps and arrival tolerances. Conservative on purpose: a base that creeps
    finishes the demo; one that lunges knocks things over."""

    v_max: float = 0.35  # m/s
    w_max: float = 1.2  # rad/s
    pos_tol: float = 0.05  # m
    ang_tol: float = 0.1  # rad

    def __post_init__(self) -> None:
        for name in ("v_max", "w_max", "pos_tol", "ang_tol"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"{name} must be positive and finite, got {value!r}")


class DifferentialGotoController:
    """Steers a base that cannot strafe to a world pose: turn toward it, drive, then face
    the goal's heading."""

    def __init__(
        self, limits: Limits = Limits(), kp_lin: float = 1.2, kp_ang: float = 2.2
    ) -> None:
        for name, gain in (("kp_lin", kp_lin), ("kp_ang", kp_ang)):
            if not (math.isfinite(gain) and gain > 0.0):
                raise ValueError(f"{name} must be positive and finite, got {gain!r}")
        self.limits = limits
        self.kp_lin = kp_lin
        self.kp_ang = kp_ang

    def step(self, base: Pose, goal: Pose) -> tuple[Action, bool]:
        """The command for a base at `base` driving to `goal`, and whether it has arrived.

        A pure function of the two poses. The action never carries base_vy, and it is the
        zero Action once arrived.
        """
        _check_finite(base, "base")
        _check_finite(goal, "goal")
        lim = self.limits
        forward, left, heading_error = pose_error(base, goal)
        remaining = distance(base, goal)
        if remaining <= lim.pos_tol:
            if abs(heading_error) < lim.ang_tol:
                return Action(), True
            return Action(base_wz=_clip(self.kp_ang * heading_error, lim.w_max)), False
        bearing = math.atan2(left, forward)
        # Equal to kp_lin * forward: full speed dead ahead, zero at the side and behind.
        speed = self.kp_lin * remaining * max(0.0, math.cos(bearing))
        return (
            Action(
                base_vx=min(speed, lim.v_max),
                base_wz=_clip(self.kp_ang * bearing, lim.w_max),
            ),
            False,
        )


def run_goto(
    backend: RobotBackend,
    goal: Pose,
    limits: Limits = Limits(),
    timeout_s: float = 25.0,
    max_ticks: int = 20000,
    on_tick: Tick | None = None,
    controller: DifferentialGotoController | None = None,
) -> Result:
    """Drive `backend` to `goal`; stop and say how far off it is if it cannot get there.

    Loops observe -> controller.step -> act until the controller reports arrival, until
    `timeout_s` of Observation.t has passed, or after `max_ticks` commands (the backstop
    for a backend whose clock does not advance). `limits` builds the default controller
    and is ignored when `controller` is given. A zero Action is sent before returning or
    raising.

    AvoidingGotoController steers round what the lidar sees. That one needs the whole
    observation (its clock dates the scan) and can give up by itself ("something is
    blocking the way"); plain controllers do neither, and run exactly as they always have.
    """
    try:
        result = _goto(backend, goal, limits, timeout_s, max_ticks, on_tick, controller)
    except BaseException:
        _stop_quietly(backend)
        raise
    backend.act(Action())  # a failure to stop is not something to swallow
    return result


def _goto(
    backend: RobotBackend,
    goal: Pose,
    limits: Limits,
    timeout_s: float,
    max_ticks: int,
    on_tick: Tick | None,
    controller: DifferentialGotoController | None,
) -> Result:
    if not timeout_s > 0.0:
        raise ValueError(f"timeout_s must be positive, got {timeout_s!r}")
    if max_ticks < 0:
        raise ValueError(f"max_ticks must not be negative, got {max_ticks!r}")
    if controller is None:
        controller = DifferentialGotoController(limits)
    step_observation = getattr(controller, "step_observation", None)
    start_t: float | None = None
    ticks = 0
    while True:
        obs = backend.observe()
        if start_t is None:
            start_t = obs.t
        elapsed = obs.t - start_t
        if step_observation is not None:
            action, arrived = step_observation(obs, goal)
        else:
            action, arrived = controller.step(obs.base, goal)
        failure = getattr(controller, "failure", None)
        if arrived:
            stopped_by, why = "arrived", ""
        elif failure is not None:
            if on_tick is not None:
                on_tick(obs, Action(), None)
            detail, extra = failure
            return Result.failed(
                detail, residual_m=distance(obs.base, goal), pose=obs.base, **extra
            )
        elif elapsed >= timeout_s:
            stopped_by, why = "timeout", f"Ran out of time after {timeout_s:g} s."
        elif ticks >= max_ticks:
            stopped_by, why = "max_ticks", f"Gave up after {max_ticks} control ticks."
        else:
            if on_tick is not None:
                on_tick(obs, action, None)
            backend.act(action)
            ticks += 1
            continue
        if on_tick is not None:
            on_tick(obs, Action(), None)  # the stop run_goto sends on its way out
        data: dict[str, Any] = {"stopped_by": stopped_by, "elapsed_s": elapsed, "ticks": ticks}
        return _outcome(why, obs.base, goal, controller.limits, data)


def _outcome(why: str, base: Pose, goal: Pose, limits: Limits, data: dict[str, Any]) -> Result:
    """Arrived, or `why` it gave up plus the residual error in words; numbers in data."""
    forward, left, heading_error = pose_error(base, goal)
    remaining = distance(base, goal)
    data.update(
        distance_m=remaining, heading_error_rad=heading_error, forward_m=forward, left_m=left
    )
    if data["stopped_by"] == "arrived":
        logger.info("arrived at %s: %s", goal, data)
        return Result(ok=True, detail="Arrived.", data=data)
    where = "at the goal" if remaining <= limits.pos_tol else f"{remaining:.2f} m short"
    facing = (
        "facing the right way"
        if abs(heading_error) < limits.ang_tol
        else f"facing {math.degrees(abs(heading_error)):.0f}° off"
    )
    detail = f"{why} Stopped {where}, {facing}."
    logger.info("goto %s did not arrive: %s", goal, detail)
    return Result.failed(detail, **data)


def _stop_quietly(backend: RobotBackend) -> None:
    """Stop on the way out of an exception, without hiding that exception."""
    try:
        backend.act(Action())
    except Exception:
        logger.exception("could not send the zero Action; the base may still be moving")


def _clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _check_finite(pose: Pose, name: str) -> None:
    # A NaN would sail through every comparison and clip; localisation that broke must
    # stop the robot here, not reach the motors.
    if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.theta)):
        raise ValueError(f"{name} pose is not finite: {pose!r}")


# ------------------------------------------------------------ obstacle avoidance

ScanSource = Callable[[], "Scan | None"]
"""Returns the latest lidar Scan in the base frame (navigation/avoid.py), or
None before the first one. Called every control tick; must return at once."""


class _Avoidance:
    """What the avoiding controller needs: scan -> plan -> is there a reason
    not to just drive the plan?

    Owns the two honest ways of giving up:
      * blocked (no way round, or no fresh scan) for patience_s. A person who
        steps in front gets that long to move. The count leaks back down while
        the way is open, so a plan that flickers blocked/clear doesn't reset it.
      * the Pi's safety bubble refusing us. The planner and the bubble disagree
        about something (usually a thing the planner didn't see, or the goal
        itself): back off at creep for backoff_s, prefer the other way round,
        re-plan. max_refusals of those and it stops and says so.
    """

    def __init__(self, scan_source: ScanSource, config: AvoidConfig) -> None:
        self.config = config
        self.planner = LocalPlanner(config)
        self.scans = ScanTracker(scan_source, config.memory_s, config.stale_s)
        self.plan: Plan | None = None
        self.failure: tuple[str, dict] | None = None
        self.refusals = 0
        self._stuck_s = 0.0
        self._last_t: float | None = None
        self._was_refused = False
        self._backoff_until: float | None = None
        self._backoff_vx = 0.0
        self._last_vx = 0.0

    def sent(self, action: Action) -> Action:
        """Every action the controller returns passes through here, so a refusal
        can be answered by backing away from the way it was going."""
        self._last_vx = action.base_vx
        return action

    def turn_scale(self, obs: Observation) -> float:
        self.scans.update(obs.t, obs.base)
        scan = self.scans.current(obs.t, obs.base)
        return self.config.creep_scale if scan is None else self.planner.turn_scale(scan)

    def decide(self, obs: Observation, goal_rad: float, goal_m: float) -> Action | None:
        """Plan this tick. Returns an Action that overrides the controller's (stop,
        back off), or None: drive self.plan."""
        c = self.config
        dt = 0.0 if self._last_t is None else max(0.0, obs.t - self._last_t)
        self._last_t = obs.t
        self.scans.update(obs.t, obs.base)
        scan = self.scans.current(obs.t, obs.base)
        if scan is None:
            age = self.scans.age(obs.t)
            why = ("my lidar hasn't sent a scan yet" if math.isinf(age)
                   else f"my lidar's last scan is {age:.1f} s old")
            self.plan = self.planner.no_scan(goal_rad, goal_m, why)
        else:
            self.plan = self.planner.plan(scan, goal_rad, goal_m)

        refused = self.scans.bubble_blocked
        if refused and not self._was_refused:
            self.refusals += 1
            if self.refusals > c.max_refusals:
                self.failure = ("My safety bubble keeps stopping me: something is closer than "
                                "I can plan around, so I've stopped.",
                                {"blocked": True, "bubble_refusals": self.refusals - 1})
                return Action()
            self.planner.prefer_other_side()
            self._backoff_until = obs.t + c.backoff_s
            # Back away from the way it was going; a refused turn on the spot just stops.
            self._backoff_vx = (-math.copysign(c.backoff_mps, self._last_vx)
                                if abs(self._last_vx) > 1e-3 else 0.0)
        self._was_refused = refused

        if self._backoff_until is not None and obs.t < self._backoff_until:
            # Still refused while backing off means boxed in: that counts as stuck.
            self._stuck_s = self._stuck_s + dt if refused else self._stuck_s
            if self._stuck_s >= c.patience_s:
                self.failure = ("My safety bubble won't let me move either way, so I've stopped.",
                                {"blocked": True, "bubble_refusals": self.refusals})
                return Action()
            return Action(base_vx=self._backoff_vx)
        self._backoff_until = None

        if self.plan.status == BLOCKED or refused:
            self._stuck_s += dt
            if self._stuck_s >= c.patience_s:
                self.failure = (self.plan.reason if self.plan.status == BLOCKED else
                                "My safety bubble won't let me go this way, so I've stopped.",
                                {"blocked": True, "obstacle_m": self.plan.obstacle_m})
            return Action()
        self._stuck_s = max(0.0, self._stuck_s - dt)
        return None


class AvoidingGotoController(DifferentialGotoController):
    """DifferentialGotoController that steers round what the lidar sees.

    Identical to its parent while the way is clear: the planner hands back the
    goal's own bearing and a speed scale of 1, so every command is the same.
    With something in the way it hands back a bearing to steer for instead, and
    the same cos(bearing) gating turns the base toward it before driving.
    Forward speed is scaled by clearance; turning by what is near the corners.

    Needs the observation (time dates the scans), so use run_goto, which calls
    step_observation(). `plan` is the last plan, for a dashboard; `failure` is
    set once it has given up.
    """

    def __init__(
        self,
        scan_source: ScanSource,
        limits: Limits = Limits(),
        config: AvoidConfig = AvoidConfig(),
        kp_lin: float = 1.2,
        kp_ang: float = 2.2,
    ):
        super().__init__(limits, kp_lin, kp_ang)
        self.avoid = _Avoidance(scan_source, config)

    @property
    def plan(self) -> Plan | None:
        return self.avoid.plan

    @property
    def failure(self) -> tuple[str, dict] | None:
        return self.avoid.failure

    def step(self, base: Pose, goal: Pose) -> tuple[Action, bool]:
        raise TypeError("AvoidingGotoController needs the observation's clock: "
                        "call step_observation(obs, goal), or use run_goto")

    def step_observation(self, obs: Observation, goal: Pose) -> tuple[Action, bool]:
        _check_finite(obs.base, "base")
        _check_finite(goal, "goal")
        fwd_err, left_err, _ = pose_error(obs.base, goal)
        dist = distance(obs.base, goal)
        if dist <= self.limits.pos_tol:
            # In position; only the heading is left. Turning on the spot sweeps
            # the corners, so it is slowed when something is near them.
            action, done = super().step(obs.base, goal)
            if not done:
                action = Action(base_wz=action.base_wz * self.avoid.turn_scale(obs))
            return self.avoid.sent(action), done

        bearing = math.atan2(left_err, fwd_err)
        override = self.avoid.decide(obs, bearing, dist)
        if override is not None:
            return self.avoid.sent(override), False
        plan = self.avoid.plan
        gain = max(0.0, math.cos(plan.steer_rad))
        vx = min(self.limits.v_max, self.kp_lin * dist * gain) * plan.speed_scale
        wz = _clip(self.kp_ang * plan.steer_rad, self.limits.w_max) * plan.turn_scale
        return self.avoid.sent(Action(base_vx=vx, base_wz=wz)), False
