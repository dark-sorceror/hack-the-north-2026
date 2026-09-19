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
    start_t: float | None = None
    ticks = 0
    while True:
        obs = backend.observe()
        if start_t is None:
            start_t = obs.t
        elapsed = obs.t - start_t
        action, arrived = controller.step(obs.base, goal)
        if arrived:
            stopped_by, why = "arrived", ""
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
