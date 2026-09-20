"""Close the last half-metre onto something perception can see.

`goto` (navigation/drive.py) drives to a pose in the world frame and is only as good as
odometry. Approach works in bearing and range straight off the detector, so it stays
correct after a room's worth of drift — which is exactly when the gripper has to be right.
Same split as drive.py: `step` is a pure control law testable without a clock, and
`run_approach` owns the side effects (observing, acting, giving up, stopping).
"""

from __future__ import annotations

import math
from collections.abc import Callable

from retriever.navigation.avoid import AvoidConfig, Plan
from retriever.navigation.drive import Limits, ScanSource, Tick, _Avoidance
from retriever.types import Action, Observation, Result, Target

class ApproachController:
    """Close the last half-metre onto something perception can see.

    Works in bearing/range, never in world coordinates, so it stays correct even
    when odometry has drifted — which after crossing a room, it has.
    """

    def __init__(
        self,
        standoff_m: float = 0.25,
        limits: Limits = Limits(),
        kp_lin: float = 1.2,
        kp_ang: float = 2.2,
        range_tol: float = 0.03,
    ):
        self.standoff_m = standoff_m
        self.limits = limits
        self.kp_lin = kp_lin
        self.kp_ang = kp_ang
        self.range_tol = range_tol

    def step(self, target: Target) -> tuple[Action, bool]:
        range_err = target.range_m - self.standoff_m
        bearing = target.bearing_rad

        aligned = abs(bearing) <= self.limits.ang_tol
        if aligned and abs(range_err) <= self.range_tol:
            return Action(), True

        # Scaling forward speed by cos(bearing) makes the base slow as it turns
        # and stop entirely if the target ends up behind it, instead of driving
        # a spiral around something it is not facing.
        gain = max(0.0, math.cos(bearing))
        vx = max(-self.limits.v_max, min(self.limits.v_max, self.kp_lin * range_err * gain))
        wz = max(-self.limits.w_max, min(self.limits.w_max, self.kp_ang * bearing))
        return Action(base_vx=vx, base_wz=wz), False


# ------------------------------------------------------------------- run loop


def run_approach(
    backend,
    perceive: Callable[[Observation], Target | None],
    standoff_m: float = 0.25,
    limits: Limits = Limits(),
    timeout_s: float = 15.0,
    lost_grace_ticks: int = 15,
    max_ticks: int = 20_000,
    on_tick: Tick | None = None,
    controller=None,
) -> Result:
    """Close onto whatever `perceive` reports each tick.

    `perceive` returning None means the target is not visible this frame. A few
    dropped frames are normal — a detector missing one frame in ten is not a
    failure — so we coast briefly before admitting we lost it.

    `controller`, when given, replaces the ApproachController built from
    standoff_m and limits (it carries its own): AvoidingApproachController
    steers round obstacles but never round the target itself.
    """
    ctl = controller or ApproachController(standoff_m=standoff_m, limits=limits)
    step_observation = getattr(ctl, "step_observation", None)
    obs = backend.observe()
    t0 = obs.t
    misses = 0
    last: Target | None = None

    for _ in range(max_ticks):
        target = perceive(obs)
        if target is None:
            misses += 1
            if misses > lost_grace_ticks:
                return Result.failed(
                    "I lost sight of it while moving in.",
                    last_seen=last,
                )
            if on_tick:
                on_tick(obs, Action(), None)
            backend.act(Action())
            obs = backend.observe()
            continue

        misses = 0
        last = target
        if step_observation is not None:
            action, done = step_observation(obs, target)
        else:
            action, done = ctl.step(target)
        if done:
            return Result(
                ok=True,
                confidence=target.confidence,
                detail=f"Lined up on the {target.label}.",
                data={"range_m": target.range_m, "elapsed_s": obs.t - t0},
            )
        failure = getattr(ctl, "failure", None)
        if failure is not None:
            backend.act(Action())
            detail, data = failure
            return Result.failed(detail, range_m=target.range_m,
                                 bearing_rad=target.bearing_rad, **data)
        if obs.t - t0 > timeout_s:
            return Result.failed(
                f"I got within {target.range_m:.2f} m but couldn't line up on it.",
                range_m=target.range_m,
                bearing_rad=target.bearing_rad,
            )
        if on_tick:
            on_tick(obs, action, target)
        backend.act(action)
        obs = backend.observe()

    return Result.failed("Approach ran out of ticks before converging.")


class AvoidingApproachController(ApproachController):
    """ApproachController that steers round obstacles but never round its own
    target. The target IS the planner's goal, so its lidar returns (a bin, a
    hand-off station) fall inside goal_clear_m and are not obstacles; closing on
    them is capped at creep, which is the speed the Pi's bubble lets through to
    within a few centimetres. Reversing to the standoff is not scaled: the
    bubble looks behind for that."""

    def __init__(
        self,
        scan_source: ScanSource,
        standoff_m: float = 0.25,
        limits: Limits = Limits(),
        config: AvoidConfig = AvoidConfig(),
        kp_lin: float = 1.2,
        kp_ang: float = 2.2,
        range_tol: float = 0.03,
    ):
        super().__init__(standoff_m, limits, kp_lin, kp_ang, range_tol)
        self.avoid = _Avoidance(scan_source, config)

    @property
    def plan(self) -> Plan | None:
        return self.avoid.plan

    @property
    def failure(self) -> tuple[str, dict] | None:
        return self.avoid.failure

    def step(self, target: Target) -> tuple[Action, bool]:
        raise TypeError("AvoidingApproachController needs the observation's clock: "
                        "call step_observation(obs, target), or use run_approach")

    def step_observation(self, obs: Observation, target: Target) -> tuple[Action, bool]:
        range_err = target.range_m - self.standoff_m
        if abs(target.bearing_rad) <= self.limits.ang_tol and abs(range_err) <= self.range_tol:
            return Action(), True

        override = self.avoid.decide(obs, target.bearing_rad, target.range_m)
        if override is not None:
            return self.avoid.sent(override), False
        plan = self.avoid.plan
        gain = max(0.0, math.cos(plan.steer_rad))
        vmax = self.limits.v_max
        vx = max(-vmax, min(vmax, self.kp_lin * range_err * gain))
        if vx > 0.0:
            vx *= plan.speed_scale
        wmax = self.limits.w_max
        wz = max(-wmax, min(wmax, self.kp_ang * plan.steer_rad)) * plan.turn_scale
        return self.avoid.sent(Action(base_vx=vx, base_wz=wz)), False
