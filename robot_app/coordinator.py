"""Deterministic fetch sequence; cloud output cannot specify motor commands."""
import argparse
import asyncio
import json
import os
import time

from .protocol import Pose, Request, rpc
from .safety import QuickDecisionMaker, run_with_watchdog
from .task_decomposer import VLATaskDecomposer


class Coordinator:
    def __init__(self, robot_url=None, camera_url=None, safety=None, decomposer=None):
        self.robot_url = robot_url or os.getenv("ROBOT_URL", "ws://127.0.0.1:8766")
        self.camera_url = camera_url or os.getenv("CAMERA_URL", self.robot_url)
        self.active = None
        self.state = "idle"
        self.safety = safety or QuickDecisionMaker()
        self.decomposer = decomposer
        if self.decomposer is None and os.getenv("VLA_DECOMPOSER_ENABLED") == "1":
            self.decomposer = VLATaskDecomposer()

    async def _emergency_stop(self):
        try:
            await rpc(self.robot_url, Request(action="stop"), timeout=5)
        finally:
            self.state = "stopped"

    async def stop(self):
        if self.active and self.active is not asyncio.current_task():
            self.active.cancel()
            await asyncio.gather(self.active, return_exceptions=True)
        self.state = "stopping"
        try:
            await rpc(self.robot_url, Request(action="stop"), timeout=5)
            self.state = "stopped"
        except Exception:
            self.state = "stop_unconfirmed"
            raise
        return {"state": self.state}

    async def fetch(self, target):
        if not isinstance(target, str) or not target.strip() or len(target) > 120:
            raise ValueError("Provide a short object description")
        if self.active and not self.active.done():
            raise RuntimeError("Robot is already executing a task")
        self.active = asyncio.current_task()
        steps = []
        simulated = False
        task_plan = None
        latencies_s = {}

        async def step(action, url=None, **kwargs):
            nonlocal simulated
            self.state = action
            request = Request(action=action, **kwargs)
            decision = self.safety.validate_action_safety(
                {"action": action, "pose": request.pose, "target": request.target},
                current_state={"state": self.state}, sensor_data={})
            if not decision["safe"]:
                raise RuntimeError(f"Safety abort: {decision['reason']}")
            started = time.perf_counter()
            reply = await run_with_watchdog(
                rpc(url or self.robot_url, request), self._emergency_stop)
            latencies_s.setdefault(action, []).append(time.perf_counter() - started)
            simulated = simulated or reply.simulated
            steps.append(action)
            return reply.result

        try:
            if self.decomposer is not None:
                task_plan = await self.decomposer.adecompose(
                    f"Pick up the {target} and bring it to the starting point")
            start = Pose.model_validate((await step("save_start"))["pose"])
            observation = await step("locate", self.camera_url, target=target)
            pose = self.validate_observation(observation, start.frame)
            await step("approach", pose=pose)
            # Reobserve after moving: never grasp using the pre-navigation image.
            observation = await step("locate", self.camera_url, target=target)
            pose = self.validate_observation(observation, start.frame)
            await step("grasp", target=target, pose=pose)
            if (await step("verify_grasp")).get("held") is not True:
                raise RuntimeError("Grasp was not verified")
            await step("stow")
            await step("return_start", pose=start)
            self.state = "completed"
            result = {"state": self.state, "simulated": simulated, "steps": steps,
                      "latencies_s": latencies_s}
            if task_plan is not None:
                result["task_plan"] = task_plan
            return result
        except BaseException:
            self.state = "failed"
            try:
                await rpc(self.robot_url, Request(action="stop"), timeout=5)
            except Exception:
                self.state = "stop_unconfirmed"
            raise
        finally:
            self.active = None

    @staticmethod
    def validate_observation(observation, frame):
        pose = Pose.model_validate(observation["pose"])
        age = time.time() - float(observation["observed_at"])
        confidence = float(observation["confidence"])
        if not -1 <= age <= 2 or not 0.7 <= confidence <= 1:
            raise ValueError("Stale or uncertain object observation")
        if pose.frame != frame:
            raise ValueError("Camera observation needs a calibrated transform into the navigation frame")
        return pose


def main():
    parser = argparse.ArgumentParser(description="Exercise the fetch sequence over WebSocket")
    parser.add_argument("--target", default="bottle")
    args = parser.parse_args()
    print(json.dumps(asyncio.run(Coordinator().fetch(args.target)), indent=2))


if __name__ == "__main__":
    main()
