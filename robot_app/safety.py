"""Fast local safety checks and bounded execution watchdog."""
import asyncio
import math
import os


class QuickDecisionMaker:
    """Deterministic safety gate that runs before provider or hardware actions."""

    def validate_action_safety(self, action, current_state=None, sensor_data=None):
        current_state = current_state or {}
        sensor_data = sensor_data or {}
        if sensor_data.get("emergency_stop"):
            return {"safe": False, "reason": "Emergency stop is active"}
        if action.get("type") == "stop" or action.get("action") == "stop":
            return {"safe": True, "reason": "Stop is always permitted"}
        pose = action.get("pose")
        if pose:
            values = (pose.x, pose.y, pose.z, pose.yaw) if hasattr(pose, "x") else (
                pose.get("x"), pose.get("y"), pose.get("z", 0), pose.get("yaw", 0))
            if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in values):
                return {"safe": False, "reason": "Pose contains a non-finite value"}
            if abs(values[0]) > 10 or abs(values[1]) > 10 or not -1 <= values[2] <= 3:
                return {"safe": False, "reason": "Pose is outside the configured workspace"}
        return {"safe": True, "reason": "Local safety checks passed"}


async def run_with_watchdog(operation, emergency_stop, timeout=None):
    """Run one operation and request a stop if it exceeds its deadline."""
    deadline = timeout if timeout is not None else float(os.getenv("ROBOT_WATCHDOG_TIMEOUT", "30"))
    try:
        return await asyncio.wait_for(operation, timeout=deadline)
    except asyncio.TimeoutError:
        await emergency_stop()
        raise RuntimeError(f"Watchdog timeout after {deadline:g}s")