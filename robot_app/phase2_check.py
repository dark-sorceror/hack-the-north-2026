"""Non-motion Phase 2 preflight for camera, robot, Nav2, and Arm 101 bridges."""
import argparse
import asyncio
import json
import os

from .camera_check import check as check_camera
from .hardware_integrations import Arm101Bridge, Nav2Bridge
from .protocol import Request, rpc


async def check(args):
    robot = await rpc(args.robot_url, Request(action="save_start"), timeout=10)
    camera = await check_camera(args.camera_url, args.target, args.frame)
    nav2 = await Nav2Bridge(args.nav2_url).call("health")
    arm = await Arm101Bridge(args.arm_url).call("health")
    return {"ok": True, "motion_started": False, "robot": robot.result,
            "camera": camera, "nav2": nav2, "arm101": arm}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-url", default=os.getenv("ROBOT_URL", "ws://127.0.0.1:8766"))
    parser.add_argument("--camera-url", default=os.getenv("CAMERA_URL", "ws://127.0.0.1:8767"))
    parser.add_argument("--nav2-url", default=os.getenv("NAV2_BRIDGE_URL", "ws://127.0.0.1:8770"))
    parser.add_argument("--arm-url", default=os.getenv("ARM101_BRIDGE_URL", "ws://127.0.0.1:8771"))
    parser.add_argument("--target", default="bottle")
    parser.add_argument("--frame", default="map")
    args = parser.parse_args()
    try:
        result = asyncio.run(check(args))
    except Exception as exc:
        print(json.dumps({"ok": False, "motion_started": False, "error": str(exc)}))
        raise SystemExit(1)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()