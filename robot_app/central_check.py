"""Check the central Pi's voice, robot, and camera connections."""
import argparse
import asyncio
import json
import os

from websockets.asyncio.client import connect

from .camera_check import check as check_camera
from .protocol import Request, rpc, send_json, token
from .voice import session_config


async def check_voice(url):
    async with connect(url, additional_headers={"Authorization": "Bearer " + token()},
                       proxy=None, max_size=4 * 1024 * 1024, open_timeout=15) as ws:
        created = json.loads(await asyncio.wait_for(ws.recv(), 30))
        if created.get("type") != "session.created":
            raise RuntimeError(f"Expected session.created, received {created.get('type')}")
        await send_json(ws, session_config())
        updated = json.loads(await asyncio.wait_for(ws.recv(), 30))
        if updated.get("type") != "session.updated":
            raise RuntimeError(f"Expected session.updated, received {updated.get('type')}")
    return {"ok": True, "url": url}


async def check_robot(url):
    reply = await rpc(url, Request(action="save_start"), timeout=15)
    if "pose" not in reply.result:
        raise RuntimeError("Robot did not return a starting pose")
    return {"ok": True, "url": url, "simulated": reply.simulated,
            "pose": reply.result["pose"]}


async def check_all(voice_url, robot_url, camera_url, target, frame):
    voice, robot, camera = await asyncio.gather(
        check_voice(voice_url), check_robot(robot_url), check_camera(camera_url, target, frame))
    return {"ok": True, "voice": voice, "robot": robot, "camera": camera}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--voice-url", default=os.getenv("VOICE_URL", "ws://127.0.0.1:8765"))
    parser.add_argument("--robot-url", default=os.getenv("ROBOT_URL", "ws://127.0.0.1:8766"))
    parser.add_argument("--camera-url", default=os.getenv("CAMERA_URL", "ws://127.0.0.1:8767"))
    parser.add_argument("--target", default="bottle")
    parser.add_argument("--frame", default="map")
    args = parser.parse_args()
    try:
        result = asyncio.run(check_all(args.voice_url, args.robot_url, args.camera_url,
                                       args.target, args.frame))
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}))
        raise SystemExit(1)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()