"""Verify an authenticated camera WebSocket and one fresh object observation."""
import argparse
import asyncio
import json
import os

from .coordinator import Coordinator
from .protocol import Request, rpc


async def check(url, target, frame):
    reply = await rpc(url, Request(action="locate", target=target), timeout=15)
    observation = reply.result
    Coordinator.validate_observation(observation, frame)
    return {"ok": True, "simulated": reply.simulated, "url": url,
            "target": target, "observation": observation}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default=os.getenv("CAMERA_URL", "ws://127.0.0.1:8767"))
    parser.add_argument("--target", default="bottle")
    parser.add_argument("--frame", default="map")
    args = parser.parse_args()
    try:
        result = asyncio.run(check(args.url, args.target, args.frame))
    except Exception as exc:
        print(json.dumps({"ok": False, "url": args.url, "error": str(exc)}))
        raise SystemExit(1)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()