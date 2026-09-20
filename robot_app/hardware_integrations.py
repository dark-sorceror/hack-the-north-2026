"""Network adapters for Nav2 and Arm 101 bridge processes."""
import asyncio
import json
import os

from websockets.asyncio.client import connect

from .protocol import Pose, Request, token


class BridgeClient:
    """Call a Pi-side hardware bridge using authenticated JSON messages."""

    def __init__(self, url, component):
        if not url.startswith(("ws://", "wss://")):
            raise ValueError(f"{component} bridge URL must use ws:// or wss://")
        self.url = url
        self.component = component

    async def call(self, action, **kwargs):
        message = {"action": action, **kwargs}
        async with connect(self.url, additional_headers={
                "Authorization": "Bearer " + token()}, proxy=None,
                max_size=65536, open_timeout=5) as ws:
            await ws.send(json.dumps(message))
            response = json.loads(await asyncio.wait_for(ws.recv(), 30))
        if not isinstance(response, dict) or response.get("ok") is not True:
            raise RuntimeError(response.get("error", f"{self.component} bridge failed"))
        return response.get("result", {})


class Nav2Bridge(BridgeClient):
    def __init__(self, url=None):
        super().__init__(url or os.getenv("NAV2_BRIDGE_URL", "ws://127.0.0.1:8770"), "Nav2")


class Arm101Bridge(BridgeClient):
    def __init__(self, url=None):
        super().__init__(url or os.getenv("ARM101_BRIDGE_URL", "ws://127.0.0.1:8771"), "Arm 101")


class BridgeHardware:
    """Composite real driver; motion remains impossible until both bridges respond."""

    simulated = False

    def __init__(self, nav2=None, arm=None):
        self.nav2 = nav2 or Nav2Bridge()
        self.arm = arm or Arm101Bridge()

    async def execute(self, request: Request):
        if request.action == "locate":
            raise ValueError("locate must be served by the camera service")
        if request.action in {"save_start", "approach", "return_start"}:
            kwargs = {}
            if request.pose is not None:
                kwargs["pose"] = request.pose.model_dump()
            return await self.nav2.call(request.action, **kwargs)
        if request.action in {"grasp", "verify_grasp", "stow"}:
            kwargs = {"target": request.target} if request.target else {}
            if request.pose is not None:
                kwargs["pose"] = request.pose.model_dump()
            return await self.arm.call(request.action, **kwargs)
        raise ValueError(f"Unsupported hardware action: {request.action}")

    async def stop(self):
        await asyncio.gather(self.nav2.call("stop"), self.arm.call("stop"))