import asyncio
import json

from websockets.asyncio.server import serve

from robot_app.camera_feed import WebSocketCameraFeed
from robot_app.hardware_integrations import BridgeHardware
from robot_app.protocol import Pose, Request


def test_bridge_hardware_routes_navigation_and_arm_actions(monkeypatch):
    monkeypatch.setenv("ROBOT_TOKEN", "test-token-for-local-tests-only-123")
    calls = []

    class FakeBridge:
        def __init__(self, name):
            self.name = name

        async def call(self, action, **kwargs):
            calls.append((self.name, action, kwargs))
            if action == "save_start":
                return {"pose": Pose(frame="map", x=0, y=0).model_dump()}
            if action == "verify_grasp":
                return {"held": True}
            return {"completed": True}

    async def scenario():
        driver = BridgeHardware(FakeBridge("nav2"), FakeBridge("arm"))
        await driver.execute(Request(action="save_start"))
        await driver.execute(Request(action="approach", pose=Pose(frame="map", x=1, y=0)))
        await driver.execute(Request(action="grasp", target="bottle"))
        await driver.execute(Request(action="verify_grasp"))

    asyncio.run(scenario())
    assert [call[:2] for call in calls] == [
        ("nav2", "save_start"), ("nav2", "approach"),
        ("arm", "grasp"), ("arm", "verify_grasp")]


def test_persistent_camera_feed_counts_binary_frames(monkeypatch):
    monkeypatch.setenv("ROBOT_TOKEN", "test-token-for-local-tests-only-123")

    async def camera_server(ws):
        for _ in range(3):
            await ws.send(b"jpeg-frame")
        await ws.close()

    async def scenario():
        async with serve(camera_server, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            feed = WebSocketCameraFeed(f"ws://127.0.0.1:{port}")
            received = []
            stats = await feed.run(lambda payload, observed_at: received.append((payload, observed_at)),
                                   max_frames=3)
            return stats, received

    stats, received = asyncio.run(scenario())
    assert stats.frames == 3
    assert stats.first_frame_s is not None
    assert [payload for payload, _ in received] == [b"jpeg-frame"] * 3