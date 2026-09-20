import asyncio
import time

from websockets.asyncio.server import serve

from robot_app.camera import CameraCalibration, CameraDriver, CapturedFrame, Detection
from robot_app.camera_bridge import BirdseyeSource
from robot_app.hardware import HardwareService
from robot_app.protocol import Request, rpc


class FakeSource:
    def capture(self):
        return CapturedFrame("frame", time.time(), None)

    def point_in_camera(self, frame, pixel_x, pixel_y):
        assert frame.image == "frame"
        assert (pixel_x, pixel_y) == (20, 30)
        return (1.0, 0.5, 0.7)

    def close(self):
        pass


class FakeDetector:
    def detect(self, image, target):
        assert image == "frame"
        assert target == "bottle"
        return Detection(10, 20, 30, 40, 0.93)


def test_camera_locate_returns_calibrated_observation(monkeypatch):
    monkeypatch.setenv("ROBOT_TOKEN", "test-token-for-local-tests-only-123")

    async def scenario():
        camera = CameraDriver(FakeSource(), FakeDetector(),
                              CameraCalibration("map", (2.0, 0.0, 0.0)))
        async with serve(HardwareService(camera).handler, "127.0.0.1", 0) as service:
            url = f"ws://127.0.0.1:{service.sockets[0].getsockname()[1]}"
            reply = await rpc(url, Request(action="locate", target="bottle"))
            observation = reply.result
            assert observation["pose"]["x"] == 3.0
            assert observation["pose"]["frame"] == "map"
            assert observation["confidence"] == 0.93

    asyncio.run(scenario())


def test_birdseye_source_maps_image_center_to_map_center():
    source = BirdseyeSource("ws://camera/feed", (2.0, 3.0), (0.01, -0.02), 0.7)
    frame = CapturedFrame(type("Image", (), {"size": (640, 480)})(), time.time())

    assert source.point_in_camera(frame, 320, 240) == (2.0, 3.0, 0.7)
    assert source.point_in_camera(frame, 420, 190) == (3.0, 4.0, 0.7)