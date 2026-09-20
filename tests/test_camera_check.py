import asyncio

from websockets.asyncio.server import serve

from robot_app.camera import CameraCalibration, CameraDriver, CapturedFrame, Detection
from robot_app.camera_check import check
from robot_app.hardware import HardwareService


class Source:
    def capture(self):
        import time
        return CapturedFrame("frame", time.time())

    def point_in_camera(self, frame, pixel_x, pixel_y):
        return (1, 2, 3)

    def close(self):
        pass


class Detector:
    def detect(self, image, target):
        return Detection(0, 0, 10, 10, 0.9)


def test_camera_check_validates_remote_observation(monkeypatch):
    monkeypatch.setenv("ROBOT_TOKEN", "test-token-for-local-tests-only-123")

    async def scenario():
        driver = CameraDriver(Source(), Detector(), CameraCalibration("map"))
        async with serve(HardwareService(driver).handler, "127.0.0.1", 0) as service:
            url = f"ws://127.0.0.1:{service.sockets[0].getsockname()[1]}"
            result = await check(url, "bottle", "map")
            assert result["ok"] is True
            assert result["simulated"] is False

    asyncio.run(scenario())