import asyncio
import json
import time

from websockets.asyncio.server import serve

from robot_app.camera import CameraCalibration, CameraDriver, CapturedFrame, Detection
from robot_app.central_check import check_all
from robot_app.hardware import HardwareService, SimulatedHardware


class Source:
    def capture(self):
        return CapturedFrame("frame", time.time())

    def point_in_camera(self, frame, pixel_x, pixel_y):
        return (1, 2, 3)

    def close(self):
        pass


class Detector:
    def detect(self, image, target):
        return Detection(0, 0, 10, 10, 0.9)


async def service_url(service):
    return f"ws://127.0.0.1:{service.sockets[0].getsockname()[1]}"


def test_central_check_validates_all_three_links(monkeypatch):
    monkeypatch.setenv("ROBOT_TOKEN", "test-token-for-local-tests-only-123")

    async def voice(ws):
        await ws.send(json.dumps({"type": "session.created"}))
        assert json.loads(await ws.recv())["type"] == "session.update"
        await ws.send(json.dumps({"type": "session.updated"}))

    async def scenario():
        robot = HardwareService(SimulatedHardware())
        camera = HardwareService(CameraDriver(Source(), Detector(), CameraCalibration("map")))
        async with serve(robot.handler, "127.0.0.1", 0) as robot_server, \
                serve(camera.handler, "127.0.0.1", 0) as camera_server, \
                serve(voice, "127.0.0.1", 0) as voice_server:
            result = await check_all(await service_url(voice_server),
                                     await service_url(robot_server),
                                     await service_url(camera_server), "bottle", "map")
            assert result["ok"] is True
            assert result["robot"]["simulated"] is True
            assert result["camera"]["observation"]["target"] == "bottle"

    asyncio.run(scenario())