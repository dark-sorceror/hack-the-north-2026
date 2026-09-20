import asyncio
import json
import time

import pytest
from websockets.asyncio.client import connect
from websockets.asyncio.server import serve

from robot_app.coordinator import Coordinator
from robot_app.hardware import HardwareService, SimulatedHardware
from robot_app.protocol import Request, rpc
from robot_app.voice import perform_tool, session_config


@pytest.fixture(autouse=True)
def credentials(monkeypatch):
    monkeypatch.setenv("ROBOT_TOKEN", "test-token-for-local-tests-only-123")


async def with_robot(test, driver=None):
    driver = driver or SimulatedHardware()
    async with serve(HardwareService(driver).handler, "127.0.0.1", 0) as server:
        url = f"ws://127.0.0.1:{server.sockets[0].getsockname()[1]}"
        await test(url, driver)


def test_fetch_returns_to_start():
    async def check(url, driver):
        outcome = await Coordinator(url, url).fetch("bottle")
        assert outcome["state"] == "completed"
        assert outcome["simulated"] is True
        assert outcome["steps"] == ["save_start", "locate", "approach", "locate", "grasp", "verify_grasp", "stow", "return_start"]
        assert driver.pose.x == driver.pose.y == 0
        assert driver.held
    asyncio.run(with_robot(check))


def test_duplicate_and_expired_commands():
    async def check(url, driver):
        request = Request(action="save_start")
        await rpc(url, request)
        with pytest.raises(RuntimeError, match="Duplicate"):
            await rpc(url, request)
        with pytest.raises(RuntimeError, match="Expired"):
            await rpc(url, Request(action="save_start", expires_at=time.time() - 1))
        assert driver.history == ["save_start"]
    asyncio.run(with_robot(check))


def test_unauthenticated_client_rejected():
    async def check(url, driver):
        async with connect(url, proxy=None) as ws:
            await ws.wait_closed()
            assert ws.close_code == 1008
        assert not driver.history
    asyncio.run(with_robot(check))


@pytest.mark.parametrize("change", [
    {"observed_at": 0}, {"confidence": 0.1}, {"confidence": float("nan")},
    {"pose": {"frame": "camera", "x": 0, "y": 0}},
    {"pose": {"frame": "map", "x": float("nan"), "y": 0}},
])
def test_bad_observations_rejected(change):
    observation = {"observed_at": time.time(), "confidence": 0.95,
                   "pose": {"frame": "map", "x": 1, "y": 0}}
    observation.update(change)
    with pytest.raises(ValueError):
        Coordinator.validate_observation(observation, "map")


def test_failed_grasp_stops_without_returning():
    class FailedGrasp(SimulatedHardware):
        async def execute(self, request):
            result = await super().execute(request)
            if request.action == "verify_grasp":
                result["held"] = False
            return result
    async def check(url, driver):
        with pytest.raises(RuntimeError, match="not verified"):
            await Coordinator(url, url).fetch("bottle")
        assert driver.stopped.is_set()
        assert "return_start" not in driver.history
    asyncio.run(with_robot(check, FailedGrasp()))


def test_stop_interrupts_active_task():
    class SlowHardware(SimulatedHardware):
        async def execute(self, request):
            self.history.append(request.action)
            await asyncio.sleep(10)
    async def check(url, driver):
        coordinator = Coordinator(url, url)
        task = asyncio.create_task(coordinator.fetch("bottle"))
        while not driver.history:
            await asyncio.sleep(0.01)
        result = await coordinator.stop()
        assert result["state"] == "stopped"
        assert task.cancelled()
        assert driver.stopped.is_set()
    asyncio.run(with_robot(check, SlowHardware()))


def test_disconnect_stops_hardware():
    class SlowHardware(SimulatedHardware):
        async def execute(self, request):
            self.history.append(request.action)
            await asyncio.sleep(10)
    async def check(url, driver):
        async with connect(url, proxy=None, additional_headers={
            "Authorization": "Bearer test-token-for-local-tests-only-123"}) as ws:
            await ws.send(Request(action="save_start").model_dump_json())
            while not driver.history:
                await asyncio.sleep(0.01)
        await asyncio.wait_for(driver.stopped.wait(), 1)
    asyncio.run(with_robot(check, SlowHardware()))


def test_tool_rejects_arbitrary_motion():
    with pytest.raises(ValueError):
        asyncio.run(perform_tool(Coordinator(), {"name": "set_motor", "arguments": '{"speed": 100}'}))
    config = session_config()["session"]
    assert config["turn_detection"]["type"] == "server_vad"
    assert config["input_audio_format"] == "pcm"
