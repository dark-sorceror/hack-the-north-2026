import asyncio
import json

from websockets.asyncio.server import serve

from robot_app import voice
from robot_app.hardware import HardwareService, SimulatedHardware


def test_continuous_voice_tool_fetch_round_trip(monkeypatch):
    monkeypatch.setenv("ROBOT_TOKEN", "test-token-for-local-tests-only-123")
    played = []
    outcomes = []

    class FakeAudio:
        def __init__(self, *args):
            self.input = asyncio.Queue()
            self.input.put_nowait(b"\x00\x00" * 320)

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def append(self, data):
            played.append(data)

        def clear(self):
            pass

    monkeypatch.setattr(voice, "Audio", FakeAudio)

    async def scenario():
        driver = SimulatedHardware()
        async def provider(ws):
            await ws.send(json.dumps({"type": "session.created"}))
            configuration = json.loads(await ws.recv())
            assert configuration["session"]["turn_detection"]["type"] == "server_vad"
            await ws.send(json.dumps({"type": "session.updated"}))
            audio = json.loads(await ws.recv())
            assert audio["type"] == "input_audio_buffer.append"
            await ws.send(json.dumps({"type": "response.done", "response": {
                "status": "completed", "output": [{"type": "function_call", "name": "fetch_object",
                "call_id": "fetch-1", "arguments": '{"target":"bottle"}'}]}}))
            result = json.loads(await ws.recv())
            assert result["item"]["type"] == "function_call_output"
            outcomes.append(json.loads(result["item"]["output"]))
            assert json.loads(await ws.recv())["type"] == "response.create"
            await ws.send(json.dumps({"type": "response.audio.delta", "delta": "AAAA"}))
            await ws.close()

        async with serve(HardwareService(driver).handler, "127.0.0.1", 0) as hardware:
            hardware_url = f"ws://127.0.0.1:{hardware.sockets[0].getsockname()[1]}"
            monkeypatch.setenv("ROBOT_URL", hardware_url)
            monkeypatch.setenv("CAMERA_URL", hardware_url)
            async with serve(provider, "127.0.0.1", 0) as provider_server:
                monkeypatch.setenv("VOICE_URL", f"ws://127.0.0.1:{provider_server.sockets[0].getsockname()[1]}")
                await asyncio.wait_for(voice.run(), 5)
        assert driver.stopped.is_set()

    asyncio.run(scenario())
    assert outcomes[0]["state"] == "completed"
    assert outcomes[0]["simulated"] is True
    assert played == [b"\x00\x00\x00"]


def test_fetch_order_reaches_vision_endpoint_twice(monkeypatch):
    monkeypatch.setenv("ROBOT_TOKEN", "test-token-for-local-tests-only-123")
    vision_calls = []

    class FakeCoordinator:
        async def fetch(self, target):
            vision_calls.extend([target, target])
            return {"state": "completed", "simulated": True,
                    "steps": ["save_start", "locate", "approach", "locate", "grasp"]}

    async def scenario():
        return await voice.perform_tool(FakeCoordinator(), {
            "name": "fetch_object", "arguments": '{"target":"bottle"}'})

    result = asyncio.run(scenario())
    assert result["state"] == "completed"
    assert vision_calls == ["bottle", "bottle"]
