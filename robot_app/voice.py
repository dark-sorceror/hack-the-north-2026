"""Continuous Qwen voice with server VAD, tool calls, and interruptible playback."""
import argparse
import asyncio
import base64
import json
import os
import struct
import threading

from websockets.asyncio.client import connect

from .coordinator import Coordinator
from .protocol import send_json, token


def session_config():
    session = {
        "modalities": ["text", "audio"], "voice": "Tina",
        "input_audio_format": "pcm", "output_audio_format": "pcm",
        "turn_detection": {"type": "server_vad", "threshold": 0.5, "silence_duration_ms": 800},
        "instructions": (
            "You are a hands-free robot assistant. Call fetch_object only when the user asks "
            "to fetch a clearly identified object and bring it to the starting point. Ask a short "
            "question if the object is ambiguous. Call stop_robot immediately when asked to stop "
            "or cancel. Never invent coordinates or claim success before tool feedback. "
            "If feedback says simulated, explicitly say this was a simulation. "
            "You cannot release an object or perform other physical actions. Keep speech concise."),
        "tools": [
            {"type": "function", "function": {
                "name": "fetch_object", "description": "Fetch an object and return to the saved start pose",
                "parameters": {"type": "object", "properties": {"target": {"type": "string"}},
                               "required": ["target"], "additionalProperties": False}}},
            {"type": "function", "function": {
                "name": "stop_robot", "description": "Cancel the current robot task",
                "parameters": {"type": "object", "properties": {}, "additionalProperties": False}}}
        ]
    }
    return {"type": "session.update", "session": session}


class Audio:
    def __init__(self, input_device=None, output_device=None):
        import sounddevice as sd
        self.loop = asyncio.get_running_loop()
        self.input = asyncio.Queue(maxsize=25)
        self.playback = bytearray()
        self.lock = threading.Lock()
        self.input_device = input_device
        self.output_device = output_device
        self.output_samplerate = int(os.getenv("VOICE_OUTPUT_RATE", "24000"))
        self.sd = sd

    def receive(self, data, frames, timing, status):
        self.loop.call_soon_threadsafe(self.enqueue, bytes(data))

    def enqueue(self, data):
        if self.input.full():
            self.input.get_nowait()  # Drop old audio instead of accumulating latency.
        self.input.put_nowait(data)

    def output(self, out, frames, timing, status):
        size = len(out)
        with self.lock:
            chunk = self.playback[:size]
            del self.playback[:size]
        out[:] = bytes(chunk) + bytes(size - len(chunk))

    def append(self, data):
        if self.output_samplerate == 48000:
            samples = struct.iter_unpack("<h", data[:len(data) - len(data) % 2])
            data = b"".join(struct.pack("<hh", sample[0], sample[0]) for sample in samples)
        with self.lock:
            if len(self.playback) + len(data) > self.output_samplerate * 2 * 10:
                raise RuntimeError("Audio playback fell more than 10 seconds behind")
            self.playback.extend(data)

    def clear(self):
        with self.lock:
            self.playback.clear()

    def __enter__(self):
        self.mic = self.sd.RawInputStream(samplerate=16000, channels=1, dtype="int16",
                                          blocksize=320, callback=self.receive, device=self.input_device)
        self.speaker = self.sd.RawOutputStream(samplerate=self.output_samplerate, channels=1,
                               dtype="int16", blocksize=self.output_samplerate // 50,
                               callback=self.output, device=self.output_device)
        self.mic.start()
        try:
            self.speaker.start()
        except BaseException:
            self.mic.close()
            self.speaker.close()
            raise
        return self

    def __exit__(self, *args):
        self.mic.close()
        self.speaker.close()


async def perform_tool(coordinator, item):
    arguments = json.loads(item.get("arguments", "{}"))
    if not isinstance(arguments, dict):
        raise ValueError("Tool arguments must be an object")
    if item["name"] == "stop_robot" and not arguments:
        return await coordinator.stop()
    if item["name"] == "fetch_object" and set(arguments) == {"target"}:
        print(f"Order detected: fetch_object target={arguments['target']}", flush=True)
        return await coordinator.fetch(arguments["target"])
    raise ValueError("Unsupported tool or arguments")


async def run(input_device=None, output_device=None):
    coordinator = Coordinator()
    tools = set()
    seen = set()
    async with connect(os.getenv("VOICE_URL", "ws://127.0.0.1:8765"),
                       additional_headers={"Authorization": "Bearer " + token()},
                       proxy=None, max_size=4 * 1024 * 1024, open_timeout=10) as ws:
        created = json.loads(await asyncio.wait_for(ws.recv(), 30))
        if created.get("type") != "session.created":
            raise RuntimeError("Expected provider session.created")
        await send_json(ws, session_config())
        updated = json.loads(await asyncio.wait_for(ws.recv(), 30))
        if updated.get("type") != "session.updated":
            raise RuntimeError("Provider did not accept realtime audio/tool configuration")
        with Audio(input_device, output_device) as audio:
            response_idle = asyncio.Event()
            response_idle.set()
            needs_response = asyncio.Event()

            async def request_responses():
                while True:
                    await needs_response.wait()
                    await response_idle.wait()
                    needs_response.clear()
                    response_idle.clear()
                    await send_json(ws, {"type": "response.create"})

            async def upload():
                while True:
                    await send_json(ws, {"type": "input_audio_buffer.append",
                                         "audio": base64.b64encode(await audio.input.get()).decode()})

            async def execute(item):
                try:
                    result = await perform_tool(coordinator, item)
                except asyncio.CancelledError:
                    result = {"state": "cancelled"}
                except Exception as exc:
                    result = {"state": "failed", "error": str(exc)}
                print(json.dumps(result), flush=True)
                await send_json(ws, {"type": "conversation.item.create", "item": {
                    "type": "function_call_output", "call_id": item["call_id"],
                    "output": json.dumps(result)}})
                needs_response.set()

            async def download():
                async for raw in ws:
                    event = json.loads(raw)
                    kind = event.get("type")
                    if kind == "response.created":
                        response_idle.clear()
                    elif kind == "input_audio_buffer.speech_started":
                        audio.clear()
                    elif kind == "response.audio.delta":
                        audio.append(base64.b64decode(event["delta"], validate=True))
                    elif kind == "response.audio_transcript.done":
                        print("Assistant:", event.get("transcript", ""), flush=True)
                    elif kind in {"conversation.item.input_audio_transcription.completed",
                                  "input_audio_buffer.transcription.completed"}:
                        transcript = event.get("transcript", "")
                        if transcript:
                            print("You:", transcript, flush=True)
                    elif kind == "response.done":
                        response_idle.set()
                        if event.get("response", {}).get("status") != "completed":
                            continue
                        for item in event.get("response", {}).get("output", []):
                            if item.get("type") != "function_call" or item.get("status", "completed") != "completed":
                                continue
                            call_id = item.get("call_id")
                            if not call_id or call_id in seen:
                                continue
                            if len(seen) >= 10000:
                                raise RuntimeError("Session tool limit reached")
                            seen.add(call_id)
                            task = asyncio.create_task(execute(item))
                            tools.add(task)
                    elif kind == "error":
                        raise RuntimeError("Voice provider error")
                    for task in list(tools):
                        if task.done():
                            task.result()
                            tools.remove(task)

            print("Listening continuously. Say 'bring me the bottle' or 'stop'.", flush=True)
            tasks = [asyncio.create_task(upload()), asyncio.create_task(download()),
                     asyncio.create_task(request_responses())]
            try:
                done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                for task in done:
                    task.result()
            finally:
                for task in [*tasks, *tools]:
                    task.cancel()
                await asyncio.gather(*tasks, *tools, return_exceptions=True)
                try:
                    await coordinator.stop()
                except Exception:
                    print("Stop could not be confirmed; check the local controller.", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list-devices", action="store_true")
    parser.add_argument("--input-device", type=int)
    parser.add_argument("--output-device", type=int)
    args = parser.parse_args()
    if args.list_devices:
        import sounddevice
        print(sounddevice.query_devices())
    else:
        asyncio.run(run(args.input_device, args.output_device))
