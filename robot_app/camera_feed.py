"""Persistent authenticated WebSocket camera feed for Phase 2 hardware."""
import asyncio
import inspect
import os
import time
from dataclasses import dataclass

from websockets.asyncio.client import connect

from .protocol import token


@dataclass
class FeedStats:
    frames: int = 0
    elapsed_s: float = 0
    first_frame_s: float | None = None

    @property
    def frame_rate(self):
        return self.frames / self.elapsed_s if self.elapsed_s else 0


class WebSocketCameraFeed:
    """Read binary JPEG frames from a persistent camera WebSocket."""

    def __init__(self, url=None, max_size=4 * 1024 * 1024):
        self.url = url or os.getenv("BIRDSEYE_URL", "ws://127.0.0.1:8765/birdseye")
        if not self.url.startswith(("ws://", "wss://")):
            raise ValueError("Camera feed URL must use ws:// or wss://")
        self.max_size = max_size
        self.stats = FeedStats()

    async def run(self, callback, duration=None, max_frames=None):
        """Stream frames to callback and return measured feed statistics."""
        started = time.perf_counter()
        deadline = started + duration if duration is not None else None
        async with connect(self.url, additional_headers={
                "Authorization": "Bearer " + token()}, proxy=None,
                max_size=self.max_size, open_timeout=10) as ws:
            while max_frames is None or self.stats.frames < max_frames:
                if deadline is not None and time.perf_counter() >= deadline:
                    break
                payload = await asyncio.wait_for(ws.recv(), 10)
                if not isinstance(payload, bytes):
                    continue
                if self.stats.first_frame_s is None:
                    self.stats.first_frame_s = time.perf_counter() - started
                result = callback(payload, time.time())
                if inspect.isawaitable(result):
                    await result
                self.stats.frames += 1
        self.stats.elapsed_s = time.perf_counter() - started
        return self.stats