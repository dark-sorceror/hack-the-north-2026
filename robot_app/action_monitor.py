"""Frame-rate pacing and latency accounting for realtime action feedback."""
import asyncio
import inspect
import time
from dataclasses import dataclass


@dataclass
class MonitorStats:
    frames: int = 0
    feedback: int = 0
    elapsed_s: float = 0
    max_latency_ms: float = 0

    @property
    def frame_rate(self):
        return self.frames / self.elapsed_s if self.elapsed_s else 0

    def meets(self, minimum_fps=25, max_latency_ms=500):
        return self.frame_rate >= minimum_fps and self.max_latency_ms <= max_latency_ms


class RealtimeActionMonitor:
    """Send camera frames at a bounded rate and measure feedback latency."""

    def __init__(self, feedback, fps=30, response_limit_ms=500):
        if fps <= 0:
            raise ValueError("fps must be positive")
        self.feedback = feedback
        self.period = 1 / fps
        self.response_limit_ms = response_limit_ms
        self.stats = MonitorStats()

    async def run(self, frames, state=None, duration=None):
        started = time.perf_counter()
        next_frame = started
        deadline = started + duration if duration is not None else None
        for frame in frames:
            now = time.perf_counter()
            if deadline is not None and now >= deadline:
                break
            if now < next_frame:
                await asyncio.sleep(next_frame - now)
            sent = time.perf_counter()
            result = self.feedback(frame, state) if state is not None else self.feedback(frame)
            if inspect.isawaitable(result):
                await result
            latency_ms = (time.perf_counter() - sent) * 1000
            self.stats.frames += 1
            self.stats.feedback += 1
            self.stats.max_latency_ms = max(self.stats.max_latency_ms, latency_ms)
            next_frame = max(next_frame + self.period, time.perf_counter())
        self.stats.elapsed_s = time.perf_counter() - started
        return self.stats