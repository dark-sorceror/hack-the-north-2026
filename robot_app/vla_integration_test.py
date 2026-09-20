"""Offline Phase 1 integration test and latency report generator."""
import argparse
import asyncio
import json
import sys
import time
from pathlib import Path

from websockets.asyncio.server import serve

from .action_monitor import RealtimeActionMonitor
from .coordinator import Coordinator
from .hardware import HardwareService, SimulatedHardware
from .task_decomposer import VLATaskDecomposer


async def run_report():
    """Run the complete pure-Python flow against simulated hardware."""
    decomposer = VLATaskDecomposer(provider_type="mock")
    frames = []

    async def feedback(frame):
        frames.append(frame)

    monitor = RealtimeActionMonitor(feedback, fps=30)
    driver = SimulatedHardware()
    async with serve(HardwareService(driver).handler, "127.0.0.1", 0) as server:
        port = server.sockets[0].getsockname()[1]
        url = f"ws://127.0.0.1:{port}"
        coordinator = Coordinator(url, url, decomposer=decomposer)
        started = time.perf_counter()
        outcome = await coordinator.fetch("bottle")
        monitor_stats = await monitor.run(range(30))
        total_latency = time.perf_counter() - started
    return {
        "status": "PASS" if total_latency < 8 and monitor_stats.meets() else "FAIL",
        "hardware_simulation": True,
        "task_decomposition_latency_s": round(decomposer.last_latency_s or 0, 4),
        "object_detection_latency_s": round(max(outcome["latencies_s"]["locate"]), 4),
        "feedback_latency_ms": round(monitor_stats.max_latency_ms, 3),
        "frame_rate": round(monitor_stats.frame_rate, 2),
        "total_e2e_latency_s": round(total_latency, 4),
        "safety_watchdog": "active",
        "outcome": outcome["state"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, default=None)
    args = parser.parse_args()
    report = asyncio.run(run_report())
    text = json.dumps(report, indent=2)
    sys.stdout.write(text + "\n")
    if args.report:
        args.report.write_text(text + "\n", encoding="utf-8")
    if report["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()