import asyncio
import time

import pytest

from robot_app.action_monitor import RealtimeActionMonitor
from robot_app.safety import QuickDecisionMaker, run_with_watchdog
from robot_app.task_decomposer import VLATaskDecomposer


PLAN = {
    "object": {"class": "bottle", "confidence": 0.98},
    "subgoals": [
        {"type": "navigate", "target": [1, 0, 0], "duration_sec": 1},
        {"type": "approach_arm", "constraints": ["avoid table"]},
        {"type": "grasp", "success_criteria": ["object held"]},
        {"type": "deliver", "target": [0, 0, 0]},
    ],
}


def test_decomposer_validates_dashscope_json_and_image():
    requests = []

    def transport(payload):
        requests.append(payload)
        return {"choices": [{"message": {"content": "```json\n" + str(PLAN).replace("'", '"') + "\n```"}}]}

    decomposer = VLATaskDecomposer(api_key="test-key", transport=transport)
    result = decomposer.decompose("Pick up the bottle", b"image")
    assert result["object"]["class"] == "bottle"
    assert len(result["subgoals"]) == 4
    assert requests[0]["model"] == "qwen3.5-plus"
    assert requests[0]["messages"][0]["content"][1]["type"] == "image_url"


def test_decomposer_requires_api_key():
    with pytest.raises(RuntimeError, match="DASHSCOPE_API_KEY"):
        VLATaskDecomposer().decompose("Pick up the bottle")


def test_mock_provider_requires_no_api_key():
    decomposer = VLATaskDecomposer(provider_type="mock")
    plan = decomposer.decompose("Pick up the bottle")
    assert plan["object"]["class"] == "bottle"
    assert decomposer.last_latency_s is not None


def test_safety_rejects_workspace_escape_and_emergency_stop():
    safety = QuickDecisionMaker()
    assert safety.validate_action_safety({"pose": {"x": 11, "y": 0}})["safe"] is False
    assert safety.validate_action_safety({}, sensor_data={"emergency_stop": True})["safe"] is False


def test_watchdog_requests_emergency_stop():
    stopped = []

    async def operation():
        await asyncio.sleep(1)

    async def emergency_stop():
        stopped.append(True)

    with pytest.raises(RuntimeError, match="Watchdog timeout"):
        asyncio.run(run_with_watchdog(operation(), emergency_stop, timeout=0.001))
    assert stopped == [True]


def test_monitor_processes_30_frames_per_second_with_fast_feedback():
    seen = []

    async def feedback(frame):
        seen.append(frame)

    async def run_monitor():
        return await RealtimeActionMonitor(feedback, fps=30).run(range(30))

    stats = asyncio.run(run_monitor())
    assert stats.frames == 30
    assert stats.feedback == 30
    assert 25 <= stats.frame_rate <= 35
    assert stats.max_latency_ms < 500
    assert stats.meets(minimum_fps=25, max_latency_ms=500)
    assert seen == list(range(30))