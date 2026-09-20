import asyncio
import json
import time

from websockets.asyncio.client import connect
from websockets.asyncio.server import serve

from robot_app import cloud
from robot_app.audit import record, summarize


def test_usage_preserves_unknown_and_derives_total(tmp_path, monkeypatch):
    path = tmp_path / "usage.jsonl"
    monkeypatch.setenv("YIBU_AUDIT_LOG", str(path))
    unknown = record({}, "model", "secret-value", "wss://example.test", time.monotonic())
    assert unknown["total_tokens"] is None
    row = record({"response": {"usage": {"input_tokens": 3, "output_tokens": 4}}},
                 "model", "secret-value", "wss://example.test", time.monotonic())
    assert row["total_tokens"] == 7
    assert "secret-value" not in path.read_text()
    summarize(path, tmp_path / "summary")
    summary = json.loads((tmp_path / "summary/usage_summary.json").read_text())
    assert summary["groups"][0]["missing_total_tokens"] == 1


def test_cloud_relays_audio_and_accounts_response(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBOT_TOKEN", "test-token-for-local-tests-only-123")
    monkeypatch.setenv("YIBU_API_KEY", "fake-provider-key")
    monkeypatch.setenv("YIBU_AUDIT_LOG", str(tmp_path / "usage.jsonl"))

    async def scenario():
        async def provider(ws):
            assert ws.request.headers["Authorization"] == "Bearer fake-provider-key"
            await ws.send(json.dumps({"type": "session.created", "session": {"model": "qwen3.5-omni-plus-realtime"}}))
            event = json.loads(await ws.recv())
            assert event["type"] == "input_audio_buffer.append"
            assert event["audio"] == "AAAA"
            await ws.send(json.dumps({"type": "response.audio.delta", "delta": "AAAA"}))
            await ws.send(json.dumps({"type": "response.done", "response": {
                "status": "completed", "usage": {"input_tokens": 4, "output_tokens": 5}}}))
            await ws.wait_closed()

        async with serve(provider, "127.0.0.1", 0) as upstream:
            monkeypatch.setenv("YIBU_ENDPOINT", f"ws://127.0.0.1:{upstream.sockets[0].getsockname()[1]}")
            async with serve(cloud.relay, "127.0.0.1", 0) as relay:
                async with connect(f"ws://127.0.0.1:{relay.sockets[0].getsockname()[1]}", proxy=None,
                                   additional_headers={"Authorization": "Bearer test-token-for-local-tests-only-123"}) as ws:
                    assert json.loads(await ws.recv())["type"] == "session.created"
                    await ws.send(json.dumps({"type": "input_audio_buffer.append", "audio": "AAAA"}))
                    assert json.loads(await ws.recv())["type"] == "response.audio.delta"
                    assert json.loads(await ws.recv())["type"] == "response.done"

    asyncio.run(scenario())
    rows = [json.loads(line) for line in (tmp_path / "usage.jsonl").read_text().splitlines()]
    assert len(rows) == 1
    assert rows[0]["total_tokens"] == 9
