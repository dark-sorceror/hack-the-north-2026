"""Authenticated voice relay: provider credentials stay on the cloud host."""
import argparse
import asyncio
import json
import os
import time
from urllib.parse import urlencode

from websockets.asyncio.client import connect
from websockets.asyncio.server import serve

from .audit import record
from .protocol import authenticated, token


ALLOWED = {"session.update", "input_audio_buffer.append", "conversation.item.create",
           "response.create", "response.cancel"}


async def relay(client):
    if not authenticated(client):
        await client.close(1008, "Unauthorized")
        return
    key = os.environ["YIBU_API_KEY"]
    model = os.getenv("YIBU_MODEL", "qwen3.5-omni-plus-realtime")
    endpoint = os.getenv("YIBU_ENDPOINT", "wss://yibuapi.com/v1/realtime")
    url = endpoint + ("&" if "?" in endpoint else "?") + urlencode({"model": model})
    started = time.monotonic()
    received_response = False
    pending_response = False
    failed = False
    tasks = []
    try:
        async with connect(url, additional_headers={"Authorization": "Bearer " + key},
                           proxy=None, open_timeout=30, max_size=4 * 1024 * 1024) as upstream:
            async def upload():
                async for raw in client:
                    event = json.loads(raw)
                    if not isinstance(event, dict) or event.get("type") not in ALLOWED:
                        raise ValueError("Unsupported voice event")
                    await upstream.send(raw)

            async def download():
                nonlocal started, received_response, pending_response, failed
                async for raw in upstream:
                    event = json.loads(raw)
                    kind = event.get("type")
                    if kind == "session.created":
                        actual = event.get("session", {}).get("model")
                        if actual and actual != model:
                            raise ValueError("Provider model mismatch")
                    if kind == "response.created":
                        started = time.monotonic()
                        pending_response = True
                    if kind == "error":
                        # Do not forward provider errors which may echo credentials/input.
                        details = event.get("error", {})
                        if isinstance(details, dict):
                            summary = {name: details.get(name) for name in ("type", "code", "message")
                                       if details.get(name) is not None}
                        else:
                            summary = {"type": type(details).__name__}
                        print(f"Provider error: {summary}", flush=True)
                        raise RuntimeError("Provider rejected a realtime event")
                    if kind == "response.done":
                        status = event.get("response", {}).get("status")
                        record(event, model, key, endpoint, started, ok=status not in {"failed", "cancelled"})
                        received_response = True
                        pending_response = False
                    await client.send(raw)

            tasks = [asyncio.create_task(upload()), asyncio.create_task(download())]
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                task.result()
    except Exception:
        failed = True
        await client.close(1011, "Voice provider session failed; check configuration and model access")
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        if failed or pending_response or not received_response:
            record({}, model, key, endpoint, started, ok=False, error="session_ended_without_complete_accounting")
        await client.close()


async def run(host, port):
    token()
    if not os.getenv("YIBU_API_KEY"):
        raise RuntimeError("Set YIBU_API_KEY on the cloud host")
    if not os.getenv("YIBU_ENDPOINT", "wss://yibuapi.com/v1/realtime").startswith("wss://"):
        raise RuntimeError("Provider endpoint must use wss://")
    async with serve(relay, host, port, max_size=4 * 1024 * 1024):
        print(f"Voice relay listening on {host}:{port}", flush=True)
        await asyncio.Future()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    asyncio.run(run(args.host, args.port))
