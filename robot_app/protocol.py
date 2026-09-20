"""Small, validated command vocabulary. No generated code or raw motor values."""
import asyncio
import json
import os
import secrets
import time
from typing import Literal
from uuid import uuid4

from pydantic import BaseModel, ConfigDict, Field
from websockets.asyncio.client import connect


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)


class Pose(StrictModel):
    frame: str = Field(min_length=1, max_length=80)
    x: float
    y: float
    z: float = 0
    yaw: float = 0


class Request(StrictModel):
    id: str = Field(default_factory=lambda: uuid4().hex, min_length=1, max_length=80)
    expires_at: float = Field(default_factory=lambda: time.time() + 120)
    action: Literal["save_start", "locate", "approach", "grasp", "verify_grasp", "stow", "return_start", "stop"]
    target: str | None = Field(default=None, max_length=120)
    pose: Pose | None = None


class Reply(StrictModel):
    id: str
    ok: bool
    simulated: bool
    result: dict = Field(default_factory=dict)
    error: str | None = None


def token():
    value = os.environ.get("ROBOT_TOKEN", "")
    if len(value) < 24:
        raise RuntimeError("Set ROBOT_TOKEN to a shared random secret of at least 24 characters")
    return value


def authenticated(ws):
    supplied = ws.request.headers.get_all("Authorization")
    return len(supplied) == 1 and secrets.compare_digest(supplied[0], "Bearer " + token())


async def rpc(url, request: Request, timeout=120):
    # No automatic retries: uncertain movement must never be replayed.
    async with asyncio.timeout(timeout):
        async with connect(url, additional_headers={"Authorization": "Bearer " + token()},
                           proxy=None, max_size=65536, open_timeout=5) as ws:
            await ws.send(request.model_dump_json())
            reply = Reply.model_validate_json(await ws.recv())
            if reply.id != request.id:
                raise RuntimeError("Mismatched command acknowledgement")
            if not reply.ok:
                raise RuntimeError(reply.error or "Remote command failed")
            return reply


async def send_json(ws, value):
    await ws.send(json.dumps(value))
