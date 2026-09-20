"""DashScope-compatible task decomposition with strict plan validation."""
import asyncio
import base64
import json
import os
import re
import time
from urllib import request

from pydantic import BaseModel, ConfigDict, Field


class Subgoal(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)

    type: str = Field(pattern="^(navigate|approach_arm|grasp|deliver)$")
    target: list[float] | None = None
    constraints: list[str] = Field(default_factory=list)
    success_criteria: list[str] = Field(default_factory=list)
    duration_sec: float | None = Field(default=None, ge=0, le=600)


class TaskPlan(BaseModel):
    model_config = ConfigDict(extra="forbid")

    object: dict
    subgoals: list[Subgoal] = Field(min_length=1, max_length=20)


class VLATaskDecomposer:
    """Call Qwen 3.5 Plus through the DashScope OpenAI-compatible endpoint."""

    def __init__(self, endpoint=None, model=None, api_key=None, timeout=10, transport=None,
                 provider_type=None):
        self.provider_type = provider_type or os.getenv("VLA_PROVIDER", "qwen")
        self.endpoint = endpoint or os.getenv(
            "DASHSCOPE_ENDPOINT",
            "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions",
        )
        self.model = model or os.getenv("DASHSCOPE_DECOMPOSER_MODEL", "qwen3.5-plus")
        self.api_key = api_key if api_key is not None else os.getenv("DASHSCOPE_API_KEY")
        self.timeout = timeout
        self.transport = transport or (self._mock if self.provider_type == "mock" else self._post)
        self.last_latency_s = None

    def decompose(self, voice_command: str, image_bytes=None, image_media_type="image/jpeg") -> dict:
        if not isinstance(voice_command, str) or not voice_command.strip():
            raise ValueError("voice_command must be a non-empty string")
        if self.provider_type != "mock" and not self.api_key:
            raise RuntimeError("Set DASHSCOPE_API_KEY to enable task decomposition")
        started = time.perf_counter()
        content = [{"type": "text", "text": self._prompt(voice_command)}]
        if image_bytes is not None:
            encoded = base64.b64encode(image_bytes).decode("ascii")
            content.append({"type": "image_url", "image_url": {
                "url": f"data:{image_media_type};base64,{encoded}"}})
        payload = {"model": self.model, "temperature": 0.1, "max_tokens": 1200,
                   "response_format": {"type": "json_object"},
                   "messages": [{"role": "user", "content": content}]}
        try:
            response = self.transport(payload)
            text = response["choices"][0]["message"]["content"]
            return TaskPlan.model_validate(self._parse_json(text)).model_dump()
        except (KeyError, IndexError, TypeError, ValueError) as exc:
            raise RuntimeError("Task provider returned an invalid task plan") from exc
        finally:
            self.last_latency_s = time.perf_counter() - started

    async def adecompose(self, voice_command: str, image_bytes=None, image_media_type="image/jpeg"):
        return await asyncio.to_thread(self.decompose, voice_command, image_bytes, image_media_type)

    def _post(self, payload):
        if not self.api_key:
            raise RuntimeError("Set DASHSCOPE_API_KEY to enable task decomposition")
        body = json.dumps(payload).encode("utf-8")
        req = request.Request(self.endpoint, data=body, headers={
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        }, method="POST")
        with request.urlopen(req, timeout=self.timeout) as response:
            return json.load(response)

    @staticmethod
    def _mock(payload):
        command = payload["messages"][0]["content"][0]["text"]
        command = command.rsplit("COMMAND:", 1)[-1].strip()
        target = command.split("the ", 1)[-1].split(" and bring", 1)[0].strip(" .")
        plan = {"object": {"class": target, "confidence": 1.0}, "subgoals": [
            {"type": "navigate", "constraints": ["stay within workspace"]},
            {"type": "approach_arm", "constraints": ["avoid collisions"]},
            {"type": "grasp", "success_criteria": ["object held"]},
            {"type": "deliver", "success_criteria": ["return to start"]},
        ]}
        return {"choices": [{"message": {"content": json.dumps(plan)}}]}

    @staticmethod
    def _parse_json(text):
        if not isinstance(text, str):
            raise ValueError("Response content is not text")
        cleaned = re.sub(r"^```(?:json)?\s*|\s*```$", "", text.strip())
        start, end = cleaned.find("{"), cleaned.rfind("}")
        if start < 0 or end < start:
            raise ValueError("No JSON object in response")
        return json.loads(cleaned[start:end + 1])

    @staticmethod
    def _prompt(command):
        return ("Decompose this robot fetch command into JSON with keys object and subgoals. "
                "Each subgoal type must be navigate, approach_arm, grasp, or deliver. "
                "Include only measurable targets, constraints, success_criteria, and duration_sec. "
                "Do not invent coordinates when the scene does not provide them.\n\n"
                f"COMMAND: {command}")