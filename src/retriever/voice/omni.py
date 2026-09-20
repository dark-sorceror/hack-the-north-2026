"""One multimodal turn against an OMNI model (Qwen-Omni) over an OpenAI-compatible API.

WHY raw HTTP instead of the `openai` SDK: the request body is the thing most
likely to be wrong on the day (the relay is undocumented), so it is built by
two PURE functions -- `build_messages` and `build_request` -- that the tests
pin down, and posted byte-for-byte as built. The SDK would re-type the payload
and hide Qwen-specific stream fields (`delta.audio`) behind `model_extra`.
Stdlib `urllib` keeps this module importable with zero dependencies.

Wire format, from Alibaba Model Studio's Qwen-Omni docs (see docs/omni-setup.md):

    POST {base}/chat/completions
    {"model": ..., "messages": [...], "stream": true,
     "stream_options": {"include_usage": true},
     "modalities": ["text", "audio"], "audio": {"voice": "Ethan", "format": "wav"}}

    image part  {"type": "image_url",   "image_url":   {"url": "data:image/jpeg;base64,..."}}
    audio part  {"type": "input_audio", "input_audio": {"data": "data:;base64,...", "format": "wav"}}

    stream      data: {"choices": [{"delta": {"content": "Hel"}}]}
                data: {"choices": [{"delta": {"audio": {"data": "<b64 PCM16 24 kHz mono>"}}}]}
                data: {"choices": [], "usage": {...}}
                data: [DONE]

Audio output REQUIRES stream=true, so this client always streams -- which is
also what the latency score wants.
"""

from __future__ import annotations

import base64
import binascii
import json
import os
import re
import struct
import time
import urllib.error
import urllib.request
from collections.abc import Callable, Iterable, Iterator, Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Protocol, runtime_checkable

from retriever.voice.audio import pcm16_to_wav, synth_chime

DEFAULT_BASE_URL = "https://yibuapi.com/v1"
DEFAULT_MODEL = "qwen3.5-omni-flash"
# Ethan exists in both the Qwen3.5-Omni and Qwen3-Omni-Flash voice lists, so the
# default survives a model swap. (Qwen3.5-Omni's own default is Tina.)
DEFAULT_VOICE = "Ethan"
OUTPUT_SAMPLE_RATE = 24000  # Qwen-Omni streams PCM16 mono at 24 kHz

TextCallback = Callable[[str], None]
AudioCallback = Callable[[bytes], None]
# (url, headers, body, timeout_s) -> iterable of raw SSE lines. Injected in tests.
Transport = Callable[[str, Mapping[str, str], bytes, float], Iterable[bytes | str]]


class OmniError(RuntimeError):
    """Anything that went wrong talking to the model, phrased for a human."""


@dataclass(frozen=True)
class OmniConfig:
    """Everything that is still a guess lives here, so fixing a guess is an env var."""

    base_url: str = DEFAULT_BASE_URL
    api_key: str = ""
    model: str = DEFAULT_MODEL
    voice: str = DEFAULT_VOICE
    audio_output: bool = True       # False -> modalities ["text"] (e.g. qwen3.8-omni-flash)
    timeout_s: float = 60.0
    # Alibaba requires input_audio.data as "data:;base64,<b64>". OpenAI itself
    # wants bare base64. If the relay rejects the audio, flip this.
    audio_data_uri: bool = True
    # Some OpenAI-compatible relays drop or reject the system role. If replies
    # ignore the persona, fold the system prompt into the user turn instead.
    system_as_user: bool = False
    max_tokens: int | None = None

    @classmethod
    def from_env(cls, env: Mapping[str, str] | None = None) -> OmniConfig:
        env = os.environ if env is None else env

        def flag(name: str, default: bool) -> bool:
            v = env.get(name)
            if v is None or v.strip() == "":
                return default
            return v.strip().lower() in ("1", "true", "yes", "on")

        max_tokens = env.get("OMNI_MAX_TOKENS", "").strip()
        return cls(
            base_url=(env.get("OMNI_BASE_URL") or DEFAULT_BASE_URL).rstrip("/"),
            api_key=(env.get("OMNI_API_KEY") or "").strip(),
            model=env.get("OMNI_MODEL") or DEFAULT_MODEL,
            voice=env.get("OMNI_VOICE") or DEFAULT_VOICE,
            audio_output=flag("OMNI_AUDIO_OUTPUT", True),
            timeout_s=float(env.get("OMNI_TIMEOUT") or 60.0),
            audio_data_uri=not flag("OMNI_AUDIO_RAW_B64", False),
            system_as_user=flag("OMNI_SYSTEM_AS_USER", False),
            max_tokens=int(max_tokens) if max_tokens else None,
        )


@dataclass(frozen=True)
class OmniReply:
    """One finished turn. Latencies are measured from just before the request is sent."""

    text: str
    audio_wav: bytes | None
    first_token_s: float | None
    total_s: float
    first_audio_s: float | None = None
    sample_rate: int = OUTPUT_SAMPLE_RATE
    usage: dict[str, Any] | None = None
    model: str = ""

    @property
    def audio_seconds(self) -> float:
        if not self.audio_wav or len(self.audio_wav) <= 44:
            return 0.0
        return (len(self.audio_wav) - 44) / 2 / self.sample_rate


@runtime_checkable
class OmniBackend(Protocol):
    """The real client and the mock both satisfy this; nothing above cares which.

    `on_text` gets text deltas as they stream. `on_audio` gets raw PCM16 mono
    chunks at `OmniReply.sample_rate` -- feed them to a player to start speaking
    before the reply is finished.
    """

    def ask(
        self,
        *,
        text: str | None = None,
        image_jpeg: bytes | None = None,
        audio_wav: bytes | None = None,
        system: str | None = None,
        history: Sequence[Mapping[str, Any]] = (),
        on_text: TextCallback | None = None,
        on_audio: AudioCallback | None = None,
    ) -> OmniReply: ...


# ---------------------------------------------------------------- request building


def image_data_url(image: bytes) -> str:
    """JPEG (or PNG) bytes -> data URL. Sniffs the magic so a PNG isn't mislabelled."""
    mime = "image/png" if image[:8] == b"\x89PNG\r\n\x1a\n" else "image/jpeg"
    return f"data:{mime};base64,{base64.b64encode(image).decode('ascii')}"


def audio_data_field(audio: bytes, data_uri: bool = True) -> str:
    b64 = base64.b64encode(audio).decode("ascii")
    # "data:;base64," with an EMPTY media type is what Alibaba's docs show.
    return f"data:;base64,{b64}" if data_uri else b64


def build_messages(
    *,
    text: str | None = None,
    image_jpeg: bytes | None = None,
    audio_wav: bytes | None = None,
    system: str | None = None,
    history: Sequence[Mapping[str, Any]] = (),
    audio_format: str = "wav",
    audio_data_uri: bool = True,
    system_as_user: bool = False,
) -> list[dict[str, Any]]:
    """Pure: inputs -> OpenAI-compatible `messages`.

    Part order is image, audio, text -- media first, the way Alibaba's examples
    do it, so the text reads as an instruction about what came before.
    History entries are passed through unchanged; they must be text-only,
    because Qwen-Omni rejects non-text assistant content.
    """
    if not (text and text.strip()) and image_jpeg is None and audio_wav is None:
        raise ValueError("nothing to send: need at least one of text, image, audio")

    messages: list[dict[str, Any]] = []
    if system and not system_as_user:
        messages.append({"role": "system", "content": system})
    messages.extend(dict(m) for m in history)

    parts: list[dict[str, Any]] = []
    if image_jpeg is not None:
        parts.append({"type": "image_url", "image_url": {"url": image_data_url(image_jpeg)}})
    if audio_wav is not None:
        parts.append(
            {
                "type": "input_audio",
                "input_audio": {
                    "data": audio_data_field(audio_wav, audio_data_uri),
                    "format": audio_format,
                },
            }
        )
    body_text = (text or "").strip()
    if system and system_as_user:
        body_text = f"{system}\n\n{body_text}".strip()
    if body_text:
        parts.append({"type": "text", "text": body_text})

    messages.append({"role": "user", "content": parts})
    return messages


def build_request(config: OmniConfig, messages: list[dict[str, Any]]) -> dict[str, Any]:
    """Pure: config + messages -> the exact JSON body that gets POSTed."""
    body: dict[str, Any] = {
        "model": config.model,
        "messages": messages,
        "stream": True,  # mandatory for audio output, and it's what makes first-token fast
        "stream_options": {"include_usage": True},
    }
    if config.audio_output:
        body["modalities"] = ["text", "audio"]
        body["audio"] = {"voice": config.voice, "format": "wav"}
    else:
        body["modalities"] = ["text"]
    if config.max_tokens:
        body["max_tokens"] = config.max_tokens
    return body


# ---------------------------------------------------------------- stream parsing


def iter_sse_json(
    lines: Iterable[bytes | str], other: list[str] | None = None
) -> Iterator[dict[str, Any]]:
    """SSE lines -> JSON payloads. Stops at [DONE]; skips comments and keep-alives.

    Lines that aren't SSE at all go into `other`, so the caller can tell
    "relay ignored stream=true and sent plain JSON" apart from "empty reply".
    """
    for raw in lines:
        line = raw.decode("utf-8", "replace") if isinstance(raw, bytes) else raw
        line = line.strip()
        if not line or line.startswith(":"):
            continue
        if not line.startswith("data:"):
            if other is not None and not line.startswith(("event:", "id:", "retry:")):
                other.append(line)
            continue
        payload = line[5:].strip()
        if payload == "[DONE]":
            return
        try:
            obj = json.loads(payload)
        except json.JSONDecodeError:
            continue  # a relay's garbled keep-alive shouldn't kill the turn
        if isinstance(obj, dict):
            yield obj


class PcmAssembler:
    """Turns a stream of base64 audio fragments into aligned PCM16 bytes.

    Defensive on purpose, because the docs don't pin the chunking down:
      * fragments may be independently padded base64 OR one base64 stream cut
        at arbitrary points -- decode only whole 4-char quanta, carry the rest;
      * the first bytes may be a WAV header instead of raw PCM -- strip it and
        take the sample rate from it;
      * a chunk may end mid-sample -- carry the odd byte so playback never
        gets a half int16 (which sounds like loud static).
    """

    def __init__(self, sample_rate: int = OUTPUT_SAMPLE_RATE):
        self.sample_rate = sample_rate
        self._b64 = ""
        self._head = b""
        self._header_checked = False
        self._odd = b""
        self._pcm = bytearray()

    def feed(self, b64: str) -> bytes:
        self._b64 += re.sub(r"\s+", "", b64)
        n = len(self._b64) - len(self._b64) % 4
        if n == 0:
            return b""
        chunk, self._b64 = self._b64[:n], self._b64[n:]
        try:
            data = base64.b64decode(chunk)
        except (binascii.Error, ValueError):
            return b""
        return self._push(data)

    def _push(self, data: bytes) -> bytes:
        if not self._header_checked:
            self._head += data
            if len(self._head) < 12 and self._head == b"RIFF"[: len(self._head)]:
                return b""  # too short to tell yet
            if self._head[:4] == b"RIFF":
                idx = self._head.find(b"data", 12)
                if idx < 0 or len(self._head) < idx + 8:
                    return b""  # header not complete yet
                fmt = self._head.find(b"fmt ", 12)
                if 0 <= fmt and len(self._head) >= fmt + 16:
                    self.sample_rate = struct.unpack("<I", self._head[fmt + 12 : fmt + 16])[0]
                data = self._head[idx + 8 :]
            else:
                data = self._head
            self._header_checked = True
            self._head = b""
        data = self._odd + data
        if len(data) % 2:
            data, self._odd = data[:-1], data[-1:]
        else:
            self._odd = b""
        self._pcm += data
        return bytes(data)

    @property
    def pcm(self) -> bytes:
        return bytes(self._pcm)


# ---------------------------------------------------------------- transport


def urllib_transport(
    url: str, headers: Mapping[str, str], body: bytes, timeout_s: float
) -> Iterator[bytes]:
    """POST and yield response lines as they arrive. Stdlib only."""
    req = urllib.request.Request(url, data=body, headers=dict(headers), method="POST")
    try:
        resp = urllib.request.urlopen(req, timeout=timeout_s)
    except urllib.error.HTTPError as e:
        detail = ""
        try:
            detail = e.read().decode("utf-8", "replace")[:500]
        except Exception:
            pass
        raise OmniError(_explain_http(e.code, detail)) from e
    except urllib.error.URLError as e:
        raise OmniError(f"Could not reach {url}: {e.reason}. Check OMNI_BASE_URL and wifi.") from e
    except TimeoutError as e:
        raise OmniError(f"Timed out after {timeout_s:.0f}s connecting to {url}.") from e
    with resp:
        try:
            yield from resp
        except TimeoutError as e:
            raise OmniError(f"Stream stalled for {timeout_s:.0f}s; gave up.") from e


def _explain_http(code: int, detail: str) -> str:
    hint = {
        401: "the key was rejected -- check OMNI_API_KEY",
        403: "the key lacks access to this model or is out of credit",
        404: "wrong path or model -- check OMNI_BASE_URL (should end in /v1) and OMNI_MODEL",
        429: "rate limited -- slow down or check remaining credit",
    }.get(code, "")
    if code == 400 and "audio" in detail.lower():
        hint = "audio rejected -- try OMNI_AUDIO_RAW_B64=1, or OMNI_AUDIO_OUTPUT=0 for text-only"
    msg = f"HTTP {code} from the OMNI endpoint"
    if hint:
        msg += f": {hint}"
    return f"{msg}. Server said: {detail}" if detail else msg


# ---------------------------------------------------------------- clients


class OmniClient:
    """The real thing. One `ask()` = one streamed chat-completions request."""

    def __init__(self, config: OmniConfig | None = None, transport: Transport | None = None):
        self.config = config or OmniConfig.from_env()
        self.transport = transport or urllib_transport

    def ask(
        self,
        *,
        text: str | None = None,
        image_jpeg: bytes | None = None,
        audio_wav: bytes | None = None,
        system: str | None = None,
        history: Sequence[Mapping[str, Any]] = (),
        on_text: TextCallback | None = None,
        on_audio: AudioCallback | None = None,
    ) -> OmniReply:
        cfg = self.config
        if not cfg.api_key:
            raise OmniError("OMNI_API_KEY is not set. Use MockOmniClient / --mock to run offline.")
        messages = build_messages(
            text=text,
            image_jpeg=image_jpeg,
            audio_wav=audio_wav,
            system=system,
            history=history,
            audio_data_uri=cfg.audio_data_uri,
            system_as_user=cfg.system_as_user,
        )
        body = json.dumps(build_request(cfg, messages)).encode("utf-8")
        headers = {
            "Authorization": f"Bearer {cfg.api_key}",
            "Content-Type": "application/json",
            "Accept": "text/event-stream",
        }
        url = f"{cfg.base_url}/chat/completions"

        t0 = time.perf_counter()
        first_token: float | None = None
        first_audio: float | None = None
        text_parts: list[str] = []
        transcript_parts: list[str] = []
        usage: dict[str, Any] | None = None
        model = ""
        pcm = PcmAssembler()
        other: list[str] = []

        def handle(obj: dict[str, Any], key: str) -> None:
            nonlocal first_token, first_audio, usage, model
            if "error" in obj and not obj.get("choices"):
                err = obj["error"]
                msg = err.get("message") if isinstance(err, dict) else str(err)
                raise OmniError(f"Model returned an error: {msg}")
            model = obj.get("model") or model
            if obj.get("usage"):
                usage = obj["usage"]
            for choice in obj.get("choices") or ():
                delta = choice.get(key) or {}
                content = delta.get("content")
                if isinstance(content, str) and content:
                    if first_token is None:
                        first_token = time.perf_counter() - t0
                    text_parts.append(content)
                    if on_text:
                        on_text(content)
                audio = delta.get("audio")
                if isinstance(audio, dict):
                    tr = audio.get("transcript")
                    if isinstance(tr, str) and tr:
                        transcript_parts.append(tr)
                    data = audio.get("data")
                    if isinstance(data, str) and data:
                        new = pcm.feed(data)
                        if new:
                            if first_audio is None:
                                first_audio = time.perf_counter() - t0
                            if on_audio:
                                on_audio(new)

        n_events = 0
        for obj in iter_sse_json(self.transport(url, headers, body, cfg.timeout_s), other):
            n_events += 1
            handle(obj, "delta")
        if n_events == 0 and other:
            # The relay ignored stream=true (or sent a 200 with a JSON error body).
            try:
                whole = json.loads("\n".join(other))
            except json.JSONDecodeError:
                raise OmniError(f"Unrecognised response from the endpoint: {other[0][:200]}")
            if isinstance(whole, dict):
                handle(whole, "message")
        total = time.perf_counter() - t0

        text_out = "".join(text_parts)
        if not text_out and transcript_parts:
            # Older Qwen-Omni variants put the words only in audio.transcript.
            text_out = "".join(transcript_parts)
            if on_text:
                on_text(text_out)
        raw = pcm.pcm
        return OmniReply(
            text=text_out,
            audio_wav=pcm16_to_wav(raw, pcm.sample_rate) if raw else None,
            first_token_s=first_token,
            total_s=total,
            first_audio_s=first_audio,
            sample_rate=pcm.sample_rate,
            usage=usage,
            model=model or cfg.model,
        )


# Keyword -> reply for the mock. Deliberately covers BOTH candidate niches
# (fetch my belongings / sort litter) so the demo works whichever one wins.
_MOCK_OBJECTS = (
    "inhaler", "keys", "wallet", "phone", "glasses", "medication", "pill bottle",
    "remote", "water bottle", "mug", "cup", "can", "bottle", "wrapper", "banana peel",
)


def _mock_reply(text: str, has_image: bool, has_audio: bool, audio_s: float) -> str:
    t = (text or "").lower()
    obj = next((o for o in _MOCK_OBJECTS if o in t), None)
    if any(k in t for k in ("sort", "recycl", "trash", "litter", "compost", "throw", "bin")):
        obj = obj or "plastic bottle"
        return (
            f"That looks like a {obj}. It goes in recycling, I'll sort it now.\n"
            f'INTENT: {{"intent": "sort", "object": "{obj}", "bin": "recycling"}}'
        )
    if "where" in t:
        obj = obj or "keys"
        return (
            f"I don't see your {obj} right now. I last saw them near the door.\n"
            f'INTENT: {{"intent": "where_is", "object": "{obj}"}}'
        )
    if any(k in t for k in ("bring", "fetch", "get me", "grab", "hand me", "need my")) or obj:
        obj = obj or "item"
        seen = "I can see it on the table. " if has_image else ""
        return (
            f"{seen}Getting your {obj} now.\n"
            f'INTENT: {{"intent": "fetch", "object": "{obj}"}}'
        )
    if has_audio and not t.strip():
        return (
            f"I heard {audio_s:.1f} seconds of audio, but mock mode cannot understand speech. "
            "Set OMNI_API_KEY to talk to the real model.\n"
            'INTENT: {"intent": "none"}'
        )
    if has_image:
        return (
            "I can see a table with a few objects on it. What would you like me to do?\n"
            'INTENT: {"intent": "none"}'
        )
    return 'I\'m listening. What do you need?\nINTENT: {"intent": "none"}'


class MockOmniClient:
    """Canned, keyword-routed replies with a synthesized chime as the "voice".

    Runs the real `build_messages` so the payload path is exercised offline,
    and records every request in `.calls` for tests to inspect.
    """

    def __init__(
        self,
        replies: Mapping[str, str] | None = None,
        delay_s: float = 0.0,
        chunk_chars: int = 12,
        sample_rate: int = OUTPUT_SAMPLE_RATE,
    ):
        self.replies = dict(replies or {})
        self.delay_s = delay_s
        self.chunk_chars = max(1, chunk_chars)
        self.sample_rate = sample_rate
        self.calls: list[dict[str, Any]] = []

    def ask(
        self,
        *,
        text: str | None = None,
        image_jpeg: bytes | None = None,
        audio_wav: bytes | None = None,
        system: str | None = None,
        history: Sequence[Mapping[str, Any]] = (),
        on_text: TextCallback | None = None,
        on_audio: AudioCallback | None = None,
    ) -> OmniReply:
        t0 = time.perf_counter()
        messages = build_messages(
            text=text, image_jpeg=image_jpeg, audio_wav=audio_wav, system=system, history=history
        )
        self.calls.append({"messages": messages, "text": text})

        lowered = (text or "").lower()
        reply = next((r for k, r in self.replies.items() if k.lower() in lowered), None)
        if reply is None:
            audio_s = max(0.0, (len(audio_wav) - 44) / 32000) if audio_wav else 0.0
            reply = _mock_reply(text or "", image_jpeg is not None, audio_wav is not None, audio_s)

        if self.delay_s:
            time.sleep(self.delay_s)  # pretend network time-to-first-token
        first_token = time.perf_counter() - t0
        for i in range(0, len(reply), self.chunk_chars):
            if on_text:
                on_text(reply[i : i + self.chunk_chars])

        pcm = synth_chime(self.sample_rate)
        first_audio = time.perf_counter() - t0
        if on_audio:
            step = self.sample_rate // 10 * 2  # 100 ms chunks, like a real stream
            for i in range(0, len(pcm), step):
                on_audio(pcm[i : i + step])
        return OmniReply(
            text=reply,
            audio_wav=pcm16_to_wav(pcm, self.sample_rate),
            first_token_s=first_token,
            total_s=time.perf_counter() - t0,
            first_audio_s=first_audio,
            sample_rate=self.sample_rate,
            model="mock",
        )
