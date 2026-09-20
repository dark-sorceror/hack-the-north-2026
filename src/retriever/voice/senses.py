"""See + hear + speak in one turn: camera frame and utterance go to the OMNI
model TOGETHER, and the robot answers out loud as itself.

WHY one model call instead of STT -> LLM -> TTS: the request is usually about
the scene ("is that my inhaler?", "which bin does this go in?"), so the model
needs the words and the pixels at the same time, and one round trip beats
three for latency.

The model never drives the robot. It may end its reply with a machine-read
line --  INTENT: {"intent": "fetch", "object": "inhaler"}  -- and a separate
planner decides whether and how to act on it. That keeps a hallucinated
sentence from ever becoming a motor command without a check in between.
"""

from __future__ import annotations

import json
import re
from collections import deque
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol, runtime_checkable

from retriever.voice.audio import (
    AudioError,
    find_cut_sample,
    pcm16_to_wav,
    record_push_to_talk,
    to_model_wav,
    trim_pcm16,
    wav_to_pcm16,
)
from retriever.voice.omni import AudioCallback, OmniBackend, OmniReply, TextCallback

INTENTS = ("fetch", "sort", "where_is", "none")

DEFAULT_ROLE = (
    "You are Retriever, a small home robot with a camera, a microphone, a speaker, "
    "a wheeled base and one arm. You help people by fetching their belongings "
    "and by sorting litter into the right bin."
)

SYSTEM_PROMPT_TEMPLATE = """{role}

You are talking OUT LOUD through your speaker to the person in front of you.
- Reply in one or two short sentences, in the language the person used. No lists, no markdown, no emojis.
- The image, if any, is what your camera sees right now. Use it: say where the thing is or what it is when that helps.
- The audio, if any, is the person speaking to you. Answer what they actually said.
- Never pretend. If you cannot see the object or are unsure, say so plainly and ask.
- You do not move on your own. When the person wants something done, say in a few words what you will do.

Always finish with exactly one final line, written on one line with double quotes:
INTENT: {{"intent": "<fetch|sort|where_is|none>", "object": "<short object name or null>"}}
  fetch    = bring an object to the person ("bring me my keys")
  sort     = put an item in the right bin; also add "bin": "recycling", "compost" or "trash"
  where_is = the person asks where something is
  none     = chatting, describing, or answering a question
You may add "seen": true or false for whether the object is visible in the image."""

DEFAULT_SYSTEM_PROMPT = SYSTEM_PROMPT_TEMPLATE.format(role=DEFAULT_ROLE)

_SYNONYMS = {
    "fetch": "fetch", "bring": "fetch", "get": "fetch", "retrieve": "fetch", "grab": "fetch",
    "hand": "fetch", "hand_over": "fetch", "deliver": "fetch", "pick_up": "fetch",
    "pickup": "fetch", "give": "fetch",
    "sort": "sort", "recycle": "sort", "dispose": "sort", "throw_away": "sort",
    "trash": "sort", "tidy": "sort", "clean": "sort", "clean_up": "sort", "bin": "sort",
    "where_is": "where_is", "whereis": "where_is", "where": "where_is", "find": "where_is",
    "locate": "where_is", "search": "where_is", "look_for": "where_is",
    "none": "none", "null": "none", "no": "none", "nothing": "none", "chat": "none",
    "answer": "none", "describe": "none", "no_action": "none", "": "none",
}
_OBJECT_KEYS = ("object", "item", "target", "obj", "thing")
_NAME_KEYS = ("intent", "action", "type")


@dataclass(frozen=True)
class Intent:
    """What the model thinks the person wants. A hint for the planner, never a command.

    `valid` is True only when the line was strict JSON with a known intent
    name; `intent`/`object` are still best-effort filled from sloppy output.
    """

    intent: str = "none"
    object: str | None = None
    args: dict[str, Any] = field(default_factory=dict)
    raw: str | None = None
    valid: bool = False

    @property
    def actionable(self) -> bool:
        return self.intent != "none"

    def to_dict(self) -> dict[str, Any]:
        return {"intent": self.intent, "object": self.object, **self.args}


# ---------------------------------------------------------------- intent parsing

_MARKER = re.compile(r"(?i)\bintent\b[*_`\s]*[:=]")


def _depth_at(s: str, pos: int) -> int:
    """Brace depth at `pos`, ignoring braces inside double-quoted strings."""
    depth, in_str, esc = 0, False, False
    for ch in s[:pos]:
        if in_str:
            if esc:
                esc = False
            elif ch == "\\":
                esc = True
            elif ch == '"':
                in_str = False
        elif ch == '"':
            in_str = True
        elif ch == "{":
            depth += 1
        elif ch == "}":
            depth = max(0, depth - 1)
    return depth


def _match_braces(s: str, start: int) -> int:
    """Index just past the brace closing s[start] == '{', or len(s) if it never closes."""
    depth, in_str, esc = 0, False, False
    for i in range(start, len(s)):
        ch = s[i]
        if in_str:
            if esc:
                esc = False
            elif ch == "\\":
                esc = True
            elif ch == '"':
                in_str = False
        elif ch == '"':
            in_str = True
        elif ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return i + 1
    return len(s)


def _locate_intent(s: str) -> tuple[int, int, str, bool] | None:
    """-> (start, end, candidate, is_json) of the intent section in `s`, or None."""
    markers = [m for m in _MARKER.finditer(s) if _depth_at(s, m.start()) == 0]
    if markers:
        m = markers[-1]
        # Swallow decoration before the marker on the same line ("**INTENT:**", "- INTENT:").
        start = m.start()
        while start > 0 and s[start - 1] in " \t*_`>#-":
            start -= 1
        j = m.end()
        while j < len(s) and s[j] in " \t*_`":
            j += 1
        if s.startswith("json", j):
            j += 4
        while j < len(s) and s[j] in " \t\r\n`":
            j += 1
        if j < len(s) and s[j] == "{":
            end = _match_braces(s, j)
            cand = s[j:end]
            while end < len(s) and s[end] in " \t`":
                end += 1
            return start, end, cand, True
        nl = s.find("\n", j)
        end = len(s) if nl < 0 else nl
        return start, end, s[j:end].strip(" \t`*_\"'."), False
    # No marker: the model may have emitted a bare {"intent": ...} object.
    k = s.lower().rfind('"intent"')
    if k < 0:
        k = s.lower().rfind("'intent'")
    if k < 0:
        return None
    b = s.rfind("{", 0, k)
    if b < 0:
        return None
    end = _match_braces(s, b)
    return b, end, s[b:end], True


def _loads_lenient(cand: str) -> tuple[dict[str, Any] | None, bool]:
    """-> (dict or None, parsed_strictly)."""
    try:
        obj = json.loads(cand)
        return (obj, True) if isinstance(obj, dict) else (None, False)
    except (json.JSONDecodeError, ValueError):
        pass
    s = cand.strip().strip("`")
    if '"' not in s:
        s = s.replace("'", '"')
    s = re.sub(r"\bNone\b", "null", s)
    s = re.sub(r"\bTrue\b", "true", s)
    s = re.sub(r"\bFalse\b", "false", s)
    s = re.sub(r'([{,]\s*)([A-Za-z_][\w ]*?)\s*:', r'\1"\2":', s)  # bare keys

    def _quote_bare(m: re.Match[str]) -> str:
        v = m.group(1).strip()
        if v in ("true", "false", "null") or re.fullmatch(r"-?\d+(\.\d+)?", v):
            return f": {v}{m.group(2)}"
        return f': "{v}"{m.group(2)}'

    s = re.sub(r':\s*([A-Za-z_][\w \-]*?)\s*([,}])', _quote_bare, s)  # bare values
    s = re.sub(r",\s*([}\]])", r"\1", s)  # trailing commas
    s += "}" * max(0, s.count("{") - s.count("}"))
    try:
        obj = json.loads(s)
        return (obj, False) if isinstance(obj, dict) else (None, False)
    except (json.JSONDecodeError, ValueError):
        return None, False


def _regex_pairs(cand: str) -> dict[str, Any]:
    pairs = re.findall(r"[\"']?(\w+)[\"']?\s*[:=]\s*[\"']?([^\"',{}\n]*)[\"']?", cand)
    return {k.lower(): v.strip() for k, v in pairs if v.strip()}


def _norm_name(name: Any) -> str:
    return re.sub(r"[\s\-]+", "_", str(name or "").strip().lower())


def _clean_object(v: Any) -> str | None:
    if v is None or isinstance(v, (dict, list, bool)):
        return None
    s = str(v).strip().strip("\"'.").strip()
    return None if s.lower() in ("", "null", "none", "n/a", "nothing", "unknown") else s


def _make_intent(data: Mapping[str, Any], raw: str, strict: bool) -> Intent:
    data = {str(k).lower(): v for k, v in data.items()}
    name_raw = next((data[k] for k in _NAME_KEYS if k in data), "none")
    key = _norm_name(name_raw)
    name = _SYNONYMS.get(key, "none")
    obj = _clean_object(next((data[k] for k in _OBJECT_KEYS if k in data), None))
    args = {k: v for k, v in data.items() if k not in _NAME_KEYS + _OBJECT_KEYS}
    if name == "none" and key not in _SYNONYMS:
        args["unknown_intent"] = str(name_raw)
    return Intent(intent=name, object=obj, args=args, raw=raw, valid=strict and key in INTENTS)


def parse_intent(text: Any) -> Intent:
    """Pull the INTENT line out of a reply. Never raises: bad input -> Intent('none').

    Tolerates: no intent line, prose after it, markdown decoration, code
    fences, single quotes, unquoted keys/values, trailing commas, a missing
    closing brace, the bare form `INTENT: fetch keys`, and a JSON object
    with an "intent" key but no INTENT: prefix.
    """
    try:
        if not isinstance(text, str) or not text.strip():
            return Intent()
        s = text.replace("“", '"').replace("”", '"')
        s = s.replace("‘", "'").replace("’", "'")
        loc = _locate_intent(s)
        if loc is None:
            return Intent()
        _, _, cand, is_json = loc
        if not is_json:
            words = cand.split(None, 1)
            if not words:
                return Intent(raw=cand)
            data: dict[str, Any] = {"intent": words[0].strip(",:;")}
            if len(words) > 1:
                data["object"] = words[1]
            return _make_intent(data, cand, strict=False)
        parsed, strict = _loads_lenient(cand)
        if parsed is None:
            parsed, strict = _regex_pairs(cand), False
        return _make_intent(parsed, cand, strict)
    except Exception:  # belt and braces: a parser must not take the robot down
        return Intent(raw=text if isinstance(text, str) else None)


def strip_intent(text: str) -> str:
    """The reply minus its INTENT section: what a human should read or hear."""
    try:
        if not isinstance(text, str):
            return ""
        loc = _locate_intent(text)
        if loc is None:
            return text.strip()
        start, end, _, _ = loc
        left, right = text[:start].rstrip(), text[end:].lstrip()
        out = f"{left} {right}" if left and right else left or right
        out = re.sub(r"```(?:json)?", "", out)  # a spoken reply never needs code fences
        out = re.sub(r"[ \t]+\n", "\n", out)
        return re.sub(r"\n{3,}", "\n\n", out).strip()
    except Exception:
        return text.strip() if isinstance(text, str) else ""


def _alnum(s: str) -> int:
    return sum(ch.isalnum() for ch in s)


def spoken_tail_fraction(text: str) -> float | None:
    """If the intent section is at the END of `text`, the fraction of speech
    before it (by letter count, a decent proxy for speaking time). Else None."""
    loc = _locate_intent(text) if isinstance(text, str) else None
    if loc is None:
        return None
    start, end, _, _ = loc
    if _alnum(text[end:]) > 0:
        return None  # intent isn't last; don't guess where to cut
    total = _alnum(text)
    return _alnum(text[:start]) / total if total else None


# ---------------------------------------------------------------- camera / mic


class CameraError(RuntimeError):
    pass


@runtime_checkable
class Camera(Protocol):
    def grab(self) -> bytes | None:
        """Latest frame as JPEG bytes, or None if there is no frame right now."""
        ...


@runtime_checkable
class Microphone(Protocol):
    def record(self) -> bytes | None:
        """One utterance as WAV bytes, or None if the user cancelled."""
        ...


class StaticCamera:
    """A fixed JPEG -- for tests, replays, and venues where the webcam fights you."""

    def __init__(self, jpeg: bytes):
        self.jpeg = jpeg

    @classmethod
    def from_file(cls, path: str | Path) -> StaticCamera:
        return cls(Path(path).read_bytes())

    def grab(self) -> bytes | None:
        return self.jpeg


class WebcamCamera:
    """cv2 webcam. Keeps the device open: reopening costs ~1 s on macOS."""

    def __init__(self, index: int = 0, warmup_frames: int = 5, quality: int = 85):
        try:
            import cv2
        except ImportError as e:
            raise CameraError("opencv-python is not installed") from e
        self._cv2 = cv2
        self.quality = quality
        self._cap = cv2.VideoCapture(index)
        if not self._cap.isOpened():
            raise CameraError(
                f"Could not open camera {index}. macOS: System Settings > Privacy & Security > "
                "Camera -> enable your terminal app. Or pass --image PATH."
            )
        for _ in range(warmup_frames):  # first frames are often black while exposure settles
            self._cap.read()

    def grab(self) -> bytes | None:
        # Drop frames buffered while we were waiting on the model, so the robot
        # describes NOW, not two seconds ago.
        for _ in range(2):
            self._cap.grab()
        ok, frame = self._cap.read()
        if not ok or frame is None:
            return None
        ok, buf = self._cv2.imencode(".jpg", frame, [self._cv2.IMWRITE_JPEG_QUALITY, self.quality])
        return buf.tobytes() if ok else None

    def close(self) -> None:
        self._cap.release()


class PushToTalkMic:
    """Enter to start, Enter to stop."""

    def __init__(self, max_s: float = 30.0, already_started: bool = False):
        self.max_s = max_s
        self.already_started = already_started

    def record(self) -> bytes | None:
        return record_push_to_talk(max_s=self.max_s, already_started=self.already_started)


def downscale_jpeg(jpeg: bytes, max_side: int = 640, quality: int = 75) -> bytes:
    """Shrink a frame before upload. Image tokens and upload time both scale with
    pixels, and 640 px is plenty to tell a mug from an inhaler. Returns the input
    unchanged if it is already small or can't be decoded."""
    try:
        import cv2
        import numpy as np
    except ImportError:
        return jpeg
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return jpeg
    h, w = img.shape[:2]
    if max(h, w) <= max_side:
        return jpeg
    s = max_side / float(max(h, w))
    img = cv2.resize(img, (max(1, round(w * s)), max(1, round(h * s))), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes() if ok else jpeg


# ---------------------------------------------------------------- the turn


@dataclass(frozen=True)
class Turn:
    reply: OmniReply
    intent: Intent
    said: str                    # reply minus the INTENT line
    audio_wav: bytes | None      # reply audio trimmed to `said` (INTENT line not spoken)
    cut_sample: int | None       # where to stop a streaming player; None = play it all
    image_sent: bool
    audio_sent: bool


class Senses:
    """Camera + mic + OMNI model, with a short rolling memory of the conversation.

    History is text only (Qwen-Omni rejects non-text assistant turns) and
    old frames are NOT resent -- each turn pays for one image, not five.
    """

    def __init__(
        self,
        omni: OmniBackend,
        camera: Camera | None = None,
        mic: Microphone | None = None,
        system_prompt: str | None = None,
        history_turns: int = 4,
        max_image_side: int = 640,
    ):
        self.omni = omni
        self.camera = camera
        self.mic = mic
        self.system_prompt = system_prompt or DEFAULT_SYSTEM_PROMPT
        self.max_image_side = max_image_side
        self._history: deque[tuple[dict[str, Any], dict[str, Any]]] = deque(maxlen=history_turns)

    @property
    def history(self) -> list[dict[str, Any]]:
        return [m for pair in self._history for m in pair]

    def reset(self) -> None:
        self._history.clear()

    def look(self) -> bytes | None:
        if self.camera is None:
            return None
        frame = self.camera.grab()
        return downscale_jpeg(frame, self.max_image_side) if frame else None

    def turn(
        self,
        text: str | None = None,
        audio_wav: bytes | None = None,
        image_jpeg: bytes | None = None,
        *,
        look: bool = True,
        listen: bool = False,
        context: str | None = None,
        on_text: TextCallback | None = None,
        on_audio: AudioCallback | None = None,
    ) -> Turn:
        """One exchange. Explicit `image_jpeg` / `audio_wav` win over the camera / mic.

        `context` is for facts the robot knows but can't see -- e.g. memory's
        last sighting of the keys -- so `where_is` can be answered honestly.
        """
        image = downscale_jpeg(image_jpeg, self.max_image_side) if image_jpeg else None
        if image is None and look:
            image = self.look()
        audio = audio_wav
        if audio is None and listen and self.mic is not None:
            audio = self.mic.record()
        if audio is not None:
            try:
                audio = to_model_wav(audio)
            except AudioError:
                pass  # send as-is; the model accepts other rates, just slower to upload

        prompt = (text or "").strip()
        if context:
            prompt = f"{prompt}\n\n[What you already know: {context.strip()}]".strip()

        reply = self.omni.ask(
            text=prompt or None,
            image_jpeg=image,
            audio_wav=audio,
            system=self.system_prompt,
            history=self.history,
            on_text=on_text,
            on_audio=on_audio,
        )

        intent = parse_intent(reply.text)
        said = strip_intent(reply.text)
        trimmed, cut = _trim_reply_audio(reply)

        user_note = (text or "").strip() or ("(spoke to you out loud)" if audio else "(no words)")
        if image is not None:
            user_note += " [showed you a camera frame]"
        self._history.append(
            ({"role": "user", "content": user_note}, {"role": "assistant", "content": reply.text})
        )
        return Turn(
            reply=reply,
            intent=intent,
            said=said,
            audio_wav=trimmed,
            cut_sample=cut,
            image_sent=image is not None,
            audio_sent=audio is not None,
        )


def _trim_reply_audio(reply: OmniReply) -> tuple[bytes | None, int | None]:
    if not reply.audio_wav:
        return None, None
    frac = spoken_tail_fraction(reply.text)
    if frac is None:
        return reply.audio_wav, None
    try:
        pcm, rate, _ = wav_to_pcm16(reply.audio_wav)
        cut = find_cut_sample(pcm, rate, frac)
        if cut >= len(pcm) // 2:
            return reply.audio_wav, None
        return pcm16_to_wav(trim_pcm16(pcm, rate, cut), rate), cut
    except Exception:
        return reply.audio_wav, None
