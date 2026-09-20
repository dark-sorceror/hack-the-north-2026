#!/usr/bin/env python3
"""The same voice turn as `mac_voice.py`, but on OpenAI instead of Qwen-Omni.

    python3 scripts/openai_voice.py                      # one turn, fixed 4 s
    python3 scripts/openai_voice.py --push-to-talk
    python3 scripts/openai_voice.py --mode direct       # one gpt-audio call
    python3 scripts/openai_voice.py --bench 3 --say "go get that goose"

WHY A SECOND VOICE PATH AT ALL. The Qwen-Omni relay works, but its latency on
byte-identical input measured 2.96, 4.53, 12.09, 14.30 and 19.38 s across runs
-- a 12.1 s median. Nineteen seconds of silence after someone speaks does not
read as a slow robot, it reads as a broken one, and we cannot tell the judges
which of those two they will get.

Measured here over 17 runs per mode, same synthesised utterance every time:

    approach   runs   min    median   p90     max
    direct      17    1.84   2.28     3.00    10.07
    pipeline    17    1.85   2.51     3.19     7.57
    Qwen-Omni    5    2.96  12.09    19.38    19.38

Read that honestly: the MEDIAN improves 5x and 15 of 17 runs land under 3.5 s,
which is the difference between a demo that feels alive and one that does not.
The TAIL does not go away -- one direct run took 10.07 s. This is a better bet,
not a fixed problem, and anything built on it still wants a spoken "one moment"
if a reply has not landed in ~3 s.

WHAT IT COSTS, and this is the whole design problem. Qwen-Omni takes audio and
an image in ONE call. **OpenAI has no model that does.** `gpt-audio` (and
`-mini`, and `1.5`) reject `image_url` outright -- "This model does not support
image_url content" -- and the Responses API answers "Audio input is not
available" for every model on this account. The direct equivalent of what we
already have does not exist here, so the image has to get in some other way.
Two ways, both implemented, `--mode` picks:

  direct            One `gpt-audio` call: speech in, speech out. The camera
                    frame is captioned by a separate cheap vision call that
                    runs DURING the recording -- see `_Scene` -- so the
                    sentence describing the frame is already in hand when the
                    person stops talking and costs no wall-clock latency. The
                    model gets the scene as text rather than pixels.

  pipeline          transcribe -> vision chat WITH the real image -> TTS.
    (default)       Three calls for ~0.2 s more median, and the only mode where
                    a model genuinely looks at the frame. Each stage fails
                    separately, which is worth something at 4 a.m. It also
                    CANNOT speak the intent tag, because only the spoken line
                    is ever handed to the speech model -- see below.

`pipeline` is the default despite being the slower of the two, because 0.23 s of
median is not perceptible and the two things it buys are: the model sees actual
pixels, and it is structurally incapable of saying "Intent" out loud. `direct`
is there for the lowest median and as the honest like-for-like against Qwen.

TWO THINGS THAT BITE. `gpt-audio` reads the `INTENT=` line out loud: it is in
the text, so it goes into the speech, and a 2 s sentence comes back as 5.4 s of
audio ending in "intent fetch goose". `_trim_tag` cuts it back off using the
helpers in `audio.py` that this exact problem was written for. It is a
heuristic on a waveform, so it is not free -- see that function for the failure
it still had at 1-in-6 before the cap went in. `pipeline` has no such problem by
construction. And OpenAI's WAV responses carry a placeholder RIFF length --
`wave` reports a 1.8 s clip as 89478 s -- so every TTS call here asks for
headerless `pcm` and puts its own header on.

`urllib`, not the `openai` SDK, even though the SDK is installed here: this file
has to run on the Pi and on the QNX board too, and `retriever.voice.omni` set
the precedent for the same reason. The prompt, `Heard` and `parse_intent` are
imported from `qnx_voice` -- one source of truth, so the two paths cannot drift.

Needs OPENAI_API_KEY in the environment. The seam is `listen()` -> `Heard` and
`speak()`, so this is a drop-in for `MacVoice` and `QnxVoice` in central_pi.py.
"""

from __future__ import annotations

import argparse
import base64
import http.client
import json
import logging
import os
import statistics
import sys
import threading
import time
import urllib.error
import urllib.request
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from retriever.voice.audio import (AudioError, find_cut_sample, pcm16_to_wav,
                                   play_wav, record_push_to_talk, to_model_wav,
                                   trim_pcm16, wav_to_pcm16)

from mac_voice import _cue                          # same laptop, same beep
from qnx_voice import SYSTEM, Heard, parse_intent    # one source of truth for the prompt

LOG = logging.getLogger("voice")

API = "https://api.openai.com/v1"
AUDIO_MODEL = "gpt-audio"        # gpt-4o-audio-preview is gone; this is its successor
DECIDE_MODEL = "gpt-4.1"         # fastest vision chat measured here, and no reasoning tax
STT_MODEL = "gpt-transcribe"
TTS_MODEL = "tts-1"              # gpt-4o-mini-tts costs the same latency for 1.7x the bytes
SCENE_MODEL = "gpt-4.1-mini"     # only ever runs in parallel, so cheap beats good
VOICE = "alloy"
OUT_RATE = 24000                 # what OpenAI returns for both `pcm` and gpt-audio


class OpenAIError(RuntimeError):
    """Anything the API said no to, phrased for a human."""


# ---------------------------------------------------------------- HTTP (stdlib)


def _key() -> str:
    k = (os.environ.get("OPENAI_API_KEY") or "").strip()
    if not k:
        raise OpenAIError("OPENAI_API_KEY is not set in the environment")
    return k


def _send(req: urllib.request.Request, timeout: float, tries: int = 2) -> bytes:
    """POST, and turn every failure into an OpenAIError the caller can survive.

    The broad `(OSError, HTTPException)` arm is not laziness. A dropped keep-alive
    raises `http.client.RemoteDisconnected`, which is in neither `urllib.error`
    branch, so it escaped this function and killed a benchmark on run 10 of 10.
    In the mission loop that is the robot falling over because a socket closed.
    Connection-level failures get one retry; an HTTP status never does, because
    a 400 will be a 400 again and a 429 deserves to be seen.
    """
    for attempt in range(tries):
        try:
            return urllib.request.urlopen(req, timeout=timeout).read()
        except urllib.error.HTTPError as e:
            detail = e.read()[:400].decode("utf-8", "replace")
            try:
                detail = json.loads(detail)["error"]["message"]
            except Exception:
                pass
            raise OpenAIError(f"{e.code} from {req.selector}: {detail}") from e
        except (OSError, http.client.HTTPException) as e:
            reason = getattr(e, "reason", e)
            if attempt + 1 >= tries:
                raise OpenAIError(f"cannot reach the API: {reason}") from e
            LOG.warning("%s dropped (%s) -- retrying once", req.selector, reason)
    raise OpenAIError("unreachable")            # for type checkers only


def _post_json(path: str, payload: dict, timeout: float = 60.0) -> dict:
    req = urllib.request.Request(
        f"{API}{path}", data=json.dumps(payload).encode(),
        headers={"Authorization": f"Bearer {_key()}", "Content-Type": "application/json"})
    return json.loads(_send(req, timeout))


def _post_bytes(path: str, payload: dict, timeout: float = 60.0) -> bytes:
    """For /audio/speech, which answers with the audio itself, not JSON."""
    req = urllib.request.Request(
        f"{API}{path}", data=json.dumps(payload).encode(),
        headers={"Authorization": f"Bearer {_key()}", "Content-Type": "application/json"})
    return _send(req, timeout)


def _post_wav(path: str, fields: dict, wav: bytes, timeout: float = 60.0) -> dict:
    """Multipart upload, hand-rolled because /audio/transcriptions takes a file."""
    b = "----retriever" + uuid.uuid4().hex
    body = b"".join(
        f"--{b}\r\nContent-Disposition: form-data; name=\"{k}\"\r\n\r\n{v}\r\n".encode()
        for k, v in fields.items())
    body += (f"--{b}\r\nContent-Disposition: form-data; name=\"file\"; "
             f"filename=\"hear.wav\"\r\nContent-Type: audio/wav\r\n\r\n").encode()
    body += wav + f"\r\n--{b}--\r\n".encode()
    req = urllib.request.Request(
        f"{API}{path}", data=body,
        headers={"Authorization": f"Bearer {_key()}",
                 "Content-Type": f"multipart/form-data; boundary={b}"})
    return json.loads(_send(req, timeout))


# ---------------------------------------------------------------- the three calls


def transcribe(wav: bytes, *, model: str = STT_MODEL, timeout: float = 60.0) -> str:
    # language=en is not politeness: it stops the recogniser hedging into another
    # language on a short, noisy clip, which is half of the drift problem.
    d = _post_wav("/audio/transcriptions",
                  {"model": model, "language": "en", "response_format": "json"},
                  wav, timeout)
    return (d.get("text") or "").strip()


def synth(text: str, *, model: str = TTS_MODEL, voice: str = VOICE,
          timeout: float = 60.0) -> bytes:
    """Text -> 24 kHz mono WAV. Asks for `pcm` and adds the header here, because
    OpenAI's own WAV header carries a placeholder length that `wave` believes."""
    pcm = _post_bytes("/audio/speech",
                      {"model": model, "voice": voice, "input": text,
                       "response_format": "pcm"}, timeout)
    return pcm16_to_wav(pcm, OUT_RATE, 1)


def _image_part(jpeg: bytes, detail: str = "low") -> dict:
    return {"type": "image_url",
            "image_url": {"url": "data:image/jpeg;base64," + base64.b64encode(jpeg).decode(),
                          "detail": detail}}


# ---------------------------------------------------------------- spoken-tag repair


def _trim_tag(wav: bytes, raw_text: str, spoken: str) -> bytes:
    """Cut the spoken `INTENT=...` off the end of the reply audio.

    gpt-audio reads its whole text answer aloud, tag included. The fraction of
    the letters that are human-facing is a good proxy for the fraction of the
    speaking time, and `find_cut_sample` snaps that estimate to the pause before
    the tag, so the cut lands between sentences rather than mid-word.

    THE SNAP IS CAPPED AT THE ESTIMATE, which is the whole reason this is not
    just a call to `find_cut_sample`. It searches a window either side, and the
    gap between "intent" and "fetch" is quieter than the gap before "intent" --
    so left alone it happily snaps to a pause INSIDE the tag and the robot says
    "...right away. Intent." That leaked on 1 run in 6 before the cap. Cutting
    early only ever clips the tail of a word, which the fade makes inaudible.
    """
    def alnum(s: str) -> int:
        return sum(c.isalnum() for c in s)

    total = alnum(raw_text)
    if not wav or not total or not spoken:
        return wav
    frac = alnum(spoken) / total
    if frac >= 0.995:
        return wav
    try:
        pcm, rate, _ch = wav_to_pcm16(wav)
        n = len(pcm) // 2
        cut = min(find_cut_sample(pcm, rate, frac), int(n * frac))
        if cut >= n:
            return wav
        return pcm16_to_wav(trim_pcm16(pcm, rate, cut), rate, 1)
    except Exception as exc:
        LOG.warning("could not trim the spoken tag: %s", exc)
        return wav


# ---------------------------------------------------------------- scene prefetch


class _Scene:
    """One sentence about the camera frame, fetched while the person is talking.

    This is the whole trick that makes `direct` mode viable. gpt-audio will not
    accept an image, but the frame is available the instant the microphone opens
    -- seconds before there is any audio to send -- so the caption is free in
    wall-clock terms. By the time the recording ends it is already sitting here.
    """

    def __init__(self, jpeg: bytes | None, timeout: float = 20.0) -> None:
        self.text: str | None = None
        self._thread: threading.Thread | None = None
        if not jpeg:
            return
        self._thread = threading.Thread(target=self._run, args=(jpeg, timeout), daemon=True)
        self._thread.start()

    def _run(self, jpeg: bytes, timeout: float) -> None:
        try:
            d = _post_json("/chat/completions", {
                "model": SCENE_MODEL, "max_tokens": 60,
                "messages": [
                    {"role": "system", "content":
                     "Name what is on the floor in front of the robot in one short "
                     "English clause. Objects only, no commentary."},
                    {"role": "user", "content": [_image_part(jpeg)]}]}, timeout)
            self.text = (d["choices"][0]["message"]["content"] or "").strip()
        except Exception as exc:
            LOG.debug("scene caption failed, continuing without it: %s", exc)

    def get(self, deadline_s: float = 1.5) -> str | None:
        """Whatever arrived. Never blocks the turn for long -- the caption is a
        nice-to-have and the person's words are not."""
        if self._thread is not None:
            self._thread.join(deadline_s)
        return self.text


# ---------------------------------------------------------------- the voice


class OpenAIVoice:
    """A `Voice` for central_pi.py, on OpenAI. Same seam as MacVoice."""

    def __init__(self, camera=None, *, seconds: float = 4.0,
                 push_to_talk: bool = False, target: str = "goose",
                 mode: str = "pipeline", scene: bool = True,
                 timeout_s: float = 60.0) -> None:
        self.camera = camera            # anything with .latest() -> (frame, t)
        self.seconds = seconds
        self.push_to_talk = push_to_talk
        self.target = target
        self.mode = mode
        self.scene = scene
        self.timeout_s = timeout_s
        self.test_audio: bytes | None = None   # a WAV to use instead of the mic
        self.timing: dict[str, float] = {}

    # -- capture ---------------------------------------------------------
    def ready_cue(self) -> None:
        try:
            play_wav(_cue())
        except AudioError as exc:
            LOG.warning("no cue: %s", exc)

    def _frame_jpeg(self) -> bytes | None:
        """The robot's view, not the laptop's."""
        if self.camera is None:
            return None
        frame, _t = self.camera.latest()
        if frame is None:
            return None
        import cv2
        ok, enc = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return enc.tobytes() if ok else None

    def _record_fixed(self, rate: int = 16000) -> bytes:
        import numpy as np
        import sounddevice as sd
        buf = sd.rec(int(rate * self.seconds), samplerate=rate, channels=1, dtype="int16")
        sd.wait()
        return pcm16_to_wav(np.asarray(buf, dtype="<i2").tobytes(), rate, 1)

    def _capture(self) -> bytes | None:
        """Returns 16 kHz mono WAV, or None. `test_audio` stands in for the mic."""
        if self.test_audio is not None:
            return to_model_wav(self.test_audio)
        try:
            if self.push_to_talk:
                audio = record_push_to_talk()
            else:
                self.ready_cue()
                LOG.info("listening %.1fs -- speak now", self.seconds)
                audio = self._record_fixed()
        except Exception as exc:
            LOG.warning("capture failed: %s", exc)
            return None
        try:
            return to_model_wav(audio)
        except Exception:
            return audio

    # -- the seam central_pi.py uses -------------------------------------
    def listen(self) -> Heard | None:
        jpeg = self._frame_jpeg()
        # Started before the capture, not after: in `direct` mode this call is
        # racing the person's sentence, and that is the point of it.
        scene = _Scene(jpeg, self.timeout_s) if (jpeg and self.scene
                                                 and self.mode == "direct") else None

        audio = self._capture()
        if not audio:
            return None

        t0 = time.time()
        try:
            if self.mode == "pipeline":
                raw, reply_wav, clean = self._turn_pipeline(audio, jpeg)
            else:
                raw, reply_wav, clean = self._turn_direct(audio, scene)
        except OpenAIError as exc:
            LOG.warning("OpenAI failed: %s", exc)
            return None
        self.timing["total_s"] = time.time() - t0

        kind, obj, spoken = parse_intent(raw)
        LOG.info("heard -> intent=%s object=%s say=%r (%.2fs)",
                 kind, obj, spoken, self.timing["total_s"])

        # `clean` audio was synthesised from the spoken line alone and has no tag
        # in it. Trimming it anyway lops off a real clause -- that cost us the
        # end of "...but I can't see it here" before this flag existed.
        if reply_wav:
            self._play(reply_wav if clean else _trim_tag(reply_wav, raw, spoken))

        if kind == "stop":
            return Heard(text="stop", target=None, reply=None)
        if kind == "fetch":
            return Heard(text=spoken or "fetch", target=obj or self.target, reply=None)
        return Heard(text=spoken or "", target=None, reply=None)

    def _turn_direct(self, audio: bytes,
                     scene: _Scene | None) -> tuple[str, bytes | None, bool]:
        """One call: speech in, speech out. The frame arrives as a caption."""
        parts: list[dict] = []
        note = scene.get() if scene is not None else None
        if note:
            # Labelled as the camera's report, never as the person's words --
            # the prompt is explicit that the object comes from the person, and
            # this text must not be mistaken for something they said.
            parts.append({"type": "text",
                          "text": f"(robot camera sees: {note}. The person's request "
                                  f"is the audio; use THEIR word for the object.)"})
        parts.append({"type": "input_audio",
                      "input_audio": {"data": base64.b64encode(audio).decode(), "format": "wav"}})
        LOG.info("asking %s (%d B audio, %s)", AUDIO_MODEL, len(audio),
                 f"scene: {note!r}" if note else "no scene")

        t = time.time()
        d = _post_json("/chat/completions", {
            "model": AUDIO_MODEL,
            "modalities": ["text", "audio"],
            "audio": {"voice": VOICE, "format": "pcm16"},
            "messages": [{"role": "system", "content": SYSTEM},
                         {"role": "user", "content": parts}]}, self.timeout_s)
        self.timing["chat_s"] = time.time() - t

        msg = d["choices"][0]["message"]
        a = msg.get("audio") or {}
        # The text lives in `audio.transcript` when audio is on; `content` is null.
        raw = (msg.get("content") or a.get("transcript") or "").strip()
        wav = pcm16_to_wav(base64.b64decode(a["data"]), OUT_RATE, 1) if a.get("data") else None
        return raw, wav, False          # the model read its own tag aloud; needs trimming

    def _turn_pipeline(self, audio: bytes,
                       jpeg: bytes | None) -> tuple[str, bytes | None, bool]:
        """Three calls, and the only mode where the model sees the actual pixels."""
        t = time.time()
        said = transcribe(audio, timeout=self.timeout_s)
        self.timing["stt_s"] = time.time() - t
        LOG.info("transcript: %r", said)
        if not said:
            return "INTENT=none", None, True

        parts: list[dict] = [{"type": "text", "text": said}]
        if jpeg:
            parts.append(_image_part(jpeg))
        t = time.time()
        d = _post_json("/chat/completions", {
            "model": DECIDE_MODEL,
            "messages": [{"role": "system", "content": SYSTEM},
                         {"role": "user", "content": parts}]}, self.timeout_s)
        self.timing["decide_s"] = time.time() - t
        raw = (d["choices"][0]["message"]["content"] or "").strip()

        # Only the spoken line is synthesised, so unlike `direct` the tag can
        # never be read aloud -- it is never handed to the speech model at all.
        _kind, _obj, spoken = parse_intent(raw)
        wav = None
        if spoken:
            t = time.time()
            wav = synth(spoken, timeout=self.timeout_s)
            self.timing["tts_s"] = time.time() - t
        return raw, wav, True

    def speak(self, text: str) -> None:
        """Narration from the mission loop. A TTS call, not a chat round trip."""
        if not text:
            return
        print(f"  robot: {text}", flush=True)
        try:
            self._play(synth(text, timeout=self.timeout_s))
        except OpenAIError as exc:
            LOG.warning("speak failed: %s", exc)

    def _play(self, wav: bytes) -> None:
        try:
            play_wav(wav)
        except AudioError as exc:
            LOG.warning("playback failed: %s", exc)


# ---------------------------------------------------------------- cli


def _bench(voice: OpenAIVoice, runs: int, modes: list[str]) -> int:
    """The measurement this file exists to produce. Same WAV every run, so the
    spread is the API's and not the microphone's."""
    rows = []
    for mode in modes:
        voice.mode = mode
        times, ok = [], 0
        for i in range(runs):
            t = time.time()
            heard = voice.listen()
            el = time.time() - t
            good = bool(heard and heard.target == voice.target)
            ok += good
            times.append(el)
            print(f"  {mode:9s} run {i + 1}: {el:5.2f}s  "
                  f"{'OK ' if good else 'BAD'} {heard}", flush=True)
        rows.append((mode, times, ok))
    print(f"\n  {'approach':10s} {'runs':>4s} {'min':>7s} {'median':>7s} {'max':>7s}  intent")
    for mode, times, ok in rows:
        print(f"  {mode:10s} {len(times):4d} {min(times):6.2f}s "
              f"{statistics.median(times):6.2f}s {max(times):6.2f}s  {ok}/{len(times)}")
    return 0 if all(ok == len(t) for _m, t, ok in rows) else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stream", default="192.168.4.2:5577",
                    help="the robot's camera, so the model sees what the robot sees")
    ap.add_argument("--frame", help="a JPEG to use when the stream is unreachable")
    ap.add_argument("--listen", type=float, default=4.0, metavar="SECONDS")
    ap.add_argument("--push-to-talk", action="store_true")
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--no-scene", action="store_true",
                    help="direct mode: skip the parallel camera caption")
    ap.add_argument("--mode", default="pipeline", choices=("direct", "pipeline"))
    ap.add_argument("--say", metavar="TEXT",
                    help="synthesise this instead of recording -- a repeatable "
                         "stand-in for the microphone")
    ap.add_argument("--audio", metavar="WAV", help="feed this WAV in as the person's speech")
    ap.add_argument("--bench", type=int, metavar="N", help="time N runs of each mode")
    ap.add_argument("--quiet", action="store_true", help="do not play anything")
    args = ap.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(name)s: %(message)s")

    cam = None
    if args.frame:
        cam = _StillCamera(Path(args.frame).read_bytes())
    elif not args.no_camera:
        from floor_detector import DepthStreamFrames
        h, _, p = args.stream.partition(":")
        cam = DepthStreamFrames(h, int(p or 5577)).start()
        time.sleep(3)
        if cam.latest()[0] is None:
            LOG.warning("no frame from %s -- continuing without the camera", args.stream)
            cam = None

    v = OpenAIVoice(cam, seconds=args.listen, push_to_talk=args.push_to_talk,
                    mode=args.mode, scene=not args.no_scene)
    if args.audio:
        v.test_audio = Path(args.audio).read_bytes()
    elif args.say:
        # Rendering the utterance rather than recording one is what makes the
        # benchmark mean anything: every run sends byte-identical audio, so the
        # spread below is the API's and not the room's.
        LOG.info("synthesising the test utterance: %r", args.say)
        try:
            v.test_audio = to_model_wav(synth(args.say))
        except OpenAIError as exc:
            print(f"  cannot synthesise the test utterance: {exc}", file=sys.stderr)
            return 2
    if args.quiet:
        v._play = lambda wav: None      # benchmarking a speaker helps nobody

    if args.bench:
        return _bench(v, args.bench, ["direct", "pipeline"])
    heard = v.listen()
    print(f"  -> {heard}")
    return 0 if heard else 1


class _StillCamera:
    """A fixed JPEG behind the camera seam, for when :5577 is not up."""

    def __init__(self, jpeg: bytes) -> None:
        import cv2
        import numpy as np
        self._frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)

    def latest(self):
        return self._frame, time.time()


if __name__ == "__main__":
    raise SystemExit(main())
