#!/usr/bin/env python3
"""Ears and a mouth for the mission loop, borrowed from the QNX board over SSH.

    python3 scripts/qnx_voice.py --listen 4        # record, ask OMNI, speak the reply

WHY IT IS SPLIT LIKE THIS. The microphone and speaker are on the QNX central Pi;
the detector needs torch and lives on the Mac. Rather than move either, this runs
on the Mac and reaches through SSH for the two things only the board can do:
capture audio and play it. Everything in between -- the model call, the
resampling, the intent -- happens here.

`sounddevice` is not an option on that board. PortAudio's ALSA backend wants
Linux's libasound and QNX's /dev/snd is not that; the failure is silent, which is
worse than loud. `~/supervisor/qnx_audio.py` goes at QNX's own audio stack
directly, and this file just drives it.

SAMPLE RATES ARE NOT NEGOTIABLE. Qwen-Omni returns 24 kHz. The Jabra SPEAK 510 on
that board refuses 24 kHz outright and wants 48 kHz. `retriever.voice.audio`
already knows how to resample and needs no sounddevice to do it, so the
conversion happens here before the WAV ever goes back over the wire.
"""

from __future__ import annotations

import argparse
import logging
import re
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.voice.audio import pcm16_to_wav, resample, to_model_wav
from retriever.voice.omni import OmniClient, OmniConfig

LOG = logging.getLogger("voice")

BOARD_RATE = 48000          # what the Jabra on the QNX board will accept
QNX_AUDIO = "~/supervisor/qnx_audio.py"

# One short spoken sentence, then a machine-readable line. Asking for the tag on
# its own line is what keeps it out of the speech: anything after the first line
# is stripped before the words are spoken.
SYSTEM = (
    "ALWAYS REPLY IN ENGLISH. Never use Chinese or any other language, whatever "
    "language the person speaks to you in. Every spoken word you produce must be "
    "English.\n"
    "You are the voice of a robot that fetches things for a person.\n"
    "THE OBJECT COMES FROM THE PERSON, NOT THE PICTURE. Whatever they name is what "
    "you fetch, and you must use THEIR word for it. The picture only tells you what "
    "is nearby; never substitute something you can see for the thing they asked for. "
    "If they say 'goose' the object is goose, even if no goose is visible.\n"
    "If you cannot make out what they asked for, do not guess an object.\n"
    "Reply with ONE short spoken sentence, natural and warm, under 12 words.\n"
    "Then on a SECOND line write exactly one tag:\n"
    "  INTENT=fetch:THEIR_WORD   when they want something fetched\n"
    "  INTENT=stop               when they want the robot to stop\n"
    "  INTENT=none               when it is neither, or you could not tell\n"
    "Never say the tag out loud, never use angle brackets, never explain it."
)


_TAG = re.compile(r"INTENT\s*=\s*(fetch|stop|none)\s*:?\s*([^\n<>]*)", re.I)


@dataclass(frozen=True)
class Heard:
    """Matches scripts/central_pi.py's Heard."""
    text: str
    target: str | None = None
    reply: str | None = None


def parse_intent(raw: str) -> tuple[str, str | None, str]:
    """(kind, object, what to actually say). The tag never reaches the speaker."""
    kind, obj = "none", None
    m = _TAG.search(raw or "")
    if m:
        kind = m.group(1).lower()
        obj = (m.group(2) or "").strip().strip(".,") or None
    spoken = _TAG.sub("", raw or "").strip()
    spoken = spoken.splitlines()[0].strip() if spoken else ""
    return kind, obj, spoken


class QnxVoice:
    """A `Voice` for central_pi.py, with the board's mic and speaker behind SSH."""

    def __init__(self, board: str, jump: str | None = None, *, seconds: float = 4.0,
                 camera=None, target: str = "goose") -> None:
        self.board, self.jump, self.seconds = board, jump, seconds
        self.camera = camera            # anything with .latest() -> (frame, t)
        self.target = target
        self.client = OmniClient(OmniConfig.from_env())

    # -- ssh plumbing ----------------------------------------------------
    def _ssh(self, cmd: str, timeout: float = 40.0) -> tuple[int, str]:
        argv = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8"]
        if self.jump:
            argv += ["-J", self.jump]
        argv += [self.board, cmd]
        p = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
        return p.returncode, (p.stderr or p.stdout or "").strip()

    def _scp(self, src: str, dst: str, timeout: float = 40.0) -> bool:
        argv = ["scp", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", "-q"]
        if self.jump:
            argv += ["-J", self.jump]
        argv += [src, dst]
        return subprocess.run(argv, capture_output=True, timeout=timeout).returncode == 0


    def ready_cue(self) -> None:
        """A short rising two-tone beep, played just before the mic opens.

        Generated here rather than asked of the model: it must be instant and it
        must work when the network or the API is having a bad day, because its
        whole job is to tell a person the exact moment to start talking. Speech
        would cost a round trip and arrive late, which is worse than no cue at all.
        """
        import array
        import math
        rate = BOARD_RATE
        pcm = array.array("h")
        for freq, secs in ((880.0, 0.09), (1320.0, 0.11)):
            n = int(rate * secs)
            for i in range(n):
                # a short fade at each end, so it clicks on neither edge
                env = min(1.0, i / (0.01 * rate), (n - i) / (0.01 * rate))
                pcm.append(int(9000 * env * math.sin(2 * math.pi * freq * i / rate)))
        wav = pcm16_to_wav(pcm.tobytes(), rate, 1)
        with tempfile.TemporaryDirectory() as tmp:
            local = Path(tmp) / "cue.wav"
            local.write_bytes(wav)
            if not self._scp(str(local), f"{self.board}:/tmp/cue.wav"):
                return
        self._ssh(f"python3 {QNX_AUDIO} play /tmp/cue.wav", timeout=20.0)

    # -- the seam central_pi.py uses -------------------------------------
    def listen(self) -> Heard | None:
        # Tell the person the mic is open BEFORE opening it. Without this they
        # start talking into a recorder that has not started and the first word
        # of every command is lost.
        self.ready_cue()
        LOG.info("listening %.1fs -- speak now", self.seconds)
        rc, out = self._ssh(f"python3 {QNX_AUDIO} record {self.seconds:g} /tmp/hear.wav")
        if rc != 0:
            LOG.warning("record failed: %s", out)
            return None

        with tempfile.TemporaryDirectory() as tmp:
            local = str(Path(tmp) / "hear.wav")
            jump = f"-J {self.jump} " if self.jump else ""
            if not self._scp(f"{self.board}:/tmp/hear.wav", local):
                LOG.warning("could not fetch the recording")
                return None
            audio = Path(local).read_bytes()

        # The Jabra records at 48 kHz; the model only wants 16 kHz mono. Sending the
        # raw capture means uploading ~3x the bytes over a link that already goes
        # Mac -> nav Pi -> board, and upload was the single biggest slice of the
        # round trip. Downsample here, where there is CPU to spare.
        raw_len = len(audio)
        try:
            audio = to_model_wav(audio)
            LOG.info("audio %d B -> %d B for the model", raw_len, len(audio))
        except Exception as exc:
            LOG.warning("could not downsample, sending raw: %s", exc)

        jpeg = None
        if self.camera is not None:
            frame, _t = self.camera.latest()
            if frame is not None:
                import cv2
                ok, enc = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    jpeg = enc.tobytes()

        LOG.info("asking OMNI (%d B audio, %s)", len(audio),
                 f"{len(jpeg)} B image" if jpeg else "no image")
        try:
            r = self.client.ask(audio_wav=audio, image_jpeg=jpeg, system=SYSTEM)
        except Exception as exc:
            LOG.warning("OMNI failed: %s", exc)
            return None

        kind, obj, spoken = parse_intent(r.text or "")
        LOG.info("heard -> intent=%s object=%s say=%r (%.2fs)",
                 kind, obj, spoken, r.total_s)

        # Speak the model's own audio when we have it: it is already warm and in
        # the right voice, and it costs nothing extra.
        if r.audio_wav:
            self._say_wav(r.audio_wav, r.sample_rate or 24000)

        if kind == "stop":
            return Heard(text="stop", target=None, reply=None)
        if kind == "fetch":
            return Heard(text=spoken or "fetch", target=obj or self.target,
                         reply=None)          # already spoken above
        return Heard(text=spoken or "", target=None, reply=None)

    def speak(self, text: str) -> None:
        """Narration from the mission loop. Short, so a tiny TTS call is fine."""
        if not text:
            return
        print(f"  robot: {text}", flush=True)
        try:
            r = self.client.ask(text=f"Say exactly this in English and nothing else: {text}",
                                system="You speak only English. Repeat the words given, nothing more.")
        except Exception as exc:
            LOG.warning("speak failed: %s", exc)
            return
        if r.audio_wav:
            self._say_wav(r.audio_wav, r.sample_rate or 24000)

    # -- playback --------------------------------------------------------
    def _say_wav(self, wav: bytes, rate: int) -> None:
        """Resample to what the board's speaker accepts, then play it there."""
        try:
            wav48 = _to_board_rate(wav, rate)
        except Exception as exc:
            LOG.warning("resample failed: %s", exc)
            return
        with tempfile.TemporaryDirectory() as tmp:
            local = Path(tmp) / "say.wav"
            local.write_bytes(wav48)
            if not self._scp(str(local), f"{self.board}:/tmp/say.wav"):
                LOG.warning("could not send the reply to the board")
                return
        rc, out = self._ssh(f"python3 {QNX_AUDIO} play /tmp/say.wav", timeout=60.0)
        if rc != 0:
            LOG.warning("playback failed: %s", out)


def _to_board_rate(wav: bytes, rate: int) -> bytes:
    """24 kHz from the model -> 48 kHz for the Jabra. Mono, 16-bit throughout."""
    import wave
    import io
    import array
    with wave.open(io.BytesIO(wav), "rb") as w:
        ch, width, src_rate, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
        pcm = w.readframes(n)
    if width != 2:
        raise ValueError(f"expected 16-bit audio, got {width * 8}-bit")
    samples = array.array("h")
    samples.frombytes(pcm)
    if ch == 2:                                   # the model sends mono; be safe
        samples = array.array("h", samples[0::2])
    if src_rate != BOARD_RATE:
        samples = array.array("h", (int(v) for v in resample(samples, src_rate, BOARD_RATE)))
    return pcm16_to_wav(samples.tobytes(), BOARD_RATE, 1)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--board", default="qnxuser@192.168.4.2")
    ap.add_argument("--jump", default="hao@10.0.0.111",
                    help="the board is behind the nav Pi; '' for none")
    ap.add_argument("--listen", type=float, default=4.0, metavar="SECONDS")
    ap.add_argument("--stream", default="192.168.4.2:5577",
                    help="camera stream, so the model can see while it listens")
    ap.add_argument("--no-camera", action="store_true")
    args = ap.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(name)s: %(message)s")
    cam = None
    if not args.no_camera:
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from floor_detector import DepthStreamFrames
        h, _, p = args.stream.partition(":")
        cam = DepthStreamFrames(h, int(p or 5577)).start()
        time.sleep(3)

    v = QnxVoice(args.board, args.jump or None, seconds=args.listen, camera=cam)
    print(f"  speak now ({args.listen:g}s)...", flush=True)
    heard = v.listen()
    print(f"  -> {heard}")
    return 0 if heard else 1


if __name__ == "__main__":
    raise SystemExit(main())
