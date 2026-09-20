#!/usr/bin/env python3
"""Ears and a mouth on the laptop, with the camera still on the robot.

    python3 scripts/voice_mac.py                    # one turn, fixed 4 s
    python3 scripts/voice_mac.py --push-to-talk     # Enter to start, Enter to stop

WHY THIS EXISTS, given `qnx_voice.py` already does the same job. It does the same
job better -- the microphone genuinely belongs on the central Pi -- but that
board's USB bus cannot carry a D435i and a USB speakerphone at the same time. It
collapsed four times, always taking the camera down with the audio, and always
within minutes of a reboot. Between a camera and a microphone on a bus that can
only feed one, the camera is the one that has to stay.

So: capture and playback move here, the camera stays on the board, and NOTHING
ELSE CHANGES. The model call is identical, the frame still comes from the robot's
own D435i over :5577, and the reply is still one Qwen-Omni request carrying image
and audio together. Which machine holds the microphone was never part of that.

The seam is the same one `central_pi.py` already consumes -- `listen()` returning
a `Heard`, and `speak()` -- so this file is a drop-in for `QnxVoice`.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path


from retriever.voice.audio import (AudioError, play_wav, pcm16_to_wav,
                                   record_push_to_talk, to_model_wav)
from retriever.voice.omni import OmniClient, OmniConfig

from retriever.voice.qnx import SYSTEM, Heard, parse_intent      # one source of truth for the prompt

LOG = logging.getLogger("voice")


def _cue(rate: int = 24000) -> bytes:
    """The same rising two-tone beep the board played, made locally.

    Generated rather than requested from the model: its entire job is to mark the
    instant the microphone opens, so it has to be instant and has to work when the
    network is having a bad day.
    """
    import array
    import math
    pcm = array.array("h")
    for freq, secs in ((880.0, 0.09), (1320.0, 0.11)):
        n = int(rate * secs)
        for i in range(n):
            env = min(1.0, i / (0.01 * rate), (n - i) / (0.01 * rate))
            pcm.append(int(9000 * env * math.sin(2 * math.pi * freq * i / rate)))
    return pcm16_to_wav(pcm.tobytes(), rate, 1)


def _record_fixed(seconds: float, rate: int = 16000) -> bytes:
    """A fixed-length capture. No keypress, because the robot loop has no keyboard."""
    import numpy as np
    import sounddevice as sd
    frames = int(rate * seconds)
    buf = sd.rec(frames, samplerate=rate, channels=1, dtype="int16")
    sd.wait()
    return pcm16_to_wav(np.asarray(buf, dtype="<i2").tobytes(), rate, 1)


class MacVoice:
    """A `Voice` for central_pi.py: local mic and speaker, robot's camera."""

    def __init__(self, camera=None, *, seconds: float = 4.0,
                 push_to_talk: bool = False, target: str = "goose") -> None:
        self.camera = camera            # anything with .latest() -> (frame, t)
        self.seconds = seconds
        self.push_to_talk = push_to_talk
        self.target = target
        self.client = OmniClient(OmniConfig.from_env())

    def ready_cue(self) -> None:
        try:
            play_wav(_cue())
        except AudioError as exc:
            LOG.warning("no cue: %s", exc)

    def _frame_jpeg(self) -> bytes | None:
        """The robot's view, not the laptop's. The model is asked about the floor
        in front of the robot, so the frame must come from the robot."""
        if self.camera is None:
            return None
        frame, _t = self.camera.latest()
        if frame is None:
            return None
        import cv2
        ok, enc = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return enc.tobytes() if ok else None

    def listen(self) -> Heard | None:
        try:
            if self.push_to_talk:
                audio = record_push_to_talk()
            else:
                self.ready_cue()
                LOG.info("listening %.1fs -- speak now", self.seconds)
                audio = _record_fixed(self.seconds)
        except Exception as exc:
            LOG.warning("capture failed: %s", exc)
            return None

        # Already 16 kHz mono from either path, but be explicit: the model wants
        # exactly that, and a push-to-talk device could hand back something else.
        try:
            audio = to_model_wav(audio)
        except Exception:
            pass

        jpeg = self._frame_jpeg()
        LOG.info("asking OMNI (%d B audio, %s)", len(audio),
                 f"{len(jpeg)} B robot frame" if jpeg else "no image")
        try:
            r = self.client.ask(audio_wav=audio, image_jpeg=jpeg, system=SYSTEM)
        except Exception as exc:
            LOG.warning("OMNI failed: %s", exc)
            return None

        kind, obj, spoken = parse_intent(r.text or "")
        LOG.info("heard -> intent=%s object=%s say=%r (%.2fs)",
                 kind, obj, spoken, r.total_s)
        if r.audio_wav:
            self._play(r.audio_wav)

        if kind == "stop":
            return Heard(text="stop", target=None, reply=None)
        if kind == "fetch":
            return Heard(text=spoken or "fetch", target=obj or self.target, reply=None)
        return Heard(text=spoken or "", target=None, reply=None)

    def speak(self, text: str) -> None:
        if not text:
            return
        print(f"  robot: {text}", flush=True)
        try:
            r = self.client.ask(
                text=f"Say exactly this in English and nothing else: {text}",
                system="You speak only English. Repeat the words given, nothing more.")
        except Exception as exc:
            LOG.warning("speak failed: %s", exc)
            return
        if r.audio_wav:
            self._play(r.audio_wav)

    def _play(self, wav: bytes) -> None:
        # No resampling dance here: the laptop's output device takes whatever rate
        # the model returns. That whole 24k->48k step existed only for the Jabra.
        try:
            play_wav(wav)
        except AudioError as exc:
            LOG.warning("playback failed: %s", exc)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stream", default="192.168.4.2:5577",
                    help="the robot's camera, so the model sees what the robot sees")
    ap.add_argument("--listen", type=float, default=4.0, metavar="SECONDS")
    ap.add_argument("--push-to-talk", action="store_true")
    ap.add_argument("--no-camera", action="store_true")
    args = ap.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(name)s: %(message)s")
    cam = None
    if not args.no_camera:
        from retriever.perception.floor import DepthStreamFrames
        h, _, p = args.stream.partition(":")
        cam = DepthStreamFrames(h, int(p or 5577)).start()
        time.sleep(3)

    v = MacVoice(cam, seconds=args.listen, push_to_talk=args.push_to_talk)
    heard = v.listen()
    print(f"  -> {heard}")
    return 0 if heard else 1


if __name__ == "__main__":
    raise SystemExit(main())
