"""Microphone in, speaker out, and the WAV plumbing between them.

WHY the imports are lazy: `sounddevice` loads PortAudio at import time and
numpy is heavy, and neither should be needed to build a request or run the
payload tests. Only the functions that touch hardware or do DSP import them.

Formats at the boundaries:
  * to the model:   16 kHz mono PCM16 WAV (speech models want 16 kHz; more is
                    just upload latency)
  * from the model: PCM16 mono at 24 kHz, streamed in chunks

macOS gotcha worth the paragraph: when the terminal app has NO microphone
permission, CoreAudio does not raise -- it hands you perfect digital silence.
So "recorded all zeros" is reported as a permission problem, not a quiet room.
"""

from __future__ import annotations

import array
import io
import math
import queue
import shutil
import subprocess
import sys
import tempfile
import threading
import wave
from collections.abc import Callable
from pathlib import Path
from typing import Any

INPUT_RATE = 16000

MIC_PERMISSION_HINT = (
    "macOS: System Settings > Privacy & Security > Microphone -> enable your terminal app "
    "(Terminal / iTerm / VS Code), then fully quit and reopen it."
)


class AudioError(RuntimeError):
    """A hardware or format problem, with a message that says what to do about it."""


# ---------------------------------------------------------------- WAV codec (stdlib)


def pcm16_to_wav(pcm: bytes, rate: int, channels: int = 1) -> bytes:
    """Raw little-endian PCM16 -> WAV bytes."""
    if len(pcm) % 2:
        pcm = pcm[:-1]
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(channels)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(pcm)
    return buf.getvalue()


def wav_to_pcm16(wav: bytes) -> tuple[bytes, int, int]:
    """WAV bytes -> (PCM16 bytes, rate, channels). Stdlib only for 16-bit input.

    8/24/32-bit integer WAVs are converted with numpy; float WAVs (which the
    `wave` module refuses) go through soundfile if it is installed.
    """
    try:
        with wave.open(io.BytesIO(wav), "rb") as w:
            rate, ch, width = w.getframerate(), w.getnchannels(), w.getsampwidth()
            frames = w.readframes(w.getnframes())
    except (wave.Error, EOFError) as e:
        return _wav_via_soundfile(wav, e)
    if width == 2:
        return frames, rate, ch
    np = _numpy()
    if width == 1:
        x = (np.frombuffer(frames, np.uint8).astype(np.int16) - 128) << 8
    elif width == 3:
        b = np.frombuffer(frames, np.uint8).reshape(-1, 3)
        x = (b[:, 2].astype(np.int8).astype(np.int16) << 8) | b[:, 1].astype(np.int16)
    elif width == 4:
        x = (np.frombuffer(frames, "<i4") >> 16).astype(np.int16)
    else:
        raise AudioError(f"unsupported WAV sample width: {width} bytes")
    return x.astype("<i2").tobytes(), rate, ch


def _wav_via_soundfile(wav: bytes, cause: Exception) -> tuple[bytes, int, int]:
    try:
        import soundfile as sf
    except ImportError:
        raise AudioError(f"cannot decode this WAV ({cause}); install soundfile") from cause
    try:
        data, rate = sf.read(io.BytesIO(wav), dtype="int16", always_2d=True)
    except Exception as e:
        raise AudioError(f"cannot decode audio: {e}") from e
    return data.astype("<i2").tobytes(), int(rate), int(data.shape[1])


def decode_wav(wav: bytes):
    """WAV bytes -> (float32 mono samples in [-1, 1], rate). Mixes channels down."""
    np = _numpy()
    pcm, rate, ch = wav_to_pcm16(wav)
    x = np.frombuffer(pcm, "<i2").astype(np.float32) / 32768.0
    if ch > 1:
        x = x[: len(x) - len(x) % ch].reshape(-1, ch).mean(axis=1)
    return x, rate


def encode_wav(samples, rate: int) -> bytes:
    """Mono samples (float in [-1, 1], or int16) -> 16-bit WAV bytes."""
    np = _numpy()
    x = np.asarray(samples)
    if x.dtype != np.int16:
        x = np.clip(np.round(x.astype(np.float64) * 32767.0), -32768, 32767).astype(np.int16)
    return pcm16_to_wav(x.astype("<i2").tobytes(), rate)


def resample(samples, src_rate: int, dst_rate: int):
    """Linear-interpolation resampler with a box pre-filter when downsampling.

    Not audiophile, but speech into a model does not need it to be, and it
    keeps scipy out of the dependency list. The box filter stops 48 kHz mic
    hiss from aliasing into the speech band on the way down to 16 kHz.
    """
    np = _numpy()
    x = np.asarray(samples, dtype=np.float32)
    if src_rate == dst_rate or len(x) == 0:
        return x.copy()
    if dst_rate < src_rate:
        k = int(round(src_rate / dst_rate))
        if k > 1:
            x = np.convolve(x, np.ones(k, dtype=np.float32) / k, mode="same")
    n_out = max(1, int(round(len(x) * dst_rate / src_rate)))
    t = np.arange(n_out, dtype=np.float64) * (src_rate / dst_rate)
    return np.interp(t, np.arange(len(x), dtype=np.float64), x).astype(np.float32)


def to_model_wav(wav: bytes, rate: int = INPUT_RATE) -> bytes:
    """Any WAV -> 16 kHz mono PCM16 WAV, the format sent to the model."""
    pcm, src_rate, ch = wav_to_pcm16(wav)
    if src_rate == rate and ch == 1:
        return pcm16_to_wav(pcm, rate)
    x, src = decode_wav(wav)
    return encode_wav(resample(x, src, rate), rate)


def wav_duration_s(wav: bytes) -> float:
    try:
        with wave.open(io.BytesIO(wav), "rb") as w:
            return w.getnframes() / float(w.getframerate())
    except Exception:
        return 0.0


def synth_chime(rate: int = 24000, seconds: float = 0.4) -> bytes:
    """A short two-note chime as PCM16. The mock client's "voice"; stdlib only."""
    n = int(rate * seconds)
    half = n // 2
    out = array.array("h")
    for i in range(n):
        f = 660.0 if i < half else 880.0
        env = min(1.0, i / (0.01 * rate), (n - i) / (0.05 * rate))
        out.append(int(0.25 * 32767 * env * math.sin(2 * math.pi * f * i / rate)))
    if sys.byteorder == "big":
        out.byteswap()
    return out.tobytes()


# ---------------------------------------------------------------- trimming


def find_cut_sample(pcm: bytes, rate: int, fraction: float, window_s: float = 0.6) -> int:
    """Where to stop playback so the robot says `fraction` of what it generated.

    WHY: the model speaks its whole text reply, including the machine-read
    `INTENT: {...}` line at the end. We know what fraction of the text is
    human-facing, estimate the matching sample, then snap to the quietest
    30 ms frame nearby -- the pause between the last sentence and the intent
    line -- so the cut lands between words instead of inside one.
    """
    n = len(pcm) // 2
    if fraction >= 0.995 or n == 0:
        return n
    est = int(n * max(0.0, fraction))
    try:
        np = _numpy()
    except AudioError:
        return est
    x = np.frombuffer(pcm[: n * 2], "<i2").astype(np.float32)
    frame = max(1, int(0.03 * rate))
    lo = max(0, est - int(window_s * rate))
    hi = min(n - frame, est + int(window_s * rate))
    if hi <= lo:
        return est
    starts = np.arange(lo, hi, max(1, frame // 2))
    energy = np.array([float(np.mean(x[s : s + frame] ** 2)) for s in starts])
    # Prefer the frame closest to the estimate among near-silent ties.
    quiet = energy <= energy.min() * 1.5 + 1e-3
    candidates = starts[quiet]
    best = candidates[np.argmin(np.abs(candidates + frame // 2 - est))]
    return int(best + frame // 2)


def trim_pcm16(pcm: bytes, rate: int, end_sample: int, fade_s: float = 0.03) -> bytes:
    """Cut PCM16 at `end_sample` with a short fade so the cut doesn't click."""
    end = max(0, min(len(pcm) // 2, end_sample))
    a = array.array("h")
    a.frombytes(pcm[: end * 2])
    if sys.byteorder == "big":
        a.byteswap()
    fade = min(len(a), int(fade_s * rate))
    for i in range(fade):
        j = len(a) - fade + i
        a[j] = int(a[j] * (1.0 - (i + 1) / fade))
    if sys.byteorder == "big":
        a.byteswap()
    return a.tobytes()


# ---------------------------------------------------------------- hardware


def _numpy():
    try:
        import numpy as np
    except ImportError as e:
        raise AudioError("numpy is required for this audio operation: pip install numpy") from e
    return np


def _sounddevice():
    try:
        import sounddevice as sd
    except ImportError as e:
        raise AudioError("sounddevice is not installed: pip install sounddevice") from e
    except OSError as e:  # PortAudio shared library missing
        raise AudioError(
            f"PortAudio is unavailable ({e}). macOS: brew install portaudio; "
            "Linux: sudo apt install libportaudio2"
        ) from e
    return sd


def _check_device(sd: Any, kind: str) -> None:
    try:
        sd.query_devices(kind=kind)
    except Exception as e:
        what = "microphone" if kind == "input" else "speaker/headphone output"
        extra = f" {MIC_PERMISSION_HINT}" if kind == "input" else ""
        raise AudioError(f"No default {what} found ({e}).{extra}") from e


def record_push_to_talk(
    rate: int = INPUT_RATE,
    max_s: float = 30.0,
    wait: Callable[[str], Any] = input,
    log: Callable[[str], Any] = print,
    already_started: bool = False,
) -> bytes:
    """Enter to start, Enter to stop. Returns 16 kHz mono PCM16 WAV bytes.

    `already_started=True` skips the first Enter, for callers whose own prompt
    already consumed it. Raises AudioError with a fix-it message if there is no
    mic, the device refuses the stream, or macOS silently withheld permission.
    """
    sd = _sounddevice()
    _check_device(sd, "input")
    chunks: list[bytes] = []

    def callback(indata, frames, time_info, status):  # PortAudio thread
        chunks.append(bytes(indata))

    if not already_started:
        wait("Press Enter to talk... ")
    device_rate = rate
    try:
        stream = sd.RawInputStream(samplerate=rate, channels=1, dtype="int16", callback=callback)
    except Exception:
        # Some USB mics only do 44.1/48 kHz; record native and resample after.
        try:
            device_rate = int(sd.query_devices(kind="input")["default_samplerate"])
            stream = sd.RawInputStream(
                samplerate=device_rate, channels=1, dtype="int16", callback=callback
            )
        except Exception as e:
            raise AudioError(f"Could not open the microphone: {e}. {MIC_PERMISSION_HINT}") from e
    with stream:
        log("Recording... press Enter to stop.")
        wait("")
    pcm = b"".join(chunks)[: int(max_s * device_rate) * 2]

    if len(pcm) < int(0.2 * device_rate) * 2:
        raise AudioError("Recording was too short -- hold for at least half a second.")
    samples = array.array("h")
    samples.frombytes(pcm[: len(pcm) - len(pcm) % 2])
    peak = max(abs(v) for v in samples) if samples else 0
    if peak == 0:
        raise AudioError(f"The microphone returned pure silence. {MIC_PERMISSION_HINT}")
    if peak < 300:
        log("warning: recording is very quiet -- is the right input device selected?")

    wav = pcm16_to_wav(pcm, device_rate)
    return wav if device_rate == rate else to_model_wav(wav, rate)


def play_wav(wav: bytes) -> None:
    """Blocking playback of WAV bytes on the default output."""
    sd = _sounddevice()
    _check_device(sd, "output")
    pcm, rate, ch = wav_to_pcm16(wav)
    np = _numpy()
    data = np.frombuffer(pcm, "<i2").reshape(-1, ch)
    try:
        sd.play(data, rate)
        sd.wait()
    except Exception as e:
        raise AudioError(f"Playback failed: {e}") from e


class StreamingPlayer:
    """Plays PCM16 chunks as they stream in, so the robot starts talking at the
    first audio chunk instead of after the whole reply -- this is most of the
    perceived-latency win.

    `set_limit(n)` stops playback at sample n even if more audio arrives; that
    is how the spoken INTENT line gets cut off (see find_cut_sample).
    """

    def __init__(self, rate: int = 24000, prebuffer_s: float = 0.15):
        sd = _sounddevice()
        _check_device(sd, "output")
        self.rate = rate
        self._prebuffer = int(prebuffer_s * rate) * 2
        self._q: queue.Queue[bytes | None] = queue.Queue()
        self._limit: int | None = None
        self.written = 0  # samples handed to the device
        self.error: Exception | None = None
        try:
            self._stream = sd.RawOutputStream(samplerate=rate, channels=1, dtype="int16")
            self._stream.start()
        except Exception as e:
            raise AudioError(f"Could not open audio output: {e}") from e
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def feed(self, pcm: bytes) -> None:
        if pcm:
            self._q.put(pcm)

    def set_limit(self, n_samples: int | None) -> None:
        self._limit = n_samples

    def _run(self) -> None:
        pending = b""
        started = False
        while True:
            chunk = self._q.get()
            if chunk is not None:
                pending += chunk
                if not started and len(pending) < self._prebuffer:
                    continue  # small head start avoids an underrun crackle
            started = True
            if pending:
                if self._limit is not None:
                    pending = pending[: max(0, self._limit - self.written) * 2]
                if pending:
                    try:
                        self._stream.write(pending)
                    except Exception as e:
                        self.error = e
                        return
                    self.written += len(pending) // 2
                pending = b""
            if chunk is None:
                return

    def finish(self, timeout: float | None = 60.0) -> None:
        """Flush everything queued (respecting the limit), then close."""
        self._q.put(None)
        self._thread.join(timeout)
        try:
            self._stream.stop()  # PortAudio drains queued buffers before stopping
            self._stream.close()
        except Exception:
            pass


# ---------------------------------------------------------------- system TTS (macOS)


def have_system_say() -> bool:
    return sys.platform == "darwin" and shutil.which("say") is not None


def say_text(text: str) -> bool:
    """Speak with macOS `say`. The fallback when a reply comes back without audio."""
    if not text.strip() or not have_system_say():
        return False
    try:
        subprocess.run(["say", text], check=False, timeout=60)
        return True
    except Exception:
        return False


def say_to_wav(text: str, rate: int = INPUT_RATE) -> bytes:
    """Synthesize a test utterance with macOS `say` -> 16 kHz WAV.

    Lets you exercise the model's speech UNDERSTANDING without a mic or a
    quiet room: `demo_omni.py --say "bring me my keys"`.
    """
    if not have_system_say():
        raise AudioError("--say needs macOS `say`; record a WAV and use --audio instead.")
    # The default system voice is often a Siri voice, which `say -o` renders as
    # ~5 ms of silence. Fall back to voices that can write to a file.
    for voice in (None, "Samantha", "Alex", "Daniel"):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "utt.wav"
            cmd = ["say", "-o", str(out), "--file-format=WAVE", f"--data-format=LEI16@{rate}"]
            cmd += ["-v", voice] if voice else []
            try:
                subprocess.run(cmd + [text], check=True, timeout=60, capture_output=True)
                wav = out.read_bytes()
            except (subprocess.SubprocessError, OSError):
                continue
        if wav_duration_s(wav) >= 0.3:
            return to_model_wav(wav, rate)
    raise AudioError("macOS `say` produced no audio; record a WAV and use --audio instead.")
