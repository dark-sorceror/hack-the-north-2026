"""Voice layer: payload format, stream parsing, intent parsing, WAV plumbing.

Never touches the network, the mic, or the speakers: the HTTP transport is a
fake that replays canned SSE lines, and audio is synthesized in memory.
numpy/cv2-dependent tests skip themselves when those aren't installed.
"""

import base64
import io
import json
import math
import random
import sys
import unittest
import wave
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.voice.audio import (
    find_cut_sample,
    pcm16_to_wav,
    synth_chime,
    trim_pcm16,
    wav_to_pcm16,
)
from retriever.voice.omni import (
    MockOmniClient,
    OmniBackend,
    OmniClient,
    OmniConfig,
    OmniError,
    PcmAssembler,
    build_messages,
    build_request,
    iter_sse_json,
)
from retriever.voice.senses import (
    DEFAULT_SYSTEM_PROMPT,
    Intent,
    Senses,
    StaticCamera,
    parse_intent,
    spoken_tail_fraction,
    strip_intent,
)

try:
    import numpy as np
except ImportError:  # core tests still run without it
    np = None

try:
    import cv2
except ImportError:
    cv2 = None

# Smallest thing that starts and ends like a JPEG; the mock never decodes it.
FAKE_JPEG = b"\xff\xd8\xff\xe0" + b"\x00" * 32 + b"\xff\xd9"


def tone_pcm(freq=440.0, seconds=0.5, rate=16000, amp=0.3):
    return b"".join(
        int(amp * 32767 * math.sin(2 * math.pi * freq * i / rate)).to_bytes(2, "little", signed=True)
        for i in range(int(seconds * rate))
    )


def sse(obj):
    return f"data: {json.dumps(obj)}\n".encode()


def fake_transport(lines, sink=None):
    def transport(url, headers, body, timeout_s):
        if sink is not None:
            sink.update(url=url, headers=dict(headers), body=json.loads(body), timeout=timeout_s)
        yield from lines

    return transport


LIVE_CFG = OmniConfig(api_key="sk-test", base_url="https://relay.example/v1", model="qwen3.5-omni-flash")


# ---------------------------------------------------------------- request format


class TestBuildMessages(unittest.TestCase):
    def setUp(self):
        self.wav = pcm16_to_wav(tone_pcm(seconds=0.1), 16000)
        self.msgs = build_messages(
            text="bring me my inhaler",
            image_jpeg=FAKE_JPEG,
            audio_wav=self.wav,
            system="be a robot",
            history=[{"role": "user", "content": "hi"}, {"role": "assistant", "content": "hello"}],
        )

    def test_system_then_history_then_user(self):
        roles = [m["role"] for m in self.msgs]
        self.assertEqual(roles, ["system", "user", "assistant", "user"])
        self.assertEqual(self.msgs[0]["content"], "be a robot")

    def test_image_part_is_a_jpeg_data_url_of_the_same_bytes(self):
        part = self.msgs[-1]["content"][0]
        self.assertEqual(part["type"], "image_url")
        url = part["image_url"]["url"]
        self.assertTrue(url.startswith("data:image/jpeg;base64,"))
        self.assertEqual(base64.b64decode(url.split(",", 1)[1]), FAKE_JPEG)

    def test_audio_part_uses_input_audio_with_alibaba_prefix(self):
        part = self.msgs[-1]["content"][1]
        self.assertEqual(part["type"], "input_audio")
        self.assertEqual(part["input_audio"]["format"], "wav")
        data = part["input_audio"]["data"]
        self.assertTrue(data.startswith("data:;base64,"))
        self.assertEqual(base64.b64decode(data.split(",", 1)[1]), self.wav)

    def test_text_part_last(self):
        self.assertEqual(self.msgs[-1]["content"][2], {"type": "text", "text": "bring me my inhaler"})
        self.assertEqual(len(self.msgs[-1]["content"]), 3)

    def test_raw_base64_option_for_strict_openai_relays(self):
        msgs = build_messages(audio_wav=self.wav, audio_data_uri=False)
        data = msgs[-1]["content"][0]["input_audio"]["data"]
        self.assertFalse(data.startswith("data:"))
        self.assertEqual(base64.b64decode(data), self.wav)

    def test_png_is_labelled_png(self):
        png = b"\x89PNG\r\n\x1a\n" + b"\x00" * 16
        url = build_messages(image_jpeg=png)[-1]["content"][0]["image_url"]["url"]
        self.assertTrue(url.startswith("data:image/png;base64,"))

    def test_system_can_be_folded_into_the_user_turn(self):
        msgs = build_messages(text="hi", system="be a robot", system_as_user=True)
        self.assertEqual([m["role"] for m in msgs], ["user"])
        self.assertEqual(msgs[0]["content"][0]["text"], "be a robot\n\nhi")

    def test_audio_only_turn_has_no_empty_text_part(self):
        parts = build_messages(audio_wav=self.wav)[-1]["content"]
        self.assertEqual([p["type"] for p in parts], ["input_audio"])

    def test_nothing_to_send_is_an_error(self):
        with self.assertRaises(ValueError):
            build_messages(text="   ")


class TestBuildRequest(unittest.TestCase):
    def test_audio_output_request_shape(self):
        body = build_request(LIVE_CFG, [{"role": "user", "content": "hi"}])
        self.assertEqual(body["model"], "qwen3.5-omni-flash")
        self.assertIs(body["stream"], True)  # mandatory for audio output
        self.assertEqual(body["stream_options"], {"include_usage": True})
        self.assertEqual(body["modalities"], ["text", "audio"])
        self.assertEqual(body["audio"], {"voice": "Ethan", "format": "wav"})
        json.dumps(body)  # must be serialisable as-is

    def test_text_only_mode_omits_audio_block(self):
        cfg = OmniConfig(audio_output=False)
        body = build_request(cfg, [{"role": "user", "content": "hi"}])
        self.assertEqual(body["modalities"], ["text"])
        self.assertNotIn("audio", body)

    def test_config_from_env(self):
        cfg = OmniConfig.from_env(
            {
                "OMNI_BASE_URL": "https://x.example/v1/",
                "OMNI_API_KEY": " k ",
                "OMNI_MODEL": "qwen3.5-omni-plus",
                "OMNI_VOICE": "Tina",
                "OMNI_AUDIO_RAW_B64": "1",
                "OMNI_MAX_TOKENS": "200",
            }
        )
        self.assertEqual(cfg.base_url, "https://x.example/v1")
        self.assertEqual(cfg.api_key, "k")
        self.assertEqual((cfg.model, cfg.voice), ("qwen3.5-omni-plus", "Tina"))
        self.assertFalse(cfg.audio_data_uri)
        self.assertEqual(build_request(cfg, [])["max_tokens"], 200)

    def test_env_defaults(self):
        cfg = OmniConfig.from_env({})
        self.assertEqual(cfg.base_url, "https://yibuapi.com/v1")
        self.assertEqual(cfg.api_key, "")
        self.assertTrue(cfg.audio_output and cfg.audio_data_uri)


# ---------------------------------------------------------------- stream parsing


class TestStream(unittest.TestCase):
    def test_sse_skips_noise_and_stops_at_done(self):
        lines = [b": keep-alive\n", b"\n", b"data: {bad json\n", sse({"a": 1}), b"data: [DONE]\n", sse({"b": 2})]
        self.assertEqual(list(iter_sse_json(lines)), [{"a": 1}])

    def test_assembler_handles_split_base64_odd_bytes_and_padding(self):
        pcm = tone_pcm(seconds=0.05, rate=24000)
        b64 = base64.b64encode(pcm).decode()
        a = PcmAssembler()
        out = b""
        for i in range(0, len(b64), 7):  # 7 is deliberately not a multiple of 4
            out += a.feed(b64[i : i + 7])
            self.assertEqual(len(out) % 2, 0)
        self.assertEqual(out, pcm)
        # independently padded chunks, each an odd number of bytes
        a2 = PcmAssembler()
        pieces = [pcm[i : i + 101] for i in range(0, len(pcm), 101)]
        out2 = b"".join(a2.feed(base64.b64encode(p).decode()) for p in pieces)
        self.assertEqual(a2.pcm, out2)
        self.assertEqual(out2, pcm[: len(pcm) - len(pcm) % 2])

    def test_assembler_strips_a_wav_header_and_reads_its_rate(self):
        pcm = tone_pcm(seconds=0.02, rate=16000)
        b64 = base64.b64encode(pcm16_to_wav(pcm, 16000)).decode()
        a = PcmAssembler()
        out = b"".join(a.feed(b64[i : i + 16]) for i in range(0, len(b64), 16))
        self.assertEqual(out, pcm)
        self.assertEqual(a.sample_rate, 16000)

    def test_client_streams_text_and_audio(self):
        pcm = tone_pcm(seconds=0.2, rate=24000)
        b64 = base64.b64encode(pcm).decode()
        half = len(b64) // 2
        lines = [
            sse({"choices": [{"delta": {"role": "assistant", "content": ""}, "index": 0}]}),
            sse({"choices": [{"delta": {"content": "Getting your "}, "index": 0}]}),
            sse({"choices": [{"delta": {"audio": {"data": b64[:half]}}, "index": 0}]}),
            sse({"choices": [{"delta": {"content": 'inhaler.\nINTENT: {"intent": "fetch"}'}}]}),
            sse({"choices": [{"delta": {"audio": {"data": b64[half:]}}, "finish_reason": "stop"}]}),
            sse({"choices": [], "usage": {"total_tokens": 321}, "model": "qwen3.5-omni-flash"}),
            b"data: [DONE]\n",
        ]
        sink, deltas, chunks = {}, [], []
        client = OmniClient(LIVE_CFG, transport=fake_transport(lines, sink))
        reply = client.ask(
            text="bring my inhaler", image_jpeg=FAKE_JPEG, on_text=deltas.append, on_audio=chunks.append
        )
        self.assertEqual(reply.text, 'Getting your inhaler.\nINTENT: {"intent": "fetch"}')
        self.assertEqual("".join(deltas), reply.text)
        self.assertEqual(b"".join(chunks), pcm)
        got, rate, ch = wav_to_pcm16(reply.audio_wav)
        self.assertEqual((got, rate, ch), (pcm, 24000, 1))
        self.assertAlmostEqual(reply.audio_seconds, 0.2, places=3)
        self.assertIsNotNone(reply.first_token_s)
        self.assertIsNotNone(reply.first_audio_s)
        self.assertGreaterEqual(reply.total_s, reply.first_token_s)
        self.assertEqual(reply.usage, {"total_tokens": 321})
        # what went over the wire
        self.assertEqual(sink["url"], "https://relay.example/v1/chat/completions")
        self.assertEqual(sink["headers"]["Authorization"], "Bearer sk-test")
        self.assertEqual(sink["body"]["modalities"], ["text", "audio"])
        self.assertEqual(sink["body"]["messages"][-1]["content"][0]["type"], "image_url")

    def test_transcript_only_stream_still_yields_text(self):
        lines = [sse({"choices": [{"delta": {"audio": {"transcript": "Hello there."}}}]})]
        reply = OmniClient(LIVE_CFG, transport=fake_transport(lines)).ask(text="hi")
        self.assertEqual(reply.text, "Hello there.")
        self.assertIsNone(reply.audio_wav)

    def test_error_in_stream_raises_omni_error(self):
        lines = [sse({"error": {"message": "model not found", "type": "invalid_request_error"}})]
        with self.assertRaises(OmniError) as cm:
            OmniClient(LIVE_CFG, transport=fake_transport(lines)).ask(text="hi")
        self.assertIn("model not found", str(cm.exception))

    def test_non_streamed_json_body_is_still_understood(self):
        pcm = tone_pcm(seconds=0.05, rate=24000)
        body = json.dumps(
            {"choices": [{"message": {"content": "Hi.", "audio": {"data": base64.b64encode(pcm).decode()}}}]},
            indent=2,
        )
        lines = [ln.encode() + b"\n" for ln in body.splitlines()]
        reply = OmniClient(LIVE_CFG, transport=fake_transport(lines)).ask(text="hi")
        self.assertEqual(reply.text, "Hi.")
        self.assertEqual(wav_to_pcm16(reply.audio_wav)[0], pcm)

    def test_json_error_with_200_status_raises(self):
        lines = [b'{"error": {"message": "insufficient quota"}}\n']
        with self.assertRaises(OmniError) as cm:
            OmniClient(LIVE_CFG, transport=fake_transport(lines)).ask(text="hi")
        self.assertIn("insufficient quota", str(cm.exception))

    def test_garbage_body_raises_instead_of_returning_empty(self):
        with self.assertRaises(OmniError):
            OmniClient(LIVE_CFG, transport=fake_transport([b"<html>502 Bad Gateway</html>\n"])).ask(text="hi")

    def test_missing_key_fails_before_any_network(self):
        def boom(*a):
            raise AssertionError("transport must not be called")

        with self.assertRaises(OmniError):
            OmniClient(OmniConfig(api_key=""), transport=boom).ask(text="hi")


# ---------------------------------------------------------------- mock client


class TestMockClient(unittest.TestCase):
    def test_satisfies_backend_protocol(self):
        self.assertIsInstance(MockOmniClient(), OmniBackend)
        self.assertIsInstance(OmniClient(LIVE_CFG), OmniBackend)

    def test_roundtrip(self):
        mock = MockOmniClient()
        deltas, chunks = [], []
        reply = mock.ask(
            text="Can you bring me my inhaler?", image_jpeg=FAKE_JPEG,
            on_text=deltas.append, on_audio=chunks.append,
        )
        self.assertEqual("".join(deltas), reply.text)
        self.assertIn("INTENT:", reply.text)
        pcm, rate, _ = wav_to_pcm16(reply.audio_wav)
        self.assertEqual(rate, 24000)
        self.assertEqual(b"".join(chunks), pcm)
        self.assertGreater(reply.audio_seconds, 0.1)
        self.assertEqual(parse_intent(reply.text).to_dict(), {"intent": "fetch", "object": "inhaler"})
        # the mock runs the real message builder
        types = [p["type"] for p in mock.calls[-1]["messages"][-1]["content"]]
        self.assertEqual(types, ["image_url", "text"])

    def test_covers_both_niches(self):
        mock = MockOmniClient()
        self.assertEqual(parse_intent(mock.ask(text="sort this can").text).intent, "sort")
        self.assertEqual(parse_intent(mock.ask(text="where is my phone").text).intent, "where_is")
        self.assertEqual(parse_intent(mock.ask(text="hello").text).intent, "none")

    def test_custom_replies(self):
        mock = MockOmniClient(replies={"dance": "No.\nINTENT: none"})
        self.assertEqual(mock.ask(text="please DANCE").text, "No.\nINTENT: none")


# ---------------------------------------------------------------- intent parsing


class TestParseIntent(unittest.TestCase):
    def test_valid(self):
        i = parse_intent('Getting it.\nINTENT: {"intent": "fetch", "object": "inhaler"}')
        self.assertEqual((i.intent, i.object, i.valid), ("fetch", "inhaler", True))
        self.assertTrue(i.actionable)

    def test_extra_keys_land_in_args(self):
        i = parse_intent('INTENT: {"intent": "sort", "object": "can", "bin": "recycling", "seen": true}')
        self.assertEqual(i.args, {"bin": "recycling", "seen": True})
        self.assertEqual(i.to_dict()["bin"], "recycling")

    def test_missing(self):
        for text in ("Sure, happy to help.", "", None, 42, "   "):
            i = parse_intent(text)
            self.assertEqual(i, Intent())
            self.assertFalse(i.actionable)

    def test_malformed_json_is_repaired_not_trusted(self):
        cases = {
            'INTENT: {"intent": "fetch", "object": "keys"': ("fetch", "keys"),       # no closing brace
            "INTENT: {'intent': 'sort', 'object': 'can',}": ("sort", "can"),         # quotes + comma
            "INTENT: {intent: fetch, object: water bottle}": ("fetch", "water bottle"),
            'INTENT: {"intent": "fetch", "object": None}': ("fetch", None),
            "INTENT: fetch my keys": ("fetch", "my keys"),                          # bare form
            "INTENT: none": ("none", None),
        }
        for text, want in cases.items():
            i = parse_intent(text)
            self.assertEqual((i.intent, i.object), want, text)
            self.assertFalse(i.valid, text)

    def test_garbage_never_raises(self):
        for text in ("INTENT: {{{", "INTENT:", 'INTENT: {"intent": [1, 2', "INTENT: }}}{", "{" * 50):
            self.assertEqual(parse_intent(text).intent, "none")
        rng = random.Random(0)
        alphabet = 'INTENT:{}[]"\',: intentfetchsort\n\\'
        for _ in range(500):
            s = "".join(rng.choice(alphabet) for _ in range(rng.randint(0, 60)))
            self.assertIsInstance(parse_intent(s), Intent)
            self.assertIsInstance(strip_intent(s), str)

    def test_extra_prose_and_decoration(self):
        text = 'Right away. **INTENT:** {"intent": "where_is", "object": "phone"} Anything else?'
        i = parse_intent(text)
        self.assertEqual((i.intent, i.object, i.valid), ("where_is", "phone", True))
        self.assertEqual(strip_intent(text), "Right away. Anything else?")
        fenced = 'OK.\n```json\nINTENT: {"intent": "none"}\n```'
        self.assertTrue(parse_intent(fenced).valid)
        self.assertEqual(strip_intent(fenced), "OK.")

    def test_uses_the_last_intent_line(self):
        text = 'You said INTENT: none earlier.\nINTENT: {"intent": "fetch", "object": "mug"}'
        self.assertEqual(parse_intent(text).object, "mug")

    def test_json_without_prefix(self):
        i = parse_intent('On it {"intent": "bring", "object": "glasses"}')
        self.assertEqual((i.intent, i.object), ("fetch", "glasses"))

    def test_unknown_intent_is_none_but_kept(self):
        i = parse_intent('INTENT: {"intent": "dance", "object": "robot"}')
        self.assertEqual(i.intent, "none")
        self.assertEqual(i.args["unknown_intent"], "dance")
        self.assertFalse(i.valid)

    def test_braces_inside_strings(self):
        i = parse_intent('INTENT: {"intent": "fetch", "object": "a {weird} thing"}')
        self.assertEqual(i.object, "a {weird} thing")

    def test_strip_and_spoken_fraction(self):
        text = 'Getting your keys now.\nINTENT: {"intent": "fetch", "object": "keys"}'
        self.assertEqual(strip_intent(text), "Getting your keys now.")
        f = spoken_tail_fraction(text)
        self.assertTrue(0.3 < f < 0.7, f)
        self.assertIsNone(spoken_tail_fraction("No intent here."))
        self.assertIsNone(spoken_tail_fraction('INTENT: {"intent": "none"}\nThen more words.'))


# ---------------------------------------------------------------- WAV plumbing


class TestWav(unittest.TestCase):
    def test_pcm_wav_roundtrip_is_exact(self):
        pcm = tone_pcm(seconds=0.1)
        wav = pcm16_to_wav(pcm, 16000)
        with wave.open(io.BytesIO(wav)) as w:
            self.assertEqual((w.getframerate(), w.getnchannels(), w.getsampwidth()), (16000, 1, 2))
        self.assertEqual(wav_to_pcm16(wav), (pcm, 16000, 1))

    def test_odd_byte_pcm_is_truncated_not_corrupted(self):
        self.assertEqual(wav_to_pcm16(pcm16_to_wav(b"\x01\x02\x03", 8000))[0], b"\x01\x02")

    def test_chime_is_nonsilent_and_bounded(self):
        pcm = synth_chime(24000, 0.4)
        self.assertEqual(len(pcm), int(24000 * 0.4) * 2)
        self.assertTrue(any(pcm))

    def test_trim_cuts_and_fades(self):
        pcm = tone_pcm(seconds=0.5, amp=0.5)
        out = trim_pcm16(pcm, 16000, 4000)
        self.assertEqual(len(out), 8000)
        self.assertEqual(out[-2:], b"\x00\x00")  # faded to zero, no click


@unittest.skipIf(np is None, "numpy not installed")
class TestWavNumpy(unittest.TestCase):
    def test_encode_decode_roundtrip(self):
        from retriever.voice.audio import decode_wav, encode_wav

        x = (0.5 * np.sin(2 * np.pi * 300 * np.arange(1600) / 16000)).astype(np.float32)
        y, rate = decode_wav(encode_wav(x, 16000))
        self.assertEqual(rate, 16000)
        self.assertLess(float(np.max(np.abs(x - y))), 1e-3)

    def test_resample_keeps_duration_and_pitch(self):
        from retriever.voice.audio import resample

        src = 48000
        x = np.sin(2 * np.pi * 440 * np.arange(src) / src).astype(np.float32)  # 1 s at 440 Hz
        y = resample(x, src, 16000)
        self.assertEqual(len(y), 16000)
        peak_hz = np.argmax(np.abs(np.fft.rfft(y))) * 16000 / len(y)
        self.assertAlmostEqual(peak_hz, 440, delta=2)

    def test_to_model_wav_normalises_stereo_44k(self):
        from retriever.voice.audio import to_model_wav

        n = 44100 // 2
        t = np.arange(n) / 44100
        stereo = np.stack([np.sin(2 * np.pi * 200 * t), np.sin(2 * np.pi * 200 * t)], axis=1)
        pcm = (stereo * 16000).astype("<i2").tobytes()
        wav = pcm16_to_wav(pcm, 44100, channels=2)
        out, rate, ch = wav_to_pcm16(to_model_wav(wav))
        self.assertEqual((rate, ch), (16000, 1))
        self.assertAlmostEqual(len(out) / 2 / 16000, 0.5, places=2)

    def test_24bit_wav_is_converted(self):
        x = np.array([0, 1 << 22, -(1 << 22), (1 << 23) - 1], dtype=np.int32)
        raw = b"".join(int(v).to_bytes(3, "little", signed=True) for v in x)
        buf = io.BytesIO()
        with wave.open(buf, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(3)
            w.setframerate(16000)
            w.writeframes(raw)
        pcm, _, _ = wav_to_pcm16(buf.getvalue())
        self.assertEqual(list(np.frombuffer(pcm, "<i2")), [0, 1 << 14, -(1 << 14), 32767])

    def test_cut_snaps_into_the_pause_before_the_intent_line(self):
        rate = 24000
        speech, gap, intent_speech = tone_pcm(seconds=1.0, rate=rate), b"\x00\x00" * int(0.3 * rate), tone_pcm(seconds=0.8, rate=rate)
        pcm = speech + gap + intent_speech
        # a sloppy estimate (45% instead of the true ~48-60% boundary) still lands in the gap
        cut = find_cut_sample(pcm, rate, 0.45)
        self.assertTrue(rate * 1.0 <= cut <= rate * 1.3, cut / rate)
        self.assertEqual(find_cut_sample(pcm, rate, 1.0), len(pcm) // 2)


# ---------------------------------------------------------------- senses


def _jpeg(w=1280, h=720):
    if cv2 is None or np is None:
        return FAKE_JPEG
    img = np.zeros((h, w, 3), np.uint8)
    cv2.rectangle(img, (w // 3, h // 3), (w // 2, h // 2), (0, 0, 255), -1)
    return cv2.imencode(".jpg", img)[1].tobytes()


class TestSenses(unittest.TestCase):
    def setUp(self):
        self.mock = MockOmniClient()
        self.senses = Senses(self.mock, camera=StaticCamera(_jpeg()), history_turns=2)
        self.wav = pcm16_to_wav(tone_pcm(seconds=0.5, rate=16000), 16000)

    def test_image_and_audio_turn_gives_reply_and_intent(self):
        turn = self.senses.turn(text="bring me my inhaler", audio_wav=self.wav)
        self.assertTrue(turn.image_sent and turn.audio_sent)
        self.assertEqual(turn.intent.intent, "fetch")
        self.assertEqual(turn.intent.object, "inhaler")
        self.assertNotIn("INTENT", turn.said)
        self.assertTrue(turn.said)
        self.assertIsNotNone(turn.audio_wav)
        msgs = self.mock.calls[-1]["messages"]
        self.assertEqual(msgs[0], {"role": "system", "content": DEFAULT_SYSTEM_PROMPT})
        self.assertEqual([p["type"] for p in msgs[-1]["content"]], ["image_url", "input_audio", "text"])

    def test_audio_only_turn(self):
        turn = self.senses.turn(audio_wav=self.wav, look=False)
        self.assertFalse(turn.image_sent)
        self.assertTrue(turn.audio_sent)
        self.assertEqual(turn.intent.intent, "none")

    def test_spoken_intent_line_is_trimmed_from_reply_audio(self):
        turn = self.senses.turn(text="bring me my keys")
        self.assertIsNotNone(turn.cut_sample)
        full = len(wav_to_pcm16(turn.reply.audio_wav)[0]) // 2
        trimmed = len(wav_to_pcm16(turn.audio_wav)[0]) // 2
        self.assertEqual(trimmed, turn.cut_sample)
        self.assertLess(trimmed, full)

    def test_history_is_text_only_and_rolls(self):
        for word in ("keys", "phone", "wallet"):
            self.senses.turn(text=f"bring me my {word}")
        hist = self.senses.history
        self.assertEqual(len(hist), 4)  # 2 turns kept
        self.assertTrue(all(isinstance(m["content"], str) for m in hist))
        self.assertIn("phone", hist[0]["content"])
        sent = self.mock.calls[-1]["messages"]
        self.assertEqual([m["role"] for m in sent], ["system", "user", "assistant", "user", "assistant", "user"])
        self.senses.reset()
        self.assertEqual(self.senses.history, [])

    def test_context_reaches_the_model(self):
        self.senses.turn(text="where are my keys", context="keys last seen on the kitchen table")
        text = self.mock.calls[-1]["messages"][-1]["content"][-1]["text"]
        self.assertIn("kitchen table", text)

    @unittest.skipIf(cv2 is None or np is None, "opencv not installed")
    def test_frames_are_downscaled_before_upload(self):
        self.senses.turn(text="what do you see")
        url = self.mock.calls[-1]["messages"][-1]["content"][0]["image_url"]["url"]
        img = cv2.imdecode(np.frombuffer(base64.b64decode(url.split(",", 1)[1]), np.uint8), 1)
        self.assertEqual(max(img.shape[:2]), 640)


if __name__ == "__main__":
    unittest.main()
