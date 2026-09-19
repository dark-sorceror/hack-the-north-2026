"""The wire protocol: every message round-trips, every way a line can be wrong is refused."""

from __future__ import annotations

import dataclasses
import json
import random
import subprocess
import sys
import unittest
from pathlib import Path
from typing import Any

from retriever.bridge.protocol import (
    CLIENT_MESSAGES,
    MAX_JOINTS,
    MAX_LINE_BYTES,
    MAX_NAME_LEN,
    MAX_TEXT_LEN,
    PROTOCOL_VERSION,
    SERVER_MESSAGES,
    Act,
    ClearEstop,
    Error,
    Estop,
    Heartbeat,
    Hello,
    ProtocolError,
    ProtocolVersionError,
    State,
    decode,
    encode,
)

SRC = Path(__file__).resolve().parents[1] / "src"

HELLO = Hello(
    counts_per_rev=4096,
    wheel_radius_m=0.048,
    track_width_m=0.3,
    scrub_factor=1.4,
    state_hz=50.0,
    timeout_ms=300,
    motion_timeout_ms=500,
)
STATE = State(
    seq=42,
    t=1234.5,
    left_ticks=4095,
    right_ticks=-17,
    joints={"shoulder_pan": 0.1, "gripper": 0.9},
    gripper_load=0.6,
    battery=0.87,
    estop=True,
    watchdog_tripped=True,
)
BARE_STATE = State(seq=0, t=0.0, left_ticks=0, right_ticks=0)
SAMPLES = (
    Act(
        seq=42,
        base_vx=0.25,
        base_wz=-0.5,
        joints={"shoulder_pan": 0.1, "elbow_flex": -1.2},
        gripper=0.8,
        vacuum=True,
    ),
    Act(seq=0),
    Heartbeat(),
    Estop(reason="operator pressed the red button"),
    Estop(),
    ClearEstop(),
    HELLO,
    STATE,
    BARE_STATE,
    Error(reason="'state' is not valid in this direction"),
)


def wire(**fields: Any) -> bytes:
    """A hand-written line, as someone would type it into nc (json.dumps writes NaN as NaN)."""
    return json.dumps({"v": PROTOCOL_VERSION, **fields}).encode() + b"\n"


def fields_of(msg: Any) -> dict[str, Any]:
    """The message's wire fields, as a dict to edit into a broken line."""
    obj = json.loads(encode(msg))
    del obj["v"]
    return obj


class ProtocolTestCase(unittest.TestCase):
    def assertRejected(
        self, line: bytes | str, pattern: str, allowed: tuple[type, ...] | None = None
    ) -> ProtocolError:
        with self.assertRaisesRegex(ProtocolError, pattern) as caught:
            decode(line, allowed)
        self.assertLessEqual(len(str(caught.exception)), MAX_TEXT_LEN)
        return caught.exception


class TestRoundTrip(ProtocolTestCase):
    def test_every_message_round_trips(self) -> None:
        self.assertEqual(
            {type(m) for m in SAMPLES}, set(CLIENT_MESSAGES) | set(SERVER_MESSAGES)
        )
        for msg in SAMPLES:
            with self.subTest(msg=msg):
                line = encode(msg)
                self.assertEqual(decode(line), msg)
                self.assertEqual(decode(line.decode()), msg)
                self.assertEqual(encode(decode(line)), line)

    def test_encoding_is_one_compact_ascii_line(self) -> None:
        self.assertEqual(encode(Heartbeat()), b'{"v":1,"type":"heartbeat"}\n')
        self.assertEqual(
            encode(Act(seq=7, base_vx=0.2)),
            b'{"v":1,"type":"act","seq":7,"base_vx":0.2,"base_wz":0.0,'
            b'"joints":{},"gripper":null,"vacuum":null}\n',
        )
        for msg in SAMPLES:
            with self.subTest(msg=msg):
                line = encode(msg)
                self.assertIsInstance(line, bytes)
                self.assertTrue(line.endswith(b"\n"))
                self.assertEqual(line.count(b"\n"), 1)
                self.assertTrue(line.startswith(b'{"v":1,"type":"%s"' % msg.TYPE.encode()))
                self.assertTrue(line.isascii())

    def test_non_ascii_text_survives_on_an_ascii_line(self) -> None:
        msg = Estop(reason="caf\u00e9 \u2615 \U0001d11e")
        line = encode(msg)
        self.assertTrue(line.isascii())
        self.assertEqual(decode(line), msg)

    def test_defaults_fill_optional_fields(self) -> None:
        self.assertEqual(decode(wire(type="act", seq=5)), Act(seq=5))
        state = decode(wire(type="state", seq=1, t=2.0, left_ticks=3, right_ticks=4))
        self.assertEqual(state, State(seq=1, t=2.0, left_ticks=3, right_ticks=4))
        self.assertEqual((state.joints, state.battery, state.estop), ({}, 1.0, False))
        self.assertEqual(decode(wire(type="estop")), Estop(reason=""))

    def test_numbers_decode_as_floats_and_integers_as_ints(self) -> None:
        act = decode(wire(type="act", seq=3, base_vx=1, joints={"elbow_flex": 2}))
        self.assertIsInstance(act, Act)
        self.assertIsInstance(act.seq, int)
        self.assertIsInstance(act.base_vx, float)
        self.assertIsInstance(act.joints["elbow_flex"], float)

    def test_line_terminator_is_optional_and_may_be_crlf(self) -> None:
        for line in (
            b'{"v":1,"type":"heartbeat"}',
            b'{"v":1,"type":"heartbeat"}\n',
            b'{"v":1,"type":"heartbeat"}\r\n',
            bytearray(b'{"v":1,"type":"heartbeat"}\n'),
            '{"v":1,"type":"heartbeat"}\n',
        ):
            with self.subTest(line=line):
                self.assertEqual(decode(line), Heartbeat())

    def test_largest_valid_message_fits_in_one_line(self) -> None:
        # 64 joints whose 64-character names each escape to 12 ASCII bytes per character.
        joints = {
            "\U0001d11e" * (MAX_NAME_LEN - 1) + chr(0x1F600 + i): -1.7976931348623157e308
            for i in range(MAX_JOINTS)
        }
        msg = State(
            seq=2**53,
            t=-1.7976931348623157e308,
            left_ticks=-(2**53),
            right_ticks=-(2**53),
            joints=joints,
            gripper_load=-2.2250738585072014e-308,
            battery=-2.2250738585072014e-308,
        )
        line = encode(msg)
        self.assertLessEqual(len(line) - 1, MAX_LINE_BYTES)
        self.assertEqual(decode(line), msg)


class TestEncodeValidates(ProtocolTestCase):
    def test_nan_velocity_is_refused_at_the_laptop(self) -> None:
        with self.assertRaisesRegex(ProtocolError, r"act\.base_vx must be a finite number"):
            encode(Act(seq=1, base_vx=float("nan")))
        with self.assertRaisesRegex(ProtocolError, r"act\.base_wz must be a finite number"):
            encode(Act(seq=1, base_wz=float("-inf")))
        with self.assertRaisesRegex(ProtocolError, r"act\.joints\['elbow_flex'\]"):
            encode(Act(seq=1, joints={"elbow_flex": float("inf")}))

    def test_every_rule_applies_on_the_way_out(self) -> None:
        cases = [
            (Act(seq=-1), r"act\.seq must be an integer from 0"),
            (Act(seq=True), r"act\.seq .* got true"),
            (Act(seq=1, joints={"gripper": 0.5}), r"must not contain 'gripper'"),
            (Act(seq=1, vacuum=1), r"act\.vacuum must be true or false"),
            (Estop(reason="x" * (MAX_TEXT_LEN + 1)), r"\(501 characters\)"),
            (Error(reason=None), r"error\.reason must be a string"),
            (State(seq=0, t=float("nan"), left_ticks=0, right_ticks=0), r"state\.t"),
            (State(seq=0, t=0.0, left_ticks=0.5, right_ticks=0), r"state\.left_ticks"),
        ]
        for msg, pattern in cases:
            with self.subTest(msg=msg), self.assertRaisesRegex(ProtocolError, pattern):
                encode(msg)

    def test_only_protocol_messages_encode(self) -> None:
        for thing in (None, "heartbeat", {"v": 1, "type": "heartbeat"}, object()):
            with self.subTest(thing=thing), self.assertRaisesRegex(ProtocolError, "not a"):
                encode(thing)


class TestLineRules(ProtocolTestCase):
    def test_oversize_line_is_refused(self) -> None:
        heartbeat = b'{"v":1,"type":"heartbeat"}'
        exactly = heartbeat + b" " * (MAX_LINE_BYTES - len(heartbeat))
        self.assertEqual(decode(exactly + b"\n"), Heartbeat())
        self.assertRejected(exactly + b" \n", f"the limit is {MAX_LINE_BYTES}")
        self.assertRejected(exactly.decode() + " ", "limit")
        self.assertRejected(b"x" * (10 * MAX_LINE_BYTES), "limit")

    def test_limit_counts_bytes_not_characters(self) -> None:
        # Two UTF-8 bytes per character: under the limit in characters, over it in bytes.
        reason = "\u00e9" * (MAX_LINE_BYTES // 2)
        self.assertRejected(json.dumps({"reason": reason}, ensure_ascii=False), "limit")

    def test_empty_line_is_refused(self) -> None:
        for line in (b"", b"\n", b"\r\n", b"   \t \n", ""):
            with self.subTest(line=line):
                self.assertRejected(line, "empty line")

    def test_invalid_utf8_is_refused(self) -> None:
        for line in (b'{"v":1,"type":"estop","reason":"\xff"}', b'{"v":1,"a":"\xc3', b"\x80"):
            with self.subTest(line=line):
                self.assertRejected(line, "not valid UTF-8")
        self.assertRejected('{"v":1,"type":"estop","reason":"\ud800"}', "not valid UTF-8")

    def test_invalid_json_is_refused(self) -> None:
        for line in (
            b'{"v":1,',
            b"estop",
            b"{'v':1,'type':'estop'}",
            b'{"v":1,"type":"heartbeat"}{"v":1,"type":"heartbeat"}',
        ):
            with self.subTest(line=line[:40]):
                self.assertRejected(line, "invalid JSON")

    def test_non_object_is_refused(self) -> None:
        for line, got in (
            (b"[]", "an array"),
            (b"1", "1"),
            (b'"go"', "'go'"),
            (b"null", "null"),
        ):
            with self.subTest(line=line):
                self.assertRejected(line, f"must be a JSON object, got {got}")

    def test_duplicate_keys_are_refused(self) -> None:
        self.assertRejected(
            b'{"v":1,"type":"act","seq":1,"base_vx":0.1,"base_vx":2.0}',
            "duplicate key 'base_vx'",
        )
        self.assertRejected(
            b'{"v":1,"type":"act","seq":1,"joints":{"elbow_flex":0,"elbow_flex":1}}',
            "duplicate key 'elbow_flex'",
        )

    def test_input_that_is_not_a_line_is_refused(self) -> None:
        for thing in (None, 42, ["{}"], memoryview(b"{}")):
            with self.subTest(thing=thing):
                self.assertRejected(thing, "must be bytes or str")


class TestEnvelope(ProtocolTestCase):
    def test_missing_or_wrong_version_is_a_version_error(self) -> None:
        self.assertRejected(b'{"type":"heartbeat"}', 'missing protocol version "v"')
        for version in (2, 0, -1, "1", 1.0, True, None, [1]):
            with self.subTest(version=version):
                line = json.dumps({"v": version, "type": "heartbeat"})
                with self.assertRaisesRegex(ProtocolVersionError, "not supported"):
                    decode(line)

    def test_version_is_checked_before_anything_a_newer_peer_might_add(self) -> None:
        with self.assertRaises(ProtocolVersionError):
            decode(b'{"v":2,"type":"teleport","warp":9}')

    def test_version_error_is_a_protocol_error_is_a_value_error(self) -> None:
        self.assertTrue(issubclass(ProtocolVersionError, ProtocolError))
        self.assertTrue(issubclass(ProtocolError, ValueError))

    def test_bad_type_is_a_protocol_error_but_not_a_version_error(self) -> None:
        for line, pattern in (
            (wire(), 'missing message "type"'),
            (wire(type=7), '"type" must be a string, got 7'),
            (wire(type=None), '"type" must be a string, got null'),
            (wire(type="teleport"), "unknown message type 'teleport'"),
        ):
            with self.subTest(line=line):
                error = self.assertRejected(line, pattern)
                self.assertNotIsInstance(error, ProtocolVersionError)

    def test_message_from_the_wrong_direction_is_refused(self) -> None:
        for msg in SAMPLES:
            to_pi = type(msg) in CLIENT_MESSAGES
            wrong_way = SERVER_MESSAGES if to_pi else CLIENT_MESSAGES
            right_way = CLIENT_MESSAGES if to_pi else SERVER_MESSAGES
            with self.subTest(msg=msg):
                self.assertEqual(decode(encode(msg), right_way), msg)
                self.assertRejected(
                    encode(msg), f"^'{msg.TYPE}' is not valid in this direction$", wrong_way
                )

    def test_directions_partition_the_messages(self) -> None:
        self.assertFalse(set(CLIENT_MESSAGES) & set(SERVER_MESSAGES))


class TestFieldRules(ProtocolTestCase):
    def test_sideways_velocity_is_refused_because_a_tank_cannot_strafe(self) -> None:
        for name, value in (("base_vy", 0.3), ("vy", -0.1), ("base_vy", 0)):
            with self.subTest(name=name, value=value):
                line = wire(type="act", seq=1, **{name: value})
                self.assertRejected(line, f"'{name}'.*tank base cannot strafe")
        # The strafe explanation wins over a missing seq and over other unknown fields.
        self.assertRejected(wire(type="act", speed=1, base_vy=0.3), "cannot strafe")

    def test_unknown_fields_are_refused_and_listed(self) -> None:
        self.assertRejected(wire(type="heartbeat", seq=1), r"heartbeat has unknown .* 'seq'")
        self.assertRejected(
            wire(type="act", seq=1, speed=1, turbo=True),
            r"act has unknown field\(s\) 'speed', 'turbo'",
        )
        # Only an act is told about strafing; elsewhere vy is just another unknown field.
        self.assertRejected(wire(**{**fields_of(STATE), "vy": 0.0}), "unknown field.*'vy'")

    def test_many_long_unknown_fields_still_make_a_short_error(self) -> None:
        extra = {f"field_{i:03d}_" + "x" * 60: i for i in range(200)}
        self.assertRejected(wire(type="heartbeat", **extra), r"'field_000_x+'.* and 195 more")
        self.assertEqual(len(str(ProtocolError("x" * 5000))), MAX_TEXT_LEN)

    def test_missing_required_fields_are_named(self) -> None:
        required = [
            (Act(seq=1), ["seq"]),
            (HELLO, [f.name for f in dataclasses.fields(Hello)]),
            (STATE, ["seq", "t", "left_ticks", "right_ticks"]),
            (Error(reason="x"), ["reason"]),
        ]
        for msg, names in required:
            for name in names:
                with self.subTest(type=msg.TYPE, missing=name):
                    obj = fields_of(msg)
                    del obj[name]
                    self.assertRejected(wire(**obj), rf"missing required field\(s\) '{name}'")

    def test_numbers_must_be_finite_and_not_bool(self) -> None:
        for literal in ("NaN", "Infinity", "-Infinity", "1e999", "-1e999", "1" + "0" * 400):
            for field in ("base_vx", "base_wz", "gripper"):
                with self.subTest(literal=literal, field=field):
                    line = f'{{"v":1,"type":"act","seq":1,"{field}":{literal}}}'
                    self.assertRejected(line, rf"act\.{field} must be a finite number")
            with self.subTest(literal=literal, field="joints"):
                line = f'{{"v":1,"type":"act","seq":1,"joints":{{"elbow_flex":{literal}}}}}'
                self.assertRejected(line, r"act\.joints\['elbow_flex'\] must be a finite")
        for value in (True, False, "0.1", [0.1], {"x": 0.1}):
            with self.subTest(value=value):
                self.assertRejected(wire(type="act", seq=1, base_vx=value), "finite number")
        self.assertRejected(wire(**{**fields_of(BARE_STATE), "t": float("inf")}), r"state\.t")

    def test_integers_are_whole_json_integers_within_2_to_the_53(self) -> None:
        state = fields_of(BARE_STATE)
        self.assertEqual(decode(wire(type="act", seq=2**53)), Act(seq=2**53))
        self.assertEqual(decode(wire(**{**state, "left_ticks": -(2**53)})).left_ticks, -(2**53))
        for value in (2**53 + 1, 1.5, 1.0, "1", True, None):
            with self.subTest(value=value):
                self.assertRejected(wire(type="act", seq=value), r"act\.seq must be an integer")
                line = wire(**{**state, "right_ticks": value})
                self.assertRejected(line, r"state\.right_ticks must be an integer")
        self.assertRejected(wire(type="act", seq=2**100), "a 101-bit integer")
        # Past Python's int-parsing limit (3.11+) or merely past 2**53: refused either way.
        huge = b'{"v":1,"type":"act","seq":' + b"9" * 5000 + b"}"
        self.assertRejected(huge, r"act\.seq|JSON")

    def test_counts_are_not_negative(self) -> None:
        self.assertRejected(wire(type="act", seq=-1), "from 0 to 2")
        self.assertRejected(wire(**{**fields_of(BARE_STATE), "seq": -1}), r"state\.seq")

    def test_hello_constants_must_be_positive(self) -> None:
        hello = fields_of(HELLO)
        for name in ("counts_per_rev", "timeout_ms", "motion_timeout_ms"):
            for value in (0, -1, 300.0, True):
                with self.subTest(name=name, value=value):
                    self.assertRejected(wire(**{**hello, name: value}), "an integer from 1 to")
        for name in ("wheel_radius_m", "track_width_m", "scrub_factor", "state_hz"):
            for value in (0, -0.048, 0.0, "1", float("inf")):
                with self.subTest(name=name, value=value):
                    self.assertRejected(wire(**{**hello, name: value}), rf"hello\.{name}")
        self.assertEqual(
            decode(wire(**{**hello, "state_hz": 1})), dataclasses.replace(HELLO, state_hz=1.0)
        )

    def test_booleans_are_json_booleans_only(self) -> None:
        state = fields_of(BARE_STATE)
        for value in (1, 0, "true", None):
            with self.subTest(value=value):
                self.assertRejected(wire(**{**state, "estop": value}), r"state\.estop must be")
        self.assertRejected(wire(type="act", seq=1, vacuum=0), r"act\.vacuum must be true or")

    def test_null_leaves_gripper_and_vacuum_as_they_are(self) -> None:
        act = decode(wire(type="act", seq=1, gripper=None, vacuum=None))
        self.assertEqual(act, Act(seq=1))
        self.assertRejected(wire(type="act", seq=1, gripper="open"), r"act\.gripper")

    def test_text_has_a_length_limit(self) -> None:
        longest = "x" * MAX_TEXT_LEN
        self.assertEqual(decode(wire(type="estop", reason=longest)), Estop(reason=longest))
        self.assertRejected(wire(type="estop", reason=longest + "x"), "501 characters")
        self.assertRejected(wire(type="error", reason=5), r"error\.reason must be a string")
        self.assertRejected(wire(type="error", reason=None), r"error\.reason must be a string")

    def test_joints_are_a_bounded_map_of_names_to_finite_numbers(self) -> None:
        def act(joints: Any) -> bytes:
            return wire(type="act", seq=1, joints=joints)

        most = {f"j{i}": 0.0 for i in range(MAX_JOINTS)}
        self.assertEqual(decode(act(most)).joints, most)
        self.assertRejected(act({**most, "j64": 0.0}), f"65 joints; the limit is {MAX_JOINTS}")
        longest = {"n" * MAX_NAME_LEN: 1.0}
        self.assertEqual(decode(act(longest)).joints, longest)
        self.assertRejected(act({"n" * (MAX_NAME_LEN + 1): 1.0}), "invalid joint name")
        self.assertRejected(act({"": 1.0}), "invalid joint name")
        for joints in ([0.1, 0.2], "elbow_flex", None):
            with self.subTest(joints=joints):
                self.assertRejected(act(joints), r"act\.joints must be an object")
        for angle in ("0.1", True, None, [0.1]):
            with self.subTest(angle=angle):
                self.assertRejected(act({"elbow_flex": angle}), r"\['elbow_flex'\] must be")

    def test_gripper_goes_in_its_own_field_on_an_act_but_is_a_joint_in_state(self) -> None:
        self.assertRejected(
            wire(type="act", seq=1, joints={"gripper": 0.5}), "must not contain 'gripper'"
        )
        state = decode(wire(**{**fields_of(BARE_STATE), "joints": {"gripper": 0.5}}))
        self.assertEqual(state.joints, {"gripper": 0.5})


class TestRobustness(ProtocolTestCase):
    def test_mangled_lines_raise_only_protocol_errors(self) -> None:
        rng = random.Random(1234)
        lines = [encode(msg) for msg in SAMPLES]
        noise = [b"NaN", b"1e999", b"true", b"null", b"[", b"}", b'"', b",", b"\\", b"\xff"]
        for _ in range(3000):
            line = bytearray(rng.choice(lines))
            for _ in range(rng.randint(1, 4)):
                at = rng.randrange(len(line) + 1)
                mutation = rng.randrange(4)
                if mutation == 0:
                    line[at : at + 1] = bytes([rng.randrange(256)])
                elif mutation == 1:
                    del line[at : at + rng.randint(1, 8)]
                elif mutation == 2:
                    line[at:at] = rng.choice(noise)
                else:
                    line[at:at] = line[rng.randrange(len(line) + 1) :][:12]
            try:
                msg = decode(bytes(line))
            except ProtocolError as exc:
                self.assertLessEqual(len(str(exc)), MAX_TEXT_LEN)
            else:
                self.assertEqual(decode(encode(msg)), msg)  # whatever decodes, re-encodes

    def test_pathological_nesting_is_a_protocol_error(self) -> None:
        depth = MAX_LINE_BYTES // 2 - 100
        line = b'{"v":1,"type":"estop","reason":' + b"[" * depth + b"]" * depth + b"}"
        self.assertRejected(line, "reason|recursion")  # which one depends on the interpreter

    def test_imports_bare_under_python_dash_s_without_the_rest_of_the_package(self) -> None:
        code = (
            f"import sys; sys.path.insert(0, {str(SRC)!r}); import retriever.bridge.protocol; "
            "print(sorted(m for m in sys.modules if m.startswith('retriever')))"
        )
        out = subprocess.run(
            [sys.executable, "-S", "-c", code],
            capture_output=True,
            text=True,
            timeout=30,
            check=False,
        )
        self.assertEqual(out.returncode, 0, out.stderr)
        self.assertEqual(
            out.stdout.strip(),
            "['retriever', 'retriever.bridge', 'retriever.bridge.protocol']",
        )


if __name__ == "__main__":
    unittest.main()
