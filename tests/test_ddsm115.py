"""DDSM115Driver, DDSM115Bus, the vendored ddsm115 frames, and scripts/wheel_check.py.

No hardware and no pyserial. FakeRS485 is a USB-RS485 port with DDSM115 motors
behind it, at the BYTE level: it checks every frame's CRC, answers 0x64 and
0x74 the way the Waveshare wiki lays the reply out, obeys 0xA0 mode switches,
and can be told to go silent, corrupt a reply, answer as the wrong ID, echo
our own frames, raise on write, or sleep out the read timeout like a real
port with nobody on it. With a clock it also turns the motors, at the rpm
they were commanded, so scripts get positions that move.

One modelling assumption, the same one the driver makes and wheel_check
`sides` checks on the robot: a motor's raw position rises while it turns at
+raw rpm. A mirror-mounted (flipped) motor therefore counts DOWN in raw
position when the robot drives forward.
"""

import contextlib
import importlib.util
import io
import logging
import math
import subprocess
import sys
import time
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from retriever.bridge import ddsm115 as dd  # noqa: E402
from retriever.bridge import drivers  # noqa: E402
from retriever.bridge.drivers import (  # noqa: E402
    DDSM115_COUNTS_PER_REV,
    CompositeDriver,
    DDSM115Bus,
    DDSM115Driver,
    build_real_driver,
    find_wheel_port,
    mps_to_rpm,
    parse_id_list,
    rpm_to_mps,
)
from retriever.bridge.protocol import Act, Estop  # noqa: E402
from retriever.bridge.server import BridgeCore, HardwareDriver  # noqa: E402
from retriever.navigation.kinematics import TankGeometry  # noqa: E402
from retriever.navigation.odometry import TankOdometry, unwrap_ticks  # noqa: E402

HAVE_PYSERIAL = importlib.util.find_spec("serial") is not None
CPR = DDSM115_COUNTS_PER_REV
LOG = "retriever.bridge"
logging.getLogger(LOG).setLevel(logging.CRITICAL)


def i16(v):
    return list((int(v) & 0xFFFF).to_bytes(2, "big"))


# ---------------------------------------------------------------- the fake bus


class FakeMotor:
    def __init__(self, mid, pos=1000, mode=dd.VELOCITY, err=0, temp=31):
        self.id = mid
        self.pos = pos % 65536
        self.mode = mode
        self.err = err
        self.temp = temp
        self.rpm_cmd = 0
        self.brake = False
        self.frac = 0.0
        self.silent = False          # never answers
        self.raise_on_write = None   # exception class raised when a frame is addressed to it
        self.reply = None            # bytes to answer with instead (corrupt, wrong ID)
        self.lock_mode = None        # ignores 0xA0 and stays in this mode
        self.frames = []             # ("drive", value, accel, brake) or ("query",)

    def drive_frames(self):
        return [f for f in self.frames if f[0] == "drive"]


class FakeRS485:
    """A USB-RS485 port with DDSM115 motors on the bus, byte for byte."""

    def __init__(self, motors, *, clock=None, cpr_raw=CPR, honor_timeout=False, echo=False):
        self.motors = {m.id: m for m in motors}
        self.port = "/dev/fake-rs485"
        self.timeout = None
        self.write_timeout = None
        self.writes = []
        self.closed = False
        self.clock = clock
        self._t = clock() if clock else 0.0
        self.cpr_raw = cpr_raw
        self.honor_timeout = honor_timeout
        self.echo = echo
        self._rx = b""

    def _physics(self):
        if self.clock is None:
            return
        now = self.clock()
        dt, self._t = now - self._t, now
        for m in self.motors.values():
            if m.mode == dd.VELOCITY and not m.brake and m.rpm_cmd:
                m.frac += m.rpm_cmd / 60.0 * dt * self.cpr_raw
                whole = math.trunc(m.frac)
                m.frac -= whole
                m.pos = (m.pos + whole) % self.cpr_raw

    def reset_input_buffer(self):
        self._rx = b""

    def write(self, data):
        data = bytes(data)
        assert len(data) == 10, data
        assert not self.closed, "write after close"
        self.writes.append(data)
        self._physics()
        m = self.motors.get(data[0])
        if m is not None and m.raise_on_write is not None:
            raise m.raise_on_write(f"write to motor {m.id} failed")
        if self.echo:
            self._rx = data
            return 10
        if data[1] == 0xA0:                              # no CRC, no reply
            if m is not None:
                m.mode = m.lock_mode or data[9]
            return 10
        assert dd.crc8(data[:9]) == data[9], f"bad CRC from the driver: {data.hex(' ')}"
        if m is None or m.silent:
            return 10
        if data[1] == 0x64:
            value = int.from_bytes(data[2:4], "big", signed=True)
            m.frames.append(("drive", value, data[6], data[7] == 0xFF))
            if m.mode == dd.VELOCITY:
                m.brake = data[7] == 0xFF
                m.rpm_cmd = 0 if m.brake else value
            b6, b7 = m.pos >> 8, m.pos & 0xFF
        elif data[1] == 0x74:
            m.frames.append(("query",))
            b6, b7 = m.temp, m.pos * 256 // self.cpr_raw
        else:
            raise AssertionError(f"unknown command {data.hex(' ')}")
        self._rx = m.reply or dd.frame(m.id, m.mode, *i16(0), *i16(m.rpm_cmd), b6, b7, m.err)
        return 10

    def read(self, n):
        out, self._rx = self._rx[:n], self._rx[n:]
        if not out and self.honor_timeout:
            time.sleep(self.timeout)                     # what pyserial does with no data
        return out

    def close(self):
        self.closed = True

    def frames_after(self, mark):
        return self.writes[mark:]


def roll(ser, counts, ids=(1, 2, 3, 4), flipped=(3, 4)):
    """Turn the named motors `counts` in the ROBOT-FORWARD direction."""
    for mid in ids:
        m = ser.motors[mid]
        m.pos = (m.pos + (-counts if mid in flipped else counts)) % ser.cpr_raw


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.t += s


def make(positions=(100, 32700, 5, 16000), *, motors=None, **kw):
    clock = Clock()
    motors = motors or [FakeMotor(i + 1, pos=p) for i, p in enumerate(positions)]
    ser = FakeRS485(motors, cpr_raw=kw.pop("cpr_raw", CPR))
    bus = DDSM115Bus(ser, reply_timeout_s=0.01)
    drv = DDSM115Driver(bus=bus, clock=clock, sleep=clock.sleep, **kw)
    return drv, ser, clock


def read(drv, clock, dt=0.02):
    clock.t += dt
    return drv.read_ticks()


def delta(a, b, cpr=CPR):
    return (unwrap_ticks(b[0], a[0], cpr), unwrap_ticks(b[1], a[1], cpr))


# ---------------------------------------------------------------- frames


class TestVendoredFrames(unittest.TestCase):
    def test_the_teammates_self_check_passes(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            dd.demo()
        self.assertIn("self-check ok", out.getvalue())

    def test_frames_match_the_wiki_vectors(self):
        self.assertEqual(dd.query(1).hex(" "), "01 74 00 00 00 00 00 00 00 04")
        self.assertEqual(dd.drive(1, 0).hex(" "), "01 64 00 00 00 00 00 00 00 50")
        self.assertEqual(dd.drive(1, -50).hex(" "), "01 64 ff ce 00 00 00 00 00 da")
        self.assertEqual(dd.drive(1, 30000).hex(" "), "01 64 75 30 00 00 00 00 00 a7")
        self.assertEqual(dd.drive(1, 0, brake=True).hex(" "), "01 64 00 00 00 00 00 ff 00 d1")
        self.assertEqual(dd.set_mode(3, dd.VELOCITY).hex(" "), "03 a0 00 00 00 00 00 00 00 02")

    def test_crc_is_crc8_maxim(self):
        self.assertEqual(dd.crc8(b"123456789"), 0xA1)   # the catalogue check value

    def test_parse_signed_fields_and_rejects_bad_frames(self):
        r = dd.parse(dd.frame(4, 2, *i16(-4096), *i16(-57), 0x12, 0x34, 0x08))
        self.assertEqual((r["id"], r["mode"], r["rpm"], r["err"]), (4, 2, -57, 8))
        self.assertAlmostEqual(r["current_A"], -4096 * 8 / 32767)
        self.assertEqual((r["b6"] << 8) | r["b7"], 0x1234)
        good = dd.frame(4, 2, 0, 0, 0, 0, 0, 0, 0)
        self.assertIsNone(dd.parse(good[:9] + bytes([good[9] ^ 1])))   # bad CRC
        self.assertIsNone(dd.parse(good[:9]))                           # short
        self.assertIsNone(dd.parse(b""))

    def test_the_bridge_imports_neither_pyserial_nor_ddsm115(self):
        code = ("import sys; sys.path.insert(0, 'src'); "
                "import retriever.bridge.server, retriever.bridge.drivers; "
                "bad = [m for m in ('serial', 'retriever.bridge.ddsm115') "
                "if m in sys.modules]; "
                "assert not bad, bad; "
                "from retriever.bridge import ddsm115; ddsm115.demo()")
        # -S: no site-packages at all, like a bare python3 on the Pi
        res = subprocess.run([sys.executable, "-S", "-c", code], cwd=ROOT,
                             capture_output=True, text=True, timeout=60)
        self.assertEqual(res.returncode, 0, res.stderr)
        self.assertIn("self-check ok", res.stdout)

    @unittest.skipIf(HAVE_PYSERIAL, "pyserial is installed here")
    def test_without_pyserial_the_bus_says_it_is_missing(self):
        with self.assertRaises(ImportError) as ctx:
            dd.Bus(port="/dev/null")
        self.assertIn("pyserial is not installed", str(ctx.exception))
        with self.assertRaises(ImportError):
            dd.find_port()


# ---------------------------------------------------------------- helpers


class TestHelpers(unittest.TestCase):
    def test_rpm_conversion(self):
        r = 0.05
        self.assertAlmostEqual(mps_to_rpm(2 * math.pi * r, r), 60.0)   # one turn per second
        self.assertAlmostEqual(mps_to_rpm(-0.3, 0.048), -0.3 / (2 * math.pi * 0.048) * 60)
        self.assertAlmostEqual(rpm_to_mps(mps_to_rpm(0.37, r), r), 0.37)

    def test_parse_id_list(self):
        self.assertEqual(parse_id_list("1,2"), (1, 2))
        self.assertEqual(parse_id_list("1-4"), (1, 2, 3, 4))
        self.assertEqual(parse_id_list(" 3, 4 "), (3, 4))
        self.assertEqual(parse_id_list(""), ())
        with self.assertRaises(ValueError):
            parse_id_list("left")

    def test_find_wheel_port(self):
        class P:
            def __init__(self, device, vid):
                self.device, self.vid, self.description = device, vid, "USB Single Serial"

        rs485, other = P("/dev/ttyACM1", 0x1A86), P("/dev/ttyACM0", 0x1A86)
        ftdi = P("/dev/x", 0x0403)
        self.assertEqual(find_wheel_port({"DDSM115_PORT": "/dev/env"}, [rs485]), "/dev/env")
        self.assertEqual(find_wheel_port({}, [rs485, ftdi]), "/dev/ttyACM1")
        with self.assertRaises(ConnectionError) as ctx:     # two WCH chips: refuse to guess
            find_wheel_port({}, [other, rs485])
        self.assertIn("/dev/ttyACM0", str(ctx.exception))
        self.assertIn("/dev/ttyACM1", str(ctx.exception))
        self.assertEqual(find_wheel_port({}, [other, rs485], exclude=["/dev/ttyACM0"]),
                         "/dev/ttyACM1")
        with self.assertRaises(ConnectionError):
            find_wheel_port({}, [ftdi])


# ---------------------------------------------------------------- start-up


class TestStartup(unittest.TestCase):
    def test_velocity_mode_first_then_read_only_then_brake(self):
        drv, ser, _ = make()
        kinds = [(w[0], w[1]) for w in ser.writes]
        first_drive = next(i for i, (_, cmd) in enumerate(kinds) if cmd == 0x64)
        self.assertEqual({mid for mid, cmd in kinds[:first_drive] if cmd == 0xA0}, {1, 2, 3, 4})
        self.assertTrue(all(w[9] == dd.VELOCITY for w in ser.writes if w[1] == 0xA0))
        self.assertEqual({mid for mid, cmd in kinds[:first_drive] if cmd == 0x74}, {1, 2, 3, 4})
        for m in ser.motors.values():
            self.assertEqual(m.drive_frames()[0], ("drive", 0, 0, True))     # starts BRAKED
        self.assertEqual(ser.timeout, 0.01)
        self.assertEqual(ser.write_timeout, 0.01)
        self.assertEqual(drv.counts_per_rev, 32768)
        self.assertIsNone(drv.battery())

    def test_a_missing_motor_is_named_and_the_port_closed(self):
        motors = [FakeMotor(i) for i in (1, 2, 3, 4)]
        motors[2].silent = True
        with self.assertRaises(ConnectionError) as ctx:
            make(motors=motors)
        self.assertIn("3 (no reply)", str(ctx.exception))
        self.assertIn("wheel_check.py scan", str(ctx.exception))
        self.assertTrue(all(not m.drive_frames() for m in motors))   # nothing driven

    def test_a_motor_stuck_in_position_mode_gets_no_drive_frame(self):
        motors = [FakeMotor(i) for i in (1, 2, 3, 4)]
        motors[1].lock_mode = motors[1].mode = dd.POSITION
        clock = Clock()
        ser = FakeRS485(motors)
        with self.assertRaises(ConnectionError) as ctx:
            DDSM115Driver(bus=DDSM115Bus(ser), clock=clock, sleep=clock.sleep)
        self.assertIn("2 (POSITION)", str(ctx.exception))
        self.assertTrue(all(w[1] != 0x64 for w in ser.writes))   # a 0 there = "go to 0 deg"
        self.assertTrue(ser.closed)

    def test_an_echoing_adapter_is_not_mistaken_for_motors(self):
        clock = Clock()
        ser = FakeRS485([FakeMotor(i) for i in (1, 2, 3, 4)], echo=True)
        with self.assertRaises(ConnectionError) as ctx:
            DDSM115Driver(bus=DDSM115Bus(ser), clock=clock, sleep=clock.sleep)
        self.assertIn("echoing", str(ctx.exception))

    def test_a_position_past_counts_per_rev_refuses_to_start(self):
        with self.assertRaises(ConnectionError) as ctx:
            make(positions=(100, 200, 5000, 300), counts_per_rev=4096)
        self.assertIn("wheel_check.py rev 3", str(ctx.exception))

    def test_bad_configurations(self):
        for kw in ({"left_ids": (1, 2), "right_ids": (2, 3)},    # shared ID
                   {"left_ids": (), "right_ids": (3, 4)},        # empty side
                   {"flipped_ids": (3, 9)},                      # flip on no side
                   {"accel": 300}):
            with self.subTest(kw=kw), self.assertRaises(ValueError):
                make(**kw)


# ---------------------------------------------------------------- driving


class TestDriving(unittest.TestCase):
    def setUp(self):
        self.drv, self.ser, self.clock = make(geo=TankGeometry(wheel_radius_m=0.05))
        self.mark = len(self.ser.writes)

    def last(self, mid):
        return self.ser.motors[mid].drive_frames()[-1]

    def test_forward_is_plus_rpm_and_minus_on_the_mirrored_side(self):
        v = rpm_to_mps(30, 0.05)
        self.drv.set_wheels(v, v)
        self.assertEqual(self.last(1), ("drive", 30, 0, False))
        self.assertEqual(self.last(2), ("drive", 30, 0, False))
        self.assertEqual(self.last(3), ("drive", -30, 0, False))
        self.assertEqual(self.last(4), ("drive", -30, 0, False))

    def test_turn_left_in_place(self):
        v = rpm_to_mps(20, 0.05)
        self.drv.set_wheels(-v, v)          # +wz: left side back, right side forward
        self.assertEqual([self.last(i)[1] for i in (1, 2, 3, 4)], [-20, -20, -20, -20])

    def test_rpm_is_rounded_and_clamped_to_330(self):
        self.drv.set_wheels(rpm_to_mps(12.6, 0.05), 10.0)
        self.assertEqual(self.last(1)[1], 13)
        self.assertEqual(self.last(3)[1], -330)
        self.drv.set_wheels(-10.0, 0.0)
        self.assertEqual(self.last(1)[1], -330)

    def test_one_frame_per_motor_sides_interleaved(self):
        self.drv.set_wheels(0.1, 0.1)
        self.assertEqual([w[0] for w in self.ser.frames_after(self.mark)], [1, 3, 2, 4])

    def test_zero_is_a_speed_not_a_brake(self):
        self.drv.set_wheels(0.1, 0.1)
        self.drv.set_wheels(0.0, 0.0)
        self.assertEqual({self.last(i) for i in (1, 2, 3, 4)}, {("drive", 0, 0, False)})

    def test_non_finite_speed_sends_nothing(self):
        with self.assertRaises(ValueError):
            self.drv.set_wheels(float("nan"), 0.1)
        self.assertEqual(self.ser.frames_after(self.mark), [])

    def test_other_ids_and_flips(self):
        drv, ser, _ = make(left_ids=(3, 4), right_ids=(1, 2), flipped_ids=(1, 2),
                           geo=TankGeometry(wheel_radius_m=0.05))
        drv.set_wheels(rpm_to_mps(10, 0.05), rpm_to_mps(40, 0.05))
        self.assertEqual([ser.motors[i].drive_frames()[-1][1] for i in (1, 2, 3, 4)],
                         [-40, -40, 10, 10])

    def test_accel_is_sent(self):
        drv, ser, _ = make(accel=30)
        drv.set_wheels(0.1, 0.1)
        self.assertEqual(ser.motors[1].drive_frames()[-1][2], 30)


# ---------------------------------------------------------------- odometry


class TestOdometry(unittest.TestCase):
    def test_forward_counts_up_on_both_sides(self):
        drv, ser, clock = make()
        t0 = read(drv, clock)
        roll(ser, 1000)                      # flipped motors' raw positions go DOWN
        self.assertEqual(delta(t0, read(drv, clock)), (1000, 1000))
        roll(ser, -300)
        self.assertEqual(delta(t0, read(drv, clock)), (700, 700))

    def test_across_the_u16_wrap(self):
        # motor 2 starts at 32700 and wraps up through 0; motor 3 (flipped) at 5
        # wraps down through 0 -- both while the robot drives forward.
        drv, ser, clock = make(positions=(100, 32700, 5, 16000))
        t0 = t = read(drv, clock)
        total = (0, 0)
        for _ in range(50):
            roll(ser, 700)
            nxt = read(drv, clock)
            d = delta(t, nxt)
            self.assertEqual(d, (700, 700))
            total = (total[0] + d[0], total[1] + d[1])
            t = nxt
        self.assertEqual(total, (35000, 35000))       # more than one full turn...
        self.assertEqual(delta(t0, t), (35000 - CPR, 35000 - CPR))   # ...wrapping at cpr
        self.assertTrue(all(0 <= x < CPR for x in t))

    def test_each_side_is_the_mean_of_its_two_motors(self):
        drv, ser, clock = make()
        t0 = read(drv, clock)
        roll(ser, 1000, ids=(1,))
        roll(ser, 1010, ids=(2,))
        roll(ser, 400, ids=(3, 4))
        self.assertEqual(delta(t0, read(drv, clock)), (1005, 400))

    def test_the_laptops_odometry_sees_the_right_distance_and_turn(self):
        geo = TankGeometry(wheel_radius_m=0.05, track_width_m=0.30)
        drv, ser, clock = make(geo=geo)
        odo = TankOdometry(geo=geo, counts_per_rev=drv.counts_per_rev)
        odo.update(*read(drv, clock), 0.02)
        for _ in range(32):                    # two wheel turns, straight
            roll(ser, CPR // 16)
            odo.update(*read(drv, clock), 0.02)
        self.assertAlmostEqual(odo.pose.x, 2 * 2 * math.pi * 0.05, places=6)
        self.assertAlmostEqual(odo.pose.theta, 0.0, places=9)
        for _ in range(10):                    # left back, right forward: CCW
            roll(ser, -CPR // 40, ids=(1, 2))
            roll(ser, CPR // 40, ids=(3, 4))
            odo.update(*read(drv, clock), 0.02)
        self.assertGreater(odo.pose.theta, 0.0)

    def test_back_to_back_reads_reuse_one_round(self):
        drv, ser, clock = make()
        read(drv, clock)
        mark = len(ser.writes)
        drv.read_ticks()                       # same instant: no second round
        self.assertEqual(len(ser.writes), mark)
        drv.set_wheels(0.1, 0.1)               # a command always goes out
        drv.read_ticks()                       # ...and its feedback is fresh enough
        self.assertEqual(len(ser.writes), mark + 4)

    def test_reads_resend_the_current_setpoint(self):
        drv, ser, clock = make(geo=TankGeometry(wheel_radius_m=0.05))
        drv.set_wheels(rpm_to_mps(25, 0.05), rpm_to_mps(25, 0.05))
        read(drv, clock)
        self.assertEqual(ser.motors[1].drive_frames()[-1], ("drive", 25, 0, False))
        drv.stop()
        read(drv, clock)
        self.assertEqual(ser.motors[1].drive_frames()[-1], ("drive", 0, 0, True))


# ---------------------------------------------------------------- missing replies


class TestMissingReplies(unittest.TestCase):
    def test_one_missed_reply_loses_nothing_and_is_not_an_alarm(self):
        drv, ser, clock = make()
        t0 = read(drv, clock)
        ser.motors[1].silent = True
        roll(ser, 500)
        with self.assertNoLogs(LOG, level="WARNING"):
            read(drv, clock)
        self.assertEqual(drv.diagnostics()["motors"][1]["misses"], 1)
        ser.motors[1].silent = False
        roll(ser, 500)
        self.assertEqual(delta(t0, read(drv, clock)), (1000, 1000))

    def test_a_dead_motor_is_flagged_and_its_side_keeps_counting(self):
        drv, ser, clock = make()
        t0 = read(drv, clock)
        ser.motors[1].silent = True
        with self.assertLogs(LOG, level="WARNING") as logs:
            for _ in range(10):
                roll(ser, 300)
                t = read(drv, clock)
        self.assertTrue(any("motor 1" in line and "missed" in line for line in logs.output))
        self.assertTrue(any("odometry from motor(s) [2] only" in line for line in logs.output))
        # no distance lost, not even during the misses before it was dropped
        self.assertEqual(delta(t0, t), (3000, 3000))
        diag = drv.diagnostics()
        self.assertTrue(diag["sides"]["left"]["degraded"])
        self.assertFalse(diag["sides"]["left"]["blind"])
        self.assertFalse(diag["motors"][1]["ok"])
        self.assertEqual(diag["motors"][1]["why"], "no reply")
        drv.set_wheels(0.1, 0.1)               # still allowed to drive on 3 motors

    def test_a_motor_that_comes_back_rejoins_without_a_jump(self):
        drv, ser, clock = make()
        t0 = read(drv, clock)
        ser.motors[2].silent = True
        for _ in range(5):
            roll(ser, 200)
            read(drv, clock)
        ser.motors[2].silent = False
        roll(ser, 12345, ids=(2,))             # moved a lot while we could not see it
        roll(ser, 200)
        with self.assertLogs(LOG, level="INFO") as logs:
            t = read(drv, clock)
        self.assertTrue(any("answering again" in line for line in logs.output))
        self.assertEqual(delta(t0, t), (1200, 1200))
        roll(ser, 100)
        self.assertEqual(delta(t0, read(drv, clock)), (1300, 1300))
        self.assertFalse(drv.diagnostics()["sides"]["left"]["degraded"])

    def test_a_blind_side_raises_instead_of_reading_as_stopped(self):
        drv, ser, clock = make()
        read(drv, clock)
        for mid in (3, 4):
            ser.motors[mid].silent = True
        for _ in range(drv.miss_limit - 1):
            read(drv, clock)                   # still within the tolerated misses
        with self.assertRaises(ConnectionError) as ctx:
            read(drv, clock)
        self.assertIn("right side: no motor answering", str(ctx.exception))
        self.assertTrue(drv.diagnostics()["sides"]["right"]["blind"])
        # refuses to MOVE, sending nothing...
        mark = len(ser.writes)
        with self.assertRaises(ConnectionError):
            drv.set_wheels(0.2, 0.2)
        self.assertEqual(ser.frames_after(mark), [])
        # ...but zero always goes out, and stop() brakes the motors that can hear
        drv.set_wheels(0.0, 0.0)
        with self.assertRaises(ConnectionError) as ctx:
            drv.stop()
        self.assertIn("3: no reply", str(ctx.exception))
        for mid in (1, 2):
            self.assertEqual(ser.motors[mid].drive_frames()[-1], ("drive", 0, 0, True))
        for mid in (3, 4):
            ser.motors[mid].silent = False
        read(drv, clock)                       # it recovers by itself
        drv.set_wheels(0.2, 0.2)

    def test_a_side_lost_mid_command_raises_so_the_server_stops(self):
        drv, ser, clock = make(miss_limit=1)
        ser.motors[1].silent = ser.motors[2].silent = True
        with self.assertRaises(ConnectionError):
            drv.set_wheels(0.2, 0.2)

    def test_wrong_id_and_corrupt_replies_count_as_misses(self):
        drv, ser, clock = make()
        ser.motors[1].reply = dd.frame(2, 2, 0, 0, 0, 0, 0, 0, 0)
        ser.motors[3].reply = b"\x03\x02\x00\x00"
        read(drv, clock)
        diag = drv.diagnostics()["motors"]
        self.assertEqual(diag[1]["why"], "the reply came from ID 2")
        self.assertTrue(diag[3]["why"].startswith("bad reply (4 bytes"))
        self.assertTrue(diag[2]["ok"] and diag[4]["ok"])


# ---------------------------------------------------------------- faults


class TestFaults(unittest.TestCase):
    def test_error_bits_are_decoded_and_logged_once(self):
        drv, ser, clock = make()
        ser.motors[4].err = 0x08
        with self.assertLogs(LOG, level="WARNING") as logs:
            for _ in range(5):
                read(drv, clock)
        self.assertEqual(sum("stall" in line for line in logs.output), 1)
        self.assertEqual(drv.diagnostics()["motors"][4]["errors"], ["stall"])
        ser.motors[4].err = 0x06
        read(drv, clock)
        self.assertEqual(drv.diagnostics()["motors"][4]["errors"],
                         ["overcurrent", "phase overcurrent"])

    def test_a_motor_switched_out_of_velocity_mode_is_left_alone(self):
        drv, ser, clock = make()
        ser.motors[3].mode = dd.POSITION       # someone else on the bus
        with self.assertLogs(LOG, level="ERROR"):
            drv.set_wheels(0.0, 0.0)
        before = len(ser.motors[3].drive_frames())
        with self.assertRaises(ConnectionError) as ctx:
            drv.set_wheels(0.1, 0.1)
        self.assertIn("not in VELOCITY", str(ctx.exception))
        with self.assertRaises(ConnectionError):
            read(drv, clock)
        with self.assertRaises(ConnectionError) as ctx:
            drv.stop()
        self.assertIn("3: not in VELOCITY mode", str(ctx.exception))
        self.assertEqual(len(ser.motors[3].drive_frames()), before)   # only queries since
        self.assertEqual(ser.motors[3].frames[-1], ("query",))
        self.assertEqual(ser.motors[1].drive_frames()[-1], ("drive", 0, 0, True))

    def test_a_position_past_counts_per_rev_at_runtime_stops_odometry(self):
        drv, ser, clock = make(positions=(100, 200, 300, 400), counts_per_rev=4096)
        ser.motors[2].pos = 4200
        with self.assertRaises(ConnectionError) as ctx:
            read(drv, clock)
        self.assertIn("counts_per_rev is 4096", str(ctx.exception))
        with self.assertRaises(ConnectionError):
            drv.set_wheels(0.1, 0.1)


# ---------------------------------------------------------------- stopping


class TestStop(unittest.TestCase):
    def test_stop_brakes_every_motor(self):
        drv, ser, _ = make()
        drv.set_wheels(0.3, 0.3)
        drv.stop()
        for m in ser.motors.values():
            self.assertEqual(m.drive_frames()[-1], ("drive", 0, 0, True))

    def test_stop_brakes_the_rest_even_if_one_motor_raises(self):
        drv, ser, _ = make()
        drv.set_wheels(0.3, 0.3)
        ser.motors[1].raise_on_write = OSError
        with self.assertRaises(ConnectionError) as ctx:
            drv.stop()
        self.assertIn("1: OSError", str(ctx.exception))
        for mid in (2, 3, 4):
            self.assertEqual(ser.motors[mid].drive_frames()[-1], ("drive", 0, 0, True))

    def test_ctrl_c_mid_stop_still_brakes_the_rest_then_propagates(self):
        drv, ser, _ = make()
        drv.set_wheels(0.3, 0.3)
        ser.motors[3].raise_on_write = KeyboardInterrupt
        with self.assertRaises(KeyboardInterrupt):
            drv.stop()
        for mid in (1, 2, 4):
            self.assertEqual(ser.motors[mid].drive_frames()[-1], ("drive", 0, 0, True))

    def test_close_brakes_then_releases_the_port_once(self):
        drv, ser, _ = make()
        drv.set_wheels(0.3, 0.3)
        ser.motors[2].silent = True            # an unconfirmed brake must not keep the port
        drv.close()
        drv.close()
        self.assertTrue(ser.closed)
        self.assertEqual(ser.motors[1].drive_frames()[-1], ("drive", 0, 0, True))


# ---------------------------------------------------------------- timing


class TestBoundedBlocking(unittest.TestCase):
    """The bridge calls the driver on its event loop: a dead bus must cost a
    bounded, small time per call, never a hang that starves the watchdog."""

    def test_every_call_is_bounded_when_nothing_replies(self):
        clock = Clock()
        ser = FakeRS485([FakeMotor(i) for i in (1, 2, 3, 4)], honor_timeout=True)
        drv = DDSM115Driver(bus=DDSM115Bus(ser, reply_timeout_s=0.01, write_timeout_s=0.01),
                            clock=time.monotonic, sleep=clock.sleep)
        self.assertAlmostEqual(drv.worst_case_call_s, 4 * 0.02)
        for m in ser.motors.values():
            m.silent = True
        calls = {"set_wheels(0,0)": lambda: drv.set_wheels(0.0, 0.0),
                 "read_ticks": drv.read_ticks, "stop": drv.stop}
        for name, call in calls.items():
            with self.subTest(call=name):
                time.sleep(drv.reuse_s)        # so read_ticks does a round, not a reuse
                t0 = time.perf_counter()
                with contextlib.suppress(ConnectionError):
                    call()
                elapsed = time.perf_counter() - t0
                self.assertGreaterEqual(elapsed, 4 * 0.01 * 0.9)   # it did wait for each
                self.assertLess(elapsed, drv.worst_case_call_s + 0.05)

    def test_a_dead_bus_at_start_fails_fast(self):
        clock = Clock()
        ser = FakeRS485([], honor_timeout=True)
        t0 = time.perf_counter()
        with self.assertRaises(ConnectionError):
            DDSM115Driver(bus=DDSM115Bus(ser, reply_timeout_s=0.01), sleep=clock.sleep)
        self.assertLess(time.perf_counter() - t0, 3 * 4 * 0.01 + 0.1)   # 3 tries x 4 motors


# ---------------------------------------------------------------- behind the bridge


class TestBehindTheBridge(unittest.TestCase):
    def test_act_watchdog_and_estop_through_bridge_core(self):
        geo = TankGeometry(wheel_radius_m=0.05)
        drv, ser, clock = make(geo=geo)
        comp = CompositeDriver(drv)
        self.assertIsInstance(comp, HardwareDriver)
        core = BridgeCore(comp, 0.0, geo=geo)
        v = rpm_to_mps(30, 0.05)
        self.assertIsNone(core.handle(Act(seq=1, base_vx=v), 0.01))
        self.assertEqual([ser.motors[i].drive_frames()[-1][1] for i in (1, 2, 3, 4)],
                         [30, 30, -30, -30])
        clock.t += 0.02
        state = core.snapshot(0.02)
        self.assertIsInstance(state.left_ticks, int)
        core.tick(0.5)                                   # laptop silent > 300 ms
        self.assertTrue(core.watchdog_tripped)
        self.assertEqual({m.drive_frames()[-1] for m in ser.motors.values()},
                         {("drive", 0, 0, True)})
        core.handle(Act(seq=2, base_vx=v), 0.6)
        core.handle(Estop(reason="test"), 0.61)
        self.assertEqual({m.drive_frames()[-1] for m in ser.motors.values()},
                         {("drive", 0, 0, True)})

    def test_a_blind_side_makes_the_bridge_stop_and_say_why(self):
        drv, ser, clock = make(miss_limit=1)
        core = BridgeCore(CompositeDriver(drv), 0.0)
        ser.motors[3].silent = ser.motors[4].silent = True
        clock.t += 0.02
        with self.assertRaises(ConnectionError):         # the server logs it, sends no state
            core.snapshot(0.02)
        refused = core.handle(Act(seq=1, base_vx=0.2), 0.03)
        self.assertIn("right side: no motor answering", refused)
        for mid in (1, 2):
            self.assertEqual(ser.motors[mid].drive_frames()[-1], ("drive", 0, 0, True))


class TestBuildRealDriver(unittest.TestCase):
    def test_real_wheels_get_the_ids_and_the_port(self):
        with mock.patch.object(drivers, "DDSM115Driver") as cls:
            drv = build_real_driver(None, left_ids=(5, 6), right_ids=(7, 8),
                                    flipped_ids=(7, 8), wheel_counts_per_rev=65536)
        _, kwargs = cls.call_args
        self.assertEqual(cls.call_args.args[0], None)          # auto-detect
        self.assertEqual((kwargs["left_ids"], kwargs["right_ids"], kwargs["flipped_ids"]),
                         ((5, 6), (7, 8), (7, 8)))
        self.assertEqual(kwargs["counts_per_rev"], 65536)
        drv.close()                                            # frees the wheel port
        cls.return_value.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
