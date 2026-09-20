"""RPLIDAR A2M12 driver (bridge/lidar.py): packets, scan assembly, mounting,
the serial link against a byte-level emulator, the reader thread, the fake.

No hardware and no pyserial: the link takes a serial factory, and the
emulator below answers the way Slamtec's protocol document says the lidar
does. Hand-written byte strings (not produced by our own encoder) pin the
packet layout, so an encoder/decoder pair that agree on a wrong layout fails.
"""

import logging
import math
import random
import struct
import subprocess
import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.bridge.fake_driver import FakeTankDriver
from retriever.bridge.lidar import (
    N_SAMPLE_BINS,
    AdapterMotor,
    ExternalMotor,
    FakeLidar,
    FakeTankPose,
    FakeWorld,
    GpioMotor,
    LidarError,
    LidarMount,
    Measurement,
    MeasurementParser,
    RPLidar,
    RPLidarLink,
    ScanAssembler,
    bring_up,
    default_motor,
    encode_measurement,
    parse_descriptor,
    parse_health,
    parse_info,
    parse_measurement,
    request,
    sample_bin,
    scan_to_base,
    scan_to_base_polar,
    sensor_angle,
    staleness,
)

SRC = Path(__file__).resolve().parents[1] / "src"
ROOT = Path(__file__).resolve().parents[1]
DEG = math.radians

# The reader thread logs every retry at WARNING; the unplug test causes several.
logging.getLogger("retriever.bridge.lidar").setLevel(logging.CRITICAL)


def wait_until(pred, timeout=3.0, every=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(every)
    return pred()


def revolution(n=120, start_deg=0.0, rng=lambda a: 1000.0, quality=30, first=True):
    """n packets of one clockwise revolution; the first carries S."""
    out = []
    for i in range(n):
        a = (start_deg + 360.0 * i / n) % 360.0
        out.append(encode_measurement(first and i == 0, quality, a, rng(a)))
    return out


# ---------------------------------------------------------------- packets


class TestRequests(unittest.TestCase):
    def test_commands_without_payload_are_two_bytes(self):
        for cmd, wire in ((0x25, b"\xa5\x25"), (0x40, b"\xa5\x40"), (0x20, b"\xa5\x20"),
                          (0x50, b"\xa5\x50"), (0x52, b"\xa5\x52"), (0x59, b"\xa5\x59")):
            self.assertEqual(request(cmd), wire)

    def test_payload_commands_carry_length_and_xor_checksum(self):
        # SET_MOTOR_PWM 660 = 0x0294 LE; checksum A5^F0^02^94^02 = C1.
        self.assertEqual(request(0xF0, struct.pack("<H", 660)), bytes.fromhex("a5f0029402c1"))
        # GET_ACC_BOARD_FLAG with its 4-byte reserved payload: A5^FF^04 = 5E.
        self.assertEqual(request(0xFF, b"\0\0\0\0"), bytes.fromhex("a5ff04000000005e"))

    def test_a_payload_on_a_plain_command_is_a_bug(self):
        with self.assertRaises(ValueError):
            request(0x20, b"\x01")


class TestResponses(unittest.TestCase):
    def test_descriptors_from_the_protocol_document(self):
        def fields(d):
            return d.length, d.mode, d.dtype

        def parsed(hexstr):
            return fields(parse_descriptor(bytes.fromhex(hexstr)))

        self.assertEqual(parsed("a55a0500004081"), (5, 1, 0x81))
        self.assertEqual(parsed("a55a1400000004"), (20, 0, 0x04))
        self.assertEqual(parsed("a55a0300000006"), (3, 0, 0x06))
        with self.assertRaises(LidarError):
            parse_descriptor(bytes.fromhex("a55b0500004081"))

    def test_device_info_matches_what_our_unit_reported(self):
        raw = bytes([44, 32, 1, 6]) + bytes(range(16))
        info = parse_info(raw)
        self.assertEqual((info.model, info.firmware, info.hardware), (44, (1, 32), 6))
        self.assertEqual(info.serial, bytes(range(16)).hex().upper())
        self.assertIn("firmware 1.32", str(info))

    def test_health_error_code_is_little_endian(self):
        h = parse_health(b"\x00\x00\x00")
        self.assertEqual((h.status, h.error_code, h.name), (0, 0, "good"))
        h = parse_health(b"\x02\x34\x12")
        self.assertEqual((h.status, h.error_code, h.name), (2, 0x1234, "error"))


class TestMeasurementPackets(unittest.TestCase):
    def test_hand_decoded_packet(self):
        # quality 15, S=1: b0 = 15<<2 | 0b01 = 0x3D. angle 90 deg = q6 5760:
        # b1 = (5760 & 0x7f)<<1 | C = 0x01, b2 = 5760>>7 = 0x2D. 1000 mm = q2 4000 = 0x0FA0.
        m = parse_measurement(bytes.fromhex("3d012da00f"))
        self.assertEqual(m, Measurement(start=True, quality=15, angle_deg=90.0,
                                        range_mm=1000.0))
        m = parse_measurement(bytes.fromhex("3e012da00f"))           # S=0, !S=1
        self.assertFalse(m.start)
        self.assertEqual(encode_measurement(True, 15, 90.0, 1000.0),
                         bytes.fromhex("3d012da00f"))

    def test_check_bits_reject_framing_slips(self):
        for bad in ("3f012da00f",     # S == !S == 1
                    "3c012da00f",     # S == !S == 0
                    "3d002da00f",     # C bit clear
                    "3d01ffa00f"):    # angle_q6 >= 360 * 64
            with self.subTest(bad=bad):
                self.assertIsNone(parse_measurement(bytes.fromhex(bad)))

    def test_no_return_is_a_packet_not_an_error(self):
        m = parse_measurement(encode_measurement(False, 0, 12.5, 0.0))
        self.assertIsNotNone(m)
        self.assertFalse(m.valid)

    def test_round_trip(self):
        rng = random.Random(1)
        for _ in range(500):
            q, a, d = rng.randrange(64), rng.uniform(0, 359.98), rng.uniform(0, 16000)
            m = parse_measurement(encode_measurement(bool(rng.getrandbits(1)), q, a, d))
            self.assertEqual(m.quality, q)
            self.assertAlmostEqual(m.angle_deg, a, delta=1 / 64)
            self.assertAlmostEqual(m.range_mm, d, delta=0.25)


class TestParser(unittest.TestCase):
    def test_arbitrary_chunking_and_a_garbage_prefix(self):
        packets = revolution(100)
        stream = b"\x00\x11\xff\x5a\x05" + b"".join(packets)
        rng = random.Random(2)
        p, got, i = MeasurementParser(), [], 0
        while i < len(stream):
            n = rng.randint(1, 17)
            got += p.feed(stream[i:i + n])
            i += n
        self.assertEqual([encode_measurement(m.start, m.quality, m.angle_deg, m.range_mm)
                          for m in got], packets)
        self.assertEqual(p.bad, 5)

    def test_resyncs_after_a_dropped_byte(self):
        packets = revolution(50)
        stream = bytearray(b"".join(packets))
        del stream[5 * 20 + 2]                    # one byte lost in packet 20
        got = MeasurementParser().feed(bytes(stream))
        self.assertGreaterEqual(len(got), 47)     # a slip costs a packet or two, not the stream
        self.assertEqual(encode_measurement(got[-1].start, got[-1].quality,
                                            got[-1].angle_deg, got[-1].range_mm), packets[-1])


# ---------------------------------------------------------------- scans


class TestScanAssembly(unittest.TestCase):
    def feed(self, packets, asm=None, t0=0.0, dt=0.001):
        asm = asm or ScanAssembler()
        scans = []
        for i, pkt in enumerate(packets):
            s = asm.add(parse_measurement(pkt), t0 + i * dt)
            if s is not None:
                scans.append(s)
        return scans, asm

    def test_one_scan_per_revolution_and_the_partial_first_is_dropped(self):
        partial = revolution(120, start_deg=200.0, first=False)[:40]   # joined mid-turn
        packets = partial + revolution(120) + revolution(120) + revolution(120)[:1]
        scans, asm = self.feed(packets, dt=0.1 / 120)                   # 10 rev/s
        self.assertEqual(len(scans), 2)
        self.assertEqual(asm.dropped, 1)
        self.assertEqual(len(scans[0].points), 120)
        self.assertAlmostEqual(scans[1].rev_hz, 10.0, delta=0.2)
        self.assertEqual(sum(scans[0].samples), 120)

    def test_the_wrap_splits_revolutions_when_the_start_flag_is_lost(self):
        second = revolution(120)
        # S flag lost on the wire
        second[0] = encode_measurement(False, 30, 0.0, 1000.0)
        scans, _ = self.feed(revolution(120) + second + revolution(120)[:1])
        self.assertEqual(len(scans), 2)
        self.assertTrue(all(len(s.points) == 120 for s in scans))

    def test_no_returns_are_counted_but_not_kept(self):
        rev = revolution(100, rng=lambda a: 0.0 if 80 <= a < 100 else 1500.0)
        scans, _ = self.feed(rev + revolution(100)[:1])
        s = scans[0]
        self.assertEqual(sum(s.samples), 100)
        self.assertLess(len(s.points), 100)
        self.assertTrue(all(r == 1.5 for _, r, _ in s.points))

    def test_clockwise_degrees_become_counter_clockwise_radians(self):
        """Raw 90 (clockwise) is the lidar's RIGHT; raw 270 its LEFT."""
        self.assertAlmostEqual(sensor_angle(90.0), DEG(270.0))
        self.assertAlmostEqual(sensor_angle(270.0), DEG(90.0))
        self.assertEqual(sensor_angle(0.0), 0.0)


def one_point_scan(raw_deg_cw, range_m, t=0.0):
    """A LidarScan built through the real parser: one return at a raw angle."""
    rev = [encode_measurement(True, 30, 0.0, 0.0)]
    rev += [encode_measurement(False, 30, a, range_m * 1000 if abs(a - raw_deg_cw) < 0.5 else 0)
            for a in (i * 1.0 for i in range(1, 360))]
    asm = ScanAssembler()
    for pkt in rev + rev[:1]:
        s = asm.add(parse_measurement(pkt), t)
    return s


class TestMount(unittest.TestCase):
    def test_raw_270_is_the_robots_left_upright_and_right_inverted(self):
        scan = one_point_scan(270.0, 1.0)
        (x, y, _), = scan_to_base(scan, LidarMount())
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 1.0, places=6)                       # LEFT
        (x, y, _), = scan_to_base(scan, LidarMount(inverted=True))
        self.assertAlmostEqual(y, -1.0, places=6)                      # mirrored: RIGHT

    def test_yaw_and_offset(self):
        """bearing_base = yaw - raw (upright), yaw + raw (inverted), plus the offset."""
        scan = one_point_scan(30.0, 2.0)
        m = LidarMount(0.1, -0.05, DEG(90.0))
        (x, y, _), = scan_to_base(scan, m)
        b = DEG(90.0 - 30.0)
        self.assertAlmostEqual(x, 0.1 + 2 * math.cos(b), places=6)
        self.assertAlmostEqual(y, -0.05 + 2 * math.sin(b), places=6)
        (x, y, _), = scan_to_base(scan, LidarMount(yaw_rad=DEG(90.0), inverted=True))
        b = DEG(90.0 + 30.0)
        self.assertAlmostEqual(x, 2 * math.cos(b), places=6)
        self.assertAlmostEqual(y, 2 * math.sin(b), places=6)

    def test_polar_matches_target_bearing_convention(self):
        (bearing, r), = scan_to_base_polar(one_point_scan(315.0, 1.5))
        self.assertAlmostEqual(math.degrees(bearing), 45.0, places=4)  # front-left, CCW +
        self.assertAlmostEqual(r, 1.5, places=6)

    def test_sensor_angle_of_inverts_bearing(self):
        for m in (LidarMount(), LidarMount(yaw_rad=1.0),
                  LidarMount(yaw_rad=-2.0, inverted=True)):
            for a in (0.0, 0.5, 3.0, 6.0):
                self.assertAlmostEqual(m.sensor_angle_of(m.bearing(a)), a % (2 * math.pi))


# ---------------------------------------------------------------- the link


class EmulatedA2M12:
    """A byte-level RPLIDAR on the far side of a serial port."""

    def __init__(self, health=0, acc_board=False, per_rev=120, start_deg=137.0,
                 rng=lambda a: 1200.0):
        self.health, self.acc_board, self.per_rev, self.rng = health, acc_board, per_rev, rng
        self.rx, self.tx = bytearray(), bytearray()
        self.commands: list[tuple[int, bytes]] = []
        self.checksum_errors = 0
        self.scanning = False
        self.pwm = None
        self.dtr = True
        self.closed = False
        self.unplugged = False
        self.read_delay = 0.0
        self.timeout = 0.01
        self._i = int(start_deg / 360.0 * per_rev)
        self._lock = threading.Lock()

    # pyserial surface
    def write(self, data):
        self._check()
        self.rx += data
        self._process()
        return len(data)

    @property
    def in_waiting(self):
        self._check()
        with self._lock:
            self._pump()
            return len(self.tx)

    def read(self, n=1):
        self._check()
        if self.read_delay:
            time.sleep(self.read_delay)
        with self._lock:
            self._pump()
            out = bytes(self.tx[:n])
            del self.tx[:n]
        if not out:
            time.sleep(0.001)
        return out

    def reset_input_buffer(self):
        with self._lock:
            self.tx.clear()

    def close(self):
        self.closed = True

    # the device
    def _check(self):
        if self.unplugged:
            raise OSError(5, "Input/output error")

    def _pump(self):
        if not self.scanning:
            return
        for _ in range(24):
            k = self._i % self.per_rev
            a = 360.0 * k / self.per_rev
            self.tx += encode_measurement(k == 0, 30, a, self.rng(a))
            self._i += 1

    def _answer(self, dtype, payload):
        head = bytes([0xA5, 0x5A]) + struct.pack("<I", len(payload)) + bytes([dtype])
        self.tx += head + payload

    def _process(self):
        while True:
            while self.rx and self.rx[0] != 0xA5:
                del self.rx[0]
            if len(self.rx) < 2:
                return
            cmd = self.rx[1]
            if cmd & 0x80:
                if len(self.rx) < 3 or len(self.rx) < 4 + self.rx[2]:
                    return
                n = self.rx[2]
                body, check = bytes(self.rx[:3 + n]), self.rx[3 + n]
                x = 0
                for b in body:
                    x ^= b
                if x != check:
                    self.checksum_errors += 1
                payload = body[3:]
                del self.rx[:4 + n]
            else:
                payload = b""
                del self.rx[:2]
            self.commands.append((cmd, payload))
            with self._lock:
                self._do(cmd, payload)

    def _do(self, cmd, payload):
        if cmd == 0x25:
            self.scanning = False
        elif cmd == 0x40:
            self.scanning, self.health = False, 0
            self.tx += b"RP LIDAR System.\r\nFirmware Ver 1.32\r\n"   # boot chatter
        elif cmd == 0x20:
            if self.health != 2:
                self._answer(0x81, b"")
                self.tx[-5:-1] = struct.pack("<I", 5 | (1 << 30))   # multiple-response, 5 bytes
                self.scanning = True
        elif cmd == 0x50:
            self._answer(0x04, bytes([44, 32, 1, 6]) + bytes(16))
        elif cmd == 0x52:
            self._answer(0x06, bytes([self.health, 0x01 if self.health else 0, 0]))
        elif cmd == 0x59:
            self._answer(0x15, struct.pack("<HH", 252, 126))
        elif cmd == 0xFF and self.acc_board:
            self._answer(0xFF, struct.pack("<I", 1))
        elif cmd == 0xF0:
            self.pwm = struct.unpack("<H", payload)[0]


class FakePin:
    """gpiozero DigitalOutputDevice's three members."""

    def __init__(self):
        self.value, self.closed, self.history = 0, False, []

    def on(self):
        self.value = 1
        self.history.append(1)

    def off(self):
        self.value = 0
        self.history.append(0)

    def close(self):
        self.closed = True


def factory_for(emu):
    def open_(port, baud, timeout, dtr=None):
        if dtr is not None:
            emu.dtr = dtr
        return emu
    return open_


class TestLink(unittest.TestCase):
    def link(self, emu, motor=None):
        return RPLidarLink("emu", motor=motor, serial_factory=factory_for(emu),
                           sleep=lambda s: None).open()

    def test_bring_up_then_scans(self):
        emu, pin = EmulatedA2M12(), FakePin()
        link = self.link(emu, GpioMotor(device=pin))
        info, health = bring_up(link, log_fn=lambda m: None)
        self.assertEqual((info.model, info.firmware, health.status), (44, (1, 32), 0))
        self.assertEqual(pin.value, 1)                      # motor on before the scan
        scans = []
        for s in link.scans():
            scans.append(s)
            if len(scans) == 2:
                break
        self.assertEqual(len(scans[1].points), 120)
        self.assertTrue(all(abs(r - 1.2) < 1e-9 for _, r, _ in scans[1].points))
        self.assertEqual([c for c, _ in emu.commands][:4], [0x25, 0x50, 0x52, 0x59])
        link.close()
        self.assertEqual(emu.commands[-1][0], 0x25)         # laser off first
        self.assertEqual(pin.value, 0)                      # then the motor
        self.assertTrue(emu.closed)

    def test_protection_stop_is_reset_once(self):
        emu = EmulatedA2M12(health=2)
        link = self.link(emu, ExternalMotor())
        bring_up(link, log_fn=lambda m: None)
        self.assertIn(0x40, [c for c, _ in emu.commands])
        self.assertTrue(link.scanning)

    def test_adapter_motor_uses_pwm_when_the_board_says_so(self):
        emu = EmulatedA2M12(acc_board=True)
        link = self.link(emu, AdapterMotor())
        self.assertFalse(emu.dtr)                            # cleared at open, like the SDK
        bring_up(link, log_fn=lambda m: None)
        self.assertEqual(emu.pwm, 660)
        self.assertEqual(emu.checksum_errors, 0)
        link.close()
        self.assertEqual(emu.pwm, 0)
        self.assertTrue(emu.dtr)

    def test_adapter_motor_falls_back_to_dtr(self):
        emu = EmulatedA2M12(acc_board=False)
        link = self.link(emu, AdapterMotor())
        link.motor_on()
        self.assertIsNone(emu.pwm)
        self.assertFalse(emu.dtr)                            # deasserted: the motor spins
        link.motor_off()
        self.assertTrue(emu.dtr)

    def test_silence_is_a_clear_error(self):
        class Dead(EmulatedA2M12):
            def _do(self, cmd, payload):
                pass

        link = self.link(Dead())
        with self.assertRaises(LidarError) as ctx:
            link.get_info(timeout=0.1)
        self.assertIn("wrong port or baud", str(ctx.exception))


class TestReaderThread(unittest.TestCase):
    def test_scans_arrive_and_close_stops_the_motor(self):
        emu, pin = EmulatedA2M12(), FakePin()
        lidar = RPLidar("emu", motor=GpioMotor(device=pin), serial_factory=factory_for(emu),
                        sleep=lambda s: None).start()
        self.addCleanup(lidar.close)
        self.assertTrue(wait_until(lambda: lidar.latest_scan() is not None))
        self.assertEqual(lidar.state, "scanning")
        self.assertLess(lidar.staleness(), 1.0)
        lidar.close()
        self.assertEqual(pin.value, 0)
        self.assertTrue(pin.closed)
        self.assertIn(0x25, [c for c, _ in emu.commands[-3:]])
        self.assertEqual(lidar.state, "closed")

    def test_latest_scan_never_waits_on_the_serial_port(self):
        emu = EmulatedA2M12()
        lidar = RPLidar("emu", serial_factory=factory_for(emu), sleep=lambda s: None).start()
        self.addCleanup(lidar.close)
        self.assertTrue(wait_until(lambda: lidar.latest_scan() is not None))
        emu.read_delay = 0.3                                 # the port wedges
        time.sleep(0.05)
        t0 = time.perf_counter()
        for _ in range(100):
            lidar.latest_scan()
        self.assertLess(time.perf_counter() - t0, 0.05)

    def test_unplug_goes_stale_stops_the_motor_and_recovers(self):
        plugged = {"emu": EmulatedA2M12()}

        def open_(port, baud, timeout, dtr=None):
            if plugged["emu"] is None:
                raise LidarError(f"can't open {port}: no such device")
            return plugged["emu"]

        pin = FakePin()
        lidar = RPLidar("emu", motor=GpioMotor(device=pin), serial_factory=open_,
                        retry_s=0.05, sleep=lambda s: None).start()
        self.addCleanup(lidar.close)
        self.assertTrue(wait_until(lambda: lidar.latest_scan() is not None))
        plugged["emu"].unplugged = True
        plugged["emu"] = None
        self.assertTrue(wait_until(lambda: lidar.state == "error"))
        self.assertEqual(pin.value, 0)                       # motor off while it can't scan
        first = lidar.latest_scan()
        time.sleep(0.2)
        self.assertIs(lidar.latest_scan(), first)            # ages: the bubble sees it stale
        plugged["emu"] = EmulatedA2M12()
        self.assertTrue(wait_until(lambda: lidar.latest_scan() is not first, timeout=5.0))
        self.assertEqual(pin.value, 1)


class TestMotors(unittest.TestCase):
    def test_gpio_motor_starts_off_and_closes_off(self):
        pin = FakePin()
        m = GpioMotor(device=pin)
        self.assertEqual(pin.history, [0])
        m.on()
        m.close()
        self.assertEqual((pin.value, pin.closed), (0, True))

    def test_gpio_motor_needs_a_pin(self):
        with self.assertRaises(ValueError):
            GpioMotor()

    def test_default_motor_choice(self):
        self.assertIsInstance(default_motor("/dev/ttyAMA0"), ExternalMotor)
        self.assertIsInstance(default_motor("/dev/ttyUSB0"), AdapterMotor)
        self.assertIsInstance(default_motor("/dev/cu.usbserial-110"), AdapterMotor)
        with self.assertRaises(ValueError):
            default_motor("/dev/ttyAMA0", kind="warp")


# ---------------------------------------------------------------- the fake


class Clock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t


class TestFakeLidar(unittest.TestCase):
    def test_a_wall_ahead_at_the_right_range(self):
        fl = FakeLidar(FakeWorld([(1.0, -3, 1.0, 3)]), clock=Clock())
        polar = scan_to_base_polar(fl.latest_scan())
        ahead = min((r for b, r in polar if abs(b) < DEG(1)), default=None)
        self.assertAlmostEqual(ahead, 1.0, places=6)

    def test_lazy_on_its_clock_and_freeze_goes_stale(self):
        clock = Clock()
        fl = FakeLidar(FakeWorld.room(), clock=clock, rate_hz=10)
        s1 = fl.latest_scan()
        clock.t += 0.05
        self.assertIs(fl.latest_scan(), s1)
        clock.t += 0.06
        s2 = fl.latest_scan()
        self.assertIsNot(s2, s1)
        fl.freeze()
        clock.t += 2.0
        self.assertIs(fl.latest_scan(), s2)
        self.assertAlmostEqual(staleness(fl.latest_scan(), clock()), 2.0)
        self.assertEqual(staleness(None, clock()), math.inf)

    def test_pose_and_self_parts(self):
        pose = [0.0, 0.0, 0.0]
        fl = FakeLidar(FakeWorld([(2.0, -3, 2.0, 3)]), pose_fn=lambda: tuple(pose),
                       clock=Clock(), self_parts=[(0.0, 0.3, 0.05)])
        polar = scan_to_base_polar(fl.scan_now())
        left = min(r for b, r in polar if abs(b - DEG(90)) < DEG(2))
        self.assertAlmostEqual(left, 0.25, places=3)          # the arm rides along
        pose[0] = 1.5
        ahead = min(r for b, r in scan_to_base_polar(fl.scan_now()) if abs(b) < DEG(1))
        self.assertAlmostEqual(ahead, 0.5, places=6)

    def test_dark_rays_are_counted_as_samples_but_return_nothing(self):
        fl = FakeLidar(FakeWorld.room(), clock=Clock())
        fl.dark = lambda a: a < DEG(30) or a > DEG(330)
        s = fl.scan_now()
        self.assertEqual(sum(s.samples), fl.beams)
        dark_bins = (0, 1, N_SAMPLE_BINS - 1)
        self.assertFalse(any(sample_bin(a) in dark_bins for a, _, _ in s.points))

    def test_tank_pose_follows_the_fake_driver(self):
        clock = Clock(0.0)
        drv = FakeTankDriver(clock=clock)
        pose = FakeTankPose(drv)
        pose()
        drv.set_wheels(0.2, 0.2)
        clock.t += 1.0
        x, y, th = pose()
        self.assertAlmostEqual(x, 0.2, delta=1e-3)
        self.assertAlmostEqual(y, 0.0, delta=1e-9)
        drv.set_wheels(-0.1, 0.1)                             # spin left in place
        clock.t += 1.0
        _, _, th = pose()
        self.assertAlmostEqual(th, 0.2 / drv.geo.effective_track_m, delta=1e-3)


# ---------------------------------------------------------------- the Pi


class TestPiNeedsNoLibraries(unittest.TestCase):
    def test_lidar_and_safety_import_without_pyserial_or_gpiozero(self):
        code = (
            f"import sys; sys.path.insert(0, {str(SRC)!r})\n"
            "before = set(sys.modules)\n"
            "import retriever.bridge.lidar, retriever.bridge.safety, retriever.bridge.server\n"
            "new = {m.split('.')[0] for m in set(sys.modules) - before}\n"
            "print(sorted(new - set(sys.stdlib_module_names) - {'retriever'}))\n"
        )
        out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                             timeout=30)
        self.assertEqual(out.returncode, 0, out.stderr)
        self.assertEqual(out.stdout.strip(), "[]")


class TestLidarCheckScript(unittest.TestCase):
    def run_script(self, *args):
        return subprocess.run([sys.executable, str(ROOT / "tools" / "diagnostics" / "lidar_check.py"), *args],
                              capture_output=True, text=True, timeout=30)

    def test_fake_run_reports_sectors_and_the_front(self):
        out = self.run_script("--fake", "--duration", "0.6", "--find-front")
        self.assertEqual(out.returncode, 0, out.stderr)
        self.assertIn("rev/s", out.stdout)
        self.assertIn("front-left", out.stdout)
        self.assertIn("--lidar-yaw-deg", out.stdout)

    def test_record_mask_proposes_one_sector_per_self_part(self):
        out = self.run_script("--fake", "--record-mask", "0.8", "--every", "5")
        self.assertEqual(out.returncode, 0, out.stderr)
        self.assertIn("proposed self-mask (2 sectors", out.stdout)
        self.assertIn("--lidar-mask ", out.stdout)


if __name__ == "__main__":
    unittest.main()
