"""The lidar safety bubble (bridge/safety.py) and its wiring: the rules on a
fake clock, the bridge core, the server over TCP with a fake lidar, and protocol
compatibility with laptops and Pis from before the lidar.

Scenes come from FakeLidar (bridge/lidar.py) or, where the angle convention
is the point of the test, from raw RPLIDAR bytes through the real parser.
"""

import json
import logging
import math
import random
import socket
import sys
import time
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.bridge.fake_driver import FakeTankDriver
from retriever.bridge.lidar import (
    FakeLidar,
    FakeTankPose,
    FakeWorld,
    LidarMount,
    ScanAssembler,
    encode_measurement,
    parse_measurement,
    scan_to_base_polar,
)
from retriever.bridge.protocol import (
    CLIENT_MESSAGES,
    PROTOCOL_VERSION,
    SERVER_MESSAGES,
    Act,
    Estop,
    Heartbeat,
    ProtocolError,
    Scan,
    State,
    Subscribe,
    decode,
    encode,
)
from retriever.bridge.safety import (
    BubbleConfig,
    Footprint,
    SafetyBubble,
    add_lidar_args,
    format_mask,
    in_mask,
    lidar_from_args,
    parse_mask,
    propose_mask,
)
from retriever.bridge.server import BridgeCore, BridgeServer, ServerThread

DEG = math.radians
logging.getLogger("retriever.bridge").setLevel(logging.CRITICAL)

# What a v1 laptop's strict decoder accepts on each server message, and the
# exact bytes a v1 Pi sent for one state (captured before the lidar existed).
V1_KEYS = {
    "hello": {"v", "type", "counts_per_rev", "wheel_radius_m", "track_width_m",
              "scrub_factor", "state_hz", "timeout_ms", "motion_timeout_ms"},
    "state": {"v", "type", "seq", "t", "left_ticks", "right_ticks", "joints", "gripper_load",
              "battery", "estop", "watchdog_tripped"},
    "error": {"v", "type", "reason"},
}
V1_STATE = (b'{"v":1,"type":"state","seq":3,"t":1.5,"left_ticks":4095,"right_ticks":-12,'
            b'"joints":{"gripper":0.4},"gripper_load":0.7,"battery":0.9,"estop":true,'
            b'"watchdog_tripped":false}\n')


class Clock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t


def room(*posts, walls=()):
    """A closed 6 x 6 m room, so every direction returns something."""
    w = FakeWorld.room(-3.0, -3.0, 3.0, 3.0, posts=posts)
    for wall in walls:
        w.add_wall(*wall)
    return w


def scan_of(world, mount=LidarMount(), t=0.0, beams=720, **kw):
    return FakeLidar(world, mount=mount, beams=beams, clock=lambda: t, **kw).scan_now(t)


def raw_scan(world, t=0.0):
    """A scan built the way the real lidar delivers it: raw clockwise degrees
    in RPLIDAR packets through the parser, for a lidar at the base origin with
    its 0 mark forward. bearing_base = -raw."""
    pkts = []
    for i in range(720):
        raw = i * 0.5
        r = world.raycast(0.0, 0.0, -DEG(raw))
        pkts.append(encode_measurement(i == 0, 30, raw, 0.0 if r is None else r * 1000.0))
    asm, out = ScanAssembler(), None
    for p in pkts + pkts[:1]:
        out = asm.add(parse_measurement(p), t) or out
    return out


# ---------------------------------------------------------------- the rules


class TestFootprintAndMasks(unittest.TestCase):
    def test_clearance(self):
        fp = Footprint(0.25, 0.20, 0.15)
        self.assertEqual(fp.clearance(0.0, 0.0), 0.0)
        self.assertAlmostEqual(fp.clearance(0.35, 0.0), 0.10)
        self.assertAlmostEqual(fp.clearance(-0.30, 0.0), 0.10)
        self.assertAlmostEqual(fp.clearance(0.28, 0.19), 0.05)

    def test_mask_is_a_list_and_wraps(self):
        mask = parse_mask("350:10, 90:100,200:205")
        self.assertEqual(len(mask), 3)
        for deg, want in ((355, True), (5, True), (20, False), (95, True), (203, True),
                          (300, False)):
            self.assertEqual(in_mask(DEG(deg), mask), want, deg)
        self.assertEqual(parse_mask(format_mask([(350, 10), (90, 100)])),
                         parse_mask("350:10,90:100"))
        with self.assertRaises(ValueError):
            parse_mask("90-100")

    def test_creep_settings_that_could_touch_are_refused(self):
        with self.assertRaises(ValueError):
            BubbleConfig(creep_speed_mps=0.2, creep_clearance_m=0.03)


class TestRules(unittest.TestCase):
    def setUp(self):
        self.b = SafetyBubble()

    def check(self, scan, vx, wz=0.0, now=0.0, bubble=None):
        return (bubble or self.b).check(scan, vx, wz, now)

    def test_obstacle_ahead_stops_forward_but_not_reverse_or_turning(self):
        scan = scan_of(room((0.33, 0.0, 0.03)))            # a post 5 cm off the front
        d = self.check(scan, 0.3)
        self.assertEqual((d.state, d.vx, d.wz), ("blocked", 0.0, 0.0))
        self.assertIn("ahead", d.reason)
        self.assertEqual(self.check(scan, -0.3).state, "clear")
        spin = self.check(scan, 0.0, 1.0)
        self.assertGreater(abs(spin.wz), 0.3)               # turning away is allowed
        self.assertAlmostEqual(spin.nearest_m, 0.05, delta=0.005)

    def test_stop_distance_scales_with_speed(self):
        scan = scan_of(room(walls=[(0.45, -2.0, 0.45, 2.0)]))   # a wall 20 cm off the front
        slow, fast = self.check(scan, 0.10), self.check(scan, 0.35)
        self.assertEqual(slow.state, "clear")
        self.assertEqual(fast.state, "slowing")
        self.assertLess(fast.vx, 0.35)
        self.assertGreater(fast.vx, 0.15)
        self.assertLess(slow.stop_m, fast.stop_m)
        # need() and allowed_speed() are inverses above creep
        for free in (0.1, 0.2, 0.4, 0.8):
            v = self.b.allowed_speed(free, 0.25)
            self.assertAlmostEqual(self.b.need_m(v, 0.25), free, places=9)
        speeds = [self.b.allowed_speed(f / 100, 0.25) for f in range(4, 100)]
        self.assertEqual(speeds, sorted(speeds))

    def test_creep_lets_the_robot_close_on_a_bin(self):
        def bin_at(gap):                                  # a 30 cm bin, gap cm off the front
            return scan_of(room((0.25 + gap + 0.15, 0.0, 0.15)))

        d = self.check(bin_at(0.07), 0.3)
        self.assertEqual(d.state, "slowing")
        self.assertAlmostEqual(d.vx, 0.05, places=6)      # down to creep, not to zero
        self.assertEqual(self.check(bin_at(0.07), 0.04).vx, 0.04)
        self.assertEqual(self.check(bin_at(0.03), 0.04).state, "blocked")

    def test_self_mask_ignores_the_robots_own_arm(self):
        arm = [(0.33, 0.12, 0.04)]                        # sticks out in front, to the left
        empty = FakeLidar(FakeWorld.room(-1.5, -1.5, 1.5, 1.5), self_parts=arm, beams=720,
                          clock=lambda: 0.0)
        mask = propose_mask([empty.scan_now(0.0) for _ in range(3)], near_m=0.5)
        self.assertEqual(len(mask), 1)
        scene = room()
        scan = scan_of(scene, self_parts=arm)
        self.assertEqual(self.check(scan, 0.3).state, "blocked")          # sees its own arm
        masked = SafetyBubble(BubbleConfig(mask=parse_mask(format_mask(mask))))
        self.assertEqual(self.check(scan, 0.3, bubble=masked).state, "clear")
        # ...and still sees a real obstacle outside the masked sector
        scan = scan_of(room((0.33, -0.12, 0.03)), self_parts=arm)
        self.assertEqual(self.check(scan, 0.3, bubble=masked).state, "blocked")

    def test_four_wheels_make_four_mask_sectors(self):
        wheels = [(0.15, 0.17, 0.05), (0.15, -0.17, 0.05), (-0.15, 0.17, 0.05),
                  (-0.15, -0.17, 0.05)]
        m = LidarMount(inverted=True)
        empty = FakeLidar(FakeWorld.room(-1.5, -1.5, 1.5, 1.5), mount=m, self_parts=wheels,
                          beams=720, clock=lambda: 0.0)
        mask = propose_mask([empty.scan_now(0.0) for _ in range(3)], near_m=0.45)
        self.assertEqual(len(mask), 4)
        b = SafetyBubble(BubbleConfig(mount=m, mask=parse_mask(format_mask(mask))))
        scan = scan_of(room(), mount=m, self_parts=wheels)
        self.assertEqual(self.check(scan, 0.35, bubble=b).state, "clear")    # straight: fine
        spin = self.check(scan, 0.0, 1.0, bubble=b)        # corners pass behind the wheels
        self.assertIn("masked", spin.reason)
        self.assertLess(b.footprint_speed(spin.vx, spin.wz), 0.0501)

    def test_driving_toward_a_masked_sector_is_creep_only(self):
        """Front-centre mount: the chassis hides the rear."""
        m = LidarMount(0.2, 0.0, 0.0)
        b = SafetyBubble(BubbleConfig(mount=m, mask=parse_mask("150:210")))
        scan = scan_of(room(), mount=m)
        self.assertEqual(self.check(scan, 0.3, bubble=b).state, "clear")
        back = self.check(scan, -0.3, bubble=b)
        self.assertAlmostEqual(back.vx, -0.05, places=6)
        self.assertIn("masked", back.reason)

    def test_stale_or_missing_scan_allows_creep_only(self):
        scan = scan_of(room(), t=0.0)
        fresh = self.check(scan, 0.3, now=0.2)
        self.assertEqual(fresh.state, "clear")
        old = self.check(scan, 0.3, now=0.8)
        self.assertEqual(old.state, "stale")
        self.assertAlmostEqual(old.vx, 0.05, places=6)
        self.assertIn("0.8 s old", old.reason)
        self.assertIsNone(old.nearest_m)
        none = self.check(None, 0.3)
        self.assertEqual((none.state, round(none.vx, 6)), ("stale", 0.05))
        self.assertIn("no lidar scan yet", none.reason)
        self.assertAlmostEqual(self.check(None, -0.3).vx, -0.05, places=6)
        self.assertEqual(self.check(None, 0.03).vx, 0.03)             # creep passes
        self.assertEqual(self.check(None, 0.0).state, "stale")        # idle says so too

    def test_stale_scan_can_still_forbid(self):
        scan = scan_of(room((0.33, 0.0, 0.03)), t=0.0)
        self.assertEqual(self.check(scan, 0.04, now=5.0).vx, 0.0)

    def test_dark_rays_toward_the_path_count_as_unknown(self):
        lidar = FakeLidar(room(), beams=720, clock=lambda: 0.0)
        lidar.dark = lambda a: a < DEG(60) or a > DEG(300)           # black sofa ahead
        scan = lidar.scan_now(0.0)
        d = self.check(scan, 0.3)
        self.assertAlmostEqual(d.vx, 0.05, places=6)
        self.assertIn("can't see ahead", d.reason)
        self.assertEqual(self.check(scan, -0.3).state, "clear")
        self.assertEqual(self.check(scan, 0.04).vx, 0.04)
        off = SafetyBubble(BubbleConfig(min_valid_fraction=0.0))
        self.assertEqual(self.check(scan, 0.3, bubble=off).state, "clear")

    def test_open_space_is_unknown_not_clear(self):
        scan = scan_of(FakeWorld())                                   # nothing in range
        self.assertAlmostEqual(self.check(scan, 0.3).vx, 0.05, places=6)

    def test_never_increases_changes_direction_or_curvature(self):
        rng = random.Random(7)
        for trial in range(150):
            posts = [(rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5), rng.uniform(0.02, 0.2))
                     for _ in range(rng.randint(0, 6))]
            scan = scan_of(room(*posts), beams=360) if trial % 5 else None
            vx, wz = rng.uniform(-0.5, 0.5), rng.uniform(-1.5, 1.5)
            if trial % 7 == 0:
                vx = 0.0
            d = self.check(scan, vx, wz, now=rng.choice([0.0, 0.1, 1.0]))
            with self.subTest(trial=trial, vx=vx, wz=wz):
                self.assertLessEqual(abs(d.vx), abs(vx) + 1e-12)
                self.assertLessEqual(abs(d.wz), abs(wz) + 1e-12)
                self.assertGreaterEqual(d.vx * vx, 0.0)
                self.assertGreaterEqual(d.wz * wz, 0.0)
                self.assertAlmostEqual(d.vx * wz, d.wz * vx, places=9)
                self.assertTrue(0.0 <= d.scale <= 1.0)

    def test_floor_filter_drops_one_scan_arcs_only_beyond_the_floor_range(self):
        b = SafetyBubble(BubbleConfig(floor_height_m=0.05, floor_pitch_deg=5.0))
        floor_r = b.floor_range_m                            # 0.05 / tan 5 deg = 0.57 m
        self.assertAlmostEqual(floor_r, 0.5715, places=3)
        calm = scan_of(room())
        b.obstacles(calm)
        # the nose dips: an arc of "floor" 0.60 m ahead in ONE scan
        def on(scan, x0, bubble):
            return [p for p in bubble.obstacles(scan)
                    if abs(p[0] - x0) < 0.02 and abs(p[1]) < 0.3]

        pitched = scan_of(room(walls=[(0.60, -0.3, 0.60, 0.3)]))
        self.assertEqual(on(pitched, 0.60, b), [])
        self.assertGreater(b.floor_dropped, 0)
        # a real wall persists: confirmed on the next scan
        again = scan_of(room(walls=[(0.60, -0.3, 0.60, 0.3)]), t=0.1)
        self.assertGreater(len(on(again, 0.60, b)), 10)
        # nearer than the floor range: never filtered, even the first time
        b2 = SafetyBubble(BubbleConfig(floor_height_m=0.05, floor_pitch_deg=5.0))
        b2.obstacles(calm)
        near = scan_of(room(walls=[(0.40, -0.3, 0.40, 0.3)]))
        self.assertGreater(len(on(near, 0.40, b2)), 10)
        # off by default
        self.assertGreater(len(on(pitched, 0.60, SafetyBubble())), 10)


class TestAngleConvention(unittest.TestCase):
    """RPLIDAR degrees run clockwise; the base frame is CCW, +y = LEFT. A sign
    slip here mirrors left and right, and the robot turns INTO what it sees."""

    # Front-left, where the nose goes in a left turn. (A post exactly abeam is
    # hit by the TAIL in a tight right pivot as well: the rear corner swings
    # out. That is real, and is why this test does not put it at 270 raw;
    # tests/test_lidar.py pins raw 270 -> +90 deg = left directly.)
    ARCS = ((0.2, 1.2), (0.3, 0.3))

    def setUp(self):
        self.scan = raw_scan(room((0.40, 0.30, 0.05)))

    def test_the_raw_bytes_put_the_post_on_the_left(self):
        near = min(scan_to_base_polar(self.scan), key=lambda p: p[1])
        self.assertAlmostEqual(math.degrees(near[0]), 36.9, delta=3.0)   # +bearing = left
        near_raw = min(self.scan.points, key=lambda p: p[1])
        self.assertAlmostEqual(360.0 - math.degrees(near_raw[0]), 323.1, delta=3.0)  # raw cw

    def test_left_obstacle_limits_a_left_turn_not_a_right_turn(self):
        b = SafetyBubble()
        for vx, wz in self.ARCS:
            with self.subTest(vx=vx, wz=wz):
                left = b.check(self.scan, vx, wz, 0.0)
                self.assertIn(left.state, ("slowing", "blocked"))
                self.assertIn("turning left", left.reason)
                self.assertEqual(b.check(self.scan, vx, -wz, 0.0).state, "clear")

    def test_inverted_mount_mirrors_it(self):
        b = SafetyBubble(BubbleConfig(mount=LidarMount(inverted=True)))
        for vx, wz in self.ARCS:
            with self.subTest(vx=vx, wz=wz):
                self.assertEqual(b.check(self.scan, vx, wz, 0.0).state, "clear")
                right = b.check(self.scan, vx, -wz, 0.0)
                self.assertIn(right.state, ("slowing", "blocked"))
                self.assertIn("turning right", right.reason)


# ---------------------------------------------------------------- the bridge core


class TestCoreWithLidar(unittest.TestCase):
    """BridgeCore on a fake clock with a fake lidar riding on the fake tank."""

    def setUp(self):
        self.clock = Clock()
        self.drv = FakeTankDriver(clock=self.clock)
        self.lidar = FakeLidar(room(walls=[(1.0, -2.0, 1.0, 2.0)]),
                               pose_fn=FakeTankPose(self.drv),
                               clock=self.clock, rate_hz=10.0)
        self.core = BridgeCore(self.drv, self.clock(), timeout_s=0.3, motion_timeout_s=0.5,
                               lidar=self.lidar)
        self.core.connected(self.clock())
        self.pose = FakeTankPose(self.drv)
        self.pose()

    def run_for(self, seconds, msg_fn=None, every=0.02):
        for i in range(int(round(seconds / every))):
            self.clock.t += every
            if msg_fn is not None:
                self.core.handle(msg_fn(i), self.clock())
            self.core.tick(self.clock())

    def test_driving_at_a_wall_stops_short_of_it(self):
        self.run_for(6.0, lambda i: Act(seq=i + 1, base_vx=0.3))
        x, _, _ = self.pose()
        front_gap = 1.0 - (x + 0.25)
        self.assertGreater(front_gap, 0.02)                # never touched
        self.assertLess(front_gap, 0.08)                   # but crept right up to it
        self.assertEqual(self.drv.wheel_speeds, (0.0, 0.0))
        self.assertEqual(self.core.decision.state, "blocked")
        s = self.core.snapshot(self.clock(), extended=True)
        self.assertEqual(s.bubble, "blocked")
        self.assertAlmostEqual(s.nearest_m, front_gap, delta=0.01)

    def test_arm_commands_pass_while_the_base_is_blocked(self):
        self.run_for(6.0, lambda i: Act(seq=i + 1, base_vx=0.3))
        self.core.handle(Act(seq=999, base_vx=0.3, joints={"elbow_flex": 0.7}, gripper=0.1,
                             vacuum=True), self.clock())
        self.assertEqual(self.drv._targets["elbow_flex"], 0.7)
        self.assertEqual(self.drv._targets["gripper"], 0.1)
        self.assertTrue(self.drv.vacuum)
        self.assertEqual(self.drv.wheel_speeds, (0.0, 0.0))

    def test_estop_still_wins_and_the_bubble_cannot_clear_it(self):
        self.core.handle(Estop(reason="test"), self.clock())
        self.run_for(1.0, lambda i: Act(seq=i + 1, base_vx=0.1))
        self.assertTrue(self.core.estop)
        self.assertEqual(self.drv.wheel_speeds, (0.0, 0.0))

    def test_ticks_only_lower_the_wheels_between_acts(self):
        self.core.handle(Act(seq=1, base_vx=0.3), self.clock())
        full = self.drv.wheel_speeds
        self.lidar.world.add_post(0.45, 0.0, 0.05)          # someone steps in front
        self.run_for(0.12, lambda i: Heartbeat())           # a new revolution is due
        self.assertLess(abs(self.drv.wheel_speeds[0]), abs(full[0]))
        lowered = self.drv.wheel_speeds
        self.lidar.world.posts.pop()                        # ...and steps away
        for _ in range(10):                                 # alive, but no new act
            self.run_for(0.02, lambda i: Heartbeat())
            # the Pi never speeds up alone (the aging scan may still lower it)
            self.assertLessEqual(abs(self.drv.wheel_speeds[0]), abs(lowered[0]) + 1e-12)
        self.core.handle(Act(seq=2, base_vx=0.3), self.clock())
        self.assertEqual(self.drv.wheel_speeds, full)       # the laptop's next act does

    def test_a_dead_lidar_limits_the_base_to_creep(self):
        self.run_for(0.2, lambda i: Act(seq=i + 1, base_vx=0.3))
        self.assertGreater(self.drv.wheel_speeds[0], 0.25)
        self.lidar.freeze()
        self.run_for(0.7, lambda i: Act(seq=100 + i, base_vx=0.3))
        self.assertEqual(self.core.decision.state, "stale")
        v = (self.drv.wheel_speeds[0] + self.drv.wheel_speeds[1]) / 2
        self.assertAlmostEqual(v, 0.05, places=6)

    def test_a_scan_going_stale_between_acts_lowers_the_wheels(self):
        self.core.bubble = SafetyBubble(BubbleConfig(stale_after_s=0.3))
        self.core.handle(Act(seq=1, base_vx=0.3), self.clock())
        self.lidar.freeze()
        self.run_for(0.4, lambda i: Heartbeat())            # past stale, inside the deadman
        self.assertEqual(self.core.decision.state, "stale")
        v = (self.drv.wheel_speeds[0] + self.drv.wheel_speeds[1]) / 2
        self.assertAlmostEqual(v, 0.05, places=6)

    def test_a_bubble_that_raises_stops_the_base(self):
        self.core.bubble.check = mock.Mock(side_effect=RuntimeError("bug"))
        self.core.handle(Act(seq=1, base_vx=0.3, joints={"elbow_flex": 0.2}), self.clock())
        self.assertEqual(self.drv.wheel_speeds, (0.0, 0.0))
        self.assertEqual(self.core.decision.state, "blocked")
        self.assertIn("bubble failed", self.core.decision.reason)
        self.assertEqual(self.drv._targets["elbow_flex"], 0.2)

    def test_no_lidar_means_no_bubble_and_no_new_fields(self):
        core = BridgeCore(FakeTankDriver(clock=self.clock), self.clock())
        self.assertIsNone(core.bubble)
        s = core.snapshot(self.clock(), extended=True)
        self.assertIsNone(s.bubble)
        self.assertEqual(set(json.loads(encode(s))), V1_KEYS["state"])


# ---------------------------------------------------------------- protocol


class TestProtocolExtension(unittest.TestCase):
    def test_a_v1_state_is_byte_identical(self):
        s = State(seq=3, t=1.5, left_ticks=4095, right_ticks=-12, joints={"gripper": 0.4},
                  gripper_load=0.7, battery=0.9, estop=True, watchdog_tripped=False)
        self.assertEqual(encode(s), V1_STATE)
        self.assertEqual(decode(V1_STATE), s)

    def test_new_messages_round_trip_in_their_direction(self):
        for m in (Subscribe(), Subscribe(safety=True, scan=False),
                  Scan(t=1.0, step_deg=2.0, ranges_cm=[0, 12, 300], near=[1]),
                  State(seq=1, t=0.1, left_ticks=0, right_ticks=0, bubble="slowing",
                        bubble_reason="slowing to 0.10 m/s", nearest_m=0.4, stop_m=0.2)):
            self.assertEqual(decode(encode(m)), m)
        self.assertIn(Subscribe, CLIENT_MESSAGES)
        self.assertIn(Scan, SERVER_MESSAGES)
        with self.assertRaises(ProtocolError):
            decode(encode(Subscribe()), SERVER_MESSAGES)

    def test_bad_extensions_are_protocol_errors(self):
        def line(**o):
            return json.dumps({"v": PROTOCOL_VERSION, **o})

        for bad in (line(type="scan", t=0, step_deg=2, ranges_cm=[-1]),
                    line(type="scan", t=0, step_deg=2, ranges_cm=[True]),
                    line(type="scan", t=0, step_deg=0, ranges_cm=[]),
                    line(type="scan", t=0, step_deg=2, ranges_cm=[1, 2], near=[5]),
                    line(type="scan", t=0, step_deg=2, ranges_cm=[0] * 800),
                    line(type="state", seq=0, t=0, left_ticks=0, right_ticks=0, bubble="warp"),
                    line(type="state", seq=0, t=0, left_ticks=0, right_ticks=0, nearest_m=-1),
                    line(type="subscribe", scan="yes")):
            with self.subTest(bad=bad[:70]):
                with self.assertRaises(ProtocolError):
                    decode(bad)


# ---------------------------------------------------------------- over TCP


class Raw:
    """A laptop from before the lidar: strict v1 decoding, never subscribes."""

    def __init__(self, port):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=2.0)
        self.buf = b""

    def line(self):
        while b"\n" not in self.buf:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise EOFError
            self.buf += chunk
        raw, _, self.buf = self.buf.partition(b"\n")
        return raw

    def send(self, msg):
        self.sock.sendall(encode(msg))

    def close(self):
        self.sock.close()


class LidarServerCase(unittest.TestCase):
    def serve(self, lidar=True, wall_x=1.0, **kw):
        self.drv = FakeTankDriver()
        self.lidar = None
        if lidar:
            self.lidar = FakeLidar(room(walls=[(wall_x, -2.0, wall_x, 2.0)]),
                                   pose_fn=FakeTankPose(self.drv), rate_hz=10.0)
        opts = dict(timeout_ms=300, motion_timeout_ms=500, state_hz=50.0, scan_hz=10.0)
        opts.update(kw)
        self.server = BridgeServer(self.drv, "127.0.0.1", 0, lidar=self.lidar, **opts)
        self.st = ServerThread(self.server)
        self.port = self.st.start()
        self.addCleanup(self.st.stop)


class TestServerWithLidar(LidarServerCase):
    def test_old_client_gets_exactly_the_v1_stream(self):
        self.serve()
        c = Raw(self.port)
        self.addCleanup(c.close)
        c.send(Heartbeat())
        c.send(Act(seq=1, base_vx=0.2))
        seen = set()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            obj = json.loads(c.line())
            seen.add(obj["type"])
            self.assertIn(obj["type"], V1_KEYS)
            self.assertEqual(set(obj), V1_KEYS[obj["type"]])
        self.assertIn("state", seen)
        # and the bubble still guards it: its drive is filtered all the same
        self.assertIsNotNone(self.st.call(lambda: self.server.core.decision))

    def test_server_close_closes_the_lidar(self):
        self.serve()
        closed = []
        self.lidar.close = lambda: closed.append(True)
        self.st.stop()
        self.assertEqual(closed, [True])


class TestCommandLine(unittest.TestCase):
    def parse(self, *argv):
        import argparse

        ap = argparse.ArgumentParser()
        add_lidar_args(ap)
        return ap.parse_args(list(argv))

    def test_no_lidar_by_default(self):
        self.assertEqual(lidar_from_args(self.parse()), (None, None))

    def test_fake_lidar_rides_on_the_fake_tank(self):
        args = self.parse("--fake-lidar", "--footprint", "0.3,0.2,0.18", "--lidar-x", "0.1",
                          "--lidar-yaw-deg", "180", "--lidar-inverted",
                          "--lidar-mask", "80:100,260:280")
        lidar, bubble = lidar_from_args(args, FakeTankDriver())
        self.assertIsInstance(lidar, FakeLidar)
        c = bubble.config
        self.assertEqual((c.footprint.front_m, c.mount.x_m, c.mount.inverted), (0.3, 0.1, True))
        self.assertAlmostEqual(c.mount.yaw_rad, math.pi)
        self.assertEqual(len(c.mask), 2)
        self.assertIsNotNone(lidar.latest_scan())
        with self.assertRaises(ValueError):
            lidar_from_args(args, driver=object())


if __name__ == "__main__":
    unittest.main()
