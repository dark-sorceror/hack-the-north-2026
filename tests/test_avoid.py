"""Lidar obstacle avoidance: the planner, the scan plumbing, and the controllers
end to end on TankFakeRobot with a simulated RPLIDAR (60% dropouts, 1 cm noise).

No hardware, no wall clock. The sign tests are the ones to read first: a lidar
that reports angles clockwise on a base that counts them anticlockwise is the
bug that makes a robot steer INTO the chair.
"""

import math
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.navigation.avoid import (
    BLOCKED,
    CLEAR,
    STEERING,
    AvoidConfig,
    LocalPlanner,
    Mount,
    Scan,
    ScanTracker,
    bridge_scan_source,
    parse_mask,
    reproject,
    scan_from_lidar_scan,
    scan_from_rplidar,
)
from retriever.navigation.drive import Limits
from retriever.navigation.simscan import (
    Circle,
    SimLidar,
    box,
    chair,
    occluder_mask,
    room,
    wall,
    wheels,
)
from retriever.types import Pose

DEG = math.pi / 180
ROOM = room(-2.0, -2.5, 4.5, 2.5)
CFG = AvoidConfig()


def scan_from(world, pose=Pose(), mount=Mount(), seed=0, sweeps=3, **kw) -> Scan:
    """What the planner sees from `pose`: `sweeps` revolutions merged, the way
    ScanTracker merges them on the robot (0.5 s of scans)."""
    lid = SimLidar(world, lambda: pose, lambda: 0.0, mount, seed=seed, **kw)
    scans = [lid.scan_at(pose) for _ in range(sweeps)]
    pts = tuple(p for s in scans for p in s.points)
    return Scan(0.0, pts, scans[0].ray_step_rad / sweeps, scans[0].blind)


# ------------------------------------------------------------------ signs


class TestMountSigns(unittest.TestCase):
    """RPLIDAR: clockwise from above, 0 at the front mark. Base: + is LEFT."""

    def test_a_ray_clockwise_of_the_front_mark_points_right(self):
        self.assertAlmostEqual(Mount().ray_bearing(90.0), -math.pi / 2)
        self.assertAlmostEqual(Mount().ray_bearing(350.0), 10 * DEG)   # just LEFT of front

    def test_upside_down_mirrors_the_sweep(self):
        self.assertAlmostEqual(Mount(inverted=True).ray_bearing(90.0), math.pi / 2)
        self.assertAlmostEqual(Mount(inverted=True).ray_bearing(350.0), -10 * DEG)

    def test_yaw_is_where_the_front_mark_points(self):
        m = Mount(yaw_rad=math.pi / 2)            # front mark facing the robot's left
        self.assertAlmostEqual(m.ray_bearing(0.0), math.pi / 2)
        x, y = m.to_base(0.0, 1.0)
        self.assertAlmostEqual(x, 0.0, places=9)
        self.assertAlmostEqual(y, 1.0)

    def test_the_mount_offset_is_added_in_the_base_frame(self):
        x, y = Mount(x_m=0.2, y_m=-0.05).to_base(0.0, 1.0)
        self.assertAlmostEqual(x, 1.2)
        self.assertAlmostEqual(y, -0.05)

    def test_the_simulated_lidar_speaks_the_real_units_convention(self):
        """Independent of the planner: a post front-LEFT of an upright lidar
        must come back at a raw angle just BELOW 360 (clockwise from the front)."""
        lid = SimLidar([Circle(1.0, 0.25, 0.05, p_return=1.0)], lambda: Pose(), lambda: 0.0)
        hits = [a for a, r in lid.sweep(Pose()) if r > 0]
        self.assertTrue(hits)
        self.assertTrue(all(330.0 < a < 360.0 for a in hits), hits)

    def test_mask_is_in_the_lidars_own_frame_like_the_bubbles(self):
        """'80:100' is the lidar's own counter-clockwise frame (--lidar-mask).
        Right way up that is the robot's left; upside down, its right."""
        mask = parse_mask("80:100")
        self.assertAlmostEqual(mask[0][0], 80 * DEG)
        up = Mount(mask=mask)
        self.assertTrue(up.masked(270.0))          # clockwise 270 = counter-clockwise 90
        self.assertFalse(up.masked(90.0))
        (start, width), = up.blind()
        self.assertAlmostEqual(start, 80 * DEG)
        self.assertAlmostEqual(width, 20 * DEG)
        (start, width), = Mount(inverted=True, mask=mask).blind()
        self.assertAlmostEqual(start, -100 * DEG)

    def test_a_wrapping_mask(self):
        m = Mount(mask=parse_mask("350:10"))
        self.assertTrue(m.masked(0.0) and m.masked(355.0) and m.masked(5.0))
        self.assertFalse(m.masked(20.0))


class TestObstacleOnTheLeftSteersRight(unittest.TestCase):
    """THE sign test, through the whole pipeline: world -> raw clockwise sweep
    -> Mount -> planner, for every way the lidar might be bolted on."""

    MOUNTS = [
        Mount(),
        Mount(inverted=True),
        Mount(yaw_rad=math.pi),                          # cable facing forward
        Mount(inverted=True, yaw_rad=37 * DEG, x_m=0.05, y_m=-0.03),
        Mount(x_m=0.20),                                 # front-centre mount
    ]

    def test_left_obstacle_right_turn_and_vice_versa(self):
        for m in self.MOUNTS:
            with self.subTest(mount=m):
                left = LocalPlanner().plan(scan_from(ROOM + [Circle(1.0, 0.25, 0.1)], mount=m),
                                           0.0, 3.0)
                self.assertEqual(left.status, STEERING)
                self.assertLess(left.steer_rad, 0.0, left.reason)
                self.assertIn("on the right", left.reason)
                right = LocalPlanner().plan(scan_from(ROOM + [Circle(1.0, -0.25, 0.1)], mount=m),
                                            0.0, 3.0)
                self.assertGreater(right.steer_rad, 0.0, right.reason)


# ------------------------------------------------------------------ planner


class TestPlanner(unittest.TestCase):
    def test_open_space_goes_straight_at_full_speed(self):
        for goal in (0.0, 30 * DEG, -60 * DEG):
            p = LocalPlanner().plan(scan_from(ROOM), goal, 1.5)
            self.assertEqual(p.status, CLEAR)
            self.assertEqual(p.steer_rad, goal)
            self.assertEqual((p.speed_scale, p.turn_scale), (1.0, 1.0))
            self.assertTrue(p.seen)

    def test_a_box_in_the_way_is_steered_round_and_named(self):
        p = LocalPlanner().plan(scan_from(ROOM + box(1.5, 0.0, 0.4, 0.4)), 0.0, 3.0)
        self.assertEqual(p.status, STEERING)
        self.assertGreater(abs(p.steer_rad), 20 * DEG)
        self.assertAlmostEqual(p.obstacle_m, 1.3 - CFG.radius_m, delta=0.05)
        self.assertIn("about 1.0 m ahead", p.reason)

    def test_obstacles_beyond_lookahead_are_ignored(self):
        p = LocalPlanner().plan(scan_from(ROOM + box(2.4, 0.0, 0.4, 0.4)), 0.0, 3.5)
        self.assertEqual(p.status, CLEAR)

    def test_a_dead_end_is_blocked_with_speed_zero(self):
        u = [wall(-0.2, 0.6, 1.5, 0.6), wall(-0.2, -0.6, 1.5, -0.6), wall(1.5, -0.6, 1.5, 0.6)]
        p = LocalPlanner().plan(scan_from(ROOM + u), 0.0, 3.5)
        self.assertEqual(p.status, BLOCKED)
        self.assertEqual((p.speed_scale, p.turn_scale), (0.0, 0.0))
        self.assertEqual(
            p.reason, "Something is blocking the way about 1.2 m ahead and I can't find a way around it.")

    def test_an_unseen_sector_is_not_free(self):
        """A box that returns nothing (chrome, black) is a hole in the scan. The
        direction into it is UNKNOWN, and the planner goes round it on a seen
        heading rather than into it at full speed."""
        dark = box(1.5, 0.0, 0.6, 0.6, p_return=0.0)
        p = LocalPlanner().plan(scan_from(ROOM + dark), 0.0, 3.2)
        self.assertEqual(p.sectors[0], "u")
        self.assertTrue(p.seen)
        # its heading clears the box's near corners by the body radius
        for cy in (-0.3, 0.3):
            corner_bearing = math.atan2(cy, 1.2)
            lateral = math.hypot(1.2, cy) * math.sin(abs(p.steer_rad - corner_bearing))
            self.assertGreater(lateral, CFG.radius_m)

    def test_when_nothing_is_seen_it_only_creeps(self):
        """No walls: every ray returns nothing. Nothing is free."""
        p = LocalPlanner().plan(scan_from([]), 0.0, 3.0)
        self.assertFalse(p.seen)
        self.assertLessEqual(p.speed_scale, CFG.creep_scale)
        self.assertIn("creeping", p.reason)

    def test_sparse_returns_are_unknown(self):
        """Returns in every direction, but 1 ray in 20 (5%): too few to call free."""
        pts = tuple((i * 20 * DEG / 20, 3.0) for i in range(0, 360, 20))
        p = LocalPlanner().plan(Scan(0.0, pts, 1 * DEG), 0.0, 2.0)
        self.assertFalse(p.seen)
        self.assertLessEqual(p.speed_scale, CFG.creep_scale)

    def test_a_masked_sector_counts_as_unknown_not_free(self):
        m = Mount(mask=parse_mask("60:120"))        # upright: the robot's left
        s = scan_from(ROOM, mount=m, sweeps=3)
        p = LocalPlanner().plan(s, 90 * DEG, 1.5)
        self.assertEqual(p.sectors[45], "u")        # 90 degrees
        self.assertFalse(p.status == CLEAR and p.seen)
        self.assertEqual(p.sectors[0], "f")         # dead ahead is still fine

    def test_the_goal_itself_is_not_an_obstacle(self):
        """Sent to a bin: its returns are the goal. Without goal_clear_m the
        robot would route round the very bin it was sent to."""
        bin_ = [Circle(1.2, 0.0, 0.2)]
        p = LocalPlanner().plan(scan_from(ROOM + bin_), 0.0, 1.2)
        self.assertEqual(p.status, CLEAR)
        self.assertEqual(p.steer_rad, 0.0)
        self.assertGreaterEqual(p.speed_scale, CFG.creep_scale)
        naive = LocalPlanner(AvoidConfig(goal_clear_m=0.0)).plan(scan_from(ROOM + bin_), 0.0, 1.2)
        self.assertNotEqual(naive.status, CLEAR)

    def test_closing_on_the_goal_object_slows_to_creep_not_to_zero(self):
        # the bin's face is 0.08 m from the body: an obstacle there would mean speed 0
        p = LocalPlanner().plan(scan_from(ROOM + [Circle(0.6, 0.0, 0.2)]), 0.0, 0.6)
        self.assertEqual(p.status, CLEAR)
        self.assertAlmostEqual(p.speed_scale, CFG.creep_scale)
        self.assertEqual(p.reason, "Creeping up to the goal.")

    def test_a_wall_just_beyond_the_goal_does_not_block_it(self):
        """The trip ends at the goal; a wall 0.45 m past it is not in the way."""
        p = LocalPlanner().plan(scan_from(ROOM + [wall(1.25, -1.0, 1.25, 1.0)]), 0.0, 0.8)
        self.assertEqual(p.status, CLEAR)

    def test_hysteresis_holds_one_side_between_two_equal_gaps(self):
        """A post dead ahead, gaps either side exactly alike: the noise decides
        each scan. Hysteresis keeps it going round the side it picked."""
        post = [Circle(1.0, 0.0, 0.1)]
        for hysteresis, expect_flips in ((CFG.hysteresis_deg, False), (0.0, True)):
            flips = 0
            for seed in (7, 8, 9):
                lid = SimLidar(ROOM + post, lambda: Pose(), lambda: 0.0, seed=seed)
                planner = LocalPlanner(AvoidConfig(hysteresis_deg=hysteresis))
                sides = []
                for _ in range(40):
                    scans = [lid.scan_at(Pose()) for _ in range(3)]
                    merged = Scan(0.0, tuple(p for s in scans for p in s.points),
                                  scans[0].ray_step_rad / 3)
                    sides.append(planner.plan(merged, 0.0, 3.0).steer_rad > 0)
                flips += sum(a != b for a, b in zip(sides, sides[1:]))
            with self.subTest(hysteresis=hysteresis):
                self.assertEqual(flips > 0, expect_flips, flips)

    def test_speed_stays_under_what_the_pi_bubble_allows(self):
        """safety.py rule 2: the bubble allows v with v*t + v^2/(2a) + 0.05 <= free
        (a = 0.5 m/s^2), creep (0.05 m/s) down to 3 cm. Take the reaction time
        pessimistically (0.1 s latency + a 5 Hz wire scan 0.3 s old + a 0.15 s
        revolution) and the bubble's free path as no longer than the planner's."""
        v_max, a, t = Limits().v_max, 0.5, 0.10 + 0.30 + 0.15

        def bubble_allows(free):
            if free <= 0.03:
                return 0.0
            d = free - 0.05
            v = a * (-t + math.sqrt(t * t + 2 * d / a)) if d > 0 else 0.0
            return max(v, 0.05)

        planner = LocalPlanner()
        for cm in range(0, 200):
            free = cm / 100
            pts = ((0.0, CFG.radius_m + free),)
            p = planner.plan(Scan(0.0, pts, 1 * DEG), 0.0, 3.0)
            with self.subTest(free_m=free):
                self.assertLessEqual(p.speed_scale * v_max, bubble_allows(free) + 1e-9)
        # and onto the goal, where it creeps, it creeps no faster than the bubble's creep
        self.assertLessEqual(CFG.creep_scale * v_max, 0.05)

    def test_turning_on_the_spot_near_a_wall_with_blind_corners_creeps(self):
        """Lidar upside down between four wheels: the wheels blind the diagonals,
        which is exactly where the corners sweep. A wall there: careful turns."""
        occ = wheels()
        m = Mount(inverted=True, yaw_rad=20 * DEG)
        m = Mount(inverted=True, yaw_rad=20 * DEG, mask=occluder_mask(occ, m))
        near_wall = [wall(0.0, 0.48, 0.48, 0.0)]      # 0.34 m out, square to the front-left corner
        for world, expect in ((ROOM, 1.0), (ROOM + near_wall, CFG.creep_scale)):
            lid = SimLidar(world, lambda: Pose(), lambda: 0.0, m, occluders=occ, seed=2)
            p = LocalPlanner().plan(lid.scan_at(Pose()), math.pi, 2.0)
            self.assertEqual(p.turn_scale, expect)

    def test_without_the_mask_the_wheels_look_like_obstacles(self):
        lid = SimLidar(ROOM, lambda: Pose(), lambda: 0.0, Mount(inverted=True), occluders=wheels())
        self.assertEqual(LocalPlanner().plan(lid.scan_at(Pose()), 0.0, 2.0).status, BLOCKED)

    def test_fast_enough_for_every_control_tick(self):
        s = scan_from(ROOM + chair(1.0, 0.2) + box(1.5, -0.8, 0.5, 0.5), sweeps=5)
        self.assertGreater(len(s.points), 500)
        planner = LocalPlanner()
        t0 = time.perf_counter()
        for i in range(50):
            planner.plan(s, i * DEG, 2.0)
        self.assertLess((time.perf_counter() - t0) / 50, 0.010)


# ------------------------------------------------------------------ plumbing


class TestScanPlumbing(unittest.TestCase):
    def test_reproject_moves_points_into_the_current_frame(self):
        s = Scan(0.0, ((0.0, 2.0),))                         # 2 m dead ahead of the old pose
        (b, r), = reproject(s, Pose(0, 0, 0), Pose(1.0, 0.0, math.pi / 2)).points
        self.assertAlmostEqual(r, 1.0)
        self.assertAlmostEqual(b, -math.pi / 2)              # now on the right

    def test_tracker_compensates_for_turning_since_the_scan(self):
        scan = Scan(1.0, ((0.0, 1.0),))
        tracker = ScanTracker(lambda: scan)
        tracker.update(1.0, Pose())
        tracker.update(1.2, Pose(0, 0, 0.3))                 # turned 0.3 rad left since
        (b, _), = tracker.current(1.2, Pose(0, 0, 0.3)).points
        self.assertAlmostEqual(b, -0.3)

    def test_tracker_merges_recent_scans_and_refuses_stale_ones(self):
        scans = iter([Scan(0.0, ((0.0, 1.0),)), Scan(0.1, ((0.1, 1.0),)), Scan(0.9, ((0.2, 1.0),))])
        cur = [next(scans)]
        tracker = ScanTracker(lambda: cur[0], memory_s=0.5, stale_s=0.6)
        tracker.update(0.0, Pose())
        cur[0] = next(scans)
        tracker.update(0.1, Pose())
        self.assertEqual(len(tracker.current(0.1, Pose()).points), 2)
        self.assertIsNone(tracker.current(0.8, Pose()))       # newest is 0.7 s old
        cur[0] = next(scans)
        tracker.update(0.9, Pose())
        self.assertEqual(len(tracker.current(0.9, Pose()).points), 1)  # 0.0 and 0.1 aged out

    def test_bridge_scan_source_reads_scanview_and_the_bubble(self):
        """Duck-typed on BridgeRobot.scan (ScanView) and .bubble (BubbleStatus)."""
        robot = SimpleNamespace(
            scan=SimpleNamespace(t=4.0, points=((1.0, 1.0, False), (0.0, -2.0, True))),
            bubble=SimpleNamespace(state="blocked"))
        s = bridge_scan_source(robot, Mount(mask=parse_mask("170:190")))()
        self.assertEqual(s.t, 4.0)
        self.assertAlmostEqual(s.points[0][0], math.pi / 4)
        self.assertAlmostEqual(s.points[1][0], -math.pi / 2)
        self.assertAlmostEqual(s.ray_step_rad, 2 * DEG)
        self.assertTrue(s.bubble_blocked)
        self.assertEqual(len(s.blind), 1)
        robot.bubble, robot.scan = None, None
        self.assertIsNone(bridge_scan_source(robot)())

    def test_scan_from_the_pi_drivers_lidar_scan(self):
        """bridge.lidar.LidarScan is already counter-clockwise radians."""
        ls = SimpleNamespace(t=2.0, points=((math.pi / 2, 1.0, 40),), n_raw=360)
        s = scan_from_lidar_scan(ls)
        self.assertAlmostEqual(s.points[0][0], math.pi / 2)  # left stays left
        s = scan_from_lidar_scan(ls, Mount(inverted=True))
        self.assertAlmostEqual(s.points[0][0], -math.pi / 2)

    def test_raw_samples_with_no_return_count_as_rays(self):
        s = scan_from_rplidar(0.0, [(0.0, 1.0), (1.0, 0.0), (2.0, 0.1), (3.0, 20.0)])
        self.assertEqual(len(s.points), 1)                   # 0, too near, too far dropped
        self.assertAlmostEqual(s.ray_step_rad, 2 * math.pi / 4)


if __name__ == "__main__":
    unittest.main()
