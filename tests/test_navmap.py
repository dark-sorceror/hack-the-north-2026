"""The map, the route planner and the path follower (navigation/grid.py,
pathplan.py, navigator.py). numpy only; no bridge, no clock."""

from __future__ import annotations

import math
import sys
import unittest
from importlib.util import find_spec
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

HAVE_NUMPY = find_spec("numpy") is not None
if HAVE_NUMPY:
    import numpy as np

    from retriever.navigation.drive import Limits
    from retriever.navigation.grid import OccupancyGrid, distance_field
    from retriever.navigation.navigator import FollowConfig, Mapper, PathFollower
    from retriever.navigation.odometry import integrate_twist
    from retriever.navigation.pathplan import PlannerConfig, build_costmap, plan_path

from retriever.types import Observation, Pose

CHAIR = [(1.6 + dx, 0.5 + dy) for dx in (-0.22, 0.22) for dy in (-0.22, 0.22)]


def wall(x0, y0, x1, y1, step=0.04):
    n = max(2, int(math.dist((x0, y0), (x1, y1)) / step))
    return [(x0 + (x1 - x0) * i / (n - 1), y0 + (y1 - y0) * i / (n - 1)) for i in range(n)]


def room():
    """Walls round -1.5..3.5 x -2..2: rays need something to end on to clear the floor."""
    return (wall(-1.5, -2, 3.5, -2) + wall(3.5, -2, 3.5, 2) + wall(3.5, 2, -1.5, 2)
            + wall(-1.5, 2, -1.5, -2))


def seen_from(grid, pose, world_pts, scans=3):
    """Insert what a lidar at `pose` would return from these world points
    (no occlusion: fine for sparse test worlds)."""
    c, s = math.cos(pose.theta), math.sin(pose.theta)
    base = [((x - pose.x) * c + (y - pose.y) * s, -(x - pose.x) * s + (y - pose.y) * c)
            for x, y in world_pts]
    for _ in range(scans):
        grid.insert_scan(pose, base)


def min_clearance(path, pts):
    best = math.inf
    for (ax, ay), (bx, by) in zip(path, path[1:]):
        for k in range(41):
            px, py = ax + (bx - ax) * k / 40, ay + (by - ay) * k / 40
            best = min(best, min(math.hypot(px - x, py - y) for x, y in pts))
    return best


@unittest.skipUnless(HAVE_NUMPY, "needs numpy")
class TestGrid(unittest.TestCase):
    def test_a_return_marks_its_cell_in_the_right_place(self):
        g = OccupancyGrid()
        g.insert_scan(Pose(0.0, 0.0, math.pi / 2), [(1.0, 0.0)])   # 1 m ahead, facing +y
        self.assertTrue(g.is_occupied(0.0, 1.0))
        self.assertFalse(g.is_occupied(1.0, 0.0))

    def test_rays_clear_what_they_pass_through(self):
        g = OccupancyGrid()
        g.insert((0.0, 0.0), [(1.0, 0.0)])                          # someone stood here...
        self.assertTrue(g.is_occupied(1.0, 0.0))
        g.insert((0.0, 0.0), [(2.5, 0.0)])                          # ...and walked off:
        self.assertFalse(g.is_occupied(1.0, 0.0))                   # one ray through and it's gone
        self.assertTrue(g.is_occupied(2.5, 0.0))
        ix, iy = g.cell(1.0, 0.0)
        self.assertFalse(g.seen_free()[iy, ix])                     # but not yet "seen clear"
        for _ in range(5):
            g.insert((0.0, 0.0), [(2.5, 0.0)])
        self.assertTrue(g.seen_free()[iy, ix])

    def test_a_thin_leg_survives_rays_that_graze_its_cell(self):
        """Half the scans see the leg, half shoot past through its cell: it stays."""
        g = OccupancyGrid()
        for _ in range(10):
            g.insert((0.0, 0.0), [(1.0, 0.0)])
            g.insert((0.0, 0.0), [(3.0, 0.0)])
        self.assertTrue(g.is_occupied(1.0, 0.0))

    def test_the_same_scan_never_clears_its_own_hit(self):
        g = OccupancyGrid()
        g.insert((0.0, 0.0), [(1.0, 0.0), (2.0, 0.01)])             # ray 2 crosses ray 1's cell
        self.assertTrue(g.is_occupied(1.0, 0.0))

    def test_far_returns_are_ignored(self):
        g = OccupancyGrid()
        self.assertEqual(g.insert((0.0, 0.0), [(7.5, 0.0)]), 0)
        self.assertFalse(g.occupied().any())

    def test_distance_field_is_exact(self):
        rng = np.random.default_rng(3)
        occ = rng.random((50, 60)) < 0.015
        d = distance_field(occ, 10)
        ys, xs = np.nonzero(occ)
        for yy in range(0, 50, 3):
            for xx in range(0, 60, 3):
                truth = min(11.0, float(np.sqrt(((ys - yy) ** 2 + (xs - xx) ** 2).min())))
                self.assertAlmostEqual(float(d[yy, xx]), truth, places=4)


@unittest.skipUnless(HAVE_NUMPY, "needs numpy")
class TestPlanner(unittest.TestCase):
    def grid_with(self, pts):
        g = OccupancyGrid()
        seen_from(g, Pose(), pts + room())
        return g

    def test_a_chair_is_one_obstacle(self):
        cm = build_costmap(self.grid_with(CHAIR))
        self.assertTrue(cm.is_lethal(1.6, 0.5))                     # between the legs
        self.assertFalse(cm.is_lethal(1.6, 0.5 - 0.22 - 0.45))      # beside it

    def test_routes_round_a_chair_with_room_to_spare(self):
        cm = build_costmap(self.grid_with(CHAIR))
        r = plan_path(cm, (0.0, 0.0), (2.45, 0.55))
        self.assertTrue(r.ok, r.reason)
        self.assertGreater(min_clearance(r.path, CHAIR), PlannerConfig().lethal_m - 0.02)
        self.assertLess(len(r.path), 8)                              # smoothed, not a staircase
        self.assertEqual(r.path[0], (0.0, 0.0))
        self.assertEqual(r.path[-1], (2.45, 0.55))

    def test_a_gap_too_narrow_is_closed_and_a_wide_one_is_open(self):
        lethal = PlannerConfig().lethal_m
        for gap, open_ in ((2 * lethal - 0.15, False), (2 * lethal + 0.25, True)):
            with self.subTest(gap=gap):
                # a wall across x = 1.5 with one gap centred on y = 0
                pts = wall(1.5, -2.0, 1.5, -gap / 2) + wall(1.5, gap / 2, 1.5, 2.0)
                cm = build_costmap(self.grid_with(pts))
                r = plan_path(cm, (0.0, 0.0), (3.0, 0.0))
                self.assertEqual(r.ok, open_, r.reason)

    def test_a_goal_inside_something_moves_to_the_nearest_spot_it_fits(self):
        cm = build_costmap(self.grid_with(CHAIR))
        r = plan_path(cm, (0.0, 0.0), (1.6, 0.5))
        self.assertTrue(r.ok, r.reason)
        self.assertTrue(r.moved_goal)
        self.assertFalse(cm.is_lethal(*r.goal))
        self.assertLess(math.dist(r.goal, (1.6, 0.5)), PlannerConfig().goal_snap_m + 0.1)

    def test_walled_in_is_no_path_with_a_reason(self):
        ring = [(2.0 + 0.5 * math.cos(a / 20 * math.tau), 0.5 * math.sin(a / 20 * math.tau))
                for a in range(20)]
        g = OccupancyGrid()
        g.insert((0.0, 0.0), ring * 3)
        g.insert((0.0, 0.0), [(2.0 + 0.5 * math.cos(a / 40 * math.tau), 0.5 * math.sin(a / 40 * math.tau))
                              for a in range(40)] * 2)
        cm = build_costmap(g)
        r = plan_path(cm, (0.0, 0.0), (2.0, 0.0))                   # the middle of the ring
        # the goal snaps out of the ring's lethal zone to the outside, or there is no path
        if r.ok:
            self.assertTrue(r.moved_goal)
        else:
            self.assertTrue(r.reason)

    def test_starting_too_close_to_something_plans_a_way_out(self):
        cm = build_costmap(self.grid_with(CHAIR))
        start = (1.6 - 0.22 - 0.25, 0.5)                             # 25 cm from a leg
        self.assertTrue(cm.is_lethal(*start))
        r = plan_path(cm, start, (0.0, -1.0))
        self.assertTrue(r.ok, r.reason)

    def test_is_quick(self):
        cm = build_costmap(self.grid_with(CHAIR))
        r = plan_path(cm, (-1.0, -1.5), (3.0, 1.5))
        self.assertTrue(r.ok, r.reason)
        self.assertLess(r.ms, 250.0)


@unittest.skipUnless(HAVE_NUMPY, "needs numpy")
class TestCameraLayer(unittest.TestCase):
    """Obstacles a camera saw: blocking, expiring, and safe from the lidar."""

    def mapper(self):
        t = {"now": 100.0}
        return Mapper(clock=lambda: t["now"]), t

    def test_camera_points_block_and_then_expire(self):
        m, t = self.mapper()
        self.assertFalse(m.has_map())
        m.add_camera_points(Pose(), [(1.0, 0.0)], hold_s=2.0)     # 1 m ahead
        self.assertTrue(m.has_map())
        self.assertTrue(m.costmap().is_lethal(1.0, 0.0))
        t["now"] += 1.0
        self.assertTrue(m.costmap().is_lethal(1.0, 0.0))          # still fresh
        t["now"] += 1.5                                            # past hold_s
        self.assertFalse(m.costmap().is_lethal(1.0, 0.0))
        self.assertFalse(m.has_map())

    def test_seeing_them_again_refreshes_them(self):
        m, t = self.mapper()
        for _ in range(4):
            m.add_camera_points(Pose(), [(1.0, 0.0)], hold_s=1.0)
            t["now"] += 0.5
        self.assertTrue(m.costmap().is_lethal(1.0, 0.0))

    def test_the_lidar_does_not_erase_them(self):
        """The point of a separate layer: a box below the lidar's plane is
        exactly what its beams pass over."""
        m, _ = self.mapper()
        m.add_camera_points(Pose(), [(1.0, 0.0)], hold_s=60.0)
        for _ in range(8):                                         # the lidar sees only the wall
            m.add_scan(Pose(), [(3.0, 0.0)])
        ix, iy = m.grid.cell(1.0, 0.0)
        self.assertTrue(m.grid.seen_free()[iy, ix])                # the lidar calls it clear...
        self.assertTrue(m.costmap().is_lethal(1.0, 0.0))           # ...the camera still blocks it

    def test_a_route_goes_round_a_camera_only_obstacle(self):
        m, _ = self.mapper()
        box = [(1.5, y / 100) for y in range(-30, 31, 5)]          # a 0.6 m wide box, camera only
        m.add_camera_points(Pose(), box, hold_s=60.0)
        r = m.plan((0.0, 0.0), (3.0, 0.0))
        self.assertTrue(r.ok, r.reason)
        closest = min(math.hypot(px - bx, py - by)
                      for px, py in r.path for bx, by in box)
        self.assertGreater(closest, PlannerConfig().lethal_m - 0.05)

    def test_the_page_shows_them_apart_from_the_lidar(self):
        import base64
        import zlib

        m, _ = self.mapper()
        m.add_camera_points(Pose(), [(1.0, 0.0)], hold_s=60.0)
        for _ in range(3):
            m.add_scan(Pose(), [(3.0, 0.0)])
        v = m.page_view()
        cells = np.frombuffer(zlib.decompress(base64.b64decode(v["data"])), np.uint8)
        self.assertIn(4, set(cells.tolist()))                      # CAMERA, its own state


def drive(ctl, pose, goal, t_max=40.0, dt=0.05, world=None, mapper=None, lag_scan=True):
    """Close the loop with perfect kinematics; re-scan the world every 0.2 s."""
    t, trail = 0.0, [pose]
    next_scan = 0.0
    while t < t_max:
        if mapper is not None and world is not None and t >= next_scan:
            c, s = math.cos(pose.theta), math.sin(pose.theta)
            base = [((x - pose.x) * c + (y - pose.y) * s, -(x - pose.x) * s + (y - pose.y) * c)
                    for x, y in world if math.hypot(x - pose.x, y - pose.y) < 5.5]
            mapper.add_scan(pose, base)
            next_scan = t + 0.2
        a, done = ctl.step_observation(Observation(joints={}, base=pose, t=t), goal)
        if done or ctl.failure:
            return done, pose, trail
        pose = integrate_twist(pose, a.base_vx, 0.0, a.base_wz, dt)
        trail.append(pose)
        t += dt
    return False, pose, trail


@unittest.skipUnless(HAVE_NUMPY, "needs numpy")
class TestFollower(unittest.TestCase):
    def mapper(self):
        t = {"now": 0.0}
        m = Mapper(clock=lambda: t["now"])
        return m, t

    def test_drives_round_the_chair_to_the_spot_behind_it(self):
        m, _ = self.mapper()
        world = CHAIR + room()
        ctl = PathFollower(m, Limits(v_max=0.3, w_max=1.0, pos_tol=0.08))
        done, pose, trail = drive(ctl, Pose(), Pose(2.45, 0.55, 0.0), world=world, mapper=m)
        self.assertTrue(done, ctl.failure or ctl.plan)
        self.assertLess(math.hypot(pose.x - 2.45, pose.y - 0.55), 0.1)
        closest = min(math.hypot(p.x - x, p.y - y) for p in trail for x, y in CHAIR)
        self.assertGreater(closest, 0.32)            # the corners never reached a leg

    def test_gives_up_with_a_reason_when_there_is_no_way(self):
        m, _ = self.mapper()
        # boxed in: a closed ring of wall 0.6 m round the robot
        ring = [(0.6 * math.cos(a / 60 * math.tau), 0.6 * math.sin(a / 60 * math.tau)) for a in range(60)]
        ctl = PathFollower(m, Limits(v_max=0.3, w_max=1.0, pos_tol=0.08),
                           FollowConfig(patience_s=1.5, max_backoffs=0))
        done, _, _ = drive(ctl, Pose(), Pose(3.0, 0.0, 0.0), t_max=10.0, world=ring + room(), mapper=m)
        self.assertFalse(done)
        self.assertIsNotNone(ctl.failure)
        self.assertIn("way there", ctl.failure[0])

    def test_a_carrot_just_inside_the_rotate_angle_still_moves(self):
        """The stall found on the robot: alpha a hair under rotate_deg made the
        alignment ramp zero, and wz = v * curvature made that zero too, so it
        commanded 4 mm/s and no turn and sat there until the goal timed out."""
        m, _ = self.mapper()
        m.add_scan(Pose(), [(x, y) for x, y in room()])
        cfg = FollowConfig()
        ctl = PathFollower(m, Limits(v_max=0.3, w_max=1.0, pos_tol=0.08), cfg)
        for off_deg in (cfg.rotate_deg - 0.1, cfg.rotate_deg - 2.0, cfg.rotate_deg - 8.0):
            with self.subTest(off_deg=off_deg):
                ctl.path = [(0.0, 0.0), (2.0, 0.0)]
                pose = Pose(0.0, 0.0, math.radians(off_deg))   # nose off the route by ~55 deg
                a, _ = ctl.step_observation(
                    Observation(joints={}, base=pose, t=0.0), Pose(2.0, 0.0, 0.0))
                turning = abs(a.base_wz) > 0.05           # it closes the angle...
                driving = a.base_vx > cfg.creep_mps       # ...or it makes real progress
                self.assertTrue(turning or driving,
                                f"stalled at {off_deg:.1f} deg: vx={a.base_vx:.4f} wz={a.base_wz:.4f}")

    def test_backs_off_turning_so_the_retry_starts_from_a_better_angle(self):
        """Backing straight out keeps the heading that got it stuck, so it wedges
        against the next thing along. It should swing the nose toward the route
        as it reverses: a three-point turn."""
        m, _ = self.mapper()
        m.add_scan(Pose(), [(x, y) for x, y in room()])

        class Blocked:                                   # the Pi's bubble, refusing
            state = "blocked"

        cfg = FollowConfig(backoff_after_s=0.5, backoff_s=1.0)
        ctl = PathFollower(m, Limits(v_max=0.3, w_max=1.0, pos_tol=0.08), cfg,
                           bubble=lambda: Blocked())
        pose, goal = Pose(0.0, 0.0, 0.0), Pose(1.0, 1.4, 0.0)   # route heads off to the left
        t = 0.0
        for i in range(20):                              # blocked -> backoff starts
            t = i * 0.1
            ctl.step_observation(Observation(joints={}, base=pose, t=t), goal)
            if ctl._backoff_until is not None:
                break
        self.assertIsNotNone(ctl._backoff_until, "never backed off")
        act, _ = ctl.step_observation(                   # the step AFTER it decides to
            Observation(joints={}, base=pose, t=t + 0.1), goal)
        self.assertLess(act.base_vx, 0.0, "backed off without reversing")
        self.assertGreater(act.base_wz, 0.05,
                           f"reversed straight instead of turning toward the route: {act}")

        off = FollowConfig(backoff_after_s=0.5, backoff_s=1.0, reverse_to_turn=False)
        ctl2 = PathFollower(m, Limits(v_max=0.3, w_max=1.0, pos_tol=0.08), off,
                            bubble=lambda: Blocked())
        t = 0.0
        for i in range(20):
            t = i * 0.1
            ctl2.step_observation(Observation(joints={}, base=pose, t=t), goal)
            if ctl2._backoff_until is not None:
                break
        act2, _ = ctl2.step_observation(
            Observation(joints={}, base=pose, t=t + 0.1), goal)
        self.assertLess(act2.base_vx, 0.0)
        self.assertEqual(act2.base_wz, 0.0, "reverse_to_turn=False should reverse straight")

    def test_backs_off_when_the_bubble_refuses_every_other_tick(self):
        """The limit cycle seen on the robot: refused -> stop -> a stopped robot
        is safe -> bubble clear -> try again -> refused. Blocked and clear ticks
        alternated, so a symmetric counter never reached the backoff threshold
        and it flickered between "following" and "the bubble stopped me"."""
        m, _ = self.mapper()
        m.add_scan(Pose(), [(x, y) for x, y in room()])
        flip = {"n": 0}

        class Flapping:
            @property
            def state(self):
                flip["n"] += 1
                return "blocked" if flip["n"] % 2 else "clear"

        ctl = PathFollower(m, Limits(v_max=0.3, w_max=1.0, pos_tol=0.08),
                           FollowConfig(backoff_after_s=0.8),
                           bubble=lambda: Flapping())
        pose, goal = Pose(0.0, 0.0, 0.0), Pose(1.0, 1.4, 0.0)
        for i in range(120):                              # 6 s at 20 Hz
            ctl.step_observation(Observation(joints={}, base=pose, t=i * 0.05), goal)
            if ctl._backoff_until is not None:
                break
        self.assertIsNotNone(ctl._backoff_until,
                             "never backed off: blocked and clear ticks cancelled out")

    def test_sticks_to_a_route_instead_of_flipping_between_equal_ones(self):
        """Two ways round are often within a few percent, and scan noise flips
        the winner twice a second: the robot turns one way, then the other, and
        gains no ground. A committed route is kept unless clearly beaten."""
        m, _ = self.mapper()
        world = CHAIR + room()
        m.add_scan(Pose(), [(x, y) for x, y in world])
        ctl = PathFollower(m, Limits(v_max=0.3, w_max=1.0, pos_tol=0.08))
        pose, goal = Pose(0.0, 0.0, 0.0), Pose(2.45, 0.55, 0.0)
        ctl.step_observation(Observation(joints={}, base=pose, t=0.0), goal)
        first = list(ctl.path or [])
        self.assertTrue(first, "no route at all")
        side = lambda path: sum(y for _, y in path)      # which way round it goes

        flips = 0
        for i in range(1, 25):                            # 12 s of re-planning
            m.add_scan(Pose(), [(x, y) for x, y in world])   # same room, fresh scans
            ctl._planned_at = -math.inf                   # force a replan every step
            ctl.step_observation(Observation(joints={}, base=pose, t=i * 0.5), goal)
            if ctl.path and (side(ctl.path) > 0) != (side(first) > 0):
                flips += 1
                first = list(ctl.path)
        self.assertLessEqual(flips, 1, f"route flipped sides {flips} times while standing still")

    def test_waits_when_the_map_goes_stale(self):
        m, clock = self.mapper()
        m.add_scan(Pose(), [(x, y) for x, y in room()])
        clock["now"] = 5.0                                   # no scan for 5 s
        ctl = PathFollower(m, Limits(), FollowConfig(stale_scan_s=1.0))
        a, done = ctl.step_observation(Observation(joints={}, base=Pose(), t=0.0), Pose(1.0, 0.0, 0.0))
        self.assertFalse(done)
        self.assertEqual((a.base_vx, a.base_wz), (0.0, 0.0))
        self.assertEqual(ctl.plan.status, "waiting")

    def test_page_view_round_trips(self):
        import base64
        import zlib

        m, _ = self.mapper()
        for _ in range(3):
            m.add_scan(Pose(), CHAIR + room())
        v = m.page_view()
        cells = np.frombuffer(zlib.decompress(base64.b64decode(v["data"])), np.uint8)
        self.assertEqual(cells.size, v["w"] * v["h"])
        self.assertTrue((cells == 2).any() and (cells == 1).any())


if __name__ == "__main__":
    unittest.main()
