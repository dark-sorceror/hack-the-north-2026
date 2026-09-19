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

    from retriever.navigation.grid import OccupancyGrid, distance_field

from retriever.types import Pose


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


if __name__ == "__main__":
    unittest.main()
