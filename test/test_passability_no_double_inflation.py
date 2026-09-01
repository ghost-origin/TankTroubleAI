# -*- coding: utf-8 -*-
"""Regression tests for topology planning without duplicate body inflation."""
from __future__ import annotations

import os
import sys
import unittest

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PYTHON_ROOT = os.path.join(ROOT, "python")
if PYTHON_ROOT not in sys.path:
    sys.path.insert(0, PYTHON_ROOT)

from navigation_mvp import (
    A_STAR_TOPOLOGY_RADIUS_PX,
    astar,
    distance_transform_edt,
    planning_grid_from_free_dist,
    point_to_cell,
    swept_rectangle_path_clear,
)


def vertical_corridor(left_x: int, right_x: int) -> np.ndarray:
    raw = np.zeros((120, 120), dtype=bool)
    raw[:2, :] = True
    raw[-2:, :] = True
    raw[:, :2] = True
    raw[:, -2:] = True
    raw[:, left_x] = True
    raw[:, right_x] = True
    return raw


class NoDoubleInflationTests(unittest.TestCase):
    def test_production_topology_padding_is_not_full_body_inflation(self):
        self.assertEqual(A_STAR_TOPOLOGY_RADIUS_PX, 6.0)
        self.assertLess(A_STAR_TOPOLOGY_RADIUS_PX, 10.0)

    def test_exactly_passable_corridor_is_not_closed_by_coarse_astar(self):
        raw = vertical_corridor(40, 64)
        free_dist = distance_transform_edt(~raw)
        old_inflated = planning_grid_from_free_dist(free_dist, 5, 10.0)
        topology_only = planning_grid_from_free_dist(free_dist, 5, 0.0)
        start = (52.0, 20.0)
        goal = (52.0, 100.0)

        old_start = point_to_cell(start, 5, old_inflated)
        new_start = point_to_cell(start, 5, topology_only)
        new_goal = point_to_cell(goal, 5, topology_only)

        self.assertTrue(old_inflated[old_start[1], old_start[0]])
        self.assertIsNotNone(astar(new_start, new_goal, topology_only))
        self.assertTrue(swept_rectangle_path_clear([start, goal], raw))

    def test_topology_route_still_fails_when_exact_body_does_not_fit(self):
        raw = vertical_corridor(42, 62)
        free_dist = distance_transform_edt(~raw)
        topology_only = planning_grid_from_free_dist(free_dist, 5, 0.0)
        start = (52.0, 20.0)
        goal = (52.0, 100.0)

        self.assertIsNotNone(astar(
            point_to_cell(start, 5, topology_only),
            point_to_cell(goal, 5, topology_only),
            topology_only,
        ))
        self.assertFalse(swept_rectangle_path_clear([start, goal], raw))


if __name__ == "__main__":
    unittest.main()
