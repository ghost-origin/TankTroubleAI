# -*- coding: utf-8 -*-
"""P0-2 receding-horizon planner regression tests."""
from __future__ import annotations

import os
import sys
import unittest
import math

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PYTHON_ROOT = os.path.join(ROOT, "python")
if PYTHON_ROOT not in sys.path:
    sys.path.insert(0, PYTHON_ROOT)

from navigation_mvp import (
    GRID_PX,
    MAP_PX,
    WINDOW_MAX_PX,
    WINDOW_TARGET_PX,
    build_execution_window,
    extract_path_prefix,
    path_length,
    plan_to_goal,
)


def open_synthetic_map():
    raw = np.zeros((MAP_PX, MAP_PX), dtype=bool)
    raw[:2, :] = True
    raw[-2:, :] = True
    raw[:, :2] = True
    raw[:, -2:] = True
    blocked = raw.reshape(
        MAP_PX // GRID_PX, GRID_PX, MAP_PX // GRID_PX, GRID_PX
    ).any(axis=(1, 3))
    free_dist = np.full(raw.shape, 99.0, dtype=float)
    free_dist[raw] = 0.0
    return raw, blocked, free_dist


class RecedingHorizonTests(unittest.TestCase):
    def test_extract_prefix_interpolates_across_waypoints(self):
        path = [(0.0, 0.0), (80.0, 0.0), (80.0, 100.0)]
        prefix = extract_path_prefix(path, 120.0)
        self.assertEqual(prefix, [(0.0, 0.0), (80.0, 0.0), (80.0, 40.0)])
        self.assertAlmostEqual(path_length(prefix), 120.0)

    def test_long_goal_returns_validated_local_window(self):
        raw, blocked, free_dist = open_synthetic_map()
        goal = (470.0, 285.0)
        result = plan_to_goal((80.0, 285.0), goal, raw, blocked, free_dist=free_dist)

        self.assertTrue(result.success)
        self.assertTrue(result.path_validated)
        self.assertEqual(result.target, goal)
        self.assertFalse(result.window_goal_reached)
        self.assertEqual(result.validation_reason, "tube_ok")
        self.assertGreater(result.global_path_length, WINDOW_MAX_PX)
        self.assertLessEqual(result.window_path_length, WINDOW_TARGET_PX + 1e-6)

    def test_short_goal_keeps_full_validated_route(self):
        raw, blocked, free_dist = open_synthetic_map()
        goal = (150.0, 285.0)
        result = plan_to_goal((80.0, 285.0), goal, raw, blocked, free_dist=free_dist)

        self.assertTrue(result.success)
        self.assertTrue(result.window_goal_reached)
        self.assertEqual(result.validation_reason, "tube_ok")
        self.assertAlmostEqual(result.window_path_length, result.global_path_length)

    def test_far_invalid_geometry_does_not_reject_current_window(self):
        raw, _, free_dist = open_synthetic_map()
        raw[120:150, 235:265] = True
        free_dist[raw] = 0.0
        global_path = [(50.0, 50.0), (250.0, 50.0), (250.0, 220.0)]

        window = build_execution_window(global_path, free_dist, raw)

        self.assertIsNotNone(window)
        self.assertFalse(window.goal_reached)
        self.assertLessEqual(window.window_path_length, WINDOW_TARGET_PX + 1e-6)

    def test_window_collision_fails_closed(self):
        raw, _, free_dist = open_synthetic_map()
        raw[35:65, 70:100] = True
        free_dist[raw] = 0.0
        global_path = [(50.0, 50.0), (300.0, 50.0)]

        self.assertIsNone(build_execution_window(global_path, free_dist, raw))

    def test_emergency_40px_window_can_stop_before_later_obstacle(self):
        raw, _, free_dist = open_synthetic_map()
        raw[35:65, 118:145] = True
        free_dist[raw] = 0.0
        global_path = [(50.0, 50.0), (300.0, 50.0)]

        window = build_execution_window(global_path, free_dist, raw)

        self.assertIsNotNone(window)
        self.assertEqual(window.target_length, 40.0)
        self.assertLessEqual(window.window_path_length, 40.0 + 1e-6)

    def test_heading_discontinuity_rejects_window(self):
        raw, _, free_dist = open_synthetic_map()
        global_path = [(50.0, 50.0), (300.0, 50.0)]

        self.assertIsNone(
            build_execution_window(global_path, free_dist, raw, start_heading=math.pi)
        )


if __name__ == "__main__":
    unittest.main()
