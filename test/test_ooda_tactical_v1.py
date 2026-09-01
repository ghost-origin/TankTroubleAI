# -*- coding: utf-8 -*-
"""Focused regression tests for OODA Tactical V1 target selection."""
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
    GRID_PX, MAP_PX, TACTICAL_MODE_REAR_ONLY, TACTICAL_MODE_V1,
    TARGET_SWITCH_DISTANCE_PX, TARGET_SWITCH_PENALTY_PX, plan,
)


def synthetic_map(barrier_x=None):
    raw = np.zeros((MAP_PX, MAP_PX), dtype=bool)
    raw[:2, :] = True
    raw[-2:, :] = True
    raw[:, :2] = True
    raw[:, -2:] = True
    if barrier_x is not None:
        raw[2:-2, barrier_x:barrier_x + GRID_PX] = True
    blocked = raw.reshape(MAP_PX // GRID_PX, GRID_PX, MAP_PX // GRID_PX, GRID_PX).any(axis=(1, 3))
    return raw, blocked


class OODATacticalV1Tests(unittest.TestCase):
    def test_open_map_candidate_reachability_is_high(self):
        raw, blocked = synthetic_map()
        result = plan((450.0, 285.0), (285.0, 285.0), 0.0, raw, blocked,
                      tactical_mode=TACTICAL_MODE_V1)
        self.assertTrue(result.success)
        self.assertGreater(result.candidate_count, 40)
        self.assertGreaterEqual(result.reachable_candidates / result.candidate_count, 0.95)

    def test_rear_unreachable_falls_back_to_non_rear_candidate(self):
        raw, blocked = synthetic_map(barrier_x=330)
        result = plan((460.0, 285.0), (250.0, 285.0), 0.0, raw, blocked,
                      tactical_mode=TACTICAL_MODE_V1)
        self.assertTrue(result.success)
        self.assertEqual(result.rear_reachable_candidates, 0)
        self.assertIn(result.target_type, ("flank", "front", "reposition"))
        baseline = plan((460.0, 285.0), (250.0, 285.0), 0.0, raw, blocked,
                        tactical_mode=TACTICAL_MODE_REAR_ONLY)
        self.assertFalse(baseline.success)

    def test_target_hysteresis_keeps_nearby_ooda_choice_stable(self):
        raw, blocked = synthetic_map()
        first = plan((450.0, 285.0), (285.0, 285.0), 0.0, raw, blocked,
                     tactical_mode=TACTICAL_MODE_V1)
        second = plan((450.0, 285.0), (291.0, 285.0), 0.0, raw, blocked,
                      tactical_mode=TACTICAL_MODE_V1,
                      preferred_rel_deg=first.relative_deg,
                      preferred_target=first.target)
        self.assertTrue(first.success)
        self.assertTrue(second.success)
        self.assertEqual(second.target_type, first.target_type)
        self.assertLessEqual(
            ((second.target[0] - first.target[0]) ** 2 + (second.target[1] - first.target[1]) ** 2) ** 0.5,
            TARGET_SWITCH_DISTANCE_PX,
        )
        switched = plan((450.0, 285.0), (285.0, 285.0), 0.0, raw, blocked,
                        tactical_mode=TACTICAL_MODE_V1,
                        preferred_target=(50.0, 50.0))
        self.assertTrue(switched.success)
        self.assertGreaterEqual(switched.switch_cost, TARGET_SWITCH_PENALTY_PX)


if __name__ == "__main__":
    unittest.main()
