# -*- coding: utf-8 -*-
"""P1E Safety Contract regression tests."""
from __future__ import annotations

import os
import sys
import unittest
from unittest import mock

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PYTHON_ROOT = os.path.join(ROOT, "python")
if PYTHON_ROOT not in sys.path:
    sys.path.insert(0, PYTHON_ROOT)

import navigation_mvp
from navigation_mvp import GRID_PX, MAP_PX, TACTICAL_MODE_REAR_ONLY, PlanResult, plan


def open_synthetic_map():
    raw = np.zeros((MAP_PX, MAP_PX), dtype=bool)
    raw[:2, :] = True
    raw[-2:, :] = True
    raw[:, :2] = True
    raw[:, -2:] = True
    blocked = raw.reshape(MAP_PX // GRID_PX, GRID_PX, MAP_PX // GRID_PX, GRID_PX).any(axis=(1, 3))
    free_dist = np.full(raw.shape, 99.0, dtype=float)
    free_dist[raw] = 0.0
    return raw, blocked, free_dist


class P1ESafetyContractTests(unittest.TestCase):
    def test_plan_result_success_without_validation_fails_closed(self):
        result = PlanResult(True, [(1.0, 1.0), (2.0, 2.0)], None, None, 1.4, 0.0, 0.1,
                            reason="bad_branch", target_type="fallback")
        self.assertFalse(result.success)
        self.assertEqual(result.path, [])
        self.assertEqual(result.reason, "bad_branch")
        self.assertEqual(result.validation_reason, "missing_path_validation")
        self.assertEqual(result.executed_path_points, 0)

    def test_fallback_tube_failure_does_not_return_raw_astar(self):
        raw, blocked, free_dist = open_synthetic_map()
        with mock.patch.object(navigation_mvp, "build_tube", return_value=None):
            result = plan((460.0, 285.0), (250.0, 285.0), 0.0, raw, blocked,
                          free_dist=free_dist, tactical_mode=TACTICAL_MODE_REAR_ONLY)

        self.assertFalse(result.success)
        self.assertEqual(result.reason, "fallback_tube_invalid")
        self.assertEqual(result.path_source, "fallback")
        self.assertFalse(result.path_validated)
        self.assertEqual(result.validation_reason, "tube_invalid")
        self.assertGreater(result.raw_path_points, 0)
        self.assertEqual(result.executed_path_points, 0)
        self.assertEqual(result.path, [])

    def test_validated_plan_sets_contract_fields(self):
        raw, blocked, free_dist = open_synthetic_map()
        safe_path = [(460.0, 285.0), (360.0, 285.0), (250.0, 285.0)]
        with mock.patch.object(navigation_mvp, "build_tube", return_value=safe_path):
            result = plan((460.0, 285.0), (250.0, 285.0), 0.0, raw, blocked,
                          free_dist=free_dist, tactical_mode=TACTICAL_MODE_REAR_ONLY)

        self.assertTrue(result.success)
        self.assertTrue(result.path_validated)
        self.assertEqual(result.validation_reason, "tube_ok")
        self.assertGreater(result.raw_path_points, 0)
        self.assertEqual(result.executed_path_points, len(result.path))


if __name__ == "__main__":
    unittest.main()
