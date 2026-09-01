# -*- coding: utf-8 -*-
"""Constant-velocity Kalman path predictor tests."""
from __future__ import annotations

import os
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PYTHON_ROOT = os.path.join(ROOT, "python")
if PYTHON_ROOT not in sys.path:
    sys.path.insert(0, PYTHON_ROOT)

from kalman_path_predictor import ConstantVelocityKalman2D


class KalmanPathPredictorTests(unittest.TestCase):
    def test_constant_velocity_forecast(self):
        predictor = ConstantVelocityKalman2D()
        for i in range(11):
            t = i * 0.1
            predictor.update(t, 100.0 + 20.0 * t, 80.0 - 10.0 * t, 20.0, -10.0)

        x, y = predictor.forecast(0.25)

        self.assertAlmostEqual(x, 125.0, delta=1.0)
        self.assertAlmostEqual(y, 67.5, delta=1.0)

    def test_forecast_path_includes_now_and_horizon(self):
        predictor = ConstantVelocityKalman2D()
        predictor.update(0.0, 10.0, 20.0, 4.0, 0.0)

        path = predictor.forecast_path(0.25, step_s=0.05)

        self.assertEqual(len(path), 6)
        self.assertAlmostEqual(path[0][0], 10.0)
        self.assertAlmostEqual(path[-1][0], 11.0)


if __name__ == "__main__":
    unittest.main()
