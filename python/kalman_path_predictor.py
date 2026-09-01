# -*- coding: utf-8 -*-
"""Small constant-velocity Kalman predictor for runtime path experiments."""
from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np

Point = Tuple[float, float]


class ConstantVelocityKalman2D:
    def __init__(self, process_accel_sigma: float = 55.0,
                 position_sigma: float = 2.5,
                 velocity_sigma: float = 18.0):
        self.process_accel_sigma = float(process_accel_sigma)
        self.position_sigma = float(position_sigma)
        self.velocity_sigma = float(velocity_sigma)
        self.state = np.zeros(4, dtype=float)
        self.covariance = np.eye(4, dtype=float)
        self.last_t: Optional[float] = None
        self.initialized = False

    @staticmethod
    def _transition(dt: float) -> np.ndarray:
        return np.array([
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ], dtype=float)

    def _predict(self, dt: float) -> None:
        if dt <= 0.0:
            return
        f = self._transition(dt)
        q = self.process_accel_sigma ** 2
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        process = q * np.array([
            [dt4 / 4.0, 0.0, dt3 / 2.0, 0.0],
            [0.0, dt4 / 4.0, 0.0, dt3 / 2.0],
            [dt3 / 2.0, 0.0, dt2, 0.0],
            [0.0, dt3 / 2.0, 0.0, dt2],
        ], dtype=float)
        self.state = f @ self.state
        self.covariance = f @ self.covariance @ f.T + process

    def update(self, t: float, x: float, y: float,
               vx: Optional[float] = None, vy: Optional[float] = None) -> None:
        t = float(t)
        if not self.initialized:
            self.state[:] = (
                float(x), float(y), float(vx or 0.0), float(vy or 0.0)
            )
            self.covariance = np.diag([
                self.position_sigma ** 2,
                self.position_sigma ** 2,
                self.velocity_sigma ** 2,
                self.velocity_sigma ** 2,
            ])
            self.last_t = t
            self.initialized = True
            return

        dt = max(0.0, min(0.5, t - float(self.last_t)))
        self._predict(dt)
        self.last_t = t

        if vx is None or vy is None:
            h = np.array([[1.0, 0.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0, 0.0]], dtype=float)
            z = np.array([x, y], dtype=float)
            r = np.eye(2, dtype=float) * self.position_sigma ** 2
        else:
            h = np.eye(4, dtype=float)
            z = np.array([x, y, vx, vy], dtype=float)
            r = np.diag([
                self.position_sigma ** 2,
                self.position_sigma ** 2,
                self.velocity_sigma ** 2,
                self.velocity_sigma ** 2,
            ])
        innovation = z - h @ self.state
        innovation_cov = h @ self.covariance @ h.T + r
        gain = np.linalg.solve(innovation_cov.T, (self.covariance @ h.T).T).T
        self.state = self.state + gain @ innovation
        identity = np.eye(4, dtype=float)
        self.covariance = (identity - gain @ h) @ self.covariance

    def forecast(self, horizon_s: float) -> Point:
        if not self.initialized:
            raise RuntimeError("predictor has no measurement")
        horizon = max(0.0, float(horizon_s))
        state = self._transition(horizon) @ self.state
        return float(state[0]), float(state[1])

    def forecast_path(self, horizon_s: float, step_s: float = 0.05) -> List[Point]:
        if not self.initialized:
            return []
        horizon = max(0.0, float(horizon_s))
        step = max(0.01, float(step_s))
        count = max(1, int(np.ceil(horizon / step)))
        return [self.forecast(horizon * i / count) for i in range(count + 1)]
