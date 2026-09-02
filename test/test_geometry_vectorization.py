# -*- coding: utf-8 -*-
"""Exact-equivalence tests for P1/P2 geometry batching."""
from __future__ import annotations

import math
import os
import sys
import unittest

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PYTHON_ROOT = os.path.join(ROOT, "python")
if PYTHON_ROOT not in sys.path:
    sys.path.insert(0, PYTHON_ROOT)

from navigation_mvp import (
    TANK_BODY_RADIUS_PX,
    _dist,
    _lerp,
    _path_headings,
    _wrap_rad,
    footprint_clearance,
    footprint_clearances,
    footprint_collides,
    footprint_collides_batch,
    swept_rectangle_path_clear,
)


def scalar_swept_path_clear(path, raw_wall, shrink=0.0):
    """Pre-P1 reference implementation retained inside the test only."""
    if len(path) < 2:
        return False
    headings = _path_headings(path)
    for p, th in zip(path, headings):
        if footprint_collides(raw_wall, p[0], p[1], th, shrink=shrink):
            return False
    for a, b, ta, tb in zip(path, path[1:], headings, headings[1:]):
        move = _dist(a, b)
        turn_arc = abs(_wrap_rad(tb - ta)) * TANK_BODY_RADIUS_PX
        n = max(1, int(math.ceil(max(move, turn_arc) / 1.0)))
        for i in range(1, n):
            u = i / n
            p = _lerp(a, b, u)
            th = ta + _wrap_rad(tb - ta) * u
            if footprint_collides(raw_wall, p[0], p[1], th, shrink=shrink):
                return False
    return True


class GeometryVectorizationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rng = np.random.default_rng(20260902)
        cls.raw_wall = np.zeros((570, 570), dtype=np.bool_)
        cls.raw_wall[:8, :] = True
        cls.raw_wall[-8:, :] = True
        cls.raw_wall[:, :8] = True
        cls.raw_wall[:, -8:] = True
        # Deterministic wall strips plus sparse pixels exercise collisions,
        # open space, rounding boundaries and early out-of-bounds cases.
        cls.raw_wall[70:500, 180:187] = True
        cls.raw_wall[250:258, 80:500] = True
        ys = cls.rng.integers(8, 562, size=500)
        xs = cls.rng.integers(8, 562, size=500)
        cls.raw_wall[ys, xs] = True
        # A deterministic float32 distance-like field is sufficient for exact
        # index/gather equivalence against the scalar clearance implementation.
        yy, xx = np.indices(cls.raw_wall.shape)
        cls.free_dist = ((xx * 0.37 + yy * 0.19) % 43.0).astype(np.float32)

    def test_batch_collision_matches_scalar_for_random_poses(self):
        # 50,000 poses x two shrink settings = 100,000 strict old/new checks.
        pose_count = 50_000
        poses = np.column_stack((
            self.rng.uniform(-20.0, 590.0, pose_count),
            self.rng.uniform(-20.0, 590.0, pose_count),
            self.rng.uniform(-math.pi, math.pi, pose_count),
        ))
        for shrink in (0.0, 2.5):
            expected = np.asarray([
                footprint_collides(self.raw_wall, *pose, shrink=shrink)
                for pose in poses
            ], dtype=np.bool_)
            actual = footprint_collides_batch(
                self.raw_wall, poses, shrink=shrink, chunk_size=73)
            np.testing.assert_array_equal(actual, expected)

    def test_batch_collision_matches_scalar_for_nondefault_edge_step(self):
        poses = np.column_stack((
            self.rng.uniform(0.0, 570.0, 500),
            self.rng.uniform(0.0, 570.0, 500),
            self.rng.uniform(-math.pi, math.pi, 500),
        ))
        expected = np.asarray([
            footprint_collides(self.raw_wall, *pose, edge_step=2.25, shrink=1.0)
            for pose in poses
        ], dtype=np.bool_)
        actual = footprint_collides_batch(
            self.raw_wall, poses, edge_step=2.25, shrink=1.0)
        np.testing.assert_array_equal(actual, expected)

    def test_vectorized_sweep_matches_pre_p1_reference(self):
        for _ in range(120):
            start = self.rng.uniform(25.0, 545.0, size=2)
            points = [tuple(start)]
            for _segment in range(4):
                step = self.rng.uniform(-45.0, 45.0, size=2)
                points.append(tuple(np.asarray(points[-1]) + step))
            for shrink in (0.0, 2.5):
                self.assertEqual(
                    swept_rectangle_path_clear(points, self.raw_wall, shrink=shrink),
                    scalar_swept_path_clear(points, self.raw_wall, shrink=shrink),
                )

    def test_joint_clearance_matches_scalar_candidates(self):
        paths = []
        for count in (2, 3, 7, 19, 44, 61):
            x = np.linspace(40.0, 520.0, count)
            y = 180.0 + 55.0 * np.sin(np.linspace(0.0, math.pi, count))
            paths.append(list(zip(x, y)))
        paths.extend([[], [(100.0, 100.0)]])
        expected = [footprint_clearance(path, self.free_dist) for path in paths]
        actual = footprint_clearances(paths, self.free_dist)
        np.testing.assert_allclose(actual, expected, rtol=0.0, atol=0.0)

    def test_empty_batches(self):
        self.assertEqual(footprint_collides_batch(
            self.raw_wall, np.empty((0, 3))).shape, (0,))
        self.assertEqual(footprint_clearances([], self.free_dist), [])


if __name__ == "__main__":
    unittest.main()
