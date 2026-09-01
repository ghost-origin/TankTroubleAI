"""A* downstream polyline refinement for clearance, independent of tactics."""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import numpy as np

Point = Tuple[float, float]


def _distance(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _sample_clearance(a: Point, b: Point, free_dist: np.ndarray,
                      min_clearance_px: float) -> Tuple[bool, float, float]:
    """Return segment validity plus its minimum and mean distance from walls."""
    h, w = free_dist.shape
    length = _distance(a, b)
    samples = max(1, int(math.ceil(length / 2.0)))
    values = []
    for i in range(samples + 1):
        t = i / samples
        x = int(round(a[0] + (b[0] - a[0]) * t))
        y = int(round(a[1] + (b[1] - a[1]) * t))
        if x < 0 or y < 0 or x >= w or y >= h:
            return False, 0.0, 0.0
        clearance = float(free_dist[y, x])
        if clearance < min_clearance_px:
            return False, clearance, clearance
        values.append(clearance)
    return True, min(values), sum(values) / len(values)


def _candidate_offsets(radius_px: float, step_px: float) -> List[Tuple[float, float]]:
    extent = int(math.floor(radius_px / step_px))
    offsets = []
    for gy in range(-extent, extent + 1):
        for gx in range(-extent, extent + 1):
            dx, dy = gx * step_px, gy * step_px
            if dx * dx + dy * dy <= radius_px * radius_px + 1e-9:
                offsets.append((dx, dy))
    return sorted(offsets, key=lambda p: p[0] * p[0] + p[1] * p[1])


def refine_polyline_clearance(path: Sequence[Point], free_dist: np.ndarray,
                              min_clearance_px: float,
                              search_radius_px: float = 20.0,
                              search_step_px: float = 5.0,
                              passes: int = 2) -> List[Point]:
    """Move only interior A* waypoints toward corridor centres.

    The endpoints and the route's LOS topology remain fixed.  Each candidate must
    retain two collision-free centreline segments, so this is post-processing of
    an A* route rather than a second global planner.
    """
    if len(path) < 3:
        return list(path)

    refined = list(path)
    offsets = _candidate_offsets(search_radius_px, search_step_px)
    for _ in range(max(1, passes)):
        changed = False
        for index in range(1, len(refined) - 1):
            anchor = path[index]
            before, current, after = refined[index - 1], refined[index], refined[index + 1]
            best = None
            for dx, dy in offsets:
                candidate = (anchor[0] + dx, anchor[1] + dy)
                left_ok, left_min, left_mean = _sample_clearance(
                    before, candidate, free_dist, min_clearance_px)
                if not left_ok:
                    continue
                right_ok, right_min, right_mean = _sample_clearance(
                    candidate, after, free_dist, min_clearance_px)
                if not right_ok:
                    continue
                shift = _distance(anchor, candidate)
                # Prefer the route bottleneck's clearance, then its average
                # clearance; a small movement cost avoids aimless jitter.
                score = min(left_min, right_min) * 100.0
                score += (left_mean + right_mean) * 0.5 - shift * 0.15
                key = (score, -shift, -abs(dx), -abs(dy))
                if best is None or key > best[0]:
                    best = (key, candidate)
            if best is not None and _distance(current, best[1]) > 1e-6:
                refined[index] = best[1]
                changed = True
        if not changed:
            break
    return refined
