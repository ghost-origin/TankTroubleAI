# -*- coding: utf-8 -*-
"""P1/P2 向量化改造的等价性验证 + 性能对比（不修改任何规划决策）。

验证项：
1. footprint_collides_batch vs footprint_collides：--n-pose 个随机位姿（默认 100000）结果一致
2. swept_rectangle_path_clear（新批量） vs legacy 逐点：随机折线 + 真实 A* 路径一致
3. footprint_clearances vs 逐条 footprint_clearance：15 组 Bezier 候选一致
4. build_tube（新） vs legacy build_tube：同输入返回逐点相同的候选
5. plan() 整体（monkey-patch legacy 对照）路径输出一致
6. 性能对比：批量 vs 逐点的倍率（collides / clearances / build_tube / plan）
"""
from __future__ import annotations

import argparse
import math
import os
import random
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'python'))

import numpy as np
import navigation_mvp as M
from navigation_mvp import (
    GRID_PX, TACTICAL_MODE_CHASE,
    TANK_BODY_RADIUS_PX,
    FOOTPRINT_EDGE_STEP_PX, FOOTPRINT_SWEEP_STEP_PX, FOOTPRINT_WRAP_SHRINK,
    TUBE_CLEARANCE_PX, TUBE_SAMPLE_STEP, TUBE_MIN_CLEARANCE,
    _path_headings, _dist, _lerp, _wrap_rad, tank_footprint_corners,
    load_polys, build_maps, astar, simplify_cells, refine_astar_polyline,
    extract_path_prefix, point_to_cell, nearest_free, cell_to_point,
)


# ---------------------------------------------------------------- legacy 逐点实现
def legacy_footprint_collides(raw_wall, x, y, heading,
                              edge_step=FOOTPRINT_EDGE_STEP_PX, shrink=0.0):
    corners = tank_footprint_corners(x, y, heading, shrink=shrink)
    h, w = raw_wall.shape
    for cx, cy in corners:
        ix, iy = int(round(cx)), int(round(cy))
        if ix < 0 or iy < 0 or ix >= w or iy >= h or raw_wall[iy, ix]:
            return True
    for a, b in zip(corners, corners[1:] + corners[:1]):
        d = _dist(a, b)
        n = max(1, int(math.ceil(d / max(edge_step, 1e-6))))
        for i in range(n + 1):
            x0, y0 = _lerp(a, b, i / n)
            ix, iy = int(round(x0)), int(round(y0))
            if ix < 0 or iy < 0 or ix >= w or iy >= h or raw_wall[iy, ix]:
                return True
    return False


def legacy_swept_rectangle_path_clear(path, raw_wall, shrink=0.0):
    if len(path) < 2:
        return False
    headings = _path_headings(path)
    for p, th in zip(path, headings):
        if legacy_footprint_collides(raw_wall, p[0], p[1], th, shrink=shrink):
            return False
    for a, b, ta, tb in zip(path, path[1:], headings, headings[1:]):
        move = _dist(a, b)
        turn_arc = abs(_wrap_rad(tb - ta)) * TANK_BODY_RADIUS_PX
        n = max(1, int(math.ceil(max(move, turn_arc) / FOOTPRINT_SWEEP_STEP_PX)))
        for i in range(1, n):
            u = i / n
            p = _lerp(a, b, u)
            th = ta + _wrap_rad(tb - ta) * u
            if legacy_footprint_collides(raw_wall, p[0], p[1], th, shrink=shrink):
                return False
    return True


def legacy_footprint_clearance(path, free_dist):
    if free_dist is None or len(path) < 2:
        return 0.0
    h, w = free_dist.shape
    best = float("inf")
    for p, th in zip(path, _path_headings(path)):
        for cx, cy in tank_footprint_corners(p[0], p[1], th):
            ix = min(w - 1, max(0, int(round(cx))))
            iy = min(h - 1, max(0, int(round(cy))))
            best = min(best, float(free_dist[iy, ix]))
    return best if best != float("inf") else 0.0


def legacy_build_tube(path, free_dist, clearance=TUBE_CLEARANCE_PX, raw_wall=None):
    if len(path) < 2:
        return None
    if len(path) < 3:
        candidate = list(path)
        if raw_wall is not None and not legacy_swept_rectangle_path_clear(
                candidate, raw_wall, shrink=FOOTPRINT_WRAP_SHRINK):
            return None
        return candidate
    ranked = []
    for radius_scale in (1.25, 1.1, 1.45, 0.95, 1.7):
        for handle_scale in (1.0, 0.85, 1.15):
            candidate = M._bezier_polyline(path, TUBE_SAMPLE_STEP, radius_scale, handle_scale)
            if candidate is None or len(candidate) < 2:
                continue
            if not M._legacy_tube_coarse_ok(candidate, free_dist):
                continue
            candidate = M.push_off_walls(candidate, free_dist, min_clearance=TUBE_MIN_CLEARANCE)
            clear = legacy_footprint_clearance(candidate, free_dist)
            soft_clearance_penalty = max(0.0, clearance - clear) * 20.0
            cost = M.path_length(candidate) + M.smoothness(candidate) * 18.0
            cost += M.curvature_variation(candidate) * 10.0 + soft_clearance_penalty
            ranked.append((cost, candidate))
    if not ranked:
        return None
    ranked.sort(key=lambda item: item[0])
    if raw_wall is None:
        return ranked[0][1]
    for _cost, candidate in ranked:
        if legacy_swept_rectangle_path_clear(candidate, raw_wall, shrink=FOOTPRINT_WRAP_SHRINK):
            return candidate
    return None


# ---------------------------------------------------------------- 数据准备
def load_maze_csv(path):
    maze = []
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            maze.append([int(v) for v in line.split(',')])
    return maze


def random_polyline(free_cells, blocked, n_steps=10):
    start = random.choice(free_cells)
    pts = [cell_to_point(start, GRID_PX)]
    cur = start
    for _ in range(n_steps):
        options = []
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nx, ny = cur[0] + dx, cur[1] + dy
            if (0 <= nx < blocked.shape[1] and 0 <= ny < blocked.shape[0]
                    and not blocked[ny, nx]):
                options.append((nx, ny))
        if not options:
            break
        cur = random.choice(options)
        pts.append(cell_to_point(cur, GRID_PX))
    return pts


def build_real_paths(free_cells, blocked, free_dist, raw_wall, count=6):
    paths = []
    for _ in range(count):
        (sx, sy) = random.choice(free_cells)
        (gx, gy) = random.choice(free_cells)
        me = (sx * GRID_PX + GRID_PX / 2, sy * GRID_PX + GRID_PX / 2)
        goal = (gx * GRID_PX + GRID_PX / 2, gy * GRID_PX + GRID_PX / 2)
        start = point_to_cell(me, GRID_PX, blocked)
        foe_c = point_to_cell(goal, GRID_PX, blocked)
        foe_free = nearest_free(foe_c, blocked, max_r=8)
        p_cells = astar(start, foe_free, blocked)
        if not p_cells:
            continue
        pc = simplify_cells(p_cells, blocked)
        p = [me] + [cell_to_point(c, GRID_PX) for c in pc[1:-1]] + [goal]
        p = refine_astar_polyline(p, free_dist, raw_wall)
        paths.append(extract_path_prefix(p, 170.0))
    return paths


# ---------------------------------------------------------------- 验证与性能
def check_poses(raw_wall, n_pose, rng_seed=20260901, batch=1000):
    rng = random.Random(rng_seed)
    W, H = raw_wall.shape[1], raw_wall.shape[0]
    mismatches = 0
    n_collide = 0
    t0 = time.perf_counter()
    poses = []
    for _ in range(n_pose):
        poses.append((rng.uniform(-60.0, W + 60.0),
                      rng.uniform(-60.0, H + 60.0),
                      rng.uniform(-math.pi, math.pi)))
    for s in range(0, n_pose, batch):
        chunk_poses = poses[s:s + batch]
        arr = np.asarray(chunk_poses, dtype=np.float64)
        new_arr = M.footprint_collides_batch(raw_wall, arr)
        # 逐点 legacy 判定
        old_arr = np.asarray(
            [M.footprint_collides(raw_wall, p[0], p[1], p[2]) for p in chunk_poses],
            dtype=np.bool_)
        if not np.array_equal(new_arr, old_arr):
            bad = np.flatnonzero(new_arr != old_arr)
            for j in bad[:5]:
                p = chunk_poses[j]
                print('  MISMATCH pose', (round(p[0], 3), round(p[1], 3), round(p[2], 6)),
                      'legacy=', old_arr[j], 'batch=', new_arr[j])
            if len(bad) > 5:
                print('  ... %d more' % (len(bad) - 5))
            mismatches += int(len(bad))
        n_collide += int(old_arr.sum())
    dt = time.perf_counter() - t0
    print(f'  poses={n_pose}  collide={n_collide} ({n_collide / n_pose:.1%})  '
          f'mismatch={mismatches}  time={dt:.2f}s  ({dt * 1000 / n_pose:.4f} ms/pose)')
    return mismatches


def bench_collides(raw_wall, n=1000, seed=7):
    rng = random.Random(seed)
    W, H = raw_wall.shape[1], raw_wall.shape[0]
    poses = np.array([[rng.uniform(0, W), rng.uniform(0, H), rng.uniform(-math.pi, math.pi)]
                      for _ in range(n)])
    t0 = time.perf_counter()
    old = [M.footprint_collides(raw_wall, p[0], p[1], p[2]) for p in poses]
    t_old = time.perf_counter() - t0
    t0 = time.perf_counter()
    new = M.footprint_collides_batch(raw_wall, poses)
    t_new = time.perf_counter() - t0
    assert any(old) == bool(new.any()), 'batch collides result differs'
    print(f'  collides x{n}: legacy {t_old * 1000:.2f} ms | batch {t_new * 1000:.2f} ms '
          f'| speedup {t_old / t_new:.1f}x')
    return t_old / t_new


def bench_clearances(paths, free_dist):
    cands = []
    for path in paths:
        for radius_scale in (1.25, 1.1, 1.45, 0.95, 1.7):
            for handle_scale in (1.0, 0.85, 1.15):
                c = M._bezier_polyline(path, TUBE_SAMPLE_STEP, radius_scale, handle_scale)
                if c is None or len(c) < 2 or not M._legacy_tube_coarse_ok(c, free_dist):
                    continue
                cands.append(M.push_off_walls(c, free_dist, min_clearance=TUBE_MIN_CLEARANCE))
    t0 = time.perf_counter()
    old = [legacy_footprint_clearance(c, free_dist) for c in cands]
    t_old = time.perf_counter() - t0
    t0 = time.perf_counter()
    new = M.footprint_clearances(cands, free_dist)
    t_new = time.perf_counter() - t0
    mism = sum(1 for a, b in zip(old, new) if abs(a - b) > 1e-12)
    print(f'  clearances x{len(cands)}: legacy {t_old * 1000:.2f} ms | batch {t_new * 1000:.2f} ms '
          f'| speedup {t_old / max(t_new, 1e-9):.1f}x | mismatch={mism}')
    assert mism == 0, 'footprint_clearances mismatch'
    return t_old / t_new


def check_swept(paths, raw_wall, rng_seed=7):
    rng = random.Random(rng_seed)
    test_paths = list(paths)
    for _ in range(40):
        test_paths.append(random_polyline(free_cells, blocked, n_steps=rng.randint(3, 14)))
    mism = 0
    t_old = t_new = 0.0
    for p in test_paths:
        t0 = time.perf_counter()
        old = legacy_swept_rectangle_path_clear(p, raw_wall, shrink=FOOTPRINT_WRAP_SHRINK)
        t_old += time.perf_counter() - t0
        t0 = time.perf_counter()
        new = M.swept_rectangle_path_clear(p, raw_wall, shrink=FOOTPRINT_WRAP_SHRINK)
        t_new += time.perf_counter() - t0
        if old != new:
            mism += 1
            print('  SWEPT MISMATCH npts=%d legacy=%s new=%s' % (len(p), old, new))
    print(f'  swept x{len(test_paths)}: legacy {t_old * 1000:.2f} ms | batch {t_new * 1000:.2f} ms '
          f'| speedup {t_old / max(t_new, 1e-9):.1f}x | mismatch={mism}')
    return mism


def check_build_tube(paths, free_dist, raw_wall):
    mism = 0
    t_old = t_new = 0.0
    for p in paths:
        t0 = time.perf_counter()
        old = legacy_build_tube(p, free_dist, raw_wall=raw_wall)
        t_old += time.perf_counter() - t0
        t0 = time.perf_counter()
        new = M.build_tube(p, free_dist, raw_wall=raw_wall)
        t_new += time.perf_counter() - t0
        if (old is None) != (new is None):
            mism += 1
            print('  TUBE None-mismatch', len(p), old is None, new is None)
            continue
        if old is None:
            continue
        if len(old) != len(new):
            mism += 1
            print('  TUBE len mismatch', len(old), len(new))
            continue
        d = max(abs(a[0] - b[0]) + abs(a[1] - b[1]) for a, b in zip(old, new))
        if d > 1e-9:
            mism += 1
            print('  TUBE point mismatch max_d=%.3g' % d)
    print(f'  build_tube x{len(paths)}: legacy {t_old * 1000:.2f} ms | new {t_new * 1000:.2f} ms '
          f'| speedup {t_old / max(t_new, 1e-9):.1f}x | mismatch={mism}')
    return mism


def check_plan(free_cells, blocked, free_dist, raw_wall, blocked_turn, count=6, seed=11):
    rng = random.Random(seed)
    cases = []
    for _ in range(count):
        (sx, sy) = rng.choice(free_cells)
        (gx, gy) = rng.choice(free_cells)
        me = (sx * GRID_PX + GRID_PX / 2, sy * GRID_PX + GRID_PX / 2)
        goal = (gx * GRID_PX + GRID_PX / 2, gy * GRID_PX + GRID_PX / 2)
        cases.append((me, goal))
    saved = (M.build_tube, M.swept_rectangle_path_clear, M.footprint_clearances)
    M.build_tube = legacy_build_tube
    M.swept_rectangle_path_clear = legacy_swept_rectangle_path_clear
    t_old = 0.0
    old_paths = []
    try:
        for me, goal in cases:
            t0 = time.perf_counter()
            pr = M.plan(me, goal, 0.0, raw_wall, blocked,
                        tactical_mode=TACTICAL_MODE_CHASE,
                        blocked_turn=blocked_turn, free_dist=free_dist, me_heading=0.0)
            t_old += time.perf_counter() - t0
            old_paths.append((pr.success, pr.reason, pr.path))
    finally:
        M.build_tube, M.swept_rectangle_path_clear, M.footprint_clearances = saved
    t_new = 0.0
    mism = 0
    for i, (me, goal) in enumerate(cases):
        t0 = time.perf_counter()
        pr = M.plan(me, goal, 0.0, raw_wall, blocked,
                    tactical_mode=TACTICAL_MODE_CHASE,
                    blocked_turn=blocked_turn, free_dist=free_dist, me_heading=0.0)
        t_new += time.perf_counter() - t0
        old_s, old_r, old_p = old_paths[i]
        if old_s != pr.success:
            mism += 1
            print('  PLAN success mismatch', old_s, pr.success, old_r, pr.reason)
            continue
        if old_p is None or pr.path is None:
            continue
        if len(old_p) != len(pr.path):
            mism += 1
            print('  PLAN len mismatch', len(old_p), len(pr.path))
            continue
        d = max(abs(a[0] - b[0]) + abs(a[1] - b[1]) for a, b in zip(old_p, pr.path))
        if d > 1e-9:
            mism += 1
            print('  PLAN point mismatch max_d=%.3g' % d)
    print(f'  plan() x{len(cases)}: legacy {t_old * 1000:.2f} ms | new {t_new * 1000:.2f} ms '
          f'| speedup {t_old / max(t_new, 1e-9):.1f}x | mismatch={mism}')
    return mism


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--n-pose', type=int, default=100000)
    ap.add_argument('--skip-plan', action='store_true')
    a = ap.parse_args()

    maze = load_maze_csv(os.path.join(ROOT, 'web_nav_logs', 'maze_latest.csv'))
    polys = load_polys(os.path.join(ROOT, 'data log', 'tile_polys.json'))
    raw_wall, _, blocked, blocked_turn, free_dist = build_maps(
        maze, polys, 10, 5, turn_radius_px=18.45, straight_radius_px=10.5)
    free_cells = [(x, y) for y in range(blocked.shape[0]) for x in range(blocked.shape[1])
                  if not blocked[y, x]]
    random.seed(7)
    print('== map:', raw_wall.shape, 'free cells:', len(free_cells))

    print('\n[1] footprint_collides vs footprint_collides_batch (%d random poses)' % a.n_pose)
    m1 = check_poses(raw_wall, a.n_pose)

    print('\n[2] swept_rectangle_path_clear legacy vs batch')
    real = build_real_paths(free_cells, blocked, free_dist, raw_wall, count=6)
    m2 = check_swept(real, raw_wall)

    print('\n[3] footprint_clearances batch vs per-path legacy (15-candidate set)')
    m3 = 0
    if real:
        bench_clearances(real[:3], free_dist)

    print('\n[4] build_tube legacy vs new (path-identical)')
    m4 = check_build_tube(real, free_dist, raw_wall)

    print('\n[5] plan() legacy vs new (path-identical)')
    m5 = 0
    if not a.skip_plan:
        m5 = check_plan(free_cells, blocked, free_dist, raw_wall, blocked_turn)

    print('\n[6] micro benchmarks')
    bench_collides(raw_wall, n=1000)
    if real:
        bench_clearances(real, free_dist)

    total = m1 + m2 + m3 + m4 + m5
    print('\n=== RESULT:', 'ALL CONSISTENT' if total == 0 else f'{total} MISMATCH(ES) ===')
    sys.exit(0 if total == 0 else 1)
