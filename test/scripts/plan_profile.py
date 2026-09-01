# -*- coding: utf-8 -*-
"""Profile plan() internals (chase mode) step by step."""
import sys, os, time, random
ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'python'))
import numpy as np
from navigation_mvp import (
    build_maps, plan, MAP_PX, TACTICAL_MODE_CHASE, load_polys,
    astar, simplify_cells, refine_astar_polyline, build_execution_window,
    point_to_cell, nearest_free, cell_to_point, GRID_PX)


def load_maze_csv(path):
    maze = []
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            maze.append([int(v) for v in line.split(',')])
    return maze


maze = load_maze_csv(os.path.join(ROOT, 'web_nav_logs', 'maze_latest.csv'))
polys = load_polys(os.path.join(ROOT, 'data log', 'tile_polys.json'))
raw_wall, _, blocked, blocked_turn, free_dist = build_maps(
    maze, polys, 10, 5, turn_radius_px=18.45, straight_radius_px=10.5)
print('blocked grid %dx%d, %d blocked cells' % (blocked.shape[1], blocked.shape[0],
                                                int(blocked.sum())))

free_cells = [(x, y) for y in range(blocked.shape[0]) for x in range(blocked.shape[1])
              if not blocked[y, x]]
random.seed(7)
pairs = []
for _ in range(10):
    a = random.choice(free_cells)
    b = random.choice(free_cells)
    pairs.append((a, b))

t_astar = []
t_simp = []
t_refine = []
t_window = []
t_total = []
ok = 0
for (sx, sy), (gx, gy) in pairs:
    me = (sx * GRID_PX + GRID_PX / 2, sy * GRID_PX + GRID_PX / 2)
    goal = (gx * GRID_PX + GRID_PX / 2, gy * GRID_PX + GRID_PX / 2)
    t0 = time.perf_counter()
    pr = plan(me, goal, 0.0, raw_wall, blocked, tactical_mode=TACTICAL_MODE_CHASE,
              blocked_turn=blocked_turn, free_dist=free_dist, me_heading=0.0)
    t_total.append((time.perf_counter() - t0) * 1000)
    ok += pr.success

    start = point_to_cell(me, GRID_PX, blocked)
    foe_c = point_to_cell(goal, GRID_PX, blocked)
    foe_free = nearest_free(foe_c, blocked, max_r=8)
    t0 = time.perf_counter()
    p_cells = astar(start, foe_free, blocked)
    t_astar.append((time.perf_counter() - t0) * 1000)
    if not p_cells:
        continue
    t0 = time.perf_counter()
    pc = simplify_cells(p_cells, blocked)
    t_simp.append((time.perf_counter() - t0) * 1000)
    p = [me] + [cell_to_point(c, GRID_PX) for c in pc[1:-1]] + [goal]
    t0 = time.perf_counter()
    p = refine_astar_polyline(p, free_dist, raw_wall)
    t_refine.append((time.perf_counter() - t0) * 1000)
    t0 = time.perf_counter()
    w = build_execution_window(p, free_dist, raw_wall, start_heading=0.0)
    t_window.append((time.perf_counter() - t0) * 1000)

for name, ts in [('astar', t_astar), ('simplify_cells', t_simp),
                 ('refine_astar_polyline', t_refine),
                 ('build_execution_window', t_window), ('plan total', t_total)]:
    ts = sorted(ts)
    if not ts:
        print('%s: n=0' % name)
        continue
    print('%-24s n=%2d min=%7.1f med=%7.1f p95=%7.1f max=%7.1f ms' % (
        name, len(ts), ts[0], ts[len(ts) // 2], ts[int(len(ts) * 0.95)], ts[-1]))
print('success: %d/%d' % (ok, len(pairs)))
