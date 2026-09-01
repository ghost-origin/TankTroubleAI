# -*- coding: utf-8 -*-
"""Profile build_tube internals on the real 170px lookahead input."""
import sys, os, time, random
ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'python'))
from navigation_mvp import (
    build_maps, plan, TACTICAL_MODE_CHASE, load_polys,
    astar, simplify_cells, refine_astar_polyline, build_execution_window,
    point_to_cell, nearest_free, cell_to_point, GRID_PX,
    extract_path_prefix, build_tube, _bezier_polyline, push_off_walls,
    swept_rectangle_path_clear, footprint_clearance, _legacy_tube_coarse_ok,
    path_length, smoothness, curvature_variation,
    TUBE_SAMPLE_STEP, TUBE_MIN_CLEARANCE, TUBE_CLEARANCE_PX, FOOTPRINT_WRAP_SHRINK)


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

free_cells = [(x, y) for y in range(blocked.shape[0]) for x in range(blocked.shape[1])
              if not blocked[y, x]]
random.seed(7)
results = []
for _ in range(6):
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
    look = extract_path_prefix(p, 170.0)
    npts = len(look)

    t_bez = []
    t_push = []
    t_swept = []
    t_clear = []
    n_coarse_ok = 0
    n_swept_ok = 0
    for radius_scale in (1.25, 1.1, 1.45, 0.95, 1.7):
        for handle_scale in (1.0, 0.85, 1.15):
            t0 = time.perf_counter()
            candidate = _bezier_polyline(look, TUBE_SAMPLE_STEP, radius_scale, handle_scale)
            t_bez.append((time.perf_counter() - t0) * 1000)
            if candidate is None or len(candidate) < 2:
                continue
            if not _legacy_tube_coarse_ok(candidate, free_dist):
                continue
            n_coarse_ok += 1
            t0 = time.perf_counter()
            candidate = push_off_walls(candidate, free_dist, min_clearance=TUBE_MIN_CLEARANCE)
            t_push.append((time.perf_counter() - t0) * 1000)
            t0 = time.perf_counter()
            ok = swept_rectangle_path_clear(candidate, raw_wall, shrink=FOOTPRINT_WRAP_SHRINK)
            t_swept.append((time.perf_counter() - t0) * 1000)
            if not ok:
                continue
            n_swept_ok += 1
            t0 = time.perf_counter()
            footprint_clearance(candidate, free_dist)
            t_clear.append((time.perf_counter() - t0) * 1000)

    # whole build_tube + whole plan for reference
    t0 = time.perf_counter()
    build_tube(look, free_dist, raw_wall=raw_wall)
    t_tube = (time.perf_counter() - t0) * 1000
    t0 = time.perf_counter()
    plan(me, goal, 0.0, raw_wall, blocked, tactical_mode=TACTICAL_MODE_CHASE,
         blocked_turn=blocked_turn, free_dist=free_dist, me_heading=0.0)
    t_plan = (time.perf_counter() - t0) * 1000
    results.append((npts, n_coarse_ok, n_swept_ok, t_bez, t_push, t_swept, t_clear, t_tube, t_plan))

print('pts | coarse_ok/15 | swept_ok | bezier ms | push ms | swept ms | clear ms | tube ms | plan ms')
for npts, nco, nso, tb, tp, ts, tc, tt, tpl in results:
    print('%3d | %2d | %2d | bez %.2f | push %.2f | swept %.2f | clear %.2f | tube %.1f | plan %.1f' % (
        npts, nco, nso,
        sum(tb) / len(tb), sum(tp) / len(tp), sum(ts) / len(ts),
        sum(tc) / len(tc) if tc else 0.0, tt, tpl))
