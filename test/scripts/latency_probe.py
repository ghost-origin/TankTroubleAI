# -*- coding: utf-8 -*-
"""Latency probe (web mode): build_maps / plan / per-frame pipeline costs.

Run with: E:\\anaconda\\python.exe test/scripts/latency_probe.py
"""
import sys, os, time, json, math, csv, random, tempfile
ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'python'))
from navigation_mvp import build_maps, plan, MAP_PX, TACTICAL_MODE_CHASE, load_polys


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
print('maze %dx%d wall=%d' % (len(maze), len(maze[0]),
                              sum(1 for r in maze for v in r if v not in (0, -1))))

polys = load_polys(os.path.join(ROOT, 'data log', 'tile_polys.json'))
print('polys entries:', len(polys) if polys else 0)

# 1) build_maps (once per maze lock)
t0 = time.perf_counter()
raw_wall, inflated, blocked, blocked_turn, free_dist = build_maps(
    maze, polys, 10, 5, turn_radius_px=18.45, straight_radius_px=10.5)
t1 = time.perf_counter()
print('build_maps: %.1f ms (once per maze lock)' % ((t1 - t0) * 1000))

# 2) plan (chase mode) x20 from random free cells
free_cells = [(x, y) for y in range(blocked.shape[0]) for x in range(blocked.shape[1])
              if not blocked[y, x]]
random.seed(1)
times = []
ok = 0
for _ in range(20):
    sx, sy = random.choice(free_cells)
    gx, gy = random.choice(free_cells)
    t0 = time.perf_counter()
    pr = plan((sx * 5.0 + 2.5, sy * 5.0 + 2.5), (gx * 5.0 + 2.5, gy * 5.0 + 2.5),
              0.0, raw_wall, blocked, tactical_mode=TACTICAL_MODE_CHASE,
              blocked_turn=blocked_turn, free_dist=free_dist, me_heading=0.0)
    times.append((time.perf_counter() - t0) * 1000)
    ok += pr.success
times.sort()
print('plan: n=%d ok=%d min=%.1f med=%.1f p95=%.1f max=%.1f ms' % (
    len(times), ok, times[0], times[len(times) // 2],
    times[int(len(times) * 0.95)], times[-1]))

# 3) per-frame JSON: typical bridge frame with/without full map
grid = [[-1] * 30 for _ in range(20)]
for y in range(10):
    for x in range(10):
        grid[y][x] = maze[y][x]
frame = {
    't': 12.34, 'layout': 'Layout 1', 'me_present': True, 'foe_present': True, 'event': '',
    'me': {'x': 300.1, 'y': 200.2, 'angle': 0.5, 'aim': 0.4, 'vx': 10.0, 'vy': 0.0, 'anim': ''},
    'foe': {'x': 320.1, 'y': 400.2, 'angle': 1.5, 'aim': 1.4, 'vx': -5.0, 'vy': 3.0, 'anim': ''},
    'bullets': [{'x': 100.0, 'y': 200.0, 'vx': 30.0, 'vy': 0.0, 'type': 44}],
    'powerups': [],
    'map': {'w': 30, 'h': 20, 'tileW': 57, 'tileH': 57, 'x': 197.5, 'y': 31.0,
            'ox': 0, 'oy': 0, 'ready': True, 'grid': grid, 'polys': polys}
}
txt_full = json.dumps(frame, separators=(',', ':'))
small = dict(frame)
small.pop('map')
txt_no_map = json.dumps(small, separators=(',', ':'))
print('frame size with map: %.1f KB / without map: %.1f KB' % (
    len(txt_full) / 1024, len(txt_no_map) / 1024))

t0 = time.perf_counter()
for _ in range(200):
    json.loads(txt_full)
t1 = time.perf_counter()
print('json.loads with map: %.3f ms/frame' % ((t1 - t0) * 1000 / 200))
t0 = time.perf_counter()
for _ in range(200):
    json.loads(txt_no_map)
t1 = time.perf_counter()
print('json.loads without map: %.3f ms/frame' % ((t1 - t0) * 1000 / 200))

# 4) CSV record cost: persistent handles + writerow + flush x2 (bot's record())
tmp1 = os.path.join(tempfile.gettempdir(), 'lp_track.csv')
tmp2 = os.path.join(tempfile.gettempdir(), 'lp_track_latest.csv')
f1 = open(tmp1, 'w', newline='')
w1 = csv.writer(f1)
f2 = open(tmp2, 'w', newline='')
w2 = csv.writer(f2)
row = [12.34, 300.1, 200.2, 0.5, 10.0, 0.0, 320.1, 400.2, 1.5, -5.0, 3.0,
       1, 0, 1, 0, 0, 0, 'chase']
t0 = time.perf_counter()
for _ in range(200):
    w1.writerow(row)
    w2.writerow(row)
    f1.flush()
    f2.flush()
t1 = time.perf_counter()
print('record (2x writerow + 2x flush): %.3f ms/frame' % ((t1 - t0) * 1000 / 200))
f1.close(); f2.close()
try:
    os.remove(tmp1); os.remove(tmp2)
except OSError:
    pass

print('done')
