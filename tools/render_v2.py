# -*- coding: utf-8 -*-
"""Render raw_wall_v2 using C2 official flipmap caches; diff vs current transformed_polygon build."""
import csv, json, math, os, sys
import numpy as np
repo = r"C:\Users\jasongeorge\deepseek harness workspace\tanktrouble12"
session = os.path.join(repo, "web_nav_logs", "20260903_125144")
sys.path.insert(0, os.path.join(repo, "python"))
from navigation_mvp import load_maze, parse_tile, build_maps, load_polys

maze = load_maze(os.path.join(session, "maze.csv"))
polys = load_polys(os.path.join(repo, "data log", "tile_polys.json"))
raw, _, _, _, _ = build_maps(maze, polys, 10, 5, turn_radius_px=18.45)

rtp = json.load(open(os.path.join(repo, "data log", "runtime_tile_polys.json"), encoding="utf-8"))
TILE = 57
W = H = 570
raw_v2 = np.zeros((H, W), dtype=bool)

def fill_poly(mask, pts):
    # rasterize pixel-space polygon points (pts: [x0,y0,x1,y1,...])
    xs = pts[0::2]; ys = pts[1::2]
    xs = [x + ox for x in xs]; ys = [y + oy for y in ys]
    xmin = max(0, int(min(xs))); xmax = min(W - 1, int(max(xs)) + 1)
    ymin = max(0, int(min(ys))); ymax = min(H - 1, int(max(ys)) + 1)
    for py in range(ymin, ymax + 1):
        for px in range(xmin, xmax + 1):
            # point-in-polygon (ray cast)
            inside = False
            n = len(xs)
            j = n - 1
            for i in range(n):
                xi, yi = xs[i], ys[i]
                xj, yj = xs[j], ys[j]
                if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi):
                    inside = not inside
                j = i
            if inside:
                mask[py, px] = True
    return mask

mismatch_tiles = []
for cy in range(10):
    for cx in range(10):
        v = maze[cy][cx]
        tid, fh, fv, fd = parse_tile(v)
        if tid <= 0 or tid >= len(rtp) or not rtp[tid]:
            continue
        fm = rtp[tid].get("flipmap")
        if not fm:
            continue
        try:
            cell = fm[fh][fv]
            pts = cell[fd]["pts_cache"] if fd < len(cell) else cell[0]["pts_cache"]
        except Exception as e:
            print("ERR tile(%d,%d) %s" % (cx, cy, e)); continue
        ox, oy = cx * TILE, cy * TILE
        fill_poly(raw_v2, pts)

# diff
diff = raw_v2 ^ raw
nd = int(diff.sum())
print("diff pixels between current raw_wall and C2-flipmap version: %d" % nd)
if nd:
    ys, xs = np.where(diff)
    print("diff bbox x=[%d,%d] y=[%d,%d]" % (xs.min(), xs.max(), ys.min(), ys.max()))
    # cluster by tile
    from collections import Counter
    cnt = Counter()
    for yy, xx in zip(ys, xs):
        cnt[(xx // 57, yy // 57)] += 1
    for (tx, ty), n in sorted(cnt.items(), key=lambda kv: -kv[1])[:12]:
        print("  tile(%d,%d): %d diff px (maze=%d)" % (tx, ty, n, maze[ty][tx]))

# ASCII around stuck area for v2
print("\nraw_v2 ASCII x=20..100 y=248..320 (#=wall)")
for yy in range(248, 321, 2):
    row = ""
    for xx in range(20, 100, 2):
        blk = raw_v2[yy:yy+2, xx:xx+2]
        row += "#" if blk.any() else "."
    print("%3d %s" % (yy, row))
np.save(os.path.join(repo, "data log", "raw_wall_v2.npy"), raw_v2)
