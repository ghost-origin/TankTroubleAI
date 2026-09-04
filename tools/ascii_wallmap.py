# -*- coding: utf-8 -*-
"""ASCII map of raw_wall around stuck area + exact wall-strip coordinates per tile."""
import csv, json, math, os, sys
import numpy as np
repo = r"C:\Users\jasongeorge\deepseek harness workspace\tanktrouble12"
session = os.path.join(repo, "web_nav_logs", "20260903_125144")
sys.path.insert(0, os.path.join(repo, "python"))
from navigation_mvp import load_maze, load_polys, build_maps, parse_tile, transformed_polygon, TILE_PX

maze = load_maze(os.path.join(session, "maze.csv"))
polys = load_polys(os.path.join(repo, "data log", "tile_polys.json"))
raw, _, _, _, _ = build_maps(maze, polys, 10, 5, turn_radius_px=18.45)

# ---- 1. ASCII map: x 20..100, y 248..320, 2px cells ----
print("raw_wall ASCII (#=wall, .=free), x=20..100, y=248..320")
hdr = "    " + "".join(str((x // 10) % 10) for x in range(20, 100, 2))
print(hdr)
for yy in range(248, 321, 2):
    row = ""
    for xx in range(20, 100, 2):
        blk = raw[yy:yy+2, xx:xx+2]
        row += "#" if blk.any() else "."
    print("%3d %s" % (yy, row))

# ---- 2. per-tile wall strips in the 4x5 window (tiles 0..4 x, 4..8 y) ----
print("\n--- tile walls (world coords), tiles x=0..3, y=4..8 ---")
for cy in range(4, 9):
    for cx in range(0, 4):
        v = maze[cy][cx]
        tid, fh, fv, fd = parse_tile(v)
        if tid <= 0 or tid >= len(polys) or not polys[tid]:
            continue
        pts = transformed_polygon(cx, cy, v, polys)
        if not pts:
            continue
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
        print("tile(%d,%d) raw=%d -> tid=%d flip(h=%d,v=%d,d=%d) poly x=[%.1f,%.1f] y=[%.1f,%.1f] pts=%s" % (
            cx, cy, v, tid, int(fh), int(fv), int(fd), min(xs), max(xs), min(ys), max(ys),
            " ".join("(%.0f,%.0f)" % (p[0], p[1]) for p in pts)))

# ---- 3. where does the tank actually roam? (from track) ----
tr = list(csv.DictReader(open(os.path.join(session, "track.csv"), encoding="utf-8")))
mx = np.array([float(x["me_x"]) for x in tr]); my = np.array([float(x["me_y"]) for x in tr])
m = (mx >= 40) & (mx <= 60) & (my >= 265) & (my <= 300)
print("\ntrack range in stuck window: x=[%.1f,%.1f] y=[%.1f,%.1f]  n=%d" % (
    mx[m].min(), mx[m].max(), my[m].min(), my[m].max(), m.sum()))
