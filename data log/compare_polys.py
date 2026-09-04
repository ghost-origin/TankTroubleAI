# -*- coding: utf-8 -*-
"""Compare runtime tile_polys with data log/tile_polys.json and show full maze."""
import json, os, csv

repo = r"C:\Users\jasongeorge\deepseek harness workspace\tanktrouble12"
dl = os.path.join(repo, "data log")

run = json.load(open(os.path.join(dl, "runtime_tile_polys.json"), encoding="utf-8"))
try:
    ours = json.load(open(os.path.join(dl, "tile_polys.json"), encoding="utf-8"))
except Exception as e:
    ours = None
    print("OURS load fail:", e)

def poly_of(tp):
    if tp is None:
        return None
    if isinstance(tp, dict):
        return tp.get("poly")
    if isinstance(tp, list):
        return tp
    return None

n = max(len(run), len(ours) if ours else 0)
print("runtime polys= %d   our polys= %s" % (len(run), len(ours) if ours else "N/A"))
for i in range(min(n, 20)):
    r = poly_of(run[i]) if i < len(run) else None
    o = poly_of(ours[i]) if ours and i < len(ours) else None
    if r is None and o is None:
        print("tid %2d: both EMPTY" % i); continue
    same = (r == o)
    print("tid %2d: same=%-5s runtime=%s  " % (i, same, r) + ("ours=" + str(o) if o != r else ""))

# maze grid full
rows = list(csv.reader(open(os.path.join(dl, "runtime_maze.csv"), encoding="utf-8")))
print("\nruntime_maze.csv rows=%d cols=%d" % (len(rows), len(rows[0])))
for y, row in enumerate(rows):
    vals = row[:30]
    nv = sum(1 for v in vals if v.strip() != "-1")
    print("row %2d: nonempty=%2d  %s" % (y, nv, vals[:14]))
