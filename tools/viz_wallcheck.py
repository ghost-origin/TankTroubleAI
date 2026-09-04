# -*- coding: utf-8 -*-
"""Render local wall map with track, path and heading arrows; check wall ahead of (44,276)."""
import csv, json, math, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

repo = r"C:\Users\jasongeorge\deepseek harness workspace\tanktrouble12"
session = os.path.join(repo, "web_nav_logs", "20260903_125144")
sys.path.insert(0, os.path.join(repo, "python"))
from navigation_mvp import load_maze, load_polys, build_maps  # noqa: E402

maze = load_maze(os.path.join(session, "maze.csv"))
polys = load_polys(os.path.join(repo, "data log", "tile_polys.json"))
raw, _, _, _, _ = build_maps(maze, polys, 10, 5, turn_radius_px=18.45)
print("raw_wall shape:", raw.shape)

tr = list(csv.DictReader(open(os.path.join(session, "track.csv"), encoding="utf-8")))
pl = list(csv.DictReader(open(os.path.join(session, "plans.csv"), encoding="utf-8")))
t = np.array([float(x["t"]) for x in tr])
mx = np.array([float(x["me_x"]) for x in tr])
my = np.array([float(x["me_y"]) for x in tr])
ang = np.array([float(x["me_angle"]) for x in tr])

# --- wall check: what is along direction 279 deg from (44.4, 276)? ---
def wall_along(x0, y0, theta_deg, maxd=60.0):
    th = math.radians(theta_deg)
    hits = []
    for d in np.arange(4.0, maxd, 1.0):
        px, py = x0 + math.cos(th) * d, y0 + math.sin(th) * d
        ix, iy = int(round(px)), int(round(py))
        if ix < 0 or iy < 0 or ix >= raw.shape[1] or iy >= raw.shape[0]:
            hits.append((d, "OOB")); break
        if raw[iy, ix]:
            hits.append((d, "WALL")); break
    return hits

print("from (44.4,276) along 279deg:", wall_along(44.4, 276.0, 279.0))
print("from (44.4,276) along 270deg:", wall_along(44.4, 276.0, 270.0))
print("from (44.4,276) along 195deg:", wall_along(44.4, 276.0, 195.0))
print("from (46.5,273) along 279deg:", wall_along(46.5, 273.0, 279.0))
print("from (57,285) along 270deg:", wall_along(57.0, 285.0, 270.0))

# --- plot ---
fig, axes = plt.subplots(1, 2, figsize=(16, 8))
for ax, (x0, x1, y0, y1) in zip(axes, [(38, 70, 245, 305), (30, 110, 220, 320)]):
    ax.imshow(raw, cmap="gray_r", origin="upper", alpha=0.85,
              extent=(0, raw.shape[1], raw.shape[0], 0))
    ax.plot(mx, my, "-", color="deepskyblue", lw=0.6, alpha=0.8, label="track")
    # heading arrows every 2s
    for k in range(0, len(t), int(2.0 / np.median(np.diff(t)))):
        th = ang[k]
        ax.arrow(mx[k], my[k], 12 * math.cos(th), 12 * math.sin(th),
                 head_width=3.5, head_length=4.5, color="red", alpha=0.75)
    for p in pl[:1]:
        pts = np.array(json.loads(p["path_points"]))
        ax.plot(pts[:, 0], pts[:, 1], "o-", color="lime", ms=3, lw=1.4, label="exec path")
    ax.plot(57.0, 285.0, "r*", ms=14, label="start(57,285)")
    ax.plot(44.4, 276.0, "ko", ms=7, label="stuck pos(44.4,276)")
    ax.set_xlim(x0, x1); ax.set_ylim(y1, y0)   # inverted y: up = up
    ax.set_title("map y-DOWN convention, axis inverted for display (up=up)")
    ax.legend(fontsize=7, loc="lower left")
    ax.grid(alpha=0.3)
plt.tight_layout()
out = os.path.join(session, "analysis_wallcheck.png")
plt.savefig(out, dpi=115)
print("saved", out)
# print maze grid for reference
print("maze:")
for row in maze:
    print(" ", row)
