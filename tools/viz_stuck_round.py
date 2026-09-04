# -*- coding: utf-8 -*-
"""Analyze a stuck round: unwrapped heading vs path angle, ALIGN error, zoomed track.
Usage: python tools/viz_stuck_round.py <session_dir> [out.png]
"""
import csv, json, math, sys, os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

session = sys.argv[1] if len(sys.argv) > 1 else r"C:\Users\jasongeorge\deepseek harness workspace\tanktrouble12\web_nav_logs\20260903_125144"
out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(session, "analysis_stuck2.png")

tr = list(csv.DictReader(open(os.path.join(session, "track.csv"), encoding="utf-8")))
pl = list(csv.DictReader(open(os.path.join(session, "plans.csv"), encoding="utf-8")))

t = np.array([float(x["t"]) for x in tr])
mx = np.array([float(x["me_x"]) for x in tr])
my = np.array([float(x["me_y"]) for x in tr])
ang_raw = np.array([float(x["me_angle"]) for x in tr])
mode = [x["controller_mode"] for x in tr]

# --- heading, unwrapped so a spin shows as a steady ramp, cuts show as discontinuity only at t0 ---
ang_uw = np.unwrap(ang_raw)
ang_deg = np.degrees(ang_uw)

# --- path first-seg angle per plan (map y-down, atan2(dy,dx)) ---
plan_t = []
plan_ang = []
for p in pl:
    pts = json.loads(p["path_points"])
    if len(pts) >= 2:
        plan_t.append(float(p["t"]))
        plan_ang.append(math.degrees(math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0])) % 360.0)
plan_t = np.array(plan_t)
plan_ang = np.array(plan_ang)

# --- ALIGN error: err = wrap(target - heading), target = path first-seg angle ---
path_deg_at = np.interp(t, plan_t, plan_ang)
err = (path_deg_at - ang_raw * 180.0 / math.pi + 180.0) % 360.0 - 180.0

# --- position-diff speed (px/s) ---
dt = np.diff(t, prepend=t[0])
spd = np.hypot(np.diff(mx, prepend=mx[0]), np.diff(my, prepend=my[0])) / np.maximum(dt, 1e-3)

fig = plt.figure(figsize=(15, 11))

# ---------- Panel 1: full map + zoomed corner ----------
ax1 = fig.add_subplot(2, 2, 1)
p = ax1.scatter(mx, my, c=t, s=4, cmap="viridis")
ax1.plot(mx, my, color="gray", lw=0.4, alpha=0.5)
ax1.plot(plan_t * 0 + 57.0, plan_t * 0 + 285.0, "r*", ms=12)
ax1.set_title("me track (color=time). Red *= path start (57,285)")
ax1.set_xlabel("x (map px); NOTE map y is DOWN")
ax1.set_ylabel("y (map, down)")
plt.colorbar(p, ax=ax1, label="t (s)")

ax1b = fig.add_subplot(2, 2, 2)
# zoom to the stuck corner, y-axis inverted so +y shows downward like the map
ax1b.set_xlim(30, 110)
ax1b.set_ylim(320, 250)  # inverted
seg_pts = mx.astype(float)
seg_pts_y = my.astype(float)
pts = np.array([seg_pts, seg_pts_y]).T.reshape(-1, 1, 2)
segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
lc = LineCollection(segs, cmap="viridis", linewidths=1.5)
lc.set_array(t[:-1])
ax1b.add_collection(lc)
ax1b.scatter(mx, my, c=t, s=3, cmap="viridis", zorder=5)
# path points
for p_ in pl[:1]:
    pts_ = json.loads(p_["path_points"])
    pp = np.array(pts_)
    ax1b.plot(pp[:, 0], pp[:, 1], "b-o", ms=3, lw=1.2, label="execution path pts (plan #0)")
ax1b.plot([57.0], [285.0], "r*", ms=15, label="start")
ax1b.set_title("ZOOM stuck corner (y axis inverted: up=up)")
ax1b.legend(loc="lower left", fontsize=8)
ax1b.set_xlabel("x")
ax1b.set_ylabel("y (inverted)")

# ---------- Panel 2: heading (unwrapped) vs path angle ----------
ax2 = fig.add_subplot(3, 2, 3)
ax2.plot(t, ang_deg, "b-", lw=0.7, label="me heading (unwrap, map y-down CW)")
ax2.plot(plan_t, plan_ang, "r.", ms=6, label="path first-seg angle (per plan)")
ax2.set_ylabel("angle (deg, map convention)")
ax2.set_title("heading vs path angle -- continuous ramps = real spinning")
ax2.legend(loc="upper left", fontsize=8)
ax2.grid(alpha=0.3)

ax2b = fig.add_subplot(3, 2, 4)
ax2b.plot(t, err, "g-", lw=0.7, label="ALIGN err = wrap(path-heading)")
ax2b.axhspan(-12.0, 12.0, color="pink", alpha=0.4)
ax2b.axhline(0, color="k", lw=0.5)
ax2b.set_ylabel("err (deg)")
ax2b.set_title("ALIGN error; pink band = |err|<12deg (would exit ALIGN)")
ax2b.legend(loc="upper left", fontsize=8)
ax2b.grid(alpha=0.3)

# ---------- Panel 3: mode + speed ----------
ax3 = fig.add_subplot(3, 2, 5)
codes = sorted(set(mode))
cmap_mode = {c: i for i, c in enumerate(codes)}
col = [cmap_mode[m] for m in mode]
for i, c in enumerate(codes):
    m = np.array([mm == c for mm in mode])
    ax3.scatter(t[m], np.full(m.sum(), i), s=3, label=c)
ax3.set_yticks(range(len(codes)))
ax3.set_yticklabels(codes, fontsize=7)
ax3.set_title("controller_mode timeline")
ax3.legend(fontsize=6, loc="upper right")

ax3b = fig.add_subplot(3, 2, 6)
ax3b.plot(t, spd, "k-", lw=0.6)
ax3b.axhline(25, color="r", ls="--", label="ALIGN near-stop threshold 25px/s")
ax3b.set_ylabel("speed px/s (pos-diff)")
ax3b.set_xlabel("t (s)")
ax3b.legend(fontsize=8)
ax3b.grid(alpha=0.3)

plt.tight_layout()
plt.savefig(out, dpi=110)
print("saved", out)
print("heading range (unwrap): %.0f ... %.0f deg -> total turns ~ %.1f" % (ang_deg.min(), ang_deg.max(), (ang_deg.max() - ang_deg.min()) / 360.0))
print("|err|>12 deg frames: %.1f%%" % (100.0 * np.mean(np.abs(err) > 12)))
print("|err|<12 deg frames: %.1f%%" % (100.0 * np.mean(np.abs(err) <= 12)))
print("ALIGN frames: %d (%.1f%%)" % (sum(1 for m_ in mode if m_ == "ALIGN_START"), 100.0 * sum(1 for m_ in mode if m_ == "ALIGN_START") / len(mode)))
