# -*- coding: utf-8 -*-
"""TankTrouble navigation MVP (offline replay + reusable planner).

MVP only:
1) Restore maze wall polygons from maze_*.csv + tile_polys.json
2) Inflate walls by a tank safety radius
3) Generate three tactical targets behind foe: -135°, -180°, -225° at fixed range
4) Keep targets that are navigable and have direct shooting LOS to foe
5) 8-neighbour A* for each legal target, choose shortest
6) LOS shortcut to reduce unnecessary A* corners
7) Replay track CSV at 1 s OODA intervals and export metrics / preview

This file is independent of the game source and does not modify the repository.
"""
from __future__ import annotations

import argparse
import csv
import glob
import heapq
import json
import math
import os
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
from PIL import Image, ImageDraw
from scipy.ndimage import distance_transform_edt

TILE_PX = 57
MAZE_SIZE = 10
MAP_PX = TILE_PX * MAZE_SIZE
TILE_ID_MASK = 0x1FFFFFFF
FLAG_H, FLAG_V, FLAG_D = 0x80000000, 0x40000000, 0x20000000

# Deliberately simple / editable MVP parameters.
GRID_PX = 5
TANK_SAFE_RADIUS_PX = 8
ATTACK_RANGES_PX = (60.0, 90.0, 120.0, 150.0, 180.0)
RELATIVE_TARGET_DEG = (-135.0, -180.0, -225.0)
OODA_PERIOD_S = 1.0

Point = Tuple[float, float]
Cell = Tuple[int, int]  # (gx, gy)


@dataclass
class PlanResult:
    success: bool
    path: List[Point]
    target: Optional[Point]
    relative_deg: Optional[float]
    path_length: float
    smoothness_rad: float
    planning_ms: float
    reason: str = ""
    valid_candidates: int = 0


def parse_tile(v: int) -> Tuple[int, bool, bool, bool]:
    uv = int(v) & 0xFFFFFFFF
    return (
        uv & TILE_ID_MASK,
        bool(uv & FLAG_H),
        bool(uv & FLAG_V),
        bool(uv & FLAG_D),
    )


def load_maze(path: str) -> List[List[int]]:
    grid = []
    with open(path, newline="", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                grid.append([int(v) for v in line.split(",")])
    if len(grid) < MAZE_SIZE or any(len(r) < MAZE_SIZE for r in grid[:MAZE_SIZE]):
        raise ValueError(f"maze is not {MAZE_SIZE}x{MAZE_SIZE}: {path}")
    return [r[:MAZE_SIZE] for r in grid[:MAZE_SIZE]]


def load_polys(path: str) -> list:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def transformed_polygon(cx: int, cy: int, raw_tile: int, polys: list) -> Optional[List[Point]]:
    if raw_tile in (0, -1):
        return None
    tid, fliph, flipv, flipd = parse_tile(raw_tile)
    if tid <= 0 or tid >= len(polys) or not polys[tid]:
        return None
    poly = polys[tid]
    pts = [(poly[i] * TILE_PX, poly[i + 1] * TILE_PX) for i in range(0, len(poly), 2)]
    if flipd:
        pts = [(y, x) for x, y in pts]
    if fliph:
        pts = [(TILE_PX - x, y) for x, y in pts]
    if flipv:
        pts = [(x, TILE_PX - y) for x, y in pts]
    ox, oy = cx * TILE_PX, cy * TILE_PX
    return [(ox + x, oy + y) for x, y in pts]


def build_maps(maze: List[List[int]], polys: list, safe_radius_px: int, grid_px: int):
    """Return raw_wall mask, inflated_wall mask, conservative planning grid."""
    img = Image.new("L", (MAP_PX, MAP_PX), 0)
    draw = ImageDraw.Draw(img)
    for cy, row in enumerate(maze):
        for cx, raw in enumerate(row):
            pts = transformed_polygon(cx, cy, raw, polys)
            if pts:
                # Polygon is the real collision shape (thin wall rectangle etc.).
                draw.polygon(pts, fill=255)

    # World boundary is also an obstacle.
    draw.rectangle([0, 0, MAP_PX - 1, MAP_PX - 1], outline=255, width=2)
    raw_wall = np.asarray(img, dtype=np.uint8) > 0

    free_dist = distance_transform_edt(~raw_wall)
    inflated = free_dist <= float(safe_radius_px)

    gw = math.ceil(MAP_PX / grid_px)
    gh = math.ceil(MAP_PX / grid_px)
    blocked = np.zeros((gh, gw), dtype=bool)
    for gy in range(gh):
        y0, y1 = gy * grid_px, min((gy + 1) * grid_px, MAP_PX)
        for gx in range(gw):
            x0, x1 = gx * grid_px, min((gx + 1) * grid_px, MAP_PX)
            # Conservative: any inflated obstacle pixel blocks this navigation cell.
            blocked[gy, gx] = bool(inflated[y0:y1, x0:x1].any())
    return raw_wall, inflated, blocked


def in_bounds_point(p: Point) -> bool:
    return 0 <= p[0] < MAP_PX and 0 <= p[1] < MAP_PX


def point_to_cell(p: Point, grid_px: int, blocked: np.ndarray) -> Cell:
    gx = min(blocked.shape[1] - 1, max(0, int(p[0] // grid_px)))
    gy = min(blocked.shape[0] - 1, max(0, int(p[1] // grid_px)))
    return gx, gy


def cell_to_point(c: Cell, grid_px: int) -> Point:
    gx, gy = c
    return (min(MAP_PX - 0.5, gx * grid_px + grid_px / 2.0),
            min(MAP_PX - 0.5, gy * grid_px + grid_px / 2.0))


def nearest_free(c: Cell, blocked: np.ndarray, max_r: int = 5) -> Optional[Cell]:
    gx, gy = c
    if 0 <= gx < blocked.shape[1] and 0 <= gy < blocked.shape[0] and not blocked[gy, gx]:
        return c
    for r in range(1, max_r + 1):
        candidates = []
        for yy in range(gy - r, gy + r + 1):
            for xx in range(gx - r, gx + r + 1):
                if max(abs(xx - gx), abs(yy - gy)) != r:
                    continue
                if 0 <= xx < blocked.shape[1] and 0 <= yy < blocked.shape[0] and not blocked[yy, xx]:
                    candidates.append((math.hypot(xx - gx, yy - gy), (xx, yy)))
        if candidates:
            candidates.sort()
            return candidates[0][1]
    return None


def octile(a: Cell, b: Cell) -> float:
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)


def astar(start: Cell, goal: Cell, blocked: np.ndarray) -> Optional[List[Cell]]:
    if blocked[start[1], start[0]] or blocked[goal[1], goal[0]]:
        return None
    pq = [(octile(start, goal), 0.0, start)]
    came = {}
    g = {start: 0.0}
    closed = set()
    dirs = [
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, math.sqrt(2)), (1, -1, math.sqrt(2)),
        (-1, 1, math.sqrt(2)), (1, 1, math.sqrt(2)),
    ]
    h, w = blocked.shape
    while pq:
        _, gc, cur = heapq.heappop(pq)
        if cur in closed:
            continue
        if cur == goal:
            out = [cur]
            while cur in came:
                cur = came[cur]
                out.append(cur)
            return out[::-1]
        closed.add(cur)
        x, y = cur
        for dx, dy, cost in dirs:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < w and 0 <= ny < h) or blocked[ny, nx]:
                continue
            if dx and dy:
                # Do not cut corners.
                if blocked[y, nx] or blocked[ny, x]:
                    continue
            nb = (nx, ny)
            ng = gc + cost
            if ng < g.get(nb, float("inf")):
                g[nb] = ng
                came[nb] = cur
                heapq.heappush(pq, (ng + octile(nb, goal), ng, nb))
    return None


def segment_clear_grid(a: Cell, b: Cell, blocked: np.ndarray) -> bool:
    x0, y0 = a
    x1, y1 = b
    dx, dy = x1 - x0, y1 - y0
    n = max(abs(dx), abs(dy))
    if n == 0:
        return not blocked[y0, x0]
    last = (x0, y0)
    for i in range(n + 1):
        t = i / n
        x = int(round(x0 + dx * t))
        y = int(round(y0 + dy * t))
        if not (0 <= x < blocked.shape[1] and 0 <= y < blocked.shape[0]) or blocked[y, x]:
            return False
        # Avoid diagonal shortcut through a blocked corner.
        if i > 0:
            lx, ly = last
            if x != lx and y != ly and (blocked[ly, x] or blocked[y, lx]):
                return False
        last = (x, y)
    return True


def simplify_cells(path: List[Cell], blocked: np.ndarray) -> List[Cell]:
    if len(path) <= 2:
        return path
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not segment_clear_grid(path[i], path[j], blocked):
            j -= 1
        out.append(path[j])
        i = j
    return out


def pixel_segment_clear(a: Point, b: Point, obstacle: np.ndarray, ignore_end_px: float = 0.0) -> bool:
    dx, dy = b[0] - a[0], b[1] - a[1]
    dist = math.hypot(dx, dy)
    n = max(1, int(math.ceil(dist)))
    for i in range(n + 1):
        t = i / n
        if ignore_end_px > 0 and dist * (1.0 - t) < ignore_end_px:
            continue
        x = int(round(a[0] + dx * t))
        y = int(round(a[1] + dy * t))
        if not (0 <= x < obstacle.shape[1] and 0 <= y < obstacle.shape[0]):
            return False
        if obstacle[y, x]:
            return False
    return True


def path_length(path: Sequence[Point]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


def smoothness(path: Sequence[Point]) -> float:
    if len(path) < 3:
        return 0.0
    angles = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(path, path[1:])]
    total = 0.0
    for a, b in zip(angles, angles[1:]):
        d = (b - a + math.pi) % (2 * math.pi) - math.pi
        total += abs(d)
    return total


def generate_targets(foe: Point, foe_angle: float, attack_ranges_px: Sequence[float]) -> List[Tuple[float, float, Point]]:
    """Three rear directions; only the radial distance is searched for robustness."""
    out = []
    for rel_deg in RELATIVE_TARGET_DEG:
        th = foe_angle + math.radians(rel_deg)
        for attack_range_px in attack_ranges_px:
            p = (foe[0] + attack_range_px * math.cos(th),
                 foe[1] + attack_range_px * math.sin(th))
            out.append((rel_deg, attack_range_px, p))
    return out


def plan(me: Point, foe: Point, foe_angle: float, raw_wall: np.ndarray, blocked: np.ndarray,
         grid_px: int = GRID_PX, attack_ranges_px: Sequence[float] = ATTACK_RANGES_PX) -> PlanResult:
    begin = time.perf_counter()
    start = nearest_free(point_to_cell(me, grid_px, blocked), blocked, max_r=5)
    if start is None:
        return PlanResult(False, [], None, None, 0, 0, (time.perf_counter()-begin)*1000,
                          reason="start_not_free")

    choices = []
    for rel_deg, attack_range_px, target in generate_targets(foe, foe_angle, attack_ranges_px):
        if not in_bounds_point(target):
            continue
        tc = point_to_cell(target, grid_px, blocked)
        if blocked[tc[1], tc[0]]:
            continue
        # Target must have direct bullet line of sight to the foe center.
        # Ignore a tiny final radius around foe center to avoid treating its adjacent wall as endpoint noise.
        if not pixel_segment_clear(target, foe, raw_wall, ignore_end_px=6.0):
            continue
        p_cells = astar(start, tc, blocked)
        if not p_cells:
            continue
        p_cells = simplify_cells(p_cells, blocked)
        p = [me] + [cell_to_point(c, grid_px) for c in p_cells[1:-1]] + [target]
        L = path_length(p)
        S = smoothness(p)
        choices.append((L, S, rel_deg, target, p))

    ms = (time.perf_counter() - begin) * 1000.0
    if not choices:
        return PlanResult(False, [], None, None, 0, 0, ms, reason="no_legal_candidate")
    choices.sort(key=lambda z: (z[0], z[1]))
    L, S, rel, target, p = choices[0]
    return PlanResult(True, p, target, rel, L, S, ms, valid_candidates=len(choices))


def load_track(path: str) -> List[dict]:
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) if v not in (None, "") else float("nan") for k, v in r.items()})
    return rows


def sample_ooda(rows: List[dict], period: float = OODA_PERIOD_S) -> List[dict]:
    if not rows:
        return []
    t0, t1 = rows[0]["t"], rows[-1]["t"]
    targets = []
    t = t0
    i = 0
    while t <= t1 + 1e-9:
        while i + 1 < len(rows) and abs(rows[i + 1]["t"] - t) <= abs(rows[i]["t"] - t):
            i += 1
        targets.append(rows[i])
        t += period
    return targets


def check_path_collision(path: Sequence[Point], inflated: np.ndarray) -> bool:
    return any(not pixel_segment_clear(a, b, inflated) for a, b in zip(path, path[1:]))


def replay(track_path: str, maze_path: str, polys_path: str, out_csv: str,
           preview_png: Optional[str] = None,
           grid_px: int = GRID_PX, safe_radius_px: int = TANK_SAFE_RADIUS_PX,
           attack_ranges_px: Sequence[float] = ATTACK_RANGES_PX) -> List[dict]:
    maze = load_maze(maze_path)
    polys = load_polys(polys_path)
    raw_wall, inflated, blocked = build_maps(maze, polys, safe_radius_px, grid_px)
    rows = load_track(track_path)
    frames = sample_ooda(rows)
    results = []
    preview_plans = []

    for cycle, r in enumerate(frames):
        me = (r["me_x"], r["me_y"])
        foe = (r["foe_x"], r["foe_y"])
        pr = plan(me, foe, r["foe_angle"], raw_wall, blocked, grid_px, attack_ranges_px)
        collides = bool(pr.success and check_path_collision(pr.path, inflated))
        results.append({
            "cycle": cycle,
            "t": round(r["t"], 3),
            "me_x": me[0], "me_y": me[1],
            "foe_x": foe[0], "foe_y": foe[1], "foe_angle": r["foe_angle"],
            "success": int(pr.success),
            "relative_deg": "" if pr.relative_deg is None else pr.relative_deg,
            "target_x": "" if pr.target is None else round(pr.target[0], 2),
            "target_y": "" if pr.target is None else round(pr.target[1], 2),
            "path_length_px": round(pr.path_length, 3),
            "smoothness_rad": round(pr.smoothness_rad, 4),
            "planning_ms": round(pr.planning_ms, 3),
            "valid_candidates": pr.valid_candidates,
            "collision": int(collides),
            "reason": pr.reason,
            "path_points": json.dumps([[round(x,1), round(y,1)] for x,y in pr.path], ensure_ascii=False),
        })
        if pr.success:
            preview_plans.append((cycle, r, pr))

    if results:
        with open(out_csv, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
            w.writeheader(); w.writerows(results)

    if preview_png:
        make_preview(preview_png, raw_wall, rows, preview_plans)
    return results


def make_preview(path: str, raw_wall: np.ndarray, track_rows: List[dict], plans):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(raw_wall, cmap="gray_r", origin="upper", extent=(0, MAP_PX, MAP_PX, 0), alpha=0.55)
    ax.plot([r["me_x"] for r in track_rows], [r["me_y"] for r in track_rows], lw=1.0, label="green/player actual")
    ax.plot([r["foe_x"] for r in track_rows], [r["foe_y"] for r in track_rows], lw=1.0, label="foe actual")
    # Plot every second successful plan. All path drawings use matplotlib default color cycle.
    for cycle, r, pr in plans:
        xs = [p[0] for p in pr.path]; ys = [p[1] for p in pr.path]
        ax.plot(xs, ys, lw=0.9, alpha=0.65)
        ax.scatter([pr.target[0]], [pr.target[1]], s=12)
    ax.set_xlim(0, MAP_PX); ax.set_ylim(MAP_PX, 0); ax.set_aspect("equal")
    ax.set_xlabel("x (px)"); ax.set_ylabel("y (px)")
    ax.set_title("Navigation MVP: 1 s tactical replanning replay")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout(); fig.savefig(path, dpi=150); plt.close(fig)


def find_pairs(data_root: str):
    pairs = []
    for track in sorted(glob.glob(os.path.join(data_root, "*", "track_*.csv"))):
        maze = os.path.join(os.path.dirname(track), os.path.basename(track).replace("track_", "maze_"))
        if os.path.exists(maze):
            pairs.append((track, maze))
    return pairs


def summarize(rows: List[dict]) -> dict:
    n = len(rows)
    ok = [r for r in rows if int(r["success"]) == 1]
    return {
        "cycles": n,
        "success_rate": (len(ok) / n) if n else 0.0,
        "collision_rate": (sum(int(r["collision"]) for r in rows) / n) if n else 0.0,
        "mean_path_px": float(np.mean([float(r["path_length_px"]) for r in ok])) if ok else float("nan"),
        "mean_smooth_rad": float(np.mean([float(r["smoothness_rad"]) for r in ok])) if ok else float("nan"),
        "mean_plan_ms": float(np.mean([float(r["planning_ms"]) for r in rows])) if rows else float("nan"),
        "p95_plan_ms": float(np.percentile([float(r["planning_ms"]) for r in rows], 95)) if rows else float("nan"),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", default=os.path.join(os.path.dirname(__file__), "data"))
    ap.add_argument("--polys", default=os.path.join(os.path.dirname(__file__), "tile_polys.json"))
    ap.add_argument("--out-dir", default=os.path.join(os.path.dirname(__file__), "mvp_output"))
    ap.add_argument("--grid-px", type=int, default=GRID_PX)
    ap.add_argument("--safe-radius", type=int, default=TANK_SAFE_RADIUS_PX)
    ap.add_argument("--attack-ranges", default=",".join(str(int(v)) for v in ATTACK_RANGES_PX),
                    help="comma separated tactical radii in pixels, e.g. 60,90,120,150,180")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    all_results = []
    summaries = []
    pairs = find_pairs(args.data_root)
    if not pairs:
        raise SystemExit("No track/maze pairs found")

    for track, maze in pairs:
        stamp = os.path.basename(track)[6:-4]
        out_csv = os.path.join(args.out_dir, f"plan_{stamp}.csv")
        preview = os.path.join(args.out_dir, f"preview_{stamp}.png")
        attack_ranges = tuple(float(v) for v in args.attack_ranges.split(",") if v.strip())
        rows = replay(track, maze, args.polys, out_csv, preview, args.grid_px, args.safe_radius, attack_ranges)
        s = summarize(rows); s["stamp"] = stamp
        summaries.append(s)
        for r in rows:
            rr = dict(r); rr["stamp"] = stamp; all_results.append(rr)
        print(stamp, json.dumps(s, ensure_ascii=False))

    summary_csv = os.path.join(args.out_dir, "summary.csv")
    with open(summary_csv, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=["stamp","cycles","success_rate","collision_rate","mean_path_px","mean_smooth_rad","mean_plan_ms","p95_plan_ms"])
        w.writeheader(); w.writerows(summaries)

    total = summarize(all_results)
    print("TOTAL", json.dumps(total, ensure_ascii=False))
    print("OUTPUT", args.out_dir)


if __name__ == "__main__":
    main()
