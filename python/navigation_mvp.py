# -*- coding: utf-8 -*-
"""TankTrouble navigation MVP (offline replay + reusable planner).

Current minimal tactical navigation:
1) Restore maze wall polygons from maze_*.csv + tile_polys.json
2) Inflate walls by a tank safety radius
3) Search five directions over the foe rear half-plane: -90°..-270°
4) Prefer attack points with LOS; allow rear staging points as a fallback
5) 8-neighbour A* + LOS path shortcut
6) Hold an already-good rear attack position to avoid oscillation
7) Penalize unnecessary tactical-side switching while keeping 1 s OODA
8) Replay track CSV at 1 s OODA intervals and export metrics / preview

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
# 坦克碰撞盒实测 31×20 → 半长 15.5 / 半高 10。
# 转弯预留与"盲目膨胀"不同：坦克矩形原地旋转 θ 扫过的包络是外接圆，
# 半径 = sqrt(15.5^2 + 10^2) ≈ 18.45px —— 这是"任意角度转弯不卡墙"的
# 精确 Minkowski 上界（用户验证后按此计算，而非拍脑袋调半径）。
# 若实验显示 18.45 封死窄通道，再考虑方向性预留（直行低/转弯高）。
TANK_SAFE_RADIUS_PX = 18.45
ATTACK_RANGES_PX = (60.0, 90.0, 120.0, 150.0, 180.0)
RELATIVE_TARGET_DEG = (-90.0, -135.0, -180.0, -225.0, -270.0)
OODA_PERIOD_S = 1.0
DIRECTION_SWITCH_PENALTY_PER_45_PX = 30.0
STAGING_PENALTY_PX = 80.0

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


def build_maps(maze: List[List[int]], polys: list, safe_radius_px: int, grid_px: int,
               turn_radius_px: Optional[float] = None,
               straight_radius_px: Optional[float] = None):
    """Return raw_wall mask, inflated_wall mask, conservative planning grid.

    方向性 Minkowski（转弯预留精确化）：
    - straight_radius_px（默认 = TANK_HW 10px，矩形半高）：直行段安全预留，
      A* 用它规划，避免 18.45px 圆形把窄通道堵死。
    - turn_radius_px（默认 = 外接圆 18.45px = sqrt(15.5^2+10^2)，坦克原地
      旋转 90° 扫过的精确包络）：转弯点校验用，确保直角转弯不卡墙。
    返回 (raw_wall, inflated, blocked_straight, blocked_turn, free_dist)；
    free_dist 为各像素到墙的距离（tube 管道校验用）。
    若未指定 turn/straight，则 blocked_straight = blocked_turn = 原单一膨胀网格
    （兼容旧调用，只返回 3 值时保持原行为）。
    """
    if turn_radius_px is None and straight_radius_px is None:
        straight_radius_px = safe_radius_px
        turn_radius_px = safe_radius_px
    elif turn_radius_px is None:
        turn_radius_px = straight_radius_px
    elif straight_radius_px is None:
        straight_radius_px = safe_radius_px

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

    def _inflate(radius: float) -> np.ndarray:
        return free_dist <= float(radius)

    inflated = _inflate(safe_radius_px)          # 兼容字段：主膨胀（straight）
    inflated_straight = _inflate(straight_radius_px)
    inflated_turn = _inflate(turn_radius_px)

    def _grid(inf: np.ndarray) -> np.ndarray:
        gw = math.ceil(MAP_PX / grid_px)
        gh = math.ceil(MAP_PX / grid_px)
        blocked = np.zeros((gh, gw), dtype=bool)
        for gy in range(gh):
            y0, y1 = gy * grid_px, min((gy + 1) * grid_px, MAP_PX)
            for gx in range(gw):
                x0, x1 = gx * grid_px, min((gx + 1) * grid_px, MAP_PX)
                # Conservative: any inflated obstacle pixel blocks this navigation cell.
                blocked[gy, gx] = bool(inf[y0:y1, x0:x1].any())
        return blocked

    blocked_straight = _grid(inflated_straight)
    blocked_turn = _grid(inflated_turn)
    return raw_wall, inflated, blocked_straight, blocked_turn, free_dist


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


# ---------------- 管状车道（Virtual Tube） ----------------
# 坦克 31×20：前进方向半长 15.5，横向半宽 10。
# 运动包线（swept volume）≠ 外接圆：
#   - 直行段包线宽 = 车宽 = 20px → 半宽 10
#   - 转弯段车体旋转，包线在弯内/弯外各外扩，取外接圆半宽 √(15.5²+10²)≈18.45
# 管道路径 = 用 Catmull-Rom 样条把折线中心线光滑化，逐采样点用"到墙距离"
# 校验包线是否与墙保持 ≥ CLEARANCE（6~8px）。
TUBE_CLEARANCE_PX = 7.0          # 管道外沿距墙最小间隙
TUBE_HALF_W = 10.0               # 直行段运动包线半宽（车宽 20/2）
TUBE_TURN_R = 18.45              # 转弯段包线（外接圆半宽，实测 R_turn）
TUBE_SAMPLE_STEP = 5.0           # 中心线采样步长（px）
TUBE_TURN_DEG = 25.0             # 转角超过此值视为转弯段


def catmull_rom_smooth(pts: Sequence[Point], step: float) -> List[Point]:
    """Catmull-Rom 样条把折线光滑为密集中心线点（不过顶点，切角圆滑）。"""
    if len(pts) < 3:
        return list(pts)
    out: List[Point] = [pts[0]]
    n = len(pts)
    for i in range(n - 1):
        p0 = pts[max(0, i - 1)]
        p1 = pts[i]
        p2 = pts[i + 1]
        p3 = pts[min(n - 1, i + 2)]
        seg_len = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        if seg_len < 1e-6:
            continue
        steps = max(1, int(math.ceil(seg_len / step)))
        for s in range(1, steps + 1):
            t = s / steps
            t2 = t * t
            t3 = t2 * t
            x = 0.5 * ((2 * p1[0]) +
                       (-p0[0] + p2[0]) * t +
                       (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2 +
                       (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3)
            y = 0.5 * ((2 * p1[1]) +
                       (-p0[1] + p2[1]) * t +
                       (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2 +
                       (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3)
            out.append((x, y))
    # 保证终点精确
    if out and pts[-1] != out[-1]:
        out.append(pts[-1])
    return out


def tube_radius_at(center: Point, prev: Point, nxt: Point) -> float:
    """该采样点的运动包线半径：转弯段用外接圆，直行段用车宽半宽。"""
    if prev is None or nxt is None:
        return TUBE_HALF_W
    a = math.atan2(prev[1] - center[1], prev[0] - center[0])
    b = math.atan2(nxt[1] - center[1], nxt[0] - center[0])
    d = abs((b - a + math.pi) % (2 * math.pi) - math.pi)
    if math.degrees(d) >= TUBE_TURN_DEG:
        return TUBE_TURN_R
    return TUBE_HALF_W


def build_tube(path: Sequence[Point], free_dist: np.ndarray,
               clearance: float = TUBE_CLEARANCE_PX) -> Optional[List[Point]]:
    """把折线路径构造成管状车道中心线。

    校验：中心线每个采样点（以运动包线半径 + clearance 为管壁到墙的距离）
    必须满足 free_dist >= tube_radius + clearance，否则返回 None（规划不出 tube）。
    free_dist 来自 build_maps 的 distance_transform_edt。
    """
    centerline = catmull_rom_smooth(list(path), TUBE_SAMPLE_STEP)
    if len(centerline) < 2:
        return None
    h, w = free_dist.shape
    for i, (cx, cy) in enumerate(centerline):
        gx = min(w - 1, max(0, int(round(cx))))
        gy = min(h - 1, max(0, int(round(cy))))
        if free_dist[gy, gx] < 1.0:
            return None   # 中心线本身碰到墙
        prev = centerline[i - 1] if i > 0 else None
        nxt = centerline[i + 1] if i + 1 < len(centerline) else None
        r = tube_radius_at((cx, cy), prev, nxt)
        if free_dist[gy, gx] < r + clearance:
            return None   # 运动包线距墙不足
    return centerline


def generate_targets(foe: Point, foe_angle: float, attack_ranges_px: Sequence[float]) -> List[Tuple[float, float, Point]]:
    """Five directions spanning the requested rear half-plane; radial distance is searched too."""
    out = []
    for rel_deg in RELATIVE_TARGET_DEG:
        th = foe_angle + math.radians(rel_deg)
        for attack_range_px in attack_ranges_px:
            p = (foe[0] + attack_range_px * math.cos(th),
                 foe[1] + attack_range_px * math.sin(th))
            out.append((rel_deg, attack_range_px, p))
    return out


def plan(me: Point, foe: Point, foe_angle: float, raw_wall: np.ndarray, blocked: np.ndarray,
         grid_px: int = GRID_PX, attack_ranges_px: Sequence[float] = ATTACK_RANGES_PX,
         preferred_rel_deg: Optional[float] = None,
         direction_switch_penalty_per_45_px: float = DIRECTION_SWITCH_PENALTY_PER_45_PX,
         staging_penalty_px: float = STAGING_PENALTY_PX,
         blocked_turn: Optional[np.ndarray] = None,
         free_dist: Optional[np.ndarray] = None) -> PlanResult:
    """Plan one OODA navigation path.

    The planner remains deliberately small. It searches fixed tactical samples in
    the foe rear half-plane, then ranks them with a cost that balances travel
    length, tactical-side stability and immediate firing LOS.

    方向性 Minkowski：
    - blocked（straight，窄 10px）：A* 网格，保留窄通道
    - blocked_turn（宽 18.45px 外接圆 = 旋转 90° 包络）：双参数校验。
      路径 LOS 简化后若转弯（转角 > TURN_CHECK_RAD）处穿 blocked_turn，
      则该候选作废（避免直角转弯卡墙），但直行段仍按窄网格通过。
    """
    begin = time.perf_counter()
    start = nearest_free(point_to_cell(me, grid_px, blocked), blocked, max_r=5)
    if start is None:
        return PlanResult(False, [], None, None, 0, 0, (time.perf_counter()-begin)*1000,
                          reason="start_not_free")

    # If we already occupy a useful rear-half attack position, do not chase a
    # newly rotated discrete candidate every OODA tick.  Re-evaluation still
    # occurs at 1 Hz; movement resumes as soon as this position becomes invalid.
    d_foe = math.hypot(me[0] - foe[0], me[1] - foe[1])
    bearing = math.atan2(me[1] - foe[1], me[0] - foe[0])
    rel_now = math.degrees((bearing - foe_angle + math.pi) % (2 * math.pi) - math.pi)
    if rel_now > 0:
        rel_now -= 360.0
    hold_min = max(35.0, min(attack_ranges_px) - 15.0) if attack_ranges_px else 45.0
    hold_max = (max(attack_ranges_px) + 20.0) if attack_ranges_px else 200.0
    if (-270.0 <= rel_now <= -90.0 and hold_min <= d_foe <= hold_max and
            pixel_segment_clear(me, foe, raw_wall, ignore_end_px=6.0)):
        ms = (time.perf_counter() - begin) * 1000.0
        return PlanResult(True, [me], me, rel_now, 0.0, 0.0, ms,
                          reason="hold_attack_position", valid_candidates=1)

    # Build cheap lower bounds first. A* is only run for candidates that can
    # still beat the best tactical cost found so far. This preserves the same
    # optimum while avoiding up to 25 full A* searches every OODA cycle.
    candidate_meta = []
    for rel_deg, attack_range_px, target in generate_targets(foe, foe_angle, attack_ranges_px):
        if not in_bounds_point(target):
            continue
        tc = point_to_cell(target, grid_px, blocked)
        if blocked[tc[1], tc[0]]:
            continue

        is_staging = not pixel_segment_clear(target, foe, raw_wall, ignore_end_px=6.0)
        switch_cost = 0.0
        if preferred_rel_deg is not None:
            switch_cost = direction_switch_penalty_per_45_px * (abs(rel_deg - preferred_rel_deg) / 45.0)
        tactical_extra = switch_cost + (staging_penalty_px if is_staging else 0.0)
        euclid_lb = math.hypot(target[0]-me[0], target[1]-me[1])
        candidate_meta.append((euclid_lb + tactical_extra, tactical_extra,
                               is_staging, rel_deg, target, tc))

    if not candidate_meta:
        ms = (time.perf_counter() - begin) * 1000.0
        return PlanResult(False, [], None, None, 0, 0, ms, reason="no_rear_candidate")

    candidate_meta.sort(key=lambda z: z[0])
    best = None
    best_cost = float('inf')
    pathable = 0

    for lower_bound, tactical_extra, is_staging, rel_deg, target, tc in candidate_meta:
        if lower_bound >= best_cost - 1e-9:
            break
        p_cells = astar(start, tc, blocked)
        if not p_cells:
            continue
        pathable += 1
        p_cells = simplify_cells(p_cells, blocked)
        p = [me] + [cell_to_point(c, grid_px) for c in p_cells[1:-1]] + [target]
        # 方向性 Minkowski：A* 用窄网格（保留窄通道），但路径的直角转弯
        # 必须通过外接圆（blocked_turn）校验，否则该候选作废（会卡墙）。
        if not path_turn_check(p, blocked_turn):
            continue
        # 管状车道（Virtual Tube）：把折线光滑成运动包线中心线，
        # 且整段包线距墙 ≥ clearance。规划不出 tube 的候选作废（不硬转圈），
        # 由 fallback_chase 兜底。成功则路径 = 光滑中心线（转弯不卡）。
        tube = build_tube(p, free_dist) if free_dist is not None else None
        if tube is None:
            continue
        L = path_length(tube)
        S = smoothness(tube)
        total_cost = L + tactical_extra
        key = (total_cost, S)
        if best is None or key < best[0]:
            best_cost = total_cost
            best = (key, is_staging, L, S, rel_deg, target, tube)

    ms = (time.perf_counter() - begin) * 1000.0
    if best is None:
        # 回退：所有尾部候选都不可达（敌人被围/死角），改规划到敌人附近
        # 最近的可达点（追敌本身），避免 bot 硬冲死路无所事事。
        foe_c = point_to_cell(foe, grid_px, blocked)
        foe_free = nearest_free(foe_c, blocked, max_r=8)
        if foe_free is not None:
            p_cells = astar(start, foe_free, blocked)
            if p_cells:
                p_cells = simplify_cells(p_cells, blocked)
                p = [me] + [cell_to_point(c, grid_px) for c in p_cells[1:-1]] + [foe]
                tube = build_tube(p, free_dist) if free_dist is not None else None
                if tube is not None:
                    p = tube
                L = path_length(p)
                S0 = smoothness(p)
                ms2 = (time.perf_counter() - begin) * 1000.0
                return PlanResult(True, p, foe, None, L, S0, ms2,
                                  reason="fallback_chase", valid_candidates=pathable)
        return PlanResult(False, [], None, None, 0, 0, ms, reason="no_rear_candidate")

    _, is_staging, L, S, rel, target, p = best
    reason = "staging_no_los" if is_staging else ""
    return PlanResult(True, p, target, rel, L, S, ms, reason=reason,
                      valid_candidates=pathable)


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


def path_turn_check(path: Sequence[Point], blocked_turn: np.ndarray,
                    turn_min_deg: float = 45.0) -> bool:
    """路径的尖锐转角是否满足宽膨胀（blocked_turn）校验。

    迷宫通道宽 57px、坦克宽 31px，弯道净空 ≈ 26px > 18.45px，
    所以直角转弯本身是可行的。turncheck 只拦截"结构上转不过去"的
    候选：最尖锐的转角（> 45°）若其转角点落在 blocked_turn 里则拒绝。
    直行段与一般换向仍由窄网格（blocked）保证，避免过度拒绝导致
    所有候选都被扣掉（退化到永久 fallback）。
    """
    if blocked_turn is None or len(path) < 3:
        return True
    angles = []
    for a, b in zip(path, path[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        if math.hypot(dx, dy) < 1e-9:
            angles.append(None)
        else:
            angles.append(math.atan2(dy, dx))
    worst = 0.0
    worst_idx = -1
    for i in range(1, len(angles)):
        if angles[i] is None or angles[i - 1] is None:
            continue
        d = abs(((angles[i] - angles[i - 1] + math.pi) % (2 * math.pi)) - math.pi)
        if d > worst:
            worst = d
            worst_idx = i
    if worst_idx < 0 or math.degrees(worst) < turn_min_deg:
        return True
    # 只校验最尖锐转角点（path[worst_idx]）：必须在宽网格可走
    for j in (worst_idx - 1, worst_idx, worst_idx + 1):
        if 0 <= j < len(path):
            px, py = path[j]
            gx = min(blocked_turn.shape[1] - 1, max(0, int(px // GRID_PX)))
            gy = min(blocked_turn.shape[0] - 1, max(0, int(py // GRID_PX)))
            if blocked_turn[gy, gx]:
                return False
    return True


def replay(track_path: str, maze_path: str, polys_path: str, out_csv: str,
           preview_png: Optional[str] = None,
           grid_px: int = GRID_PX, safe_radius_px: int = TANK_SAFE_RADIUS_PX,
           attack_ranges_px: Sequence[float] = ATTACK_RANGES_PX) -> List[dict]:
    maze = load_maze(maze_path)
    polys = load_polys(polys_path)
    raw_wall, inflated, blocked, _, free_dist = build_maps(maze, polys, safe_radius_px, grid_px)
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
