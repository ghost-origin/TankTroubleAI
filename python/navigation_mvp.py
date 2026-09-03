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
from trajectory_refinement import refine_polyline_clearance
from bot_config import CHASE_ALT_DETOUR_RATIO   # 次优路线长度上限（bot_config.py 调参）
try:
    from scipy.ndimage import distance_transform_edt
except Exception:
    def distance_transform_edt(mask: np.ndarray) -> np.ndarray:
        """Small no-SciPy fallback: 8-neighbour Euclidean distance to obstacles."""
        h, w = mask.shape
        dist = np.full((h, w), float("inf"), dtype=float)
        pq = []
        zeros = np.argwhere(~mask)
        for y, x in zeros:
            dist[y, x] = 0.0
            heapq.heappush(pq, (0.0, int(y), int(x)))
        dirs = (
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
        )
        while pq:
            d, y, x = heapq.heappop(pq)
            if d != dist[y, x]:
                continue
            for dy, dx, cost in dirs:
                yy, xx = y + dy, x + dx
                nd = d + cost
                if 0 <= yy < h and 0 <= xx < w and nd < dist[yy, xx]:
                    dist[yy, xx] = nd
                    heapq.heappush(pq, (nd, yy, xx))
        return dist

TILE_PX = 57
MAZE_SIZE = 10
MAP_PX = TILE_PX * MAZE_SIZE
TILE_ID_MASK = 0x1FFFFFFF
FLAG_H, FLAG_V, FLAG_D = 0x80000000, 0x40000000, 0x20000000

# Deliberately simple / editable MVP parameters.
GRID_PX = 5
# Coarse A* needs a small anti-wall-hugging pad, but not the tank's full 10 px
# half-width because the final swept rectangle validates the body exactly.
A_STAR_TOPOLOGY_RADIUS_PX = 10.5    # A* 只负责拓扑连通（中心线不在墙里即可），
                                   # 不承担碰撞保证。贴墙问题由 refine 阶段的
                                   # push_off_walls（free_dist 梯度推离）修复，
                                   # 最终裁决是 swept_rectangle_path_clear。
# 坦克碰撞盒实测 31×20 → 半长 15.5 / 半高 10。
# 转弯预留与"盲目膨胀"不同：坦克矩形原地旋转 θ 扫过的包络是外接圆，
# 半径 = sqrt(15.5^2 + 10^2) ≈ 18.45px —— 这是"任意角度转弯不卡墙"的
# 精确 Minkowski 上界（用户验证后按此计算，而非拍脑袋调半径）。
# 若实验显示 18.45 封死窄通道，再考虑方向性预留（直行低/转弯高）。
TANK_SAFE_RADIUS_PX = 18.45
ATTACK_RANGES_PX = (60.0, 90.0, 120.0, 150.0, 180.0)
REAR_RELATIVE_TARGET_DEG = (-90.0, -135.0, -180.0, -225.0, -270.0)
TACTICAL_RELATIVE_TARGET_DEG = (0.0, -45.0, -90.0, -135.0, -180.0, -225.0, -270.0, -315.0)
REPOSITION_RELATIVE_TARGET_DEG = TACTICAL_RELATIVE_TARGET_DEG
REPOSITION_RANGES_PX = (90.0, 150.0)
OODA_PERIOD_S = 1.0
DIRECTION_SWITCH_PENALTY_PER_45_PX = 30.0
STAGING_PENALTY_PX = 80.0

# P0-2 receding-horizon geometry. These bound how much of a global A* route is
# refined and validated now; they do not relax any footprint safety parameter.
WINDOW_MIN_PX = 70.0
WINDOW_EMERGENCY_MIN_PX = 40.0
WINDOW_TARGET_PX = 160.0
WINDOW_MAX_PX = 170.0
# 125px/s 满速时，45px 仅有约 0.36s 缓冲。提高到 105px，使正常 160px
# 执行窗口在剩余约 104px（65%）时便开始续接，规划失败也有多次重试机会。
WINDOW_REPLAN_REMAINING_PX = 105.0
WINDOW_HEADING_GATE_DEG = 45.0

TACTICAL_MODE_REAR_ONLY = "rear_only"
TACTICAL_MODE_V1 = "tactical_v1"
TACTICAL_MODE_CHASE = "chase"
DEFAULT_TACTICAL_MODE = TACTICAL_MODE_CHASE

# ---- chase 走"第二近"路线（用户方案 A，最小改动）----
# 每次 chase 规划：A* 最短路线 → 把最短路线两侧封锁带封掉后重跑 A* → 若存在
# "第二近"（次优）路线且长度 ≤ 最短×CHASE_ALT_DETOUR_RATIO → 走近第二条，
# 接触角不同（另一条走廊/房间），火控有更多锁定时机。封锁带 ≈ 走廊宽
# （±CHASE_ALT_BLOCK_RADIUS_PX），否则 A* 只是平行并线、不是换走廊。
# 无第二路线 / 超长 / 规划失败 → 原样走最短路线（回退无行为变化）。
# 次优路线长度上限 CHASE_ALT_DETOUR_RATIO（=2.5）在 bot_config.py（调参集中地）
CHASE_ALT_BLOCK_RADIUS_PX = 20.0   # 封锁带半径（最短路线两侧各半宽）
CHASE_ALT_BLOCK_CELLS = int(round(CHASE_ALT_BLOCK_RADIUS_PX / GRID_PX))  # 4 格
CHASE_ALT_BLOCK_MARGIN_CELLS = CHASE_ALT_BLOCK_CELLS + 1  # 首末各留 5 格（25px）
                                    # 不封：出发/到达口袋在封锁带外，否则次优 A*
                                    # 的起点/终点被自己的封带堵死（永远无第二路线）

# Tactical V1 scores are expressed in equivalent path pixels. Rear is a reward,
# not a reachability constraint, so a feasible flank can beat an unreachable rear.
TACTICAL_REWARD_PX = {"rear": 145.0, "flank": 85.0, "front": 20.0, "reposition": 0.0}
LOS_REWARD_PX = 45.0
NO_LOS_PENALTY_PX = 40.0
REPOSITION_PENALTY_PX = 160.0     # 重定位只是保底，蹲家蠕动会压低移动量
DEAD_END_PENALTY_PX = 85.0
OPEN_DIRECTION_REWARD_PX = 10.0
TARGET_SWITCH_PENALTY_PX = 55.0
TARGET_SWITCH_DISTANCE_PX = 60.0  # 换目标代价判定距离调大，减少 1s 级目标抖动
OPEN_DIRECTION_SAMPLE_PX = 50.0

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
    tactical_mode: str = ""
    target_type: str = ""
    candidate_count: int = 0
    reachable_candidates: int = 0
    rear_candidates: int = 0
    rear_reachable_candidates: int = 0
    line_of_sight: bool = False
    open_directions: int = 0
    tactical_score: float = 0.0
    switch_cost: float = 0.0
    path_source: str = ""
    path_validated: bool = False
    validation_reason: str = ""
    raw_path_points: int = 0
    executed_path_points: int = 0
    global_path_length: float = 0.0
    window_path_length: float = 0.0
    window_target_length: float = 0.0
    window_lookahead_length: float = 0.0
    window_goal_reached: bool = False

    def __post_init__(self) -> None:
        if self.raw_path_points == 0 and self.path:
            self.raw_path_points = len(self.path)
        if self.executed_path_points == 0 and self.path:
            self.executed_path_points = len(self.path)
        if not self.path_source:
            self.path_source = self.target_type or self.reason
        if self.success and (not self.path_validated or not self.path):
            self.success = False
            self.path = []
            self.path_length = 0.0
            self.smoothness_rad = 0.0
            self.executed_path_points = 0
            if not self.reason:
                self.reason = "planner_contract_violation"
            self.validation_reason = self.validation_reason or "missing_path_validation"


@dataclass
class ExecutionWindow:
    path: List[Point]
    global_path_length: float
    window_path_length: float
    target_length: float
    lookahead_length: float
    goal_reached: bool
    validation_reason: str


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


def planning_grid_from_free_dist(free_dist: np.ndarray, grid_px: int,
                                 radius_px: float) -> np.ndarray:
    """Build a coarse topology grid from one explicit wall-distance radius."""
    occupied = free_dist <= float(radius_px)
    h, w = occupied.shape
    gw = math.ceil(w / grid_px)
    gh = math.ceil(h / grid_px)
    blocked = np.zeros((gh, gw), dtype=bool)
    for gy in range(gh):
        y0, y1 = gy * grid_px, min((gy + 1) * grid_px, h)
        for gx in range(gw):
            x0, x1 = gx * grid_px, min((gx + 1) * grid_px, w)
            blocked[gy, gx] = bool(occupied[y0:y1, x0:x1].any())
    return blocked


def build_maps(maze: List[List[int]], polys: list, safe_radius_px: int, grid_px: int,
               turn_radius_px: Optional[float] = None,
               straight_radius_px: Optional[float] = None):
    """Return raw_wall mask, inflated_wall mask, conservative planning grid.

    方向性 Minkowski（转弯预留精确化）：
    - straight_radius_px controls only the coarse A* topology grid. Online P0-2
      passes zero here because the final swept rectangle is the single safety gate.
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

    # World boundary is also an obstacle: 2px 太薄——坦克贴着模型边界被拱出去后
    # 就会"出界跑图"（见 nav_headless_fix12 match_005 全图外圈游荡）。
    # 加厚成 26px 的环形禁带，规划/管道验证都不可能把坦克引出迷宫。
    draw.rectangle([0, 0, MAP_PX - 1, MAP_PX - 1], outline=255, width=26)
    raw_wall = np.asarray(img, dtype=np.uint8) > 0

    free_dist = distance_transform_edt(~raw_wall)

    def _inflate(radius: float) -> np.ndarray:
        return free_dist <= float(radius)

    inflated = _inflate(safe_radius_px)          # 兼容字段：主膨胀（straight）
    blocked_straight = planning_grid_from_free_dist(free_dist, grid_px, straight_radius_px)
    blocked_turn = planning_grid_from_free_dist(free_dist, grid_px, turn_radius_px)
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


def extract_path_prefix(path: Sequence[Point], distance_px: float) -> List[Point]:
    """Return a polyline prefix ending exactly at the requested arc length."""
    pts = list(path)
    if len(pts) < 2 or distance_px <= 0.0:
        return pts[:1]
    out = [pts[0]]
    travelled = 0.0
    for a, b in zip(pts, pts[1:]):
        segment = _dist(a, b)
        if segment <= 1e-9:
            continue
        remaining = distance_px - travelled
        if segment <= remaining + 1e-9:
            out.append(b)
            travelled += segment
            continue
        out.append(_lerp(a, b, max(0.0, min(1.0, remaining / segment))))
        break
    return out


def smoothness(path: Sequence[Point]) -> float:
    if len(path) < 3:
        return 0.0
    angles = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(path, path[1:])]
    total = 0.0
    for a, b in zip(angles, angles[1:]):
        d = (b - a + math.pi) % (2 * math.pi) - math.pi
        total += abs(d)
    return total


# ---------------- Virtual Tube V2：Swept Rectangle Footprint ----------------
# 坦克真实碰撞盒 31×20：沿车头方向半长 15.5，横向半宽 10。
# V2 不再用中心线 + 固定圆半径作为精确判定，而是：
#   1) A* 仍然输出折线；
#   2) 每个局部转角用 C1 cubic Bezier 候选圆滑，入/出弯切线匹配折线方向；
#   3) 候选路径必须满足最小转弯半径硬约束；
#   4) 沿路径生成带朝向的矩形车身 footprint，检查四角 + 四边；
#   5) 相邻姿态之间按位移和角度自适应补采样，近似 swept area。
TUBE_CLEARANCE_PX = 7.0
TUBE_HALF_W = 10.0
TANK_HALF_LENGTH_PX = 15.5
TANK_HALF_WIDTH_PX = 10.0
TANK_BODY_RADIUS_PX = math.hypot(TANK_HALF_LENGTH_PX, TANK_HALF_WIDTH_PX)
TUBE_TURN_R = TANK_BODY_RADIUS_PX
TUBE_SAMPLE_STEP = 4.0
FOOTPRINT_EDGE_STEP_PX = 1.0
FOOTPRINT_SWEEP_STEP_PX = 1.0
# 扫掠矩形采样容差：0 = 真实车体（与模拟/游戏碰撞口径一致）。
# 正值内缩曾允许窄口通过（规划过、模拟撞 —— 口径不一致的系统性碰撞源）。
FOOTPRINT_WRAP_SHRINK = 0.0
MIN_TURN_RADIUS_PX = 4.0     # 仅用于 Bezier 拐角切距（need_cut 基数）。
                             # 不参与候选拒绝：行驶转弯半径 R=speed/steerSpeed，
                             # 由执行端"弯前减速到 min_r*STEER_SPEED"配合实现，
                             # 急弯停车/低速可过（原地转向半径≈0）。
MIN_STRAIGHT_BETWEEN_CORNERS = 14.0   # 连续弯之间的直行衔接段最小长度（车体摆正）
TUBE_MIN_CLEARANCE = 14.0             # 曲线中心线最小 clearance（半宽10+2.5容差+1.5余量）
TURN_EPS_DEG = 6.0


def _wrap_rad(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _dist(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _lerp(a: Point, b: Point, t: float) -> Point:
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def _unit(a: Point, b: Point) -> Optional[Point]:
    d = _dist(a, b)
    if d < 1e-9:
        return None
    return ((b[0] - a[0]) / d, (b[1] - a[1]) / d)


def _append_line(out: List[Point], a: Point, b: Point, step: float) -> None:
    d = _dist(a, b)
    n = max(1, int(math.ceil(d / max(step, 1e-6))))
    for i in range(1, n + 1):
        out.append(_lerp(a, b, i / n))


def _cubic_point(p0: Point, p1: Point, p2: Point, p3: Point, t: float) -> Point:
    u = 1.0 - t
    b0 = u * u * u
    b1 = 3.0 * u * u * t
    b2 = 3.0 * u * t * t
    b3 = t * t * t
    return (
        b0 * p0[0] + b1 * p1[0] + b2 * p2[0] + b3 * p3[0],
        b0 * p0[1] + b1 * p1[1] + b2 * p2[1] + b3 * p3[1],
    )


def _append_cubic(out: List[Point], p0: Point, p1: Point, p2: Point, p3: Point,
                  step: float) -> None:
    chord = _dist(p0, p3)
    control = _dist(p0, p1) + _dist(p1, p2) + _dist(p2, p3)
    n = max(4, int(math.ceil(max(chord, control) / max(step, 1e-6))))
    for i in range(1, n + 1):
        out.append(_cubic_point(p0, p1, p2, p3, i / n))


def _turn_angle(v_in: Point, v_out: Point) -> float:
    a0 = math.atan2(v_in[1], v_in[0])
    a1 = math.atan2(v_out[1], v_out[0])
    return abs(_wrap_rad(a1 - a0))


def _bezier_polyline(pts: Sequence[Point], step: float, radius_scale: float,
                     handle_scale: float) -> Optional[List[Point]]:
    clean = [p for i, p in enumerate(pts) if i == 0 or _dist(p, pts[i - 1]) > 1e-6]
    if len(clean) < 3:
        return list(clean)

    infos = []
    for i in range(1, len(clean) - 1):
        prev, cur, nxt = clean[i - 1], clean[i], clean[i + 1]
        vin = _unit(prev, cur)
        vout = _unit(cur, nxt)
        if vin is None or vout is None:
            infos.append(None)
            continue
        turn = _turn_angle(vin, vout)
        if math.degrees(turn) < TURN_EPS_DEG:
            infos.append(None)
            continue
        len_in = _dist(prev, cur)
        len_out = _dist(cur, nxt)
        need_cut = MIN_TURN_RADIUS_PX * math.tan(turn / 2.0) * radius_scale
        cut = min(0.9 * len_in, 0.9 * len_out, max(8.0, need_cut))
        infos.append(dict(cur=cur, vin=vin, vout=vout, turn=turn, cut=cut))

    cuts = [0.0 if info is None else float(info["cut"]) for info in infos]
    # 连续弯衔接：两个拐点共享的直线段必须保留 ≥ MIN_STRAIGHT_BETWEEN_CORNERS
    # 的直行衔接（车体摆正），否则出弯即入弯 → 转向跟不上 → 外切撞墙。
    # 段太短时等比压缩两侧切角；段长 < 最小衔接长度则两拐点退化为直线通过。
    for seg_i in range(len(clean) - 1):
        seg_len = _dist(clean[seg_i], clean[seg_i + 1])
        left_corner = seg_i - 1 if 1 <= seg_i <= len(clean) - 2 else None
        right_corner = seg_i if 1 <= seg_i + 1 <= len(clean) - 2 else None
        used = 0.0
        if left_corner is not None:
            used += cuts[left_corner]
        if right_corner is not None:
            used += cuts[right_corner]
        limit = max(0.0, seg_len - MIN_STRAIGHT_BETWEEN_CORNERS)
        if used > limit and used > 1e-9:
            scale = limit / used
            if left_corner is not None:
                cuts[left_corner] *= scale
            if right_corner is not None:
                cuts[right_corner] *= scale

    corners = []
    for info, cut in zip(infos, cuts):
        if info is None:
            corners.append(None)
            continue
        cur = info["cur"]
        vin = info["vin"]
        vout = info["vout"]
        turn = info["turn"]
        if cut < 4.0:
            corners.append(None)
            continue
        radius = cut / max(math.tan(turn / 2.0), 1e-6)
        handle = (4.0 / 3.0) * math.tan(turn / 4.0) * radius * handle_scale
        q0 = (cur[0] - vin[0] * cut, cur[1] - vin[1] * cut)
        q3 = (cur[0] + vout[0] * cut, cur[1] + vout[1] * cut)
        c1 = (q0[0] + vin[0] * handle, q0[1] + vin[1] * handle)
        c2 = (q3[0] - vout[0] * handle, q3[1] - vout[1] * handle)
        corners.append((q0, c1, c2, q3))

    out: List[Point] = [clean[0]]
    cursor = clean[0]
    for i in range(1, len(clean) - 1):
        spec = corners[i - 1]
        if spec is None:
            _append_line(out, cursor, clean[i], step)
            cursor = clean[i]
            continue
        q0, c1, c2, q3 = spec
        _append_line(out, cursor, q0, step)
        _append_cubic(out, q0, c1, c2, q3, step)
        cursor = q3
    _append_line(out, cursor, clean[-1], step)
    return out


def catmull_rom_smooth(pts: Sequence[Point], step: float) -> List[Point]:
    """兼容旧入口名；V2 默认生成 C1 cubic Bezier 转弯中心线。"""
    path = _bezier_polyline(pts, step, radius_scale=1.1, handle_scale=1.0)
    return path if path is not None else list(pts)


def _path_headings(path: Sequence[Point]) -> List[float]:
    if len(path) < 2:
        return [0.0 for _ in path]
    headings: List[float] = []
    for i, p in enumerate(path):
        if i == 0:
            q = path[1]
            headings.append(math.atan2(q[1] - p[1], q[0] - p[0]))
        elif i == len(path) - 1:
            q = path[i - 1]
            headings.append(math.atan2(p[1] - q[1], p[0] - q[0]))
        else:
            a, b = path[i - 1], path[i + 1]
            headings.append(math.atan2(b[1] - a[1], b[0] - a[0]))
    return headings


def tank_footprint_corners(x: float, y: float, heading: float,
                           shrink: float = 0.0) -> List[Point]:
    """矩形车身四角。shrink>0 时向内收缩，用于"允许轻微贴边"的宽容校验。"""
    ca, sa = math.cos(heading), math.sin(heading)
    forward = (ca, sa)
    side = (-sa, ca)
    hl = max(0.0, TANK_HALF_LENGTH_PX - shrink)
    hw = max(0.0, TANK_HALF_WIDTH_PX - shrink)
    corners = []
    for lf, ls in ((1, -1), (1, 1), (-1, 1), (-1, -1)):
        corners.append((
            x + forward[0] * hl * lf + side[0] * hw * ls,
            y + forward[1] * hl * lf + side[1] * hw * ls,
        ))
    return corners


def footprint_collides(raw_wall: np.ndarray, x: float, y: float, heading: float,
                       edge_step: float = FOOTPRINT_EDGE_STEP_PX,
                       shrink: float = 0.0) -> bool:
    """检查矩形车身的四角和四边是否与墙重叠。

    shrink>0 时矩形内缩后检查 —— 对应"允许轻微贴边/擦碰"的宽容判定。
    游戏实际：坦克碰墙会反弹/滑动，砖砖擦碰是正常走迷宫的一部分（内置 AI 亦然）。
    严格零碰撞会把所有贴边路径全部拒绝（footprint_invalid 的根因）。
    """
    corners = tank_footprint_corners(x, y, heading, shrink=shrink)
    h, w = raw_wall.shape
    for cx, cy in corners:
        ix, iy = int(round(cx)), int(round(cy))
        if ix < 0 or iy < 0 or ix >= w or iy >= h or raw_wall[iy, ix]:
            return True
    for a, b in zip(corners, corners[1:] + corners[:1]):
        d = _dist(a, b)
        n = max(1, int(math.ceil(d / max(edge_step, 1e-6))))
        for i in range(n + 1):
            x0, y0 = _lerp(a, b, i / n)
            ix, iy = int(round(x0)), int(round(y0))
            if ix < 0 or iy < 0 or ix >= w or iy >= h or raw_wall[iy, ix]:
                return True
    return False


def swept_rectangle_path_clear(path: Sequence[Point], raw_wall: np.ndarray,
                               shrink: float = 0.0) -> bool:
    """沿中心线姿态检查车身 footprint；相邻姿态自适应补采样近似扫掠区。"""
    if len(path) < 2:
        return False
    headings = _path_headings(path)
    for p, th in zip(path, headings):
        if footprint_collides(raw_wall, p[0], p[1], th, shrink=shrink):
            return False
    for a, b, ta, tb in zip(path, path[1:], headings, headings[1:]):
        move = _dist(a, b)
        turn_arc = abs(_wrap_rad(tb - ta)) * TANK_BODY_RADIUS_PX
        n = max(1, int(math.ceil(max(move, turn_arc) / FOOTPRINT_SWEEP_STEP_PX)))
        for i in range(1, n):
            u = i / n
            p = _lerp(a, b, u)
            th = ta + _wrap_rad(tb - ta) * u
            if footprint_collides(raw_wall, p[0], p[1], th, shrink=shrink):
                return False
    return True


def min_curve_radius(path: Sequence[Point]) -> Optional[float]:
    vals = []
    pts = list(path)
    for i in range(len(pts) - 2):
        a, b, c = pts[i], pts[i + 1], pts[i + 2]
        ab, bc, ca = _dist(a, b), _dist(b, c), _dist(c, a)
        if min(ab, bc) < 0.5:
            continue
        area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        if area2 < 1e-5:
            continue
        r = ab * bc * ca / (2.0 * area2)
        if 1.0 < r < 10000.0:
            vals.append(r)
    return min(vals) if vals else None


def curvature_variation(path: Sequence[Point]) -> float:
    headings = []
    for a, b in zip(path, path[1:]):
        if _dist(a, b) > 1e-6:
            headings.append(math.atan2(b[1] - a[1], b[0] - a[0]))
    turns = [abs(_wrap_rad(b - a)) for a, b in zip(headings, headings[1:])]
    return sum(abs(b - a) for a, b in zip(turns, turns[1:]))


def footprint_clearance(path: Sequence[Point], free_dist: Optional[np.ndarray]) -> float:
    if free_dist is None or len(path) < 2:
        return 0.0
    h, w = free_dist.shape
    best = float("inf")
    for p, th in zip(path, _path_headings(path)):
        for cx, cy in tank_footprint_corners(p[0], p[1], th):
            ix = min(w - 1, max(0, int(round(cx))))
            iy = min(h - 1, max(0, int(round(cy))))
            best = min(best, float(free_dist[iy, ix]))
    return best if best != float("inf") else 0.0


def _legacy_tube_coarse_ok(path: Sequence[Point], free_dist: Optional[np.ndarray]) -> bool:
    """旧 Virtual Tube 只作为极粗过滤：中心线不能贴墙或越界。"""
    if free_dist is None:
        return True
    h, w = free_dist.shape
    for cx, cy in path:
        gx = min(w - 1, max(0, int(round(cx))))
        gy = min(h - 1, max(0, int(round(cy))))
        if free_dist[gy, gx] < 1.0:
            return False
    return True


def build_tube(path: Sequence[Point], free_dist: Optional[np.ndarray],
               clearance: float = TUBE_CLEARANCE_PX,
               raw_wall: Optional[np.ndarray] = None) -> Optional[List[Point]]:
    """构造 V2 C1 中心线；精确可行性以 swept rectangle footprint 为准。"""
    if len(path) < 2:
        return None
    if len(path) < 3:
        candidate = list(path)
        if raw_wall is not None and not swept_rectangle_path_clear(candidate, raw_wall, shrink=FOOTPRINT_WRAP_SHRINK):
            return None
        return candidate

    # 性能关键（profile 实测）：swept_rectangle_path_clear 单次 8-14ms（1px 步长），
    # 15 组参数各跑一次 = 130ms+，plan() 每 0.2s 一次 → 响应卡顿的根源。
    # 扫掠验证移到组合循环外：先按成本排序（评分便宜），再按成本序扫掠，
    # 第一个通过即返回 —— 与"扫掠通过的候选中取最小成本"完全等价，
    # 常见情形只跑 1 次扫掠（原 15 次）。
    ranked = []
    for radius_scale in (1.25, 1.1, 1.45, 0.95, 1.7):
        for handle_scale in (1.0, 0.85, 1.15):
            candidate = _bezier_polyline(path, TUBE_SAMPLE_STEP, radius_scale, handle_scale)
            if candidate is None or len(candidate) < 2:
                continue
            if not _legacy_tube_coarse_ok(candidate, free_dist):
                continue
            # 曲线级推离：Bezier 内切会把折线 clearance(16.5px) 削到 11px 级，
            # 真实车体间隙只剩 1-2px（arena goal 尾帧 clear=11.2 的碰撞根源）。
            # 对曲线再推一次，保证中心线 clearance ≥ TUBE_MIN_CLEARANCE。
            candidate = push_off_walls(candidate, free_dist, min_clearance=TUBE_MIN_CLEARANCE)
            clear = footprint_clearance(candidate, free_dist)
            soft_clearance_penalty = max(0.0, clearance - clear) * 20.0
            cost = path_length(candidate) + smoothness(candidate) * 18.0
            cost += curvature_variation(candidate) * 10.0 + soft_clearance_penalty
            ranked.append((cost, candidate))
    if not ranked:
        return None
    ranked.sort(key=lambda item: item[0])
    # 不再用 min_curve_radius 拒绝候选：坦克可原地转向，转弯半径与
    # 碰撞无关；碰撞安全由 swept_rectangle_path_clear 全权负责。
    if raw_wall is None:
        return ranked[0][1]
    for _cost, candidate in ranked:
        if swept_rectangle_path_clear(candidate, raw_wall, shrink=FOOTPRINT_WRAP_SHRINK):
            return candidate
    return None


def build_execution_window(path: Sequence[Point], free_dist: Optional[np.ndarray],
                           raw_wall: Optional[np.ndarray],
                           target_px: float = WINDOW_TARGET_PX,
                           max_px: float = WINDOW_MAX_PX,
                           start_heading: Optional[float] = None) -> Optional[ExecutionWindow]:
    """Validate only the near-term prefix of a global route, with look-ahead.

    A longer prefix is supplied to the curve builder so a turn just beyond the
    executed endpoint can shape the local curve. The follower receives at most
    ``target_px`` of that validated tube and replans before it is exhausted.
    """
    pts = list(path)
    if len(pts) < 2:
        return None
    if not (WINDOW_MIN_PX <= target_px <= max_px):
        raise ValueError("execution window must satisfy min <= target <= max")

    global_length = path_length(pts)
    attempts = (
        (target_px, max_px),
        (max(WINDOW_MIN_PX, min(target_px, 120.0)), min(max_px, 130.0)),
        (max(WINDOW_MIN_PX, min(target_px, 95.0)), min(max_px, 95.0)),
        (WINDOW_MIN_PX, min(max_px, WINDOW_MIN_PX)),
        (WINDOW_EMERGENCY_MIN_PX, min(max_px, WINDOW_EMERGENCY_MIN_PX)),
    )
    seen = set()
    heading_fallback = None
    for execute_px, lookahead_px in attempts:
        key = (round(execute_px, 6), round(lookahead_px, 6))
        if key in seen:
            continue
        seen.add(key)
        goal_reached = global_length <= lookahead_px + 1e-6
        validation_input = pts if goal_reached else extract_path_prefix(pts, lookahead_px)
        tube = build_tube(validation_input, free_dist, raw_wall=raw_wall)
        if tube is None:
            continue

        executed = tube
        if not goal_reached and path_length(tube) > execute_px:
            executed = extract_path_prefix(tube, execute_px)
        if len(executed) < 2:
            continue
        heading_err = 0.0
        if start_heading is not None:
            first_heading = math.atan2(
                executed[1][1] - executed[0][1],
                executed[1][0] - executed[0][0],
            )
            heading_err = abs(math.degrees(_wrap_rad(first_heading - start_heading)))
        if heading_err > WINDOW_HEADING_GATE_DEG:
            # Tank can rotate in place; a heading mismatch is not fatal. Keep
            # the best-aligned candidate as a fallback instead of discarding it.
            if heading_fallback is None or heading_err < heading_fallback[0]:
                heading_fallback = (heading_err, list(executed), global_length,
                                    path_length(executed), execute_px, lookahead_px,
                                    goal_reached)
            continue
        if raw_wall is not None and not swept_rectangle_path_clear(executed, raw_wall, shrink=FOOTPRINT_WRAP_SHRINK):
            continue

        return ExecutionWindow(
            path=list(executed),
            global_path_length=global_length,
            window_path_length=path_length(executed),
            target_length=execute_px,
            lookahead_length=lookahead_px,
            goal_reached=goal_reached,
            validation_reason="tube_ok",
        )
    if heading_fallback is not None:
        (err, executed, global_length, wpl, execute_px, lookahead_px, goal_reached) = heading_fallback
        if raw_wall is None or swept_rectangle_path_clear(executed, raw_wall, shrink=FOOTPRINT_WRAP_SHRINK):
            return ExecutionWindow(
                path=list(executed),
                global_path_length=global_length,
                window_path_length=wpl,
                target_length=execute_px,
                lookahead_length=lookahead_px,
                goal_reached=goal_reached,
                validation_reason="tube_ok_heading_soft",
            )
    return None


def push_off_walls(path: Sequence[Point], free_dist: Optional[np.ndarray],
                   min_clearance: float = 16.5, iterations: int = 4) -> List[Point]:
    """把贴墙的路径点沿 free_dist 梯度推离墙（修复"斜穿缺口擦墙"）。

    只要求"不贴墙"（min_clearance≈半长+shrink），不做外接圆膨胀 ——
    保持通道连通性，碰撞最终由 swept 裁决。**端点同样推离**：上一段收尾
    贴墙时，新窗口起点车体擦墙 → swept 起点即撞 → footprint_invalid 的根源。
    """
    if free_dist is None or len(path) < 2:
        return list(path)
    h, w = free_dist.shape
    pts = [[float(x), float(y)] for x, y in path]
    for _ in range(iterations):
        moved = False
        for i in range(len(pts)):
            x, y = pts[i]
            ix = min(w - 1, max(1, int(round(x))))
            iy = min(h - 1, max(1, int(round(y))))
            c = float(free_dist[iy, ix])
            if c >= min_clearance:
                continue
            gx = float(free_dist[iy, min(w - 1, ix + 1)]) - float(free_dist[iy, max(0, ix - 1)])
            gy = float(free_dist[min(h - 1, iy + 1), ix]) - float(free_dist[max(0, iy - 1), ix])
            gl = math.hypot(gx, gy)
            if gl < 1e-6:
                best_d = None
                bv = c
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        nx = min(w - 1, max(0, ix + dx))
                        ny = min(h - 1, max(0, iy + dy))
                        v = float(free_dist[ny, nx])
                        if v > bv:
                            bv = v
                            best_d = (dx, dy)
                if best_d is None:
                    continue
                gx, gy = float(best_d[0]), float(best_d[1])
                gl = 1.0
            step = max(2.0, min_clearance - c)
            pts[i][0] += (gx / gl) * step
            pts[i][1] += (gy / gl) * step
            moved = True
        if not moved:
            break
    return [(x, y) for x, y in pts]


def refine_astar_polyline(path: Sequence[Point], free_dist: Optional[np.ndarray],
                          raw_wall: Optional[np.ndarray] = None) -> List[Point]:
    """Keep A* topology while shifting interior LOS corners toward free-space centres."""
    if free_dist is None:
        return list(path)
    refined = refine_polyline_clearance(
        path,
        free_dist,
        min_clearance_px=TANK_HALF_WIDTH_PX + 1.0,
    )
    return push_off_walls(refined, free_dist)


def plan_to_goal(me: Point, goal: Point, raw_wall: np.ndarray, blocked: np.ndarray,
                 grid_px: int = GRID_PX, blocked_turn: Optional[np.ndarray] = None,
                 free_dist: Optional[np.ndarray] = None,
                 start_heading: Optional[float] = None) -> PlanResult:
    """Plan directly to a fixed navigation goal for benchmark/arena use.

    This deliberately reuses the exact current navigation stack:
    8-neighbour A* -> LOS simplification -> turn clearance check -> Virtual Tube.
    It has no foe/tactical sampling, so a benchmark failure points at navigation
    rather than combat target selection.
    """
    begin = time.perf_counter()
    if not in_bounds_point(goal):
        return PlanResult(False, [], None, None, 0, 0,
                          (time.perf_counter()-begin)*1000.0, reason="goal_out_of_bounds")
    # 起点格容忍（与 plan() 一致）：车中心自由即可作 A* 起点
    start = point_to_cell(me, grid_px, blocked)
    if start is None or bool(blocked[start[1], start[0]]):
        start = nearest_free(start, blocked, max_r=5)
    gc = nearest_free(point_to_cell(goal, grid_px, blocked), blocked, max_r=5)
    if start is None:
        return PlanResult(False, [], goal, None, 0, 0,
                          (time.perf_counter()-begin)*1000.0, reason="start_not_free")
    if gc is None:
        return PlanResult(False, [], goal, None, 0, 0,
                          (time.perf_counter()-begin)*1000.0, reason="goal_not_free")
    cells = astar(start, gc, blocked)
    if not cells:
        return PlanResult(False, [], goal, None, 0, 0,
                          (time.perf_counter()-begin)*1000.0, reason="no_path")
    cells = simplify_cells(cells, blocked)
    path = [me] + [cell_to_point(c, grid_px) for c in cells[1:-1]] + [goal]
    raw_path_points = len(path)
    global_path_length = path_length(path)
    window_goal_reached = True
    window_path_length = global_path_length
    window_target_length = global_path_length
    window_lookahead_length = global_path_length
    if free_dist is not None:
        path = refine_astar_polyline(path, free_dist, raw_wall)
        raw_path_points = len(path)
        window = build_execution_window(path, free_dist, raw_wall, start_heading=start_heading)
        if window is None:
            return PlanResult(False, [], goal, None, 0, 0,
                              (time.perf_counter()-begin)*1000.0, reason="footprint_invalid",
                              path_source="goal", validation_reason="tube_invalid",
                              raw_path_points=raw_path_points, executed_path_points=0,
                              global_path_length=path_length(path))
        path = window.path
        global_path_length = window.global_path_length
        window_path_length = window.window_path_length
        window_target_length = window.target_length
        window_lookahead_length = window.lookahead_length
        window_goal_reached = window.goal_reached
        validation_reason = window.validation_reason
    else:
        validation_reason = "grid_only_no_footprint"
    ms = (time.perf_counter()-begin)*1000.0
    return PlanResult(True, list(path), goal, None, path_length(path), smoothness(path),
                      ms, reason="goal_path", valid_candidates=1,
                      path_source="goal", path_validated=True,
                      validation_reason=validation_reason,
                      raw_path_points=raw_path_points,
                      executed_path_points=len(path),
                      global_path_length=global_path_length,
                      window_path_length=window_path_length,
                      window_target_length=window_target_length,
                      window_lookahead_length=window_lookahead_length,
                      window_goal_reached=window_goal_reached)


def _target_type_for_relative_deg(rel_deg: float) -> str:
    """Classify a foe-relative direction using the historical [-360, 0] convention."""
    deg = rel_deg % 360.0
    if 135.0 <= deg <= 225.0:
        return "rear"
    if 90.0 <= deg < 135.0 or 225.0 < deg <= 270.0:
        return "flank"
    return "front"


def generate_targets(foe: Point, foe_angle: float, attack_ranges_px: Sequence[float],
                     tactical_mode: str = TACTICAL_MODE_REAR_ONLY) -> List[Tuple[str, float, float, Point]]:
    """Generate target candidates without changing the grid planner."""
    rels = REAR_RELATIVE_TARGET_DEG if tactical_mode == TACTICAL_MODE_REAR_ONLY else TACTICAL_RELATIVE_TARGET_DEG
    out = []
    for rel_deg in rels:
        th = foe_angle + math.radians(rel_deg)
        target_type = _target_type_for_relative_deg(rel_deg)
        for attack_range_px in attack_ranges_px:
            p = (foe[0] + attack_range_px * math.cos(th),
                 foe[1] + attack_range_px * math.sin(th))
            out.append((target_type, rel_deg, attack_range_px, p))
    return out


def generate_reposition_targets(me: Point, foe_angle: float,
                                ranges_px: Sequence[float] = REPOSITION_RANGES_PX) -> List[Tuple[str, Optional[float], float, Point]]:
    """Sample nearby safe intermediate positions for Tactical V1 fallback."""
    out = []
    for rel_deg in REPOSITION_RELATIVE_TARGET_DEG:
        th = foe_angle + math.radians(rel_deg)
        for radius in ranges_px:
            p = (me[0] + radius * math.cos(th), me[1] + radius * math.sin(th))
            out.append(("reposition", None, radius, p))
    return out


def open_direction_count(target: Point, blocked: np.ndarray, grid_px: int = GRID_PX,
                         sample_px: float = OPEN_DIRECTION_SAMPLE_PX) -> int:
    """A V1 local-freedom heuristic, deliberately not a topology graph."""
    start = point_to_cell(target, grid_px, blocked)
    steps = max(1, int(math.ceil(sample_px / grid_px)))
    directions = ((1, 0), (1, 1), (0, 1), (-1, 1),
                  (-1, 0), (-1, -1), (0, -1), (1, -1))
    h, w = blocked.shape
    open_count = 0
    for dx, dy in directions:
        clear = True
        for step in range(1, steps + 1):
            gx, gy = start[0] + dx * step, start[1] + dy * step
            if not (0 <= gx < w and 0 <= gy < h) or blocked[gy, gx]:
                clear = False
                break
        if clear:
            open_count += 1
    return open_count


def plan(me: Point, foe: Point, foe_angle: float, raw_wall: np.ndarray, blocked: np.ndarray,
         grid_px: int = GRID_PX, attack_ranges_px: Sequence[float] = ATTACK_RANGES_PX,
         preferred_rel_deg: Optional[float] = None,
         direction_switch_penalty_per_45_px: float = DIRECTION_SWITCH_PENALTY_PER_45_PX,
         staging_penalty_px: float = STAGING_PENALTY_PX,
         blocked_turn: Optional[np.ndarray] = None,
         free_dist: Optional[np.ndarray] = None,
          tactical_mode: str = DEFAULT_TACTICAL_MODE,
          preferred_target: Optional[Point] = None,
          target_switch_penalty_px: float = TARGET_SWITCH_PENALTY_PX,
          me_heading: Optional[float] = None) -> PlanResult:
    """Plan one OODA navigation path.

    Tactical V1 scores rear, flank, front and reposition candidates. It keeps
    A* as the route finder and the swept footprint as the executable-path gate.
    ``rear_only`` retains the legacy A/B baseline.
    """
    begin = time.perf_counter()
    if tactical_mode not in (TACTICAL_MODE_REAR_ONLY, TACTICAL_MODE_V1, TACTICAL_MODE_CHASE):
        raise ValueError("unknown tactical_mode: %s" % tactical_mode)
    # 起点格容忍：车中心在自由区即可作为 A* 起点（不必满足 10.5px 半径
    # 膨胀）。前段收尾贴墙时起点格会被膨胀标墙 → nearest_free 找不到 →
    # footprint_invalid 14/20 段的根源。只有中心真在墙里才挪到最近自由格。
    start = point_to_cell(me, grid_px, blocked)
    if start is None or bool(blocked[start[1], start[0]]):
        start = nearest_free(start, blocked, max_r=5)
    if start is None:
        return PlanResult(False, [], None, None, 0, 0, (time.perf_counter()-begin)*1000,
                          reason="start_not_free", tactical_mode=tactical_mode)

    # Chase 模式：OODA 每一轮的目标就是敌人当前所在位置。
    # 跳过候选采样/评分，直接 A* → 简化 → 执行窗口，最大程度积极。
    # 目标滞回：敌人小幅移动（< TARGET_SWITCH_DISTANCE_PX）时沿用上一轮
    # 目标，避免目标抖动引发 A/B 双路跳变（路口反复掉头）。
    if tactical_mode == TACTICAL_MODE_CHASE:
        ms = (time.perf_counter() - begin) * 1000.0
        goal = foe
        if (preferred_target is not None and
                math.hypot(foe[0] - preferred_target[0], foe[1] - preferred_target[1])
                <= TARGET_SWITCH_DISTANCE_PX):
            goal = preferred_target
        foe_c = point_to_cell(goal, grid_px, blocked)
        foe_free = nearest_free(foe_c, blocked, max_r=8)
        if foe_free is None:
            return PlanResult(False, [], foe, None, 0, 0, ms,
                              reason="chase_goal_blocked", tactical_mode=tactical_mode,
                              target_type="chase", path_source="chase")
        p_cells = astar(start, foe_free, blocked)
        if not p_cells:
            return PlanResult(False, [], foe, None, 0, 0, ms,
                              reason="chase_no_path", tactical_mode=tactical_mode,
                              target_type="chase", path_source="chase")
        # ---- 走"第二近"路线（用户方案 A，最小改动）----
        # 封掉最短路线两侧封锁带后重跑 A*；存在次优路线且 ≤ 最短×比例 → 采用。
        # 失败/超长 → 原样走最短路线（无行为变化）。
        use_alt = False
        alt_cells: Optional[List[Cell]] = None
        blocked2: Optional[np.ndarray] = None
        if len(p_cells) >= CHASE_ALT_BLOCK_MARGIN_CELLS * 2 + 1:
            blocked2 = blocked.copy()
            lo = CHASE_ALT_BLOCK_MARGIN_CELLS
            hi = len(p_cells) - CHASE_ALT_BLOCK_MARGIN_CELLS
            for c in p_cells[lo:hi]:
                gy, gx = c[1], c[0]
                for dy in range(-CHASE_ALT_BLOCK_CELLS, CHASE_ALT_BLOCK_CELLS + 1):
                    y2 = gy + dy
                    if not (0 <= y2 < blocked2.shape[0]):
                        continue
                    for dx in range(-CHASE_ALT_BLOCK_CELLS, CHASE_ALT_BLOCK_CELLS + 1):
                        x2 = gx + dx
                        if 0 <= x2 < blocked2.shape[1]:
                            blocked2[y2, x2] = True
            alt_cells = astar(start, foe_free, blocked2)
            if alt_cells is not None and len(alt_cells) >= 3:
                l1 = path_length([cell_to_point(c, grid_px) for c in p_cells])
                l2 = path_length([cell_to_point(c, grid_px) for c in alt_cells])
                use_alt = l2 <= l1 * CHASE_ALT_DETOUR_RATIO
        if use_alt and alt_cells is not None:
            direct_cells = p_cells
            p_cells = alt_cells
        # 候选管线：绕行优先，直连兜底。绕行路线更长、弯更多，可能过不了扫掠
        # 校验——此时回退直连（"有绕就绕，绕不成不硬绕"），不再因为绕行校验
        # 失败而整个计划失败（第二远策略稳定触发的关键之一）。
        # 注意：绕行路线的 LOS 简化必须用"封锁后"网格 —— 原版 blocked 下直连
        # 走廊仍可视，简化会把绕行拉回直连走廊（绕行被无形抹掉）。
        src = "chase_alt" if use_alt else "chase"
        seq = [(src, alt_cells if use_alt else p_cells,
                (blocked2 if use_alt else None))]
        if use_alt:
            seq.append(("chase", direct_cells, None))
        window = None
        raw_points = 0
        for src_i, cells_i, grid_i in seq:
            cs = simplify_cells(cells_i, grid_i if grid_i is not None else blocked)
            pp = [me] + [cell_to_point(c, grid_px) for c in cs[1:-1]] + [goal]
            pp = refine_astar_polyline(pp, free_dist, raw_wall)
            raw_points = len(pp)
            win = (build_execution_window(pp, free_dist, raw_wall, start_heading=me_heading)
                   if free_dist is not None else None)
            if free_dist is None:
                # 无距离场时退回网格路径（无足迹验证）
                L = path_length(pp)
                return PlanResult(True, pp, foe, None, L, smoothness(pp), ms,
                                  reason="chase", tactical_mode=tactical_mode,
                                  target_type="chase", path_source=src_i,
                                  path_validated=True, validation_reason="grid_only_no_footprint",
                                  raw_path_points=raw_points, executed_path_points=len(pp))
            if win is not None:
                window, src = win, src_i
                break
        if window is None:
            return PlanResult(False, [], foe, None, 0, 0, ms,
                              reason="chase_tube_invalid", tactical_mode=tactical_mode,
                              target_type="chase", path_source=src,
                              validation_reason="tube_invalid", raw_path_points=raw_points)
        path = window.path
        L = path_length(path)
        return PlanResult(True, path, foe, None, L, smoothness(path), ms,
                          reason="chase", tactical_mode=tactical_mode,
                          target_type="chase", path_source=src,
                          path_validated=True, validation_reason=window.validation_reason,
                          raw_path_points=raw_points, executed_path_points=len(path),
                          global_path_length=window.global_path_length,
                          window_path_length=window.window_path_length,
                          window_target_length=window.target_length,
                          window_lookahead_length=window.lookahead_length,
                          window_goal_reached=window.goal_reached)

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
    if (tactical_mode == TACTICAL_MODE_REAR_ONLY and -270.0 <= rel_now <= -90.0 and hold_min <= d_foe <= hold_max and
            pixel_segment_clear(me, foe, raw_wall, ignore_end_px=6.0)):
        ms = (time.perf_counter() - begin) * 1000.0
        return PlanResult(True, [me], me, rel_now, 0.0, 0.0, ms,
                          reason="hold_attack_position", valid_candidates=1,
                          tactical_mode=tactical_mode, target_type="rear",
                          candidate_count=1, reachable_candidates=1,
                          rear_candidates=1, rear_reachable_candidates=1,
                          line_of_sight=True, path_source="hold",
                          path_validated=True, validation_reason="hold_position",
                          raw_path_points=1, executed_path_points=1)

    candidate_specs = generate_targets(foe, foe_angle, attack_ranges_px, tactical_mode)
    if tactical_mode == TACTICAL_MODE_V1:
        candidate_specs.extend(generate_reposition_targets(me, foe_angle))
    candidate_meta = []
    for target_type, rel_deg, attack_range_px, target in candidate_specs:
        if not in_bounds_point(target):
            continue
        tc = point_to_cell(target, grid_px, blocked)
        if blocked[tc[1], tc[0]]:
            continue
        candidate_meta.append((target_type, rel_deg, attack_range_px, target, tc))

    if not candidate_meta:
        ms = (time.perf_counter() - begin) * 1000.0
        reason = "no_rear_candidate" if tactical_mode == TACTICAL_MODE_REAR_ONLY else "no_tactical_candidate"
        return PlanResult(False, [], None, None, 0, 0, ms, reason=reason, tactical_mode=tactical_mode)

    reachable = []
    rear_candidates = sum(1 for kind, _, _, _, _ in candidate_meta if kind == "rear")
    rear_reachable = 0
    for target_type, rel_deg, _attack_range_px, target, tc in candidate_meta:
        p_cells = astar(start, tc, blocked)
        if not p_cells:
            continue
        if target_type == "rear":
            rear_reachable += 1
        p_cells = simplify_cells(p_cells, blocked)
        p = [me] + [cell_to_point(c, grid_px) for c in p_cells[1:-1]] + [target]
        map_length = path_length(p)
        p = refine_astar_polyline(p, free_dist, raw_wall)
        has_los = pixel_segment_clear(target, foe, raw_wall, ignore_end_px=6.0)
        open_dirs = open_direction_count(target, blocked, grid_px)
        switch_cost = 0.0
        if preferred_rel_deg is not None and rel_deg is not None:
            switch_cost += direction_switch_penalty_per_45_px * (abs(rel_deg - preferred_rel_deg) / 45.0)
        if preferred_target is not None and math.hypot(target[0] - preferred_target[0], target[1] - preferred_target[1]) > TARGET_SWITCH_DISTANCE_PX:
            switch_cost += target_switch_penalty_px
        if tactical_mode == TACTICAL_MODE_REAR_ONLY:
            score = map_length + switch_cost + (0.0 if has_los else staging_penalty_px)
        else:
            dead_end_cost = max(0, 2 - open_dirs) * DEAD_END_PENALTY_PX
            open_reward = max(0, open_dirs - 2) * OPEN_DIRECTION_REWARD_PX
            score = (map_length + switch_cost + dead_end_cost
                     + (0.0 if has_los else NO_LOS_PENALTY_PX)
                     + (REPOSITION_PENALTY_PX if target_type == "reposition" else 0.0)
                     - TACTICAL_REWARD_PX[target_type] - LOS_REWARD_PX * int(has_los)
                     - open_reward)
        reachable.append(dict(target_type=target_type, rel_deg=rel_deg, target=target,
                              raw_path=p, map_length=map_length, has_los=has_los,
                              open_dirs=open_dirs, score=score, switch_cost=switch_cost))

    ms = (time.perf_counter() - begin) * 1000.0
    reachable_count = len(reachable)
    best = None
    for item in sorted(reachable, key=lambda x: (x["score"], x["map_length"])):
        window = (build_execution_window(item["raw_path"], free_dist, raw_wall,
                                         start_heading=me_heading)
                  if free_dist is not None else None)
        if free_dist is None:
            item["path"] = list(item["raw_path"])
            item["global_path_length"] = path_length(item["raw_path"])
            item["window_path_length"] = item["global_path_length"]
            item["window_target_length"] = item["global_path_length"]
            item["window_lookahead_length"] = item["global_path_length"]
            item["window_goal_reached"] = True
            item["validation_reason"] = "grid_only_no_footprint"
            best = item
            break
        if window is None:
            continue
        item["path"] = window.path
        item["global_path_length"] = window.global_path_length
        item["window_path_length"] = window.window_path_length
        item["window_target_length"] = window.target_length
        item["window_lookahead_length"] = window.lookahead_length
        item["window_goal_reached"] = window.goal_reached
        item["validation_reason"] = window.validation_reason
        best = item
        break

    if best is None:
        foe_c = point_to_cell(foe, grid_px, blocked)
        foe_free = nearest_free(foe_c, blocked, max_r=8)
        if foe_free is not None:
            p_cells = astar(start, foe_free, blocked)
            if p_cells:
                p_cells = simplify_cells(p_cells, blocked)
                p = [me] + [cell_to_point(c, grid_px) for c in p_cells[1:-1]] + [foe]
                p = refine_astar_polyline(p, free_dist, raw_wall)
                raw_path_points = len(p)
                window = (build_execution_window(p, free_dist, raw_wall, start_heading=me_heading)
                          if free_dist is not None else None)
                if window is None:
                    return PlanResult(False, [], foe, None, 0, 0,
                                      (time.perf_counter() - begin) * 1000.0,
                                      reason="fallback_tube_invalid",
                                      valid_candidates=reachable_count,
                                      tactical_mode=tactical_mode,
                                      target_type="fallback",
                                      candidate_count=len(candidate_meta),
                                      reachable_candidates=reachable_count,
                                      rear_candidates=rear_candidates,
                                      rear_reachable_candidates=rear_reachable,
                                      path_source="fallback",
                                      validation_reason="tube_invalid",
                                      raw_path_points=raw_path_points,
                                      executed_path_points=0,
                                      global_path_length=path_length(p))
                p = window.path
                L = path_length(p)
                S0 = smoothness(p)
                ms2 = (time.perf_counter() - begin) * 1000.0
                return PlanResult(True, p, foe, None, L, S0, ms2,
                                  reason="fallback_chase", valid_candidates=reachable_count,
                                  tactical_mode=tactical_mode, target_type="fallback",
                                  candidate_count=len(candidate_meta), reachable_candidates=reachable_count,
                                  rear_candidates=rear_candidates, rear_reachable_candidates=rear_reachable,
                                  path_source="fallback", path_validated=True,
                                  validation_reason=window.validation_reason,
                                  raw_path_points=raw_path_points,
                                  executed_path_points=len(p),
                                  global_path_length=window.global_path_length,
                                  window_path_length=window.window_path_length,
                                  window_target_length=window.target_length,
                                  window_lookahead_length=window.lookahead_length,
                                  window_goal_reached=window.goal_reached)
        reason = "no_rear_candidate" if tactical_mode == TACTICAL_MODE_REAR_ONLY else "no_executable_tactical_candidate"
        return PlanResult(False, [], None, None, 0, 0, ms, reason=reason,
                          tactical_mode=tactical_mode, candidate_count=len(candidate_meta),
                          reachable_candidates=reachable_count, rear_candidates=rear_candidates,
                          rear_reachable_candidates=rear_reachable)

    p = best["path"]
    L = path_length(p)
    S = smoothness(p)
    reason = "staging_no_los" if tactical_mode == TACTICAL_MODE_REAR_ONLY and not best["has_los"] else ""
    return PlanResult(True, p, best["target"], best["rel_deg"], L, S, ms, reason=reason,
                      valid_candidates=reachable_count, tactical_mode=tactical_mode,
                      target_type=best["target_type"], candidate_count=len(candidate_meta),
                      reachable_candidates=reachable_count, rear_candidates=rear_candidates,
                      rear_reachable_candidates=rear_reachable, line_of_sight=best["has_los"],
                      open_directions=best["open_dirs"], tactical_score=best["score"],
                      switch_cost=best["switch_cost"], path_source=best["target_type"],
                      path_validated=True,
                      validation_reason=best["validation_reason"],
                      raw_path_points=len(best["raw_path"]),
                      executed_path_points=len(p),
                      global_path_length=best["global_path_length"],
                      window_path_length=best["window_path_length"],
                      window_target_length=best["window_target_length"],
                      window_lookahead_length=best["window_lookahead_length"],
                      window_goal_reached=best["window_goal_reached"])


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
    raw_wall, inflated, blocked, blocked_turn, free_dist = build_maps(
        maze, polys, safe_radius_px, grid_px,
        turn_radius_px=TANK_SAFE_RADIUS_PX,
        straight_radius_px=A_STAR_TOPOLOGY_RADIUS_PX,
    )
    rows = load_track(track_path)
    frames = sample_ooda(rows)
    results = []
    preview_plans = []

    for cycle, r in enumerate(frames):
        me = (r["me_x"], r["me_y"])
        foe = (r["foe_x"], r["foe_y"])
        pr = plan(me, foe, r["foe_angle"], raw_wall, blocked, grid_px, attack_ranges_px,
                  blocked_turn=blocked_turn, free_dist=free_dist)
        collides = bool(pr.success and not swept_rectangle_path_clear(pr.path, raw_wall))
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
