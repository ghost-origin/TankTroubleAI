# -*- coding: utf-8 -*-
"""Fast navigation-only benchmark simulator.

Purpose
-------
This module is intentionally *not* a combat simulator.  It reuses the current
navigation geometry (maze polygons, A*, Virtual Tube) and mirrors Construct 2's
Car behaviour closely enough to make turn/navigation regressions cheap to run.
The real game benchmark remains the final validation layer.

Current player Car properties from data.js.base:
    max speed 125 px/s, acceleration 200 px/s^2, deceleration 200 px/s^2.
The current project measured steering speed at about 3.8 rad/s; drift recovery
matches the exported Car property 200 deg/s.
"""
from __future__ import annotations

from pathlib import Path
import sys
PROJECT_ROOT = Path(__file__).resolve().parents[2]
PYTHON_ROOT = PROJECT_ROOT / 'python'
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

import json
import math
import os
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
from PIL import Image, ImageDraw

from navigation_mvp import (
    GRID_PX, MAP_PX, Point, astar, cell_to_point, load_polys, nearest_free,
    distance_transform_edt, path_length, point_to_cell,
)

MAX_SPEED = 125.0
ACCEL = 200.0
DECEL = 200.0
STEER_SPEED = 3.8                # rad/s, measured project value
MIN_STEER_SCALE = 0.15           # 原地转向最低转向速率比例（实测坦克静止可转向）
DRIFT_RECOVER = math.radians(200.0)
BODY_HALF_LENGTH = 15.5
BODY_HALF_WIDTH = 10.0
WAYPOINT_REACHED_PX = 13.0
WP_FIRST_MIN_PX = 20.0          # 初始路点最小距离（不被车身覆盖）
WP_SPACING_PX = 50.0            # 路点间距（与 navigation_bot.WP_SPACING_PX 一致）
TURN_DEADBAND_RAD = math.radians(10.0)
TURN_FULL_RAD = math.radians(25.0)
STEER_FLIP_COOL_FRAMES = 21     # 换向冷却 0.35s @ 60fps
HEADING_ALIGN_RAD = math.radians(35.0)
STOP_SPEED_PX_S = 5.0            # 点刹目标：速度降到该值即原地转
LOOKAHEAD_PX = 35.0              # 纯追踪前瞻距离：desired 指向路径前方该距离处
CRUISE_SPEED_HIGH = 70.0         # 巡航上限（与 navigation_bot 同步）
CRUISE_SPEED_LOW = 65.0
CORNER_SPEED = 60.0              # 过弯速度上限（与 navigation_bot 同步）
CORNER_MIN_SPEED = 12.0          # 过弯速度下限
CORNER_LOOKAHEAD_PX = 60.0       # 曲率前瞻窗口
CROSS_GAIN = 0.02                # cross-track 纠偏增益（rad/px）
CROSS_CORR_MAX = 0.6             # 纠偏上限（rad）
# PID（θ 转向 / y 横向 / x 纵向速度），与 bot 同步
PID_KP_TH = 10.0
PID_KI_TH = 0.0
PID_KD_TH = 0.0
PID_KP_LAT = 0.045
PID_KI_LAT = 0.004
PID_KD_LAT = 0.012
PID_KP_V = 0.025
PID_KI_V = 0.008
DT = 1.0 / 60.0
GOAL_RADIUS_PX = 20.0
MOVING_SPEED_THRESHOLD = 5.0

FLAG_H, FLAG_V, FLAG_D = 0x80000000, 0x40000000, 0x20000000
ROT_FLAGS = {0: 0, 90: FLAG_D | FLAG_H, 180: FLAG_H | FLAG_V, 270: FLAG_D | FLAG_V}


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(x: float, lo: float, hi: float) -> float:
    return min(hi, max(lo, x))


def maze_from_assets(mazes_json: str, maze_index: int = 0) -> List[List[int]]:
    """Convert assets/mazes.json [tile_id, rotation_deg] to raw Tilemap ids."""
    with open(mazes_json, encoding='utf-8-sig') as f:
        db = json.load(f)
    key = str(maze_index)
    if key not in db:
        raise KeyError('maze index not found: %s' % maze_index)
    obj = json.loads(db[key]) if isinstance(db[key], str) else db[key]
    data = obj['data']
    out = []
    for row in data:
        rr = []
        for cell in row:
            tid, deg = int(cell[0]), int(cell[1]) % 360
            rr.append(tid | ROT_FLAGS.get(deg, 0))
        out.append(rr)
    return out


def cell_path_px(cells: Sequence[Tuple[int, int]], grid_px: int = GRID_PX) -> float:
    return sum(math.hypot((b[0]-a[0])*grid_px, (b[1]-a[1])*grid_px)
               for a, b in zip(cells, cells[1:]))


def shortest_map_distance(start: Point, goal: Point, blocked: np.ndarray,
                          grid_px: int = GRID_PX) -> Optional[float]:
    sc = nearest_free(point_to_cell(start, grid_px, blocked), blocked, max_r=5)
    gc = nearest_free(point_to_cell(goal, grid_px, blocked), blocked, max_r=5)
    if sc is None or gc is None:
        return None
    cells = astar(sc, gc, blocked)
    if not cells:
        return None
    d = cell_path_px(cells, grid_px)
    sp = cell_to_point(sc, grid_px)
    gp = cell_to_point(gc, grid_px)
    d += math.hypot(start[0]-sp[0], start[1]-sp[1])
    d += math.hypot(goal[0]-gp[0], goal[1]-gp[1])
    return d


def nearest_polyline_distance(p: Point, pts: Sequence[Point]) -> float:
    if not pts:
        return float('inf')
    if len(pts) == 1:
        return math.hypot(p[0]-pts[0][0], p[1]-pts[0][1])
    px, py = p
    best = float('inf')
    for a, b in zip(pts, pts[1:]):
        ax, ay = a; bx, by = b
        dx, dy = bx-ax, by-ay
        den = dx*dx + dy*dy
        if den < 1e-12:
            d = math.hypot(px-ax, py-ay)
        else:
            t = clamp(((px-ax)*dx + (py-ay)*dy) / den, 0.0, 1.0)
            qx, qy = ax+t*dx, ay+t*dy
            d = math.hypot(px-qx, py-qy)
        if d < best:
            best = d
    return best


def _body_perimeter_points(step: float = 2.0) -> List[Point]:
    pts = []
    x = -BODY_HALF_LENGTH
    while x <= BODY_HALF_LENGTH + 1e-9:
        pts.append((x, -BODY_HALF_WIDTH)); pts.append((x, BODY_HALF_WIDTH))
        x += step
    y = -BODY_HALF_WIDTH
    while y <= BODY_HALF_WIDTH + 1e-9:
        pts.append((-BODY_HALF_LENGTH, y)); pts.append((BODY_HALF_LENGTH, y))
        y += step
    return pts


BODY_PERIMETER = _body_perimeter_points()
BODY_SCRAPE_TOL_PX = 2.5          # 擦碰容差：≤2.5px 侵入不算撞（对齐游戏滑动物理）


def body_collides(raw_wall: np.ndarray, x: float, y: float, angle: float,
                  shrink: float = BODY_SCRAPE_TOL_PX) -> bool:
    """Approximate the player's 31x20 oriented collision rectangle against pixel walls.

    shrink > 0 = 轮廓向内缩（允许 shrink px 侵入）—— 对齐游戏"擦碰反弹"
    物理：1px 级擦角在游戏里是滑动而不是撞死。
    """
    ca, sa = math.cos(angle), math.sin(angle)
    h, w = raw_wall.shape
    for lx, ly in BODY_PERIMETER:
        sx = max(0.0, abs(lx) - shrink) * (1 if lx >= 0 else -1)
        sy = max(0.0, abs(ly) - shrink) * (1 if ly >= 0 else -1)
        wx = x + ca*sx - sa*sy
        wy = y + sa*sx + ca*sy
        ix, iy = int(round(wx)), int(round(wy))
        if ix < 0 or iy < 0 or ix >= w or iy >= h or raw_wall[iy, ix]:
            return True
    return False


@dataclass
class TankState:
    x: float
    y: float
    angle: float
    speed: float = 0.0
    move_angle: Optional[float] = None

    def __post_init__(self):
        if self.move_angle is None:
            self.move_angle = self.angle


@dataclass
class Action:
    up: bool = True
    down: bool = False
    left: bool = False
    right: bool = False


class WaypointFollower:
    """Exact logic currently used by navigation_bot.action(), isolated for benchmarks."""
    def __init__(self, path: Sequence[Point], reached_px: float = WAYPOINT_REACHED_PX,
                 deadband_rad: float = TURN_DEADBAND_RAD):
        # 路点稀疏化：每 15px 保留一个（与 bot 同步）
        sparse = [tuple(path[0])]
        acc = 0.0
        for p in path[1:]:
            acc += math.hypot(p[0] - sparse[-1][0], p[1] - sparse[-1][1])
            if acc >= WP_SPACING_PX:
                sparse.append(tuple(p))
                acc = 0.0
        if len(sparse) < 2 or sparse[-1] != tuple(path[-1]):
            sparse.append(tuple(path[-1]))
        self.path = sparse
        self.reached_px = reached_px
        self.deadband_rad = deadband_rad
        self.wp_idx = 1 if len(self.path) > 1 else 0
        # 初始路点 = 第一个不被车身覆盖的路点
        while (self.wp_idx < len(self.path) - 1 and
               math.hypot(self.path[self.wp_idx][0] - self.path[0][0],
                          self.path[self.wp_idx][1] - self.path[0][1]) < WP_FIRST_MIN_PX):
            self.wp_idx += 1
        self.last_steer = 0
        self.steer_flips = 0

    def _lookahead_point(self, st: TankState):
        """纯追踪：沿路径从 wp_idx 起累计弧长，取前方 LOOKAHEAD_PX 处的点。
        指向该点=提前打舵，连续弯处 err 平滑出现，不再滞后外切。"""
        look = LOOKAHEAD_PX
        acc = 0.0
        px, py = st.x, st.y
        for i in range(self.wp_idx, len(self.path) - 1):
            x1, y1 = self.path[i]
            x2, y2 = self.path[i + 1]
            seg = math.hypot(x2 - x1, y2 - y1)
            if acc + seg >= look:
                u = (look - acc) / seg if seg > 1e-6 else 0.0
                return (x1 + (x2 - x1) * u, y1 + (y2 - y1) * u)
            acc += seg
        return self.path[-1]

    def _corner_target_speed(self):
        """曲率前瞻：lookahead 60px 窗口内路径的 min 曲率半径（8px 采样间隔
        降噪）→ 目标速度 v = min_r * STEER_SPEED（clamp 12..CORNER_SPEED）。
        直路返回 None。行驶转弯半径 R=speed/STEER_SPEED，速度高于目标会外切。"""
        if self.wp_idx >= len(self.path) - 1:
            return None
        pts = [(self.path[self.wp_idx][0], self.path[self.wp_idx][1])]
        acc = 0.0
        for i in range(self.wp_idx + 2, len(self.path), 2):   # 8px 间隔采样
            x, y = self.path[i]
            acc += math.hypot(x - pts[-1][0], y - pts[-1][1])
            if acc > CORNER_LOOKAHEAD_PX:
                break
            pts.append((x, y))
        if len(pts) < 3:
            return None
        min_r = None
        for a, b, c in zip(pts, pts[1:], pts[2:]):
            ab = math.hypot(b[0] - a[0], b[1] - a[1])
            bc = math.hypot(c[0] - b[0], c[1] - b[1])
            ca = math.hypot(c[0] - a[0], c[1] - a[1])
            if min(ab, bc) < 0.5 or ab + bc + ca < 1e-6:
                continue
            area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
            if area2 < 1e-6:
                continue
            r = ab * bc * ca / (2.0 * area2)
            if 1.0 < r < 1000.0:
                min_r = r if min_r is None else min(min_r, r)
        if min_r is None:
            return None
        return max(CORNER_MIN_SPEED, min(CORNER_SPEED, min_r * STEER_SPEED))

    def _signed_lat_error(self, st: TankState):
        """车到整条路径最近段的有符号横向距离（+ = 路径右侧）。"""
        best_d = None
        best_lat = 0.0
        for i in range(max(0, self.wp_idx - 3), min(len(self.path) - 1, self.wp_idx + 4)):
            x1, y1 = self.path[i]
            x2, y2 = self.path[i + 1]
            dx, dy = x2 - x1, y2 - y1
            seg = math.hypot(dx, dy)
            if seg < 1e-6:
                continue
            ux, uy = dx / seg, dy / seg
            rx, ry = st.x - x1, st.y - y1
            proj = rx * ux + ry * uy
            if proj < -20.0 or proj > seg + 20.0:
                continue
            lat = rx * uy - ry * ux
            d = abs(lat)
            if best_d is None or d < best_d:
                best_d = d
                best_lat = lat
        return best_lat

    def action(self, st: TankState) -> Action:
        while self.wp_idx < len(self.path):
            tx, ty = self.path[self.wp_idx]
            if math.hypot(tx-st.x, ty-st.y) <= self.reached_px:
                self.wp_idx += 1
            else:
                break
        if self.wp_idx >= len(self.path):
            return Action(up=False)
        # 贪吃蛇追踪：目标 = 第一个（最近）路点（wp 推进已在上方完成），
        # 纯 P 转向（Kp=10），无 lookahead / 横向 PID / 积分微分。
        tx, ty = self.path[self.wp_idx]
        desired = math.atan2(ty-st.y, tx-st.x)
        err = wrap(desired-st.angle)
        u_steer = PID_KP_TH * err
        self._steer_accum = getattr(self, '_steer_accum', 0.0) + min(1.0, abs(u_steer))
        steer = 0
        if self._steer_accum >= 1.0:
            self._steer_accum -= 1.0
            steer = 1 if u_steer > 0 else -1
        if steer and self.last_steer and steer != self.last_steer:
            self.steer_flips += 1
        if steer:
            self.last_steer = steer
        # x 纵向：速度 P → 油门；无 down，负输出=松油
        v_target = self._corner_target_speed() or CRUISE_SPEED_LOW
        err_v = v_target - st.speed
        u_throttle = PID_KP_V * err_v
        self._throttle_accum = getattr(self, '_throttle_accum', 0.0)
        up = False
        if u_throttle > 0:
            self._throttle_accum += min(1.0, u_throttle)
            if self._throttle_accum >= 1.0:
                self._throttle_accum -= 1.0
                up = True
        else:
            self._throttle_accum = min(self._throttle_accum, 0.5)
        if st.speed > v_target + 6.0:
            self._throttle_accum = min(self._throttle_accum, 0.3)
        return Action(up=up, left=steer < 0, right=steer > 0)


def step_car(st: TankState, action: Action, dt: float = DT) -> TankState:
    """Mirror the relevant portion of Construct 2 Car.tick()."""
    s = st.speed
    if action.up and not action.down:
        s = min(MAX_SPEED, s + ACCEL*dt)
    elif action.down and not action.up:
        s = max(-MAX_SPEED, s - DECEL*dt)
    else:
        # C2 Car uses dec*0.1 when neither/both are pressed.
        if s > 0:
            s = max(0.0, s - DECEL*dt*0.1)
        elif s < 0:
            s = min(0.0, s + DECEL*dt*0.1)

    a = st.angle
    if s < 0:
        left, right = action.right, action.left
    else:
        left, right = action.left, action.right
    # 原地转向修正：实测游戏按左右键可原地旋转（速度=0 也能转向）。
    # C2 Car 原始公式 steerScale=|s|/MAX_SPEED 在 s=0 时为 0（转不动），
    # 与实际体验不符；这里给一个最低转向速率，让静止坦克也能转向。
    steer_scale = max(abs(s) / MAX_SPEED if MAX_SPEED > 0 else 0.0, MIN_STEER_SCALE)
    if left and not right:
        a = wrap(a - STEER_SPEED*dt*steer_scale)
    elif right and not left:
        a = wrap(a + STEER_SPEED*dt*steer_scale)

    m = st.move_angle if st.move_angle is not None else st.angle
    recover = DRIFT_RECOVER*dt
    diff = wrap(a-m)
    m = wrap(m + clamp(diff, -recover, recover))
    x = st.x + math.cos(m)*s*dt
    y = st.y + math.sin(m)*s*dt
    return TankState(x=x, y=y, angle=a, speed=s, move_angle=m)


def trajectory_distance(rows: Sequence[dict]) -> float:
    return sum(math.hypot(b['x']-a['x'], b['y']-a['y']) for a,b in zip(rows, rows[1:]))


def trajectory_smoothness_per_100(rows: Sequence[dict]) -> float:
    pts = [(r['x'], r['y']) for r in rows]
    if len(pts) < 3:
        return 0.0
    L = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(pts,pts[1:]))
    if L < 1e-9:
        return 0.0
    ang = []
    for a,b in zip(pts,pts[1:]):
        if math.hypot(b[0]-a[0], b[1]-a[1]) > 1e-6:
            ang.append(math.atan2(b[1]-a[1], b[0]-a[0]))
    turn = sum(abs(wrap(b-a)) for a,b in zip(ang,ang[1:]))
    return turn/max(L,1e-9)*100.0


def moving_stats(rows: Sequence[dict], moving_threshold: float = MOVING_SPEED_THRESHOLD):
    if len(rows) < 2:
        return 0.0, 0.0, 0.0
    total_t = max(0.0, rows[-1]['t']-rows[0]['t'])
    total_d = trajectory_distance(rows)
    moving_t = 0.0
    moving_d = 0.0
    for a,b in zip(rows,rows[1:]):
        dt = max(0.0, b['t']-a['t'])
        d = math.hypot(b['x']-a['x'], b['y']-a['y'])
        v = d/dt if dt > 1e-9 else 0.0
        if v >= moving_threshold:
            moving_t += dt; moving_d += d
    avg = total_d/total_t if total_t > 1e-9 else 0.0
    moving_avg = moving_d/moving_t if moving_t > 1e-9 else 0.0
    idle_ratio = 1.0-moving_t/total_t if total_t > 1e-9 else 1.0
    return avg, moving_avg, idle_ratio


def min_turn_radius_from_track(rows: Sequence[dict]) -> Optional[float]:
    """Estimate minimum geometric turn radius from triplets, ignoring tiny moves."""
    pts = [(r['x'],r['y']) for r in rows]
    vals = []
    for i in range(0, len(pts)-2, 3):
        a,b,c = pts[i],pts[i+1],pts[i+2]
        ab=math.hypot(b[0]-a[0],b[1]-a[1]); bc=math.hypot(c[0]-b[0],c[1]-b[1]); ca=math.hypot(a[0]-c[0],a[1]-c[1])
        if min(ab,bc) < 1.0:
            continue
        area2 = abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))
        if area2 < 1e-3:
            continue
        # R = abc/(4A), area2=2A -> abc/(2*area2)
        r = ab*bc*ca/(2.0*area2)
        if 5.0 < r < 1000.0:
            vals.append(r)
    return min(vals) if vals else None


def make_open_wall_mask() -> np.ndarray:
    raw = np.zeros((MAP_PX,MAP_PX),dtype=bool)
    raw[0:2,:] = True; raw[-2:,:] = True; raw[:,0:2] = True; raw[:,-2:] = True
    return raw


def corridor_wall_mask(centerline: Sequence[Point], half_width: float) -> np.ndarray:
    """Free corridor around a reference centerline; outside is solid wall."""
    img = Image.new('L',(MAP_PX,MAP_PX),0)
    draw = ImageDraw.Draw(img)
    if len(centerline)>=2:
        draw.line(list(centerline),fill=255,width=2)
    line = np.asarray(img,dtype=np.uint8)>0
    dist = distance_transform_edt(~line)
    raw = dist > float(half_width)
    raw[0:2,:]=True; raw[-2:,:]=True; raw[:,0:2]=True; raw[:,-2:]=True
    return raw


def add_outer_wall(raw: np.ndarray, centerline: Sequence[Point], direction_sign: int,
                   offset: float = 42.0, width: int = 5) -> np.ndarray:
    """Add a wall on the outer side of a turn to expose overshoot regressions."""
    img = Image.fromarray((raw.astype(np.uint8)*255),mode='L')
    draw = ImageDraw.Draw(img)
    off = []
    n=len(centerline)
    for i in range(max(1,int(n*0.20)), max(2,int(n*0.80))):
        p=centerline[i]
        a=centerline[max(0,i-1)]; b=centerline[min(n-1,i+1)]
        dx,dy=b[0]-a[0],b[1]-a[1]
        L=math.hypot(dx,dy)
        if L<1e-9: continue
        dx/=L;dy/=L
        # screen-coordinate left normal = (dy,-dx); for right turns outer=left.
        nx,ny=direction_sign*dy, direction_sign*(-dx)
        off.append((p[0]+nx*offset,p[1]+ny*offset))
    if len(off)>=2:
        draw.line(off,fill=255,width=width)
    arr=np.asarray(img,dtype=np.uint8)>0
    return arr


def free_distance(raw_wall: np.ndarray) -> np.ndarray:
    return distance_transform_edt(~raw_wall)
