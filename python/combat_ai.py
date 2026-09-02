# -*- coding: utf-8 -*-
"""绿色坦克「半自主射击」：玩家控制移动，AI 在能打到敌人时接管左右键瞄准并开火。

行为：
1. 每 RESOLVE_EVERY_S 秒（节流）做一次「是否存在可击中敌人的发射角」扫描：
   以炮口为起点按 1° 步长扫描 360 个方向，模拟炮弹直线飞行 + 墙面镜面
   反射（最多 MAX_BOUNCES 次），找出一条能穿过敌人「预测位置」半径
   HIT_RADIUS_PX 圆的最优弹道（solve_shot）。
2. 若存在这样的发射角（不要求当前朝向已对准）→ AI 进入「接管瞄准」状态：
   - 只接管左/右键：原地转向，把坦克角度转到该发射角；
   - 不碰上/下键：玩家可以继续前进/后退；
   - 开火：每帧用「当前车头角度的实时弹道」验证确实能命中敌人预测位置
     才按下开火键（fire 脉冲 + 冷却）——不依赖历史求解角度的误差，
     避免历史角度过时导致的朝错误方向乱射、或对不准时愣住不开火。
3. 取消接管（交还左右键）：
   - 敌人被击杀 / 消失（foe 缺失 → 回合结束）；
   - 敌人离开可击中的位置（重扫发现不再存在可行发射角）。

与游戏原版 AI 对应：原版 AITrace(angle, x, y, reflections, 0) 沿指定角度做
带反射的射线探测（困难难度 reflections=2），本模块用同样的反射弹道模型，
并把反射次数上限放宽到 4。

坐标约定（与 navigation_bot 一致）：
- 地图坐标系：迷宫左上角为原点，x 向右、y 向下，单位像素；
- 角度：弧度，0 = 朝右（+x），顺时针增大（朝下为 +π/2）；
- raw_wall：570×570 布尔掩码，True = 墙，下标 raw_wall[y, x]。
"""
from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

Point = Tuple[float, float]

# 弹道模拟
MAX_BOUNCES = 2            # 最多反射次数（与游戏原版 AI 一致：normal=1 / hard=2）。
                           # 之前设 4 会把「3~4 次边框反弹扫过目标」误判为命中，
                           # 导致 AI 朝几乎错误的方向乱开火（炮弹乱弹自伤）。
MAX_PATH_PX = 760.0        # 炮弹最大总路程（约 360px/s × 2.1s，子弹按时间寿命）
STEP_PX = 1.0              # 光线步长（像素）
HIT_RADIUS_PX = 30.0       # 命中判定：弹道对敌人预测位置的最小距离 ≤ 此值
                           # （坦克命中盒半径约 30px）
MUZZLE_SAFE_PX = 36.0      # 出膛近区不计入命中判定：炮弹刚离膛的一段路程不参与
                           # min_miss 统计，避免「敌人/预测点恰好落在炮口附近」时
                           # 任何朝向的弹道都被误判为命中（无视野乱开火的根源）
WALL_MIN_GAP_PX = 4.0      # 两次撞墙之间的最小自由间隔：若反射后仍紧贴墙，说明
                           # 反射方向判错或卡进墙里（防「穿墙」模拟），放弃该弹道
MUZZLE_OFFSET_PX = 18.0    # 炮口相对坦克中心的前伸量
WALL_LOOKAHEAD_PX = 2.0    # 反射后向前推开的距离，避免立刻再次撞到同一像素
FINE_STEP_RAD = math.radians(0.25)   # 细调步长
FINE_SPAN_RAD = math.radians(2.5)    # 细调范围（围绕粗扫最优角）

# 瞄准 / 开火
AIM_DEADBAND_RAD = math.radians(3.0)   # 转向死区（进入后停止转向，防抖动）
FIRE_ALIGN_RAD = math.radians(8.0)     # 车头对准目标角在此误差内即可开火
AIM_SWITCH_RAD = math.radians(45.0)    # 新解与旧目标角差超过此值时不切换（防目标角乱跳）
U_TURN_LOCK_RAD = math.pi - 0.35       # 接近 180° 掉头时锁定转向方向，防左右抖动
FIRE_PULSE_S = 0.10                    # 按键脉冲时长
FIRE_COOLDOWN_S = 0.45                 # 开火冷却（游戏本身也限制射速，这里是 AI 侧节流）
RESOLVE_EVERY_S = 0.20                 # 「是否存在可行发射角」的重扫周期

# 弹速 / 提前量
DEFAULT_SHELL_SPEED_PX_S = 360.0
MAX_LEAD_S = 0.90
MAX_FOE_SPEED_PX_S = 600.0  # 敌人速度钳制（桥的位置差分偶发大噪声时，防预测点乱跳进墙/乱飞）


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _speed(o: Dict) -> float:
    return math.hypot(float(o.get('vx', 0.0)), float(o.get('vy', 0.0)))


def estimate_shell_speed(bullets: Sequence[Dict]) -> float:
    """从场上子弹实测速度取中位数，估计炮弹速度。"""
    vals = sorted(_speed(b) for b in bullets if 120.0 <= _speed(b) <= 1000.0)
    if not vals:
        return DEFAULT_SHELL_SPEED_PX_S
    return vals[len(vals) // 2]


def _reflect_dir(wall: np.ndarray, ix: int, iy: int,
                 px: float, py: float, dx: float, dy: float) -> Tuple[float, float]:
    """镜面反射：返回 (dx, dy) 反射后的方向。

    规则：
    - 条的「长面」（中间像素）按走向翻转：竖条 → 翻 dx，横条 → 翻 dy；
    - 条的「端点/角」（只有一侧邻居）按射线主导轴翻转（先撞到哪个面翻哪个轴），
      修正端点判错导致弹道方向不变、直接穿透薄墙的穿墙 bug；
    - 十字交点 / 孤点按进入棱边判定，最后退回全翻转。
    """
    h, w = wall.shape

    def at(cx: int, cy: int) -> bool:
        return 0 <= cx < w and 0 <= cy < h and wall[cy, cx]

    n, s = at(ix, iy - 1), at(ix, iy + 1)
    e, ww = at(ix + 1, iy), at(ix - 1, iy)
    vert = n or s
    horiz = e or ww
    endpoint = (e != ww) or (n != s)  # 横/竖条的端点或角
    if endpoint:
        # 撞端面：先撞到哪个面就翻哪个轴（水平主导 → 撞垂直面 → 翻 dx）
        return (-dx, dy) if abs(dx) >= abs(dy) else (dx, -dy)
    if vert and not horiz:
        return -dx, dy
    if horiz and not vert:
        return dx, -dy
    # 十字交点 / 孤点：按进入的棱边
    if py < iy - 0.5:
        return dx, -dy
    if py > iy + 0.5:
        return dx, -dy
    if px < ix - 0.5:
        return -dx, dy
    if px > ix + 0.5:
        return -dx, dy
    return -dx, -dy


class ShotSolution:
    """一条完整弹道（含反射）及对目标的最小距离。"""

    __slots__ = ('angle', 'bounces', 'path_len', 'miss', 'points')

    def __init__(self, angle: float, bounces: int, path_len: float,
                 miss: float, points: List[Point]):
        self.angle = angle
        self.bounces = bounces
        self.path_len = path_len
        self.miss = miss
        self.points = points

    def __repr__(self) -> str:
        return ('ShotSolution(angle=%.1fdeg bounces=%d len=%.0fpx miss=%.1fpx)'
                % (math.degrees(self.angle), self.bounces, self.path_len, self.miss))


def simulate_ray(wall: np.ndarray, x0: float, y0: float, angle: float,
                 target: Point, max_bounces: int = MAX_BOUNCES,
                 max_len: float = MAX_PATH_PX,
                 target_radius: float = HIT_RADIUS_PX,
                 min_dist_px: float = MUZZLE_SAFE_PX) -> Optional[ShotSolution]:
    """从 (x0, y0) 沿 angle 方向模拟一条带镜面反射的完整弹道。

    返回 ShotSolution（整条弹道 + 对 target 的最小距离），仅当
    最小距离 ≤ target_radius（即弹道确实能命中）；否则返回 None。
    单条弹道最坏 ~760 步，耗时亚毫秒级。

    只统计 total >= min_dist_px 且距炮口 >= min_dist_px 的路段：排除炮弹
    刚出膛及反弹绕回炮口附近的一小段，避免目标落在炮口旁时任何朝向都被
    误判为命中。
    """
    h, w = wall.shape
    dx, dy = math.cos(angle), math.sin(angle)

    # 起点若在墙内（贴墙瞄准的极端情况），沿反方向退到空地上
    x, y = x0, y0
    while 0 <= int(round(y)) < h and 0 <= int(round(x)) < w and wall[int(round(y)), int(round(x))]:
        x -= dx
        y -= dy
        if math.hypot(x - x0, y - y0) > 24.0:
            return None

    points: List[Point] = [(x, y)]
    bounced = 0
    total = 0.0
    min_miss = float('inf')
    last_sample = (x, y)
    hit_bounces: Optional[int] = None   # 首次进入命中半径时的反弹次数
    hit_len = 0.0                        # 首次进入命中半径时的路程
    last_wall_total = -1e9               # 上次撞墙时的路程（用于防卡墙/穿墙）

    while total < max_len:
        x += dx * STEP_PX
        y += dy * STEP_PX
        total += STEP_PX
        ix, iy = int(round(x)), int(round(y))
        if not (0 <= iy < h and 0 <= ix < w):
            break  # 飞出迷宫（真实迷宫有边框墙，合成测试无边框时到此为止）
        if wall[iy, ix]:
            if bounced >= max_bounces:
                break
            if total - last_wall_total < WALL_MIN_GAP_PX:
                break  # 反射后仍紧贴墙 → 反射判错/卡墙，放弃（防穿墙）
            last_wall_total = total
            dx, dy = _reflect_dir(wall, ix, iy, last_sample[0], last_sample[1], dx, dy)
            bounced += 1
            x += dx * WALL_LOOKAHEAD_PX
            y += dy * WALL_LOOKAHEAD_PX
            total += WALL_LOOKAHEAD_PX
            last_sample = (x, y)
            points.append((x, y))
            continue

        last_sample = (x, y)
        if (int(total) % 2 == 0 and total >= min_dist_px
                and math.hypot(x - x0, y - y0) >= min_dist_px):
            points.append((x, y))
            d = math.hypot(x - target[0], y - target[1])
            if d < min_miss:
                min_miss = d
            if min_miss <= target_radius and hit_bounces is None:
                hit_bounces = bounced
                hit_len = total

    if min_miss <= target_radius:
        return ShotSolution(angle, hit_bounces if hit_bounces is not None else 0,
                            hit_len, min_miss, points)
    return None


def solve_shot(wall: np.ndarray, me: Dict, target: Point,
               center_angle: Optional[float] = None) -> Optional[ShotSolution]:
    """粗扫 + 细调，返回能命中 target 的最优发射角。

    粗扫按（反射次数少、路程短）取最优，细调在最优角附近 ±2.5° 以 0.25°
    步长把弹道对准目标中心。找不到可行角返回 None。

    center_angle 非 None 时只在它附近 ±75° 窗口内粗扫（跟踪阶段快 ~2.4 倍，
    避免 360° 全扫 65ms 造成辅助线/转向滞后）；None 时全 360° 扫描（首次）。
    """
    me_x, me_y = float(me['x']), float(me['y'])
    me_angle = float(me.get('angle', 0.0))
    best: Optional[ShotSolution] = None
    best_ang_dist = float('inf')

    if center_angle is None:
        scan = range(360)
    else:
        start = int(round(math.degrees(center_angle))) - 90
        scan = [(start + i) % 360 for i in range(181)]  # ±90° 窗口，1° 步长

    for i in scan:
        a = math.radians(i)
        x0 = me_x + math.cos(a) * MUZZLE_OFFSET_PX
        y0 = me_y + math.sin(a) * MUZZLE_OFFSET_PX
        sol = simulate_ray(wall, x0, y0, a, target)
        if sol is None:
            continue
        key = (sol.bounces, sol.path_len)
        prev_key = (best.bounces, best.path_len) if best is not None else (99, 1e9)
        if best is None or key < prev_key or (key == prev_key and
                                              abs(wrap(a - me_angle)) < best_ang_dist):
            best = sol
            best_ang_dist = abs(wrap(a - me_angle))

    if best is None:
        return None

    refined = best
    steps = int(round(FINE_SPAN_RAD / FINE_STEP_RAD))
    for k in range(-steps, steps + 1):
        if k == 0:
            continue
        a = best.angle + k * FINE_STEP_RAD
        x0 = me_x + math.cos(a) * MUZZLE_OFFSET_PX
        y0 = me_y + math.sin(a) * MUZZLE_OFFSET_PX
        sol = simulate_ray(wall, x0, y0, a, target)
        if sol is None:
            continue
        if (sol.bounces, sol.miss) < (refined.bounces, refined.miss):
            refined = sol
    return refined


def _clamped_velocity(foe: Dict) -> Tuple[float, float]:
    """敌人速度钳制：桥的位置差分偶发大噪声会给出超大速度，导致预测点乱跳。"""
    vx = float(foe.get('vx', 0.0))
    vy = float(foe.get('vy', 0.0))
    sp = math.hypot(vx, vy)
    if sp > MAX_FOE_SPEED_PX_S:
        f = MAX_FOE_SPEED_PX_S / sp
        return vx * f, vy * f
    return vx, vy


def predicted_aim_point(foe: Dict, lead_t: float,
                        raw_wall: Optional[np.ndarray] = None) -> Point:
    """敌人预测位置。预测点必须在地图内且为自由空间，否则退回敌人当前位置。"""
    vx, vy = _clamped_velocity(foe)
    px = float(foe['x']) + vx * lead_t
    py = float(foe['y']) + vy * lead_t
    if raw_wall is not None:
        h, w = raw_wall.shape
        ix, iy = int(round(px)), int(round(py))
        if not (0 <= ix < w and 0 <= iy < h) or raw_wall[iy, ix]:
            return (float(foe['x']), float(foe['y']))
    return (px, py)


class CombatController:
    """半自主射击控制器：玩家控制移动，AI 接管左右键瞄准并开火。"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.fire_until = -1e9
        self.next_fire_t = -1e9
        self.shell_speed = DEFAULT_SHELL_SPEED_PX_S
        self.aim_angle: Optional[float] = None   # 锁定的目标发射角（None = 未接管）
        self.last_solve_t = -1e9
        self.turn_dir = 0
        self.last_sol: Optional[ShotSolution] = None  # 最近一次求解的弹道（调试用）

    # ------------------------------------------------------------------
    def attack(self, t: float, me: Dict, foe: Dict, bullets: Sequence[Dict],
               raw_wall: Optional[np.ndarray]) -> Dict:
        """一帧决策：有可行发射角 → 接管并「僵直」原地转向；当前朝向命中 → 开火。

        接管时返回 lock=1：桥据此锁死玩家 ⬆⬇⬅➡ 全部输入（僵直），
        由 AI 独占转向 + 开火；无可行角或敌人消失时 lock=0 交还。
        开火用「当前车头角度的实时弹道验证」，不依赖历史求解角度的误差。
        """
        out = {'keys': {'up': 0, 'down': 0, 'left': 0, 'right': 0}, 'fire': 0, 'lock': 0}
        if not me or not foe or raw_wall is None:
            self.aim_angle = None
            self.turn_dir = 0
            return out

        me_angle = float(me.get('angle', 0.0))
        me_x, me_y = float(me['x']), float(me['y'])
        self.shell_speed = max(160.0, estimate_shell_speed(bullets))

        # ---- 当前朝向（给定角度）能否命中：实时验证 ----
        def can_hit(angle: float):
            x0 = me_x + math.cos(angle) * MUZZLE_OFFSET_PX
            y0 = me_y + math.sin(angle) * MUZZLE_OFFSET_PX
            d0 = math.hypot(float(foe['x']) - me_x, float(foe['y']) - me_y)
            lead_t = min(MAX_LEAD_S, d0 / self.shell_speed)
            target = predicted_aim_point(foe, lead_t, raw_wall)
            sol = simulate_ray(raw_wall, x0, y0, angle, target)
            if sol is not None:
                lead_t = min(MAX_LEAD_S, sol.path_len / self.shell_speed)
                target = predicted_aim_point(foe, lead_t, raw_wall)
                return simulate_ray(raw_wall, x0, y0, angle, target)
            return None

        # ---- 节流重扫：是否存在能击中敌人的发射角（作为转向目标） ----
        # 修复：原条件 `aim_angle is None or t-last_solve_t>=RESOLVE_EVERY_S` 在
        # aim_angle=None（敌人墙后，98% 时间）时恒真 → 每帧都做 360° 全扫
        # （实测 32ms/帧），直接拖死 30Hz 导航循环。改为统一按时间节流，
        # aim_angle=None 时才做全扫，否则做 ±90° 窗口扫。
        if t - self.last_solve_t >= RESOLVE_EVERY_S:
            self.last_solve_t = t
            d0 = math.hypot(float(foe['x']) - me_x, float(foe['y']) - me_y)
            lead_t = min(MAX_LEAD_S, d0 / self.shell_speed)
            target = predicted_aim_point(foe, lead_t, raw_wall)
            sol = solve_shot(raw_wall, me, target, center_angle=self.aim_angle)
            if sol is not None:
                lead_t = min(MAX_LEAD_S, sol.path_len / self.shell_speed)
                target = predicted_aim_point(foe, lead_t, raw_wall)
                sol2 = solve_shot(raw_wall, me, target, center_angle=sol.angle)
                if sol2 is not None:
                    sol = sol2

            # 目标角稳定性：新解与旧目标角差太大时，若旧角度仍能命中则保留旧角度，
            # 避免「反射角对敌人位置敏感导致目标角乱跳、坦克追着乱转」。
            if (sol is not None and self.aim_angle is not None
                    and abs(wrap(sol.angle - self.aim_angle)) > AIM_SWITCH_RAD
                    and can_hit(self.aim_angle) is not None):
                sol = None  # 保留旧目标角

            if sol is not None:
                self.aim_angle = sol.angle
                self.last_sol = sol
            else:
                # 无新解：旧目标角仍可命中则保留，否则清除（交还）
                if self.aim_angle is not None and can_hit(self.aim_angle) is None:
                    self.aim_angle = None
                    self.last_sol = None

        if self.aim_angle is None:
            # 无可击中角度（或敌人已不可达）→ 取消接管，交还（lock=0）
            self.turn_dir = 0
            return out

        # ---- 接管（僵直）：朝 aim_angle 原地转向，锁死玩家输入 ----
        out['lock'] = 1
        err = wrap(self.aim_angle - me_angle)
        if abs(err) <= AIM_DEADBAND_RAD:
            self.turn_dir = 0
        elif self.turn_dir == 0 or abs(err) < U_TURN_LOCK_RAD:
            self.turn_dir = 1 if err > 0 else -1
        # 转向方向（以本仓库游戏引擎源码 c2runtime.js:35542/35546 + track.csv 实测为准）：
        #   left 键  → this.a -= steerSpeed*dt → 角度减小
        #   right 键 → this.a += steerSpeed*dt → 角度增大
        # err = wrap(aim_angle - me_angle)；err>0 = 目标在车头"角度增大"方向。
        # 故 err>0 → 按 right（增大）；err<0 → 按 left（减小）。
        # （旧注释"left 键让角度增大"是 v3 时代记录反了，导致朝反方向转、永不瞄准）
        if self.turn_dir > 0:
            out['keys']['right'] = 1
        elif self.turn_dir < 0:
            out['keys']['left'] = 1

        # ---- 开火：车头对准目标角（err≤FIRE_ALIGN）且该目标角弹道确实能命中
        # 当前敌人预测位置（can_hit 实时验证）才射。
        # 用 aim_angle 而非 me_angle 验证：反射弹道对角度极敏感，车头偏差 1°
        # 就会判不中（此前「对准了却不开火」的根因）；实时验证则避免目标角
        # 过时导致朝旧方向乱射（「乱转乱射」的根因之一）。
        if (abs(err) <= FIRE_ALIGN_RAD and t >= self.next_fire_t
                and can_hit(self.aim_angle) is not None):
            self.fire_until = t + FIRE_PULSE_S
            self.next_fire_t = t + FIRE_COOLDOWN_S
        if t < self.fire_until:
            out['fire'] = 1
        return out


# ----------------------------------------------------------------------
def _selftest() -> None:
    """合成墙掩码上验证：弹道模拟 + solve_shot + 接管/交还逻辑。"""

    def make_border():
        w = np.zeros((200, 200), dtype=bool)
        w[0, :] = True
        w[199, :] = True
        w[:, 0] = True
        w[:, 199] = True
        return w

    def check(name, cond, extra=''):
        print('  %s %s' % ('PASS' if cond else 'FAIL', name + ('  ' + extra if extra else '')))
        return cond

    ok = True

    def muzzle(me, deg):
        a = math.radians(deg)
        return (me[0] + 18.0 * math.cos(a), me[1] + 18.0 * math.sin(a))

    # 1) 直瞄命中
    wall = make_border()
    me = (40.0, 100.0)
    foe = (150.0, 100.0)
    x0, y0 = muzzle(me, 0.0)
    sol = simulate_ray(wall, x0, y0, 0.0, foe)
    ok &= check('direct hit', sol is not None and sol.bounces == 0 and sol.miss < 5.0,
                'miss=%.1f' % (sol.miss if sol else -1))

    # 2) 直瞄不中（偏 25°）
    x0, y0 = muzzle(me, 25.0)
    ok &= check('direct miss', simulate_ray(wall, x0, y0, math.radians(25.0), foe) is None)

    # 3) 1 次反射命中
    wall = make_border()
    wall[40:160, 80] = True
    foe = (115.0, 95.0)
    x0, y0 = muzzle(me, 294.0)
    sol = simulate_ray(wall, x0, y0, math.radians(294.0), foe)
    ok &= check('1-bounce hit', sol is not None and sol.bounces == 1,
                'miss=%.1f bounces=%d' % (sol.miss if sol else -1, sol.bounces if sol else -1))

    # 4) solve_shot 能找到发射角（直瞄场景，炮口未对准也应返回 0°）
    raw = make_border()
    me_d = {'x': 40.0, 'y': 100.0, 'angle': math.radians(90.0), 'aim': 0.0}
    foe_d = {'x': 150.0, 'y': 100.0, 'angle': 0.0, 'vx': 0.0, 'vy': 0.0}
    sol = solve_shot(raw, me_d, (foe_d['x'], foe_d['y']))
    ok &= check('solve_shot finds angle', sol is not None and abs(wrap(sol.angle - 0.0)) < 0.1,
                'angle=%.1fdeg' % (math.degrees(sol.angle) if sol else -1))

    # 5) solve_shot 反射场景也能找到角
    wall_r = make_border()
    wall_r[40:160, 80] = True
    sol = solve_shot(wall_r, me_d, (115.0, 95.0))
    ok &= check('solve_shot finds reflected angle', sol is not None,
                'angle=%.1fdeg bounces=%d' % (math.degrees(sol.angle) if sol else -1,
                                              sol.bounces if sol else -1))

    # 6) 完全被墙隔离 → 无可行角
    wall_closed = make_border()
    wall_closed[:, 80] = True  # 全高竖墙，左右完全隔开
    sol = solve_shot(wall_closed, me_d, (120.0, 100.0))
    ok &= check('no angle when walled off', sol is None)

    # 7) 控制器：有可行角 → 接管左右键原地转向，对准后开火
    ctrl = CombatController()
    ctrl.reset()
    # 初始朝右(0°)，敌人在正右方(0°)，已对准 → 立即开火，不转向
    me_r = dict(me_d, angle=0.0)
    a = ctrl.attack(0.0, me_r, foe_d, [], raw)
    ok &= check('fires immediately when already aligned',
                a['fire'] == 1 and a['keys']['up'] == 0 and a['keys']['down'] == 0
                and a['keys']['left'] == 0 and a['keys']['right'] == 0,
                'act=%s' % a)

    # 8) 未对准（朝下 90°，敌人在右 0°）→ 接管转向，不碰上下键，未对准不开火
    ctrl2 = CombatController()
    ctrl2.reset()
    me_down = dict(me_d, angle=math.radians(90.0))
    a = ctrl2.attack(0.0, me_down, foe_d, [], raw)
    # 朝下(π/2)→朝右(0°) 需角度减小 → left 键（游戏 left=角度减小）
    ok &= check('takes over turning when misaligned',
                a['keys']['left'] == 1 and a['keys']['right'] == 0
                and a['keys']['up'] == 0 and a['keys']['down'] == 0 and a['fire'] == 0,
                'act=%s' % a)

    # 9) 转到位后（模拟 angle 逼近 0°）→ 开火
    me_almost = dict(me_d, angle=math.radians(3.0))
    a = ctrl2.attack(0.05, me_almost, foe_d, [], raw)
    ok &= check('fires once aligned', a['fire'] == 1, 'act=%s' % a)

    # 10) 敌人离开可击中位置（隔墙）→ 重扫后取消接管，交还左右键
    ctrl3 = CombatController()
    ctrl3.reset()
    a0 = ctrl3.attack(0.0, me_down, foe_d, [], raw)  # 有解，接管
    foe_moved = {'x': 120.0, 'y': 100.0, 'angle': 0.0, 'vx': 0.0, 'vy': 0.0}
    # 触发重扫（t 前进 > RESOLVE_EVERY_S），foe 已到全高墙另一侧
    a1 = ctrl3.attack(0.25, me_down, foe_moved, [], wall_closed)
    ok &= check('releases keys when foe unreachable',
                a0['keys']['left'] == 1 and
                a1['keys'] == {'up': 0, 'down': 0, 'left': 0, 'right': 0} and a1['fire'] == 0,
                'a0=%s a1=%s' % (a0, a1))

    # 11) 敌人消失（击杀）→ 交还
    a2 = ctrl3.attack(0.3, me_down, None, [], wall_closed)
    ok &= check('releases keys when foe gone',
                a2['keys'] == {'up': 0, 'down': 0, 'left': 0, 'right': 0} and a2['fire'] == 0)

    if not ok:
        print('SELFTEST FAILED')
        raise SystemExit(1)
    print('SELFTEST PASSED')


if __name__ == '__main__':
    _selftest()
