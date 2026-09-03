# -*- coding: utf-8 -*-
"""绿色坦克「半自主射击」：玩家控制移动，AI 在能打到敌人时接管左右键瞄准并开火。

行为：
1. 「是否存在可击中敌人的发射角」由两条触发路径发现（F1 修复：旧方案只靠节流
   重扫发现机会，导航中车头扫过命中窗口（300px 处约 7°）的瞬间永远等不到
   0.2s 后的重扫 → 射击不及时）：
   - 触发方式 A（零延迟）：每帧检查「当前车头朝向」的弹道（含反射、自伤、
     预测验证 can_hit(me_angle)），命中即立即接管瞄准/开火——车头恰好扫过
     命中窗口的瞬间不再错过；
   - 触发方式 B（每 RESOLVE_EVERY_S 秒节流）：以「敌-我连线」朝向为相位对齐
     的粗扫（15°×24 条，无命中再 5°×72 条兜底）→ 命中处 ±10° 以 1° 细化，
     用 simulate_ray（直线飞行 + 墙面镜面反射，最多 MAX_BOUNCES 次）找一条
     穿过敌人「预测位置」半径 HIT_RADIUS_PX 圆的最优弹道（solve_shot）——
     覆盖「转几度就能命中」的墙后/偏移机会。
2. 任一路径找到发射角（不要求当前朝向已对准）→ AI 进入「接管瞄准」状态：
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

from kalman_path_predictor import ConstantVelocityKalman2D
from bot_config import (USE_KALMAN_PREDICTOR,   # 敌方位置预测开关（bot_config.py）
                        CONF_FIRE_MIN_DIRECT, CONF_LOCK_MIN_DIRECT,
                        FIRE_MISS_MAX_DIRECT_PX, FIRE_COOLDOWN_S,
                        RESOLVE_EVERY_S, LOCK_RELEASE_COOL_S)   # 火控门槛/节奏（bot_config.py）

Point = Tuple[float, float]

# ---- 弹速配置：config.json 的"子弹速度倍率" ----
# 实测：原版子弹峰值 ~360px/s（BASE），当前配置 0.5× → 实测峰值 ~174px/s。
# 若仍按 360 估算提前量 → lead 时间只算真实飞行的一半 → 移动目标直射必偏
# （"直射不准确"根因）；弹道最大长度同理按实际速度缩短（>实际射程的射线
# "命中"是假命中——子弹飞不到就消失）。启动时读一次配置，找不到按 1.0。
def _read_config_shell_multiplier() -> float:
    try:
        import json as _json
        import os as _os
        p = _os.path.join('config', 'config.json')
        if _os.path.exists(p):
            with open(p, encoding='utf-8-sig') as _f:
                return float(_json.load(_f).get('子弹速度倍率', 1.0))
    except Exception:
        pass
    return 1.0


CONFIG_SHELL_MULTIPLIER = _read_config_shell_multiplier()
BASE_SHELL_SPEED_PX_S = 360.0          # 原版全速子弹初速（实测峰值）
DEFAULT_SHELL_SPEED_PX_S = BASE_SHELL_SPEED_PX_S * CONFIG_SHELL_MULTIPLIER

# 弹道模拟
MAX_BOUNCES = 2            # 最多反射次数（与游戏原版 AI 一致：normal=1 / hard=2）。
                           # 之前设 4 会把「3~4 次边框反弹扫过目标」误判为命中，
                           # 导致 AI 朝几乎错误的方向乱开火（炮弹乱弹自伤）。
MAX_PATH_PX = 760.0 * CONFIG_SHELL_MULTIPLIER   # 炮弹最大总路程（≈初速 × 2.1s，
                                                # 按实际配置弹速缩放）
STEP_PX = 1.0              # 光线步长（像素）
HIT_RADIUS_PX = 18.45      # 命中判定：弹道对目标的最小距离 ≤ 此值（= 敌方坦克真实
                            # 碰撞盒 31×20 的外接圆半径 sqrt(15.5²+10²)）。原 30px 过宽：
                            # miss 20~30px 的"擦边命中"物理上打不中坦克，导致发射后子弹
                            # 继续飞行、撞墙反弹自伤（假直射/自杀根源之一）。
MUZZLE_SAFE_PX = 36.0      # 出膛近区不计入命中判定：炮弹刚离膛的一段路程不参与
                           # min_miss 统计，避免「敌人/预测点恰好落在炮口附近」时
                           # 任何朝向的弹道都被误判为命中（无视野乱开火的根源）
NEAR_KILL_PX = 45.0        # 近战特判距离：foe 距 me ≤45px 且炮口直线可达时，子弹
                           # 出生点（炮口，车头前 18px）就在 foe 车体内/紧贴 → 游戏
                           # 里出生即命中，不受 MUZZLE_SAFE 出膛盲区影响（盲区让
                           # 贴脸 foe 的弹道命中段全落在排除区 → 永远判不中 →
                           # "贴脸 foe 冲脸打不出"）。直接给朝向解（conf 0.95）。
WALL_MIN_GAP_PX = 4.0      # 两次撞墙之间的最小自由间隔：若反射后仍紧贴墙，说明
                           # 反射方向判错或卡进墙里（防「穿墙」模拟），放弃该弹道
MUZZLE_OFFSET_PX = 18.0    # 炮口相对坦克中心的前伸量
WALL_LOOKAHEAD_PX = 2.0    # 反射后向前推开的距离，避免立刻再次撞到同一像素
FINE_STEP_RAD = math.radians(0.25)   # 细调步长
FINE_SPAN_RAD = math.radians(2.5)    # 细调范围（围绕粗扫最优角）

# 瞄准 / 开火
# 停转窗（车头停进窗内 = 发射角确定，静止 FIRE_ALIGN_FRAMES 帧后才射）分叉：
# - 直射解（0 反射）：3.5°（容易停稳，直射弹道对角度不敏感，停稳即射即中）；
# - 反射解：2.5°。
# 反射窗的取值依据（实测转向满舵单步 ≈4.4°/bot帧）：离散步进 s 要能"一步落进
# 窗口 w"，必须 w ≥ s/2（窗边一步后落点 err' = err - s ∈ [w-s, w-s+…] ⊆ [-w, w]
# ⟺ s ≤ 2w）。原 0.8° ≪ 2.2°（s/2）→ 车头永远在 ±1.5~5.5° 之间来回越窗，
# 极限环干锁（"直瞄犹豫被反杀"根源：lock 站桩等一个到不了的窗口）。2.5° ≥ 2.2°
# → 从窗外任意位置最多一步必落窗内，极限环终结。
# 停转窗 = 发射门限本身（不再按 miss 余量收紧 err：can_hit 已用当前 me_angle 模拟，
# err 的几何代价已计入 miss，再收紧是双重惩罚 → 远端擦边火站桩等 err→0 被反杀）。
AIM_DEADBAND_RAD = math.radians(2.5)
FIRE_ALIGN_RAD = math.radians(5.0)     # （保留）发射条件：车头角与计算角偏差 ≤5° 才开火。
FIRE_ALIGN_FRAMES = 3                  # 开火前车头须连续 N 帧停在停转窗口内（真静止）：
                                        # fire 决策到实际发射之间车若还在转，发射角不可预测
                                        # → 验证的弹道作废（打偏/自伤根源）。直射准度来自
                                        # "发射时角度确定"，不是射得快。
                                        # 近战（弹道路程 ≤ NEAR_FIRE_PATH_PX）放宽到 1 帧：
                                        # 子弹 <0.4s 就到，角度不确定影响小（err 2.5° @
                                        # 150px ≈ 6.5px）；而贴脸 foe 绕圈时 err 每秒进出
                                        # 窗口多次，3 帧连续静止几乎凑不齐 → 永远等不到
                                        # 开火窗口就被贴脸打死（差 1~2 帧静止被反杀）。
NEAR_FIRE_PATH_PX = 150.0              # 近战弹道路程阈值（≤ 此值按近战处理）
DIRECT_STOP_RAD = math.radians(3.5)    # 直射解停转窗（≈一个转向单步，易停稳）
# 转向接近减速（PWM）已移除：停转窗 ≥ 半步长，满舵一步必落窗、无极限环；
# PWM 分时减速只拖慢瞄准（射击慢被反杀元凶之一）。常量保留备查。
TURN_SLOW_ERR_RAD = math.radians(6.0)
TURN_SLOW_DUTY = 0.4
AIM_SWITCH_RAD = math.radians(45.0)    # 新解与旧目标角差超过此值时不切换（防目标角乱跳）
U_TURN_LOCK_RAD = math.pi - 0.35       # 接近 180° 掉头时锁定转向方向，防左右抖动
FIRE_PULSE_S = 0.10                    # 按键脉冲时长
# 开火冷却 / 重扫周期 / 交还冷却 → bot_config.py（FIRE_COOLDOWN_S / RESOLVE_EVERY_S /
# LOCK_RELEASE_COOL_S），可运行中调参不改这里。

# ---- 机会发现扫描（触发方式 B 参数）----
COARSE_STEP_DEG = 15.0             # 粗扫步长：24 条射线覆盖全圆，相位对齐"敌-我连线"
                                   # 朝向（直瞄角恰在网格点上 → 最窄的直瞄窗也不漏）
COARSE_FALLBACK_STEP_DEG = 5.0     # 粗扫无命中时的二次步长（72 条）：兜住 15° 网格
                                   # 漏掉的窄反射窗（窗口 <5° 的边际解仍可能漏检，
                                   # 但那些 margin 极小、本就会被 conf 门拦下）
REFINE_SPAN_DEG = 10.0             # 命中处细化/角裕度走查范围（±10°，1° 步长）
REFINE_STEP_DEG = 1.0
REFINE_MAX_CENTERS = 6             # 最多细化几个粗扫命中角（按命中质量取前 N，控射线总量）
TRIGGER_CURRENT_SPAN_DEG = 10.0    # 触发方式 A 的角裕度走查范围（±10°）
FIRE_MISS_MAX_PX = 15.0                # 反射解发射时弹道离目标中心 ≤ 此值才打（直射
                                        # 解用 bot_config.FIRE_MISS_MAX_DIRECT_PX=18.45）：
                                        # 坦克内切圆半径 10px（31×20 矩形任何方向距中心
                                        # <10px 必穿），10~15 是角区（斜向可穿矩形角）；
                                        # 15~18.45 纯擦边命中率极低且打空后贴墙反弹
                                        # 自伤风险高。原 12 过严把 12~15 的可打角区火
                                        # 全掐（edge_miss 干等），放宽到 15 减少犹豫。
LOCK_RELEASE_TIMEOUT_S = 1.5           # 接管(lock)总超时：转向大角度（>60°）需 ~0.6s+
                                        # （满舵 ~4.4°/帧），0.8s 会在转完前掐断 → 反复
                                        # 锁-交还-冷却 → "不果断"。1.5s 给足转向时间；
                                        # 干等站桩已由 err_tol 删除 + foe 静止门槛消除。
# 交还冷却 LOCK_RELEASE_COOL_S → bot_config.py

# 弹速 / 提前量
MAX_LEAD_S = 0.90
MAX_FOE_SPEED_PX_S = 600.0  # 敌人速度钳制（桥的位置差分偶发大噪声时，防预测点乱跳进墙/乱飞）
FOE_STILL_SPD_PX_S = 30.0   # 停走检测：观测速度低于此值视为"停"（玩家/原版 AI 走停切换频繁）
FOE_STILL_FRAMES = 2        # 连续停 2 帧（~0.06s）→ 认定 foe 静止，预测直接用观测位置。
                            # 恒速卡尔曼的平滑速度在 foe 停下后仍残留旧速度（观测差分噪声大
                            # → 强平滑 → v 归零慢），预测点带着残余前置飘 10~20px —— 实测
                            # foe 静止时打的火仍偏 14px（命中半径才 18.45）→ 静止必中火
                            # 全部落空。确认必须快：foe 停下就是要开火对射，拖 0.15s 确认 +
                            # 瞄准点跳变重转 = 我方第一发慢 0.3~0.5s → 没开枪就被反杀。

# 射击置信度 C ∈ [0,1]：越高越该打。权重见 CONF_W_*，开火门槛 CONF_FIRE_MIN。
# 五项：margin(角度裕度) / bounce(反射次数) / turn(旋转角) / dist(弹道距离) / pred(预测不确定度)
CONF_W_MARGIN = 0.30        # 角度裕度：端点反射 margin≈0 会被压到后面
CONF_W_BOUNCE = 0.25        # 鼓励直射/少反射
CONF_W_MISS = 0.10          # 精确度：弹道离目标中心越近越好（擦边不如正中）
CONF_W_TURN = 0.10          # 旋转耗时越小越好
CONF_W_DIST = 0.10          # 距离适中最好（太近=贴脸躲不开，太远=飞行久）
CONF_W_PRED = 0.15          # 预测越确定越好
CONF_FIRE_MIN = 0.50        # 开火门槛：C 低于此值不开火（宁可不打，不送自伤）。
                            # 0.55→0.50：双反弹/贴墙族 conf 0.50~0.55 的火此前全被
                            # conf_low 干等拦截（站桩等 foe 动），放宽减少犹豫失败。
                            # 且 conf < 门槛的解不再锁为 aim（见 attack：锁低 conf
                            # aim = 干等 + 重扫换解时 aim 大跳反向转）。
CONF_SWITCH_MARGIN = 0.15   # 换瞄准角：新解 C 需比旧解高出此值才切换
MARGIN_FULL_DEG = 12.0      # margin 满分刻度：≥12° 记满分
DIST_OPT_PX = 180.0         # 最优弹道距离（待实测标定）
DIST_SIGMA_PX = 120.0       # 距离容忍宽度
PRED_SIGMA0_PX = 40.0       # 预测不确定度参考尺度（σ=40px → C_pred=0.5）
PRED_SIGMA_OFF_PX = 40.0    # 卡尔曼关闭时使用的固定预测不确定度（外推噪声大，
                            # 直接用该保守值进置信度：σ=40px → C_pred=0.5）
SELF_SAFE_PX = 20.0         # 自伤否决：弹道离自己中心最近距离 < 此值直接 C=0

# 坦克碰撞盒（与游戏一致：31×20，半长 15.5 / 半宽 10）——自伤判定必须用
# 矩形相交，不能用"离中心点距离"（点距离会漏判擦车/贴墙反弹穿车）
TANK_HALF_LEN_PX = 15.5
TANK_HALF_W_PX = 10.0
FIRE_SAFE_DEV_RAD = math.radians(3.0)   # 开火前偏差验证：实际发射角可能偏 ±3°，
                                        # 偏差后的弹道也不得穿过自己（防贴墙反弹自杀）
MIN_LEG_PX = 30.0                       # 反射解入射段下限：首个反射点须距炮口 ≥30px。
                                        # 入射段过短 = 贴脸打反弹：出膛 30px 内就撞墙，
                                        # 撞墙点差 1~2px（模拟网格 vs 游戏碰撞盒）→ 反弹
                                        # 方向偏 2°+，反弹段不可预测 → 乱弹/弹回自伤
                                        # （实测自杀局入射段仅 28px）。30px 拦这种极端
                                        # 贴脸；普通贴墙（炮口距墙 30~45px）的反射火恢复
                                        # ——45px 过严把贴墙正常对射全砍（贴墙局 4.9s
                                        # 一枪未发被反杀）。
# （已撤销）NEAR_B2_PATH_PX：近距双反弹否决被证明是误伤 —— 010151 会话回放显示
# b2 近火打不中主因是预测误差（停走检测已修），几何误差二次反弹只占小头；且否决
# 把"贴脸绕行 foe 的唯一解"砍成 conf=0 → 追瞄 5 秒一枪未发。恢复由 conf 的反射
# 惩罚（c_bounce 0.36）与 dev ±3° 自伤检查把关。


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

    __slots__ = ('angle', 'bounces', 'path_len', 'miss', 'points',
                 'self_clear', 'conf', 'margin_deg', 'hit_self', 'leg1')

    def __init__(self, angle: float, bounces: int, path_len: float,
                 miss: float, points: List[Point], self_clear: float = float('inf'),
                 conf: float = 0.0, margin_deg: float = 0.0, hit_self: bool = False,
                 leg1: float = float('inf')):
        self.angle = angle
        self.bounces = bounces
        self.path_len = path_len
        self.miss = miss
        self.points = points
        self.self_clear = self_clear
        self.conf = conf
        self.margin_deg = margin_deg
        self.hit_self = hit_self
        self.leg1 = leg1          # 首个反射点距炮口的路程（直射 = inf）

    def __repr__(self) -> str:
        return ('ShotSolution(angle=%.1fdeg bounces=%d len=%.0fpx miss=%.1fpx)'
                % (math.degrees(self.angle), self.bounces, self.path_len, self.miss))


def _point_in_tank(px: float, py: float, me_x: float, me_y: float,
                   me_angle: float) -> bool:
    """点是否落在坦克矩形内（车体 31×20，带车头朝向的旋转矩形）。"""
    dx, dy = px - me_x, py - me_y
    ca, sa = math.cos(me_angle), math.sin(me_angle)
    lx = dx * ca + dy * sa      # 车头方向分量
    ly = -dx * sa + dy * ca     # 横向分量
    return abs(lx) <= TANK_HALF_LEN_PX and abs(ly) <= TANK_HALF_W_PX


def _segment_hits_wall(wall: np.ndarray, a: Point, b: Point) -> bool:
    """a→b 直线是否穿墙（1px 步进采样）。命中结算用：命中点到目标中心的
    连线必须无墙，否则目标是"在墙另一侧的预测点"，反弹点擦边算中的是假命中。"""
    h, w = wall.shape
    dx, dy = b[0] - a[0], b[1] - a[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-9:
        return False
    steps = max(1, int(dist))
    for i in range(1, steps + 1):
        t = i / steps
        x, y = a[0] + dx * t, a[1] + dy * t
        ix, iy = int(round(x)), int(round(y))
        if not (0 <= ix < w and 0 <= iy < h):
            return True   # 出界视为墙（命中点与目标之间不可越界）
        if wall[iy, ix]:
            return True
    return False


def simulate_ray(wall: np.ndarray, x0: float, y0: float, angle: float,
                 target: Point, max_bounces: int = MAX_BOUNCES,
                 max_len: float = MAX_PATH_PX,
                 target_radius: float = HIT_RADIUS_PX,
                 min_dist_px: float = MUZZLE_SAFE_PX,
                 me_center: Optional[Point] = None,
                 tank: Optional[Tuple[float, float, float]] = None
                 ) -> Optional[ShotSolution]:
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
    tank_ca = tank_sa = 0.0
    if tank is not None:
        # 预计算车体系旋转量：自伤矩形检测复用，避免逐点重复 cos/sin
        # （全扫 360 角 × 760 步 = 27 万次，重复算 cos/sin 会拖慢 2~3 倍）
        tank_ca, tank_sa = math.cos(tank[2]), math.sin(tank[2])
    total = 0.0
    min_miss = float('inf')
    min_miss_pt = (x, y)     # 最小距离发生点（命中结算时验证"该点→目标"无墙）
    min_self = float('inf')  # 出膛近区之外，弹道离自己中心(me_center)的最近距离
    last_sample = (x, y)
    hit_bounces: Optional[int] = None   # 首次进入命中半径时的反弹次数
    hit_len = 0.0                        # 首次进入命中半径时的路程
    last_wall_total = -1e9               # 上次撞墙时的路程（用于防卡墙/穿墙）
    leg1 = float('inf')                  # 首个反射点距炮口的路程（防"贴墙反弹"火）

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
            if bounced == 0:
                leg1 = total              # 首个反射点路程（撞墙瞬间，未含推开量）
            dx, dy = _reflect_dir(wall, ix, iy, last_sample[0], last_sample[1], dx, dy)
            bounced += 1
            x += dx * WALL_LOOKAHEAD_PX
            y += dy * WALL_LOOKAHEAD_PX
            total += WALL_LOOKAHEAD_PX
            last_sample = (x, y)
            points.append((x, y))
            continue

        last_sample = (x, y)
        # 矩形自伤检测（内联，零函数调用/零 cos/sin 重算）：命中前弹道点
        # 进入车体 = 自伤（立即返回 hit_self 解）。贴墙反弹自杀就发生在
        # 出膛 1~3px 内，故逐点检查。
        if hit_bounces is None and tank is not None:
            tdx, tdy = x - tank[0], y - tank[1]
            if (abs(tdx * tank_ca + tdy * tank_sa) <= TANK_HALF_LEN_PX
                    and abs(-tdx * tank_sa + tdy * tank_ca) <= TANK_HALF_W_PX):
                points.append((x, y))
                return ShotSolution(angle, bounced, total, float('inf'), points,
                                    self_clear=min_self, hit_self=True, leg1=leg1)
        if (int(total) % 2 == 0 and total >= min_dist_px
                and math.hypot(x - x0, y - y0) >= min_dist_px):
            points.append((x, y))
            d = math.hypot(x - target[0], y - target[1])
            if d < min_miss:
                min_miss = d
                min_miss_pt = (x, y)
            if min_miss <= target_radius and hit_bounces is None:
                hit_bounces = bounced
                hit_len = total
            if me_center is not None and hit_bounces is None:
                # 只统计命中前路段：命中后炮弹已消失，回飞段不计入自伤
                ds = math.hypot(x - me_center[0], y - me_center[1])
                if ds < min_self:
                    min_self = ds

    if min_miss <= target_radius:
        # 命中结算把关：最小距离点到目标中心的连线必须无墙（目标盘与墙重叠时，
        # 弹道贴着墙的"擦边命中"是假命中 —— 子弹实际被墙挡住，到不了目标；
        # 实测：预测点被泛推到墙后 6px 时，一次 2 反射弹道在墙反弹点"扫过"
        # 目标被判中，实为穿过墙体的假命中 → 朝错误方向锁弹道/开火失准）。
        # 命中点本身就是射线终点前最近的自由点，此检查只在线程末尾执行一次。
        if not _segment_hits_wall(wall, min_miss_pt, target):
            return ShotSolution(angle, hit_bounces if hit_bounces is not None else 0,
                                hit_len, min_miss, points, self_clear=min_self,
                                leg1=leg1)
    return None


def trace_beam(wall: np.ndarray, x0: float, y0: float, angle: float,
               max_px: float = 90.0) -> List[Point]:
    """炮口瞄准光束预览：从 (x0,y0) 沿 angle 模拟最多 max_px 行程的折线
    （含墙面镜面反射），每 ~6px 一点 + 反射点。不判命中、不判自伤——
    纯实时预览，负载轻；开火安全由 ±3° 三次推演负责。
    """
    h, w = wall.shape
    dx, dy = math.cos(angle), math.sin(angle)
    x, y = x0, y0
    out: List[Point] = [(x, y)]
    travelled = 0.0
    bounced = 0
    last_wall_total = -1e9
    while travelled < max_px:
        x += dx * STEP_PX
        y += dy * STEP_PX
        travelled += STEP_PX
        ix, iy = int(round(x)), int(round(y))
        if not (0 <= iy < h and 0 <= ix < w):
            break
        if wall[iy, ix]:
            if bounced >= MAX_BOUNCES or travelled - last_wall_total < WALL_MIN_GAP_PX:
                break
            last_wall_total = travelled
            dx, dy = _reflect_dir(wall, ix, iy, x, y, dx, dy)
            bounced += 1
            out.append((x, y))
            continue
        if math.hypot(x - out[-1][0], y - out[-1][1]) >= 6.0:
            out.append((x, y))
    return out


def aim_beam(wall: np.ndarray, me: Dict, max_px: float = 90.0) -> List[Point]:
    """当前车头方向的炮口弹道预览（bot 地图坐标，网页激光光束可视化用）。"""
    me_x, me_y = float(me['x']), float(me['y'])
    a = float(me.get('angle', 0.0))
    x0 = me_x + math.cos(a) * MUZZLE_OFFSET_PX
    y0 = me_y + math.sin(a) * MUZZLE_OFFSET_PX
    return trace_beam(wall, x0, y0, a, max_px)


def shot_confidence(sol: Optional[ShotSolution], margin_deg: float,
                    turn_angle: float, pred_sigma: float) -> float:
    """射击置信度 C ∈ [0,1]，越高越该打。

    margin_deg：角度裕度（左右能偏多少度仍命中，来自粗扫结果）。
    turn_angle：当前车头角转到发射角的角度（rad）。
    pred_sigma：卡尔曼对敌人位置的不确定度（px）。
    """
    if sol is None or sol.hit_self:
        return 0.0                      # 自伤否决：弹道会穿过自己车体
    if sol.self_clear < SELF_SAFE_PX:
        return 0.0                      # 自伤否决：弹道会反弹回来打自己
    if sol.bounces > 0 and sol.leg1 < MIN_LEG_PX:
        return 0.0                      # 贴墙反弹否决：入射段过短，反弹方向不可预测
                                        # （模拟网格与游戏碰撞盒差 1~2px → 反弹偏 2°+，
                                        #  乱弹/弹回自伤温床）
    c_bounce = 0.6 ** sol.bounces   # 反射惩罚：0=1.0, 1=0.6, 2=0.36（强鼓励直射）
    c_dist = math.exp(-((sol.path_len - DIST_OPT_PX) ** 2) / (2.0 * DIST_SIGMA_PX ** 2))
    c_turn = max(0.0, 1.0 - abs(turn_angle) / math.pi)
    c_margin = min(1.0, margin_deg / MARGIN_FULL_DEG)
    c_miss = max(0.0, 1.0 - sol.miss / HIT_RADIUS_PX)
    c_pred = 1.0 / (1.0 + pred_sigma / PRED_SIGMA0_PX)
    return (CONF_W_MARGIN * c_margin + CONF_W_BOUNCE * c_bounce + CONF_W_MISS * c_miss
            + CONF_W_TURN * c_turn + CONF_W_DIST * c_dist + CONF_W_PRED * c_pred)


def _margin_deg(hits: Dict[int, ShotSolution], i: int, bounces: int) -> float:
    """从粗扫命中集合估算角度裕度：左右连续同反弹次数的命中跨度（度）。

    粗扫按 1° 步长，复用已有结果，零额外射线。端点反射只有孤点命中 → margin≈1°；
    直射/长面反射连续十几度命中 → margin 大。
    """
    left = i
    while left - i < 90 and ((left - 1) % 360) in hits and hits[(left - 1) % 360].bounces == bounces:
        left -= 1
    right = i
    while right - i < 90 and ((right + 1) % 360) in hits and hits[(right + 1) % 360].bounces == bounces:
        right += 1
    return float(right - left) + 1.0


def _heading_margin_deg(wall: np.ndarray, me_x: float, me_y: float,
                        me_angle: float, target: Point, me_center: Point,
                        tank_pose: Tuple[float, float, float],
                        max_span_deg: float = TRIGGER_CURRENT_SPAN_DEG) -> float:
    """当前车头朝向两侧的连续命中跨度（度）：±max_span_deg 以 1° 走查，遇未命中即停。

    触发方式 A（当前朝向已命中）时估算角裕度用 —— 只在该罕见事件发生时执行
    （+最多 20 条射线一次性），常规每帧路径不经过这里。
    """
    span = 1.0
    for sign in (1.0, -1.0):
        for step in range(1, int(round(max_span_deg)) + 1):
            a = me_angle + sign * math.radians(step)
            x0 = me_x + math.cos(a) * MUZZLE_OFFSET_PX
            y0 = me_y + math.sin(a) * MUZZLE_OFFSET_PX
            sol = simulate_ray(wall, x0, y0, a, target,
                               me_center=me_center, tank=tank_pose)
            if sol is None or sol.hit_self:
                break
            span += 1.0
    return span


def solve_shot(wall: np.ndarray, me: Dict, target: Point,
               center_angle: Optional[float] = None,
               pred_sigma: float = 0.0) -> Optional[ShotSolution]:
    """粗扫 + 细化，返回能命中 target 的最优发射角（触发方式 B）。

    扫描策略（相比旧版 1° 全扫 360° / ±90° 窗：覆盖率相同量级、射线量约 1/4，
    且相位对齐"敌-我连线"保证直瞄窄窗恰在粗扫网格点上，不漏最值钱的机会）：
    - 15° 粗扫 24 条（网格相位 = 敌-我连线朝向）；
    - 无命中 → 5° 粗扫 72 条兜底（捕捉 15° 网格漏掉的窄反射窗）；
    - 命中处 ±REFINE_SPAN_DEG 以 1° 走查（连续命中段 → 1° 命中字典，
      供 margin 复用 + 捕捉网格点之间的命中细节），只细化命中质量前
      REFINE_MAX_CENTERS 个，控制射线总量；
    - 与旧实现同口径：按置信度 C 选最优（reflect 少 + miss 小 + margin 大），
      再在最优角附近 ±2.5° 以 0.25° 细调对准目标中心。

    center_angle 保留仅作兼容（旧 ±90° 窗扫描已被全圆粗扫取代，不再影响扫描）。
    """
    me_x, me_y = float(me['x']), float(me['y'])
    me_angle = float(me.get('angle', 0.0))
    me_center = (me_x, me_y)
    tank_pose = (me_x, me_y, me_angle)

    bearing_deg = math.degrees(math.atan2(target[1] - me_y, target[0] - me_x))
    base = int(round(bearing_deg))

    def coarse_pass(step_deg: float) -> List[int]:
        found: List[int] = []
        n = int(round(360.0 / step_deg))
        for k in range(n):
            i = (base + int(round(k * step_deg))) % 360
            if i in hits:
                continue
            a = math.radians(i)
            x0 = me_x + math.cos(a) * MUZZLE_OFFSET_PX
            y0 = me_y + math.sin(a) * MUZZLE_OFFSET_PX
            sol = simulate_ray(wall, x0, y0, a, target,
                               me_center=me_center, tank=tank_pose)
            if sol is not None and not sol.hit_self:
                hits[i] = sol
                found.append(i)
        return found

    hits: Dict[int, ShotSolution] = {}
    found = coarse_pass(COARSE_STEP_DEG)
    if not found:
        found = coarse_pass(COARSE_FALLBACK_STEP_DEG)
    if not found:
        return None

    # 细化：命中处左右 ±REFINE_SPAN_DEG 以 1° 走查连续命中段
    span_steps = int(round(REFINE_SPAN_DEG / REFINE_STEP_DEG))
    centers = sorted(found, key=lambda i: (hits[i].bounces, hits[i].miss,
                                           hits[i].path_len))[:REFINE_MAX_CENTERS]
    for i0 in centers:
        for sign in (1, -1):
            for step in range(1, span_steps + 1):
                i = (i0 + sign * step) % 360
                if i in hits:
                    continue
                a = math.radians(i)
                x0 = me_x + math.cos(a) * MUZZLE_OFFSET_PX
                y0 = me_y + math.sin(a) * MUZZLE_OFFSET_PX
                sol = simulate_ray(wall, x0, y0, a, target,
                                   me_center=me_center, tank=tank_pose)
                if sol is None or sol.hit_self:
                    break
                hits[i] = sol

    # 按置信度 C 选最优（不再只比反射少/路程短——端点反射 margin≈0 被压低）
    best: Optional[ShotSolution] = None
    best_conf = -1.0
    for i, sol in hits.items():
        margin = _margin_deg(hits, i, sol.bounces)
        turn = abs(wrap(math.radians(i) - me_angle))
        conf = shot_confidence(sol, margin, turn, pred_sigma)
        if conf > best_conf:
            best_conf = conf
            best = sol
            best.conf = conf
            best.margin_deg = margin

    # 细调（保留）：在最优角附近 ±2.5° 微调对准目标中心，置信度沿用粗扫估计
    refined = best
    steps = int(round(FINE_SPAN_RAD / FINE_STEP_RAD))
    for k in range(-steps, steps + 1):
        if k == 0:
            continue
        a = best.angle + k * FINE_STEP_RAD
        x0 = me_x + math.cos(a) * MUZZLE_OFFSET_PX
        y0 = me_y + math.sin(a) * MUZZLE_OFFSET_PX
        sol = simulate_ray(wall, x0, y0, a, target, me_center=me_center,
                           tank=tank_pose)
        if sol is None or sol.hit_self:
            continue
        if (sol.bounces, sol.miss) < (refined.bounces, refined.miss):
            refined = sol
    refined.conf = best.conf
    refined.margin_deg = best.margin_deg
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
        self.aim_conf = 0.0                # 当前锁定弹道的置信度 C（开火门槛用）
        self.lock_since = None             # 本次接管开始时刻（超时交还用）
        self._lock_cool_until = -1e9       # 交还冷却截止时刻（期间不重新接管）
        self.last_firelog = {}             # 最近一帧开火判定摘要（供日志落盘）
        self._align_frames = 0             # 车头连续停在转向死区内的帧数（真静止判定）
        self._last_target = None           # 最近一次 can_hit 的瞄准点（预测 foe 位置，排查用）
        self._foe_trail = []               # 最近 foe 观测 [(t, x, y)]（停走检测用）
        self._still_frames = 0             # 连续"观测速度 < 停阈值"的帧数
        self._foe_still = False            # foe 已确认静止（预测直接用观测位置）
        self._turn_pwm = 0.0               # 转向 PWM 累加器（接近瞄准角时减速）
        self._last_fire_direct = False     # 本次开火脉冲是否为直射（脉冲容差用）
        # 敌人轨迹恒速卡尔曼：桥上报的 vx/vy 是位置差分、噪声大（偶发几百 px/s），
        # 直接外推会让预测点乱跳、瞄准点不稳定。用卡尔曼平滑位置/速度，
        # 再 forecast(lead_t) 预测子弹命中时刻敌人的位置，车头直接瞄准预测点。
        self.kalman = ConstantVelocityKalman2D()
        self._kf_ready = False

    # ------------------------------------------------------------------
    def attack(self, t: float, me: Dict, foe: Dict, bullets: Sequence[Dict],
               raw_wall: Optional[np.ndarray]) -> Dict:
        """一帧决策：有可行发射角 → 接管并「僵直」原地转向；当前朝向命中 → 开火。

        接管时返回 lock=1：桥据此锁死玩家 ⬆⬇⬅➡ 全部输入（僵直），
        由 AI 独占转向 + 开火；无可行角或敌人消失时 lock=0 交还。
        开火用「当前车头角度的实时弹道验证」，不依赖历史求解角度的误差。
        """
        out = {'keys': {'up': 0, 'down': 0, 'left': 0, 'right': 0}, 'fire': 0, 'lock': 0}
        # ---- 开火判定摘要（每帧，供 navigation_bot 写 firelog.csv 排查）----
        fd = {'t': float(t), 'aim_ok': 0, 'why': ''}
        if not me or not foe or raw_wall is None:
            self.aim_angle = None
            self.turn_dir = 0
            fd['why'] = 'no_tank'
            self.last_firelog = fd
            return out

        me_angle = float(me.get('angle', 0.0))
        me_x, me_y = float(me['x']), float(me['y'])
        self.shell_speed = max(160.0, estimate_shell_speed(bullets))

        # 每帧把敌人位置喂进恒速卡尔曼。桥上报的 vx/vy 是位置差分、噪声极大
        # （实测 ±100~600px/s），直接注入会让速度状态被噪声污染 → 预测点乱飘。
        # 只用位置观测（kalman 内部从位置序列隐式估计速度）：位置噪声仅 ~2.5px，
        # 速度估计干净；foe 停下时 v 快速收敛，配合上面的停走检测精确锁静止。
        self.kalman.update(float(t), float(foe['x']), float(foe['y']))
        self._kf_ready = True
        pred_sigma = (self.kalman.position_uncertainty() if USE_KALMAN_PREDICTOR
                      else PRED_SIGMA_OFF_PX)   # 敌人位置不确定度（供 C_pred）

        # 停走检测：foe 走停切换频繁（玩家/原版 AI），恒速卡尔曼在 foe 停下后
        # 平滑速度残留（预测点带着旧前置飘 → 静止必中火落空）。
        # 观测差分速度连续低于阈值 → 判定静止 → 预测直接用观测位置（不加提前量）。
        # 注意确认帧数要短（FOE_STILL_FRAMES=2 ≈ 0.06s）：foe 停下通常意味着它要
        # 开火对射，确认拖 0.15s + 瞄准点跳变重转 = 我方第一发慢 0.3~0.5s → 被反杀。
        self._foe_trail.append((float(t), float(foe['x']), float(foe['y'])))
        if len(self._foe_trail) > 40:
            self._foe_trail.pop(0)
        if len(self._foe_trail) >= 2:
            (t1, x1, y1), (t2, x2, y2) = self._foe_trail[-2], self._foe_trail[-1]
            dt = t2 - t1
            if 0.005 < dt < 0.2:
                spd = math.hypot(x2 - x1, y2 - y1) / dt
                if spd < FOE_STILL_SPD_PX_S:
                    self._still_frames += 1
                else:
                    self._still_frames = 0
        was_still = self._foe_still
        self._foe_still = self._still_frames >= FOE_STILL_FRAMES
        if self._foe_still and not was_still:
            # 刚确认静止：把卡尔曼状态硬置到观测位置、速度清零 —— 消除旧速度
            # 残留，且 foe 若随后再动，kalman 从干净状态起步（不会带着停止前
            # 的速度噪声外推）。
            st = self.kalman.state
            st[0], st[1] = float(foe['x']), float(foe['y'])
            st[2], st[3] = 0.0, 0.0

        def predict_foe(lead_t: float) -> Point:
            """子弹飞行 lead_t 秒后敌人的预测位置。

            foe 已确认静止（_foe_still）→ 直接用当前观测位置（静止就是静止，
            提前量只会让瞄准点飘走）。移动中 → USE_KALMAN_PREDICTOR=True 用
            卡尔曼恒速外推；False 用桥上报 vx/vy（已钳制）直接外推。
            预测点必须在地图内且为自由空间，否则退回：卡尔曼滤波后的当前位置
            → 敌人原始观测位置。避免瞄准点预测进墙/出界导致弹道模拟穿墙误判。
            """
            if self._foe_still:
                return (float(foe['x']), float(foe['y']))
            if USE_KALMAN_PREDICTOR:
                if self._kf_ready:
                    px, py = self.kalman.forecast(lead_t)
                else:
                    px, py = float(foe['x']), float(foe['y'])
            else:
                # 卡尔曼关闭：位置差分外推（桥 vx/vy 噪声大，已钳制速度上限）
                vx, vy = _clamped_velocity(foe)
                px = float(foe['x']) + vx * lead_t
                py = float(foe['y']) + vy * lead_t
            h, w = raw_wall.shape
            ok = (0 <= int(round(px)) < w and 0 <= int(round(py)) < h
                  and not raw_wall[int(round(py)), int(round(px))])
            if not ok and USE_KALMAN_PREDICTOR and self._kf_ready:
                s = self.kalman.state
                px2, py2 = float(s[0]), float(s[1])
                if (0 <= int(round(px2)) < w and 0 <= int(round(py2)) < h
                        and not raw_wall[int(round(py2)), int(round(px2))]):
                    return (px2, py2)
            if not ok:
                return (float(foe['x']), float(foe['y']))
            return (px, py)

        # ---- 当前朝向（给定角度）能否命中：实时验证 ----
        def can_hit(angle: float):
            x0 = me_x + math.cos(angle) * MUZZLE_OFFSET_PX
            y0 = me_y + math.sin(angle) * MUZZLE_OFFSET_PX
            d0 = math.hypot(float(foe['x']) - me_x, float(foe['y']) - me_y)
            # 近战特判（出膛盲区）：foe 距 me ≤45px 且炮口大致朝向它、出膛路径
            # 无障碍 → 子弹出生点（炮口前 18px）就在 foe 车体内/紧贴，游戏里
            # 出生即命中（MUZZLE_SAFE 盲区把这段排除 → 常规判定永远 no_hit，
            # 贴脸 foe 冲脸时打不出的根源）。直接视为命中（miss=0）。
            if d0 <= NEAR_KILL_PX:
                ang_foe = math.atan2(float(foe['y']) - me_y,
                                     float(foe['x']) - me_x)
                if abs(wrap(angle - ang_foe)) <= math.radians(8.0):
                    probe = simulate_ray(raw_wall, x0, y0, angle,
                                         (float(foe['x']), float(foe['y'])),
                                         target_radius=NEAR_KILL_PX,
                                         min_dist_px=0.0,
                                         me_center=(me_x, me_y),
                                         tank=(me_x, me_y, me_angle))
                    if (probe is not None and probe.bounces == 0
                            and not probe.hit_self):
                        self._last_target = (float(foe['x']), float(foe['y']))
                        return ShotSolution(angle, 0, max(1.0, d0), 0.0, [])
            lead_t = min(MAX_LEAD_S, d0 / self.shell_speed)
            target = predict_foe(lead_t)
            self._last_target = target          # 记录火控瞄准点（排查预测误差用）
            sol = simulate_ray(raw_wall, x0, y0, angle, target,
                               me_center=(me_x, me_y), tank=(me_x, me_y, me_angle))
            if sol is not None and not sol.hit_self:
                lead_t = min(MAX_LEAD_S, sol.path_len / self.shell_speed)
                target = predict_foe(lead_t)
                self._last_target = target
                sol2 = simulate_ray(raw_wall, x0, y0, angle, target,
                                    me_center=(me_x, me_y), tank=(me_x, me_y, me_angle))
                if sol2 is not None and not sol2.hit_self:
                    return sol2
            return None

        # ---- 触发方式 A（零延迟）：当前车头朝向本身就是可命中弹道 → 立即接管瞄准。
        # 修复"射击不及时"的根因：旧逻辑只靠节流重扫（RESOLVE_EVERY_S=0.2s）发现
        # 机会，导航中车头扫过命中窗口的瞬间（300px 处窗口仅 ~7°，240°/s 转向
        # 3 帧即过）永远等不到重扫 → 该开的枪全错过。每帧 1~2 条射线的 can_hit
        # 检查很便宜；角裕度走查只在此罕见事件发生时执行一次（≤20 条射线）。
        if self.aim_angle is None and t >= self._lock_cool_until:
            probe = can_hit(me_angle)
            if probe is not None:
                margin = _heading_margin_deg(
                    raw_wall, me_x, me_y, me_angle,
                    self._last_target if self._last_target is not None
                    else (float(foe['x']), float(foe['y'])),
                    (me_x, me_y), (me_x, me_y, me_angle))
                conf = shot_confidence(probe, margin, 0.0, pred_sigma)
                if conf >= CONF_FIRE_MIN:
                    probe.conf = conf
                    probe.margin_deg = margin
                    self.aim_angle = me_angle
                    self.last_sol = probe
                    self.aim_conf = conf

        # ---- 节流重扫（触发方式 B）：是否存在能击中敌人的发射角（作为转向目标）。
        # 0.2s 节流是性能下限：solve 粗扫-细化在 5~15ms 级，每帧跑会拖住 30Hz 循环
        # （旧版 360°×1° 全扫实测 32ms/帧，曾拖死导航循环；现在是 15°/5° 粗扫 +
        # 细化，射线量约 1/4）。aim 已锁定时与触发方式 A 同位竞争，由 CONF_SWITCH_
        # MARGIN 滞回决定是否切换目标角。
        if t - self.last_solve_t >= RESOLVE_EVERY_S:
            self.last_solve_t = t
            d0 = math.hypot(float(foe['x']) - me_x, float(foe['y']) - me_y)
            # 近战特判优先：foe 已贴脸（≤45px）→ 不用全扫/窗口扫，直接朝它
            # 给高 conf 解（solve 常规路径因 MUZZLE_SAFE 盲区找不到贴脸解）。
            sol = None
            if d0 <= NEAR_KILL_PX:
                ang_foe = math.atan2(float(foe['y']) - me_y,
                                     float(foe['x']) - me_x)
                x0n = me_x + math.cos(ang_foe) * MUZZLE_OFFSET_PX
                y0n = me_y + math.sin(ang_foe) * MUZZLE_OFFSET_PX
                probe = simulate_ray(raw_wall, x0n, y0n, ang_foe,
                                     (float(foe['x']), float(foe['y'])),
                                     target_radius=NEAR_KILL_PX,
                                     min_dist_px=0.0,
                                     me_center=(me_x, me_y),
                                     tank=(me_x, me_y, me_angle))
                if (probe is not None and probe.bounces == 0
                        and not probe.hit_self):
                    sol = ShotSolution(ang_foe, 0, max(1.0, d0), 0.0, [],
                                       conf=0.95, margin_deg=12.0)
            if sol is None:
                lead_t = min(MAX_LEAD_S, d0 / self.shell_speed)
                target = predict_foe(lead_t)
                sol = solve_shot(raw_wall, me, target,
                                 center_angle=self.aim_angle,
                                 pred_sigma=pred_sigma)
            # 近战特解（miss 0、conf 0.95）已最优且 foe 贴脸无 lead 意义：
            # 跳过第二段 lead 迭代（否则会被"绕远反射回环解"覆盖——贴脸 foe
            # 竟被解成 81° 绕场一周的反射，荒谬且慢）。
            near_mode = (sol is not None and d0 <= NEAR_KILL_PX
                         and sol.conf >= 0.9 and sol.miss <= 0.5)
            if sol is not None and not near_mode:
                lead_t = min(MAX_LEAD_S, sol.path_len / self.shell_speed)
                target = predict_foe(lead_t)
                sol2 = solve_shot(raw_wall, me, target, center_angle=sol.angle,
                                  pred_sigma=pred_sigma)
                if sol2 is not None:
                    sol = sol2

            # aim 冻结（诊断 fire_v3_diag：锁定期 aim 在解族间跳 ±53° → 车头
            # 反向摇摆、永不收敛 → 一枪不发；no_hit 占锁定帧 77%）。
            # 策略：旧瞄准角仍有效（can_hit 过）就绝不切换 —— 只在旧角失效时
            # 才接受重扫的新解。新解 conf 更高也不换（穿越开火会沿旋转路径
            # 顺手截获更好的窗口，不需要跳 aim）。
            if (sol is not None and self.aim_angle is not None
                    and can_hit(self.aim_angle) is not None):
                sol = None  # 冻结瞄准角：旧角仍有效，不切换

            # 低置信度解不锁为 aim（conf < 门槛的火永远打不出去）：锁它 =
            # 干等（conf_low 站桩）+ 重扫换解时 aim 从 ±1° 猛跳几十度 → 坦克
            # "往反方向旋转不射击"（实测 conf 0.36 的 aim 锁 0.5s 后 aim 跳
            # -48°）。宁可 no_aim 交还走位，等 foe 换位出现高 conf 解。
            # 直射解用放宽门槛（贴墙敌火力靠它），反射解保持原门槛。
            if sol is not None:
                sol_min = CONF_LOCK_MIN_DIRECT if sol.bounces == 0 else CONF_FIRE_MIN
                if sol.conf < sol_min:
                    sol = None

            if sol is not None:
                self.aim_angle = sol.angle
                self.last_sol = sol
                self.aim_conf = sol.conf
            else:
                # 无新解：旧目标角仍可命中则保留，否则清除（交还）
                if self.aim_angle is not None and can_hit(self.aim_angle) is None:
                    self.aim_angle = None
                    self.last_sol = None
                    self.aim_conf = 0.0

        # 交还冷却期：刚因"打不出"交还过 → 短期内不重新接管（给导航脱困时间，
        # 否则交还一帧又锁上，等于没交还）
        if t < self._lock_cool_until and self.aim_angle is not None:
            self.aim_angle = None
            self.last_sol = None
            self.aim_conf = 0.0
            self.lock_since = None

        if self.aim_angle is None:
            # 无可击中角度（或敌人已不可达）→ 取消接管，交还（lock=0）
            self.turn_dir = 0
            self.lock_since = None
            self._align_frames = 0
            fd.update(aim_ok=0, lock=0, fire=0,
                      why='cooling' if t < self._lock_cool_until else 'no_aim')
            self.last_firelog = fd
            return out

        # ---- 接管超时交还：lock 持续超过 LOCK_RELEASE_TIMEOUT_S 仍打不出去
        # （转不到位 / can_hit 不过 / 偏差验证不过）→ 强制清瞄准角交还控制权，
        # 防"锁死键盘 + 站桩挨打"（瞄不准就永远锁着）。
        if self.lock_since is None:
            self.lock_since = t
        elif (t - self.lock_since > LOCK_RELEASE_TIMEOUT_S
                and t >= self.fire_until):
            self.aim_angle = None
            self.last_sol = None
            self.aim_conf = 0.0
            self.lock_since = None
            self.turn_dir = 0
            self._align_frames = 0
            self._lock_cool_until = t + LOCK_RELEASE_COOL_S
            fd.update(aim_ok=1, lock=0, fire=0, why='lock_timeout')
            self.last_firelog = fd
            return out

        # ---- 接管（僵直）：朝 aim_angle 原地转向，锁死玩家输入 ----
        out['lock'] = 1
        err = wrap(self.aim_angle - me_angle)
        # 真实发射角验证：当前车头朝向 me_angle 的弹道（前移到这里，供「穿越开火」
        # 与下方发射门共用一次计算，避免重复 can_hit 的两条射线）。
        fire_sol = can_hit(me_angle)
        # ---- 穿越开火（fire-on-crossing）----
        # 诊断（fire_v3_diag 无头战斗）：锁定期间 77% 帧 why=no_hit（车头还在往
        # aim 转：err 中位 -8.3°、最大 103°），且 aim 会在解族间跳 ±53° → 车头
        # 反向摇摆、永不收敛 → 3 局只打出 1 发。穿越开火：转向途中只要当前朝向
        # 的弹道经过验证能命中（含自伤校验），立即松开转向键把发射角确定下来并
        # 进入发射判定 —— 不再要求「先转到 aim 再射」，窗口机会不被旋转时间吃掉，
        # 旋转路径穿过的更好窗口也会被顺手截获。
        crossing = (fire_sol is not None
                    and fire_sol.self_clear >= SELF_SAFE_PX
                    and not (fire_sol.bounces > 0 and fire_sol.leg1 < MIN_LEG_PX)
                    and fire_sol.miss <= (FIRE_MISS_MAX_DIRECT_PX
                                          if fire_sol.bounces == 0
                                          else FIRE_MISS_MAX_PX)
                    and t >= self.next_fire_t)
        if crossing:
            self.turn_dir = 0    # 已验证命中 → 停转，发射角确定（验证角 = 实际发射角）
            err = 0.0            # 发射门按"已对齐"处理（验证的就是实际发射弹道）
        # 停转窗口分叉：直射解（last_sol.bounces==0）用大窗口（易停稳），
        # 反射解用 2.5°（≥ 转向单步/2 → 一步可落窗，无极限环；原 0.8° ≪ 单步
        # 4.4°/2 导致反射解永远停不进去 → 极限环干锁被反杀）。车停在窗口内 =
        # 角度确定 → can_hit 射即中；发射精度再由 err_tol（命中余量反推）把关。
        aim_direct = (self.last_sol is not None and self.last_sol.bounces == 0)
        stop_rad = DIRECT_STOP_RAD if aim_direct else AIM_DEADBAND_RAD
        # 静止帧计数：判定"车头真的没在转"（me_angle 与上帧差 <0.2°），而不是
        # err 在不在窗内 —— aim 每 0.2s 重扫会在解族间跳（foe 移动使旧解失效，
        # b0↔b2 交替），err 跟着跳但车根本没动；用 err 判静止会把窗口打碎成
        # 1~2 帧、永远凑不齐 3 帧（"长距犹豫+轻摆"：车停着，只是瞄准目标在跳）。
        # 车没动 = 发射角确定（fire 另有 err ≤ stop_fire 管"方向对不对"，aim 跳
        # 回窗内的一帧立即射；err 长期出窗由 lock_timeout 交还兜底）。
        me_angle_delta = abs(wrap(me_angle - getattr(self, '_last_me_angle', me_angle)))
        self._last_me_angle = me_angle
        if me_angle_delta < math.radians(0.2):
            self._align_frames += 1          # 车头真的静止：累计帧数（与 err/aim 无关）
        else:
            self._align_frames = 0           # 车头在转（含惯性滑转）→ 清零
        if abs(err) <= stop_rad:
            self.turn_dir = 0
        else:
            if self.turn_dir == 0 or abs(err) < U_TURN_LOCK_RAD:
                self.turn_dir = 1 if err > 0 else -1
        fd.update(aim_ok=1, aim_deg=round(math.degrees(self.aim_angle), 2),
                  conf=round(self.aim_conf, 3),
                  err_deg=round(math.degrees(err), 2), turning=self.turn_dir)
        # 转向方向（以本仓库游戏引擎源码 c2runtime.js:35542/35546 + track.csv 实测为准）：
        #   left 键  → this.a -= steerSpeed*dt → 角度减小
        #   right 键 → this.a += steerSpeed*dt → 角度增大
        # err = wrap(aim_angle - me_angle)；err>0 = 目标在车头"角度增大"方向。
        # 故 err>0 → 按 right（增大）；err<0 → 按 left（减小）。
        # （旧注释"left 键让角度增大"是 v3 时代记录反了，导致朝反方向转、永不瞄准）
        # 满舵直转（不再 PWM 减速）：停转窗 2.5° ≥ 转向单步 4.4°/2，满舵一步必落
        # 窗内、无极限环 —— PWM 分时减速只把最后 6° 的瞄准拖慢 ~0.2s（射击慢被
        # 反杀的元凶之一），且不减单步角、对落窗无帮助，移除。
        steer = 0
        if self.turn_dir > 0:
            steer = 1
        elif self.turn_dir < 0:
            steer = -1
        if steer > 0:
            out['keys']['right'] = 1
        elif steer < 0:
            out['keys']['left'] = 1

        # ---- 开火：只用"当前车头实际朝向 me_angle"验证弹道确实能命中敌人
        # 预测位置才射（can_hit(me_angle)）。
        # 修复：原用 can_hit(aim_angle)（理想角）验证，但子弹实际从 me_angle
        # 发射 —— 车头与理想角差 err 时（哪怕 3°），反射弹道撞墙后方向偏差被
        # 放大，打不中敌人 → 子弹乱弹飞回来自伤。改成验证 me_angle 后，
        # 只有"现在这一枪真的会中"才开火；车头未转到命中角则不开火、继续转。
        # 直瞄容差大正常开火；反射需转到精确命中窗口内才射，射即中。
        # （fire_sol 已在上方接管段落计算，此处直接复用。）
        # 发射门限 = 停转窗（按"实际发射弹道" fire_sol.bounces 分窗：直射 3.5° /
        # 反射 2.5°，防 aim 是直瞄但实际弹道已变反射时按大窗放行打偏）。
        # 注意不要再用"命中余量反推 err 上限"（_fire_err_tol 已废弃）：can_hit 是
        # 用当前 me_angle（含 err）模拟的，err 的几何代价已计入 fire_sol.miss ——
        # 再按 miss 收紧 err 是双重惩罚：远端擦边火（miss 14~18）余量小 → err 要求
        # 缩到 ~1° → 车停稳在 1.9° 永远等不到 → 站桩 0.6s+ 被反杀（"长距不果断"）。
        # 静止 + 停在窗内 = 发射角确定，射不射由 can_hit 的几何命中（miss ≤ 18.45）
        # 与 conf 把关；擦边火打不中属于预测/模型误差，另用门槛解决，不再卡 err。
        fire_direct = (fire_sol is not None and fire_sol.bounces == 0)
        stop_fire = DIRECT_STOP_RAD if fire_direct else AIM_DEADBAND_RAD
        # 静止帧数：近战直瞄（弹道短）1 帧即可 —— 贴脸绕圈的 foe 让 err 高频
        # 进出停转窗，3 帧连续静止凑不齐 → 永远打不出（近战被反杀）。foe 明确
        # 移动中（≥3 帧观测且最近差分 > 阈值）也 1 帧即射：aim 随 foe 移动每
        # 0.2s 重扫跳变，3 帧静止凑不齐（实测 foe_moving 拦帧几十帧 = "迟钝
        # 迟疑"）；车真静止 1 帧（me_angle 差分确认）发射角就确定。foe 静止/
        # 未知（观测 <3 帧）的远程/反射仍 3 帧求稳。
        frames_needed = (1 if ((self._still_frames == 0
                                and len(self._foe_trail) >= 3)
                               or (fire_direct and fire_sol is not None
                                   and fire_sol.path_len <= NEAR_FIRE_PATH_PX))
                         else FIRE_ALIGN_FRAMES)
        # 穿越开火不加帧数捷径：验证用的是"转向中还带着转速"的角度，扣扳机时
        # 角度会再漂移 1~2 帧（≤16°）→ 验证作废。松键后由 _align_frames 自然
        # 累计（松键即停转：下一帧起角度冻结），fire 窗口 1~3 帧（0.03~0.1s）
        # 内用冻结后的角度重新验证再射 —— 安全且仍远快于"转到 aim 再停"。
        # 所有弹道统一要求：真静止（停在窗口内连续 frames_needed 帧）才开火。
        # 直射准度来自"发射时角度确定"（静止时 can_hit 验证角 = 实际发射角，射即中）；
        # 停转窗口已按 aim 类型分叉（直射 3.5° 易停 / 反射 2.5° 无极限环）→ 直射果断且准。
        leg_short = (fire_sol is not None and fire_sol.bounces > 0
                     and fire_sol.leg1 < MIN_LEG_PX)
        # 直射解门槛放宽（贴墙敌直射多为擦边/单侧裕度）：conf 与 miss 按弹道
        # 类型分档（bot_config.py 可调）；反射解保持原门槛（自伤风险高）。
        conf_min_now = (CONF_FIRE_MIN_DIRECT
                        if (fire_sol is not None and fire_sol.bounces == 0)
                        else CONF_FIRE_MIN)
        miss_max_now = (FIRE_MISS_MAX_DIRECT_PX
                        if (fire_sol is not None and fire_sol.bounces == 0)
                        else FIRE_MISS_MAX_PX)
        fire_ok = (t >= self.next_fire_t
                   and fire_sol is not None
                   and fire_sol.self_clear >= SELF_SAFE_PX
                   and not leg_short
                   and fire_sol.miss <= miss_max_now
                   and self.aim_conf >= conf_min_now
                   and abs(err) <= stop_fire
                   and self._align_frames >= frames_needed)
        dev_ok = True
        if fire_ok and fire_sol is not None and fire_sol.bounces > 0:
            # 偏差安全验证（反射解）：实际发射角可能偏 ±FIRE_SAFE_DEV_RAD
            # （1~2 帧滞后），偏差后的弹道若会穿过自己车体（贴墙反弹自杀的根源），
            # 这一枪不能打 —— 中心弹道能中没用，偏一点就自杀了。
            # 直射解（bounces==0）跳过：中心弹道已验 self_clear/hit_self，且直射
            # 弹道出膛即远离车身、偏差 ±3° 只会掠射撞墙不会折返回来自伤；而反射解
            # 弹道可能紧贴车身撞墙反弹（贴墙反弹自杀就在出膛 1~3px 内发生），
            # 仍需最坏情况推演。此前直射也做 ±3° 推演，把"贴墙窄缝直瞄"（偏 3°
            # 必撞缝边反弹打自己）全部误杀 → 静止 0.4s+ 站桩挨打（fire=0 死局）。
            foe_now = (float(foe['x']), float(foe['y']))
            for dev in (FIRE_SAFE_DEV_RAD, -FIRE_SAFE_DEV_RAD):
                da = me_angle + dev
                x0d = me_x + math.cos(da) * MUZZLE_OFFSET_PX
                y0d = me_y + math.sin(da) * MUZZLE_OFFSET_PX
                sd = simulate_ray(raw_wall, x0d, y0d, da, foe_now,
                                  tank=(me_x, me_y, me_angle))
                if sd is not None and sd.hit_self:
                    fire_ok = False
                    dev_ok = False
                    break
        if fire_ok:
            self.fire_until = t + FIRE_PULSE_S
            self.next_fire_t = t + FIRE_COOLDOWN_S
            self.lock_since = t          # 开火成功 → 接管有效，超时重新计时
            self._lock_cool_until = -1e9
            print('FIRE conf=%.2f bounce=%d margin=%.1fdeg len=%.0f sigma=%.0f' % (
                self.aim_conf, fire_sol.bounces,
                getattr(self.last_sol, 'margin_deg', 0.0),
                fire_sol.path_len, pred_sigma), flush=True)
        # fire 输出：脉冲内且车头仍停在窗口内才发键 —— 脉冲期间车转出窗口立即
        # 中断本脉冲（fire_until 作废），宁可不发也不让子弹在车头转动中发射
        # （发射角未定 = 验证弹道作废 = 自杀/打偏根源）。
        if t < self.fire_until:
            if abs(err) <= stop_fire:
                out['fire'] = 1
            else:
                self.fire_until = -1e9   # 中断脉冲（本发作废，下次需重新过全部条件）
        # ---- 开火判定摘要（why：本帧是否开火 / 未开火的原因，供排查）----
        if t < self.next_fire_t:
            why = 'cooldown'
        elif fire_sol is None:
            why = 'no_hit'
        elif fire_sol.self_clear < SELF_SAFE_PX:
            why = 'self_risk'
        elif leg_short:
            why = 'leg_short'
        elif self.aim_conf < conf_min_now:
            why = 'conf_low'
        elif abs(err) > stop_fire:
            why = 'not_aligned'
        elif self._align_frames < frames_needed:
            why = 'not_still'
        elif fire_sol.miss > miss_max_now:
            why = 'edge_miss'
        elif not dev_ok:
            why = 'dev_unsafe'
        else:
            why = 'fire'
        fd.update(
            hit=1 if fire_sol is not None else 0,
            bounces=fire_sol.bounces if fire_sol is not None else -1,
            miss=round(fire_sol.miss, 1) if fire_sol is not None else -1,
            self_clr=round(fire_sol.self_clear, 1) if fire_sol is not None else -1,
            pass_c=1 if self.aim_conf >= conf_min_now else 0,
            pass_a=1 if abs(err) <= stop_fire else 0,
            pass_s=1 if (fire_sol is not None
                         and fire_sol.self_clear >= SELF_SAFE_PX) else 0,
            pass_d=1 if dev_ok else 0,
            lock=out['lock'], fire=out['fire'], why=why)
        if self._last_target is not None:
            fd.update(pred_x=round(self._last_target[0], 1),
                      pred_y=round(self._last_target[1], 1))
        fd.update(foe_x=round(float(foe['x']), 1),
                  foe_y=round(float(foe['y']), 1))   # 敌人观测位置：与 pred 同行对比
        self.last_firelog = fd
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

    # 7) 直射（近战，弹道路程 ~92px ≤ NEAR_FIRE_PATH_PX）：foe 静止确认
    #    （2 帧 ≈0.06s）后 1 帧即射 —— 近战子弹 <0.3s 就到、角度不确定影响小；
    #    贴脸 foe 绕圈时 3 帧连续静止凑不齐。远战 3 帧语义由用例 13 覆盖。
    ctrl = CombatController()
    ctrl.reset()
    me_r = dict(me_d, angle=0.0)
    a1 = ctrl.attack(0.0, me_r, foe_d, [], raw)   # foe 刚出现（观测 1 帧）→ 等确认
    a2 = ctrl.attack(0.02, me_r, foe_d, [], raw)
    a3 = ctrl.attack(0.04, me_r, foe_d, [], raw)
    ok &= check('direct near: fires immediately (near 1-frame gate)',
                a1['fire'] == 1 and a3['fire'] == 1
                and a3['keys']['up'] == 0 and a3['keys']['down'] == 0
                and a3['keys']['left'] == 0 and a3['keys']['right'] == 0,
                'a1=%s a3=%s' % (a1, a3))
    # 近战 err 2°（≤3.5° 停转窗）→ 同样立即射（近距离容差大）
    ctrl0c = CombatController()
    ctrl0c.reset()
    me_2d = dict(me_d, angle=math.radians(2.0))
    c1 = ctrl0c.attack(0.0, me_2d, foe_d, [], raw)
    ok &= check('direct near err 2deg: fires immediately', c1['fire'] == 1,
                'act=%s' % c1)
    # 直射 err 5°（>3.5° 停转窗）：当前车头 5° 弹道 miss≈8px（≤内切圆 10px →
    # 真命中）→ 触发方式 A 零延迟开火。（旧语义"必须转回 0° 才射"= 犹豫；
    # 只要 can_hit(当前朝向) 验证通过就射，不追求完美对准——F1 修复目标。）
    ctrl0d = CombatController()
    ctrl0d.reset()
    a0_2 = ctrl0d.attack(0.0, dict(me_d, angle=math.radians(5.0)), foe_d, [], raw)
    ok &= check('direct err 5deg: current heading hits -> zero-delay fire (Trigger A)',
                a0_2['fire'] == 1,
                'act=%s' % a0_2)

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

    # 9) 反射场景：敌人被墙挡需反射 → 必须连续静止 FIRE_ALIGN_FRAMES 帧才开火
    #    （反射窗口窄：发射角不可预测 = 验证弹道作废）
    wall_r2 = make_border()
    wall_r2[40:160, 80] = True
    foe_r = {'x': 115.0, 'y': 95.0, 'angle': 0.0, 'vx': 0.0, 'vy': 0.0}
    ctrlR = CombatController()
    ctrlR.reset()
    # 第 1 帧：solve 找到反射角；模拟车头已转到该角（用 solve 结果验证静止门槛）
    sol_r = solve_shot(wall_r2, me_d, (foe_r['x'], foe_r['y']))
    ok &= check('reflect setup finds angle', sol_r is not None)
    if sol_r is not None:
        me_rang = dict(me_d, angle=sol_r.angle)
        a1 = ctrlR.attack(0.0, me_rang, foe_r, [], wall_r2)
        a2 = ctrlR.attack(0.02, me_rang, foe_r, [], wall_r2)
        ok &= check('reflect: no fire before 3 still frames',
                    a1['fire'] == 0 and a2['fire'] == 0, 'a2=%s' % a2)
        a3 = ctrlR.attack(0.04, me_rang, foe_r, [], wall_r2)
        ok &= check('reflect: fire after 3 still frames', a3['fire'] == 1,
                    'act=%s' % a3)
        # 反射但仅 1 帧静止（中间转出）→ 不 fire
        ctrlR2 = CombatController()
        ctrlR2.reset()
        b1 = ctrlR2.attack(0.0, me_rang, foe_r, [], wall_r2)
        b2 = ctrlR2.attack(0.02, dict(me_d, angle=sol_r.angle + math.radians(5.0)),
                           foe_r, [], wall_r2)
        ok &= check('reflect: still count resets when turning', b1['fire'] == 0 and b2['fire'] == 0,
                    'b1=%s b2=%s' % (b1, b2))

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

    # 12) 反射解残余 err 1.5°（≤ 2.5° 放宽后的停转窗）→ 静止 3 帧后照常开火。
    #     回归点：原反射死区 0.8° ≪ 转向单步 4.4°/2 → 车头永远停不进 ±0.8°，
    #     极限环干锁（"直瞄犹豫被反杀"）。窗放宽到 2.5° 后 1.5° 即视为停稳。
    if sol_r is not None:
        ctrlR3 = CombatController()
        ctrlR3.reset()
        me_err15 = dict(me_d, angle=sol_r.angle + math.radians(1.5))
        r1 = ctrlR3.attack(0.0, me_err15, foe_r, [], wall_r2)
        r2 = ctrlR3.attack(0.02, me_err15, foe_r, [], wall_r2)
        r3 = ctrlR3.attack(0.04, me_err15, foe_r, [], wall_r2)
        ok &= check('reflect err 1.5deg still fires (wide stop window)',
                    r3['fire'] == 1,
                    'act=%s' % r3)

    # 13) 发射门限 = 停转窗（err 的几何代价已由 can_hit(me_angle) 计入 miss，不再
    #     按 miss 余量收紧 err —— 那是双重惩罚，会把远端擦边火卡成站桩等 err→0）：
    #     远程直瞄（~340px，0.5× 弹速 × 0.2× 存留 → 实际射程约 360px，故只能测
    #     射程内的例子）err 1.5°（≤3.5° 窗内）停 3 帧后照常果断开火；
    #     err 5°（窗外）不射。err 0.3° 同样开火（用例 7/13 语义）。
    wall_big = np.zeros((600, 600), dtype=bool)
    wall_big[0, :] = True
    wall_big[599, :] = True
    wall_big[:, 0] = True
    wall_big[:, 599] = True
    me_far = dict(me_d, x=60.0)
    foe_far = {'x': 400.0, 'y': 100.0, 'angle': 0.0, 'vx': 0.0, 'vy': 0.0}
    ctrlF1 = CombatController()
    ctrlF1.reset()
    mf15 = dict(me_far, angle=math.radians(1.5))
    f1 = ctrlF1.attack(0.0, mf15, foe_far, [], wall_big)
    f2 = ctrlF1.attack(0.02, mf15, foe_far, [], wall_big)
    f3 = ctrlF1.attack(0.04, mf15, foe_far, [], wall_big)
    ok &= check('far err 1.5deg: fires after 3 still (miss 12.05<=15 gate)',
                f1['fire'] == 0 and f2['fire'] == 0 and f3['fire'] == 1,
                'act=%s' % f3)
    ctrlF3 = CombatController()
    ctrlF3.reset()
    f5 = ctrlF3.attack(0.0, dict(me_far, angle=math.radians(5.0)), foe_far, [], wall_big)
    ok &= check('far direct err 5deg: no fire (outside stop window)',
                f5['fire'] == 0, 'act=%s' % f5)
    ctrlF2 = CombatController()
    ctrlF2.reset()
    mf03 = dict(me_far, angle=math.radians(0.3))
    g1 = ctrlF2.attack(0.0, mf03, foe_far, [], wall_big)
    g2 = ctrlF2.attack(0.02, mf03, foe_far, [], wall_big)
    g3 = ctrlF2.attack(0.04, mf03, foe_far, [], wall_big)
    ok &= check('far direct err 0.3deg: fires after 3 still frames',
                g1['fire'] == 0 and g2['fire'] == 0 and g3['fire'] == 1,
                'g1=%s g2=%s g3=%s' % (g1, g2, g3))

    # 14) 窄缝直瞄：中心弹道穿缝直射（b=0），但 ±3° 偏差弹道必撞缝边反弹打回
    #     自己车体 —— 旧代码 dev ±3° 推演把它误杀（贴墙窄缝直瞄站桩 0.4s+
    #     挨打不射的复现）；直射解跳过 dev 检查后应照常开火。
    wall_slit = make_border()
    wall_slit[0:200, 100] = True         # 竖墙 x=100 贯穿全高（y=0..199）
    wall_slit[99:102, 100] = False       # 挖 2px 高的缝（y=99..101）→ 缝是唯一通路
    me_slit = dict(me_d, x=50.0, angle=0.0)
    foe_slit = {'x': 160.0, 'y': 100.0, 'angle': 0.0, 'vx': 0.0, 'vy': 0.0}
    sol_slit = solve_shot(wall_slit, me_slit, (foe_slit['x'], foe_slit['y']))
    ok &= check('slit setup: straight shot through slit exists',
                sol_slit is not None and sol_slit.bounces == 0,
                'b=%s' % (sol_slit.bounces if sol_slit else -1))
    if sol_slit is not None and sol_slit.bounces == 0:
        ctrlSl = CombatController()
        ctrlSl.reset()
        s1 = ctrlSl.attack(0.0, me_slit, foe_slit, [], wall_slit)
        s3 = ctrlSl.attack(0.02, me_slit, foe_slit, [], wall_slit)
        ok &= check('slit direct: fires (dev check skipped for b=0)',
                    s1['fire'] == 1 and s3['fire'] == 1,
                    's1=%s s3=%s' % (s1, s3))

    # 15) 短入射段反射解（贴墙打反弹，leg1 < MIN_LEG_PX）→ 置信度置 0 不射：
    #     入射段过短时模拟网格与游戏碰撞盒差 1~2px → 反弹方向偏 2°+，反弹段
    #     不可预测（乱弹/弹回自伤温床，自杀局入射段仅 28px）。弹道本身不穿
    #     自己（hit_self=False）也必须否决。
    wall_leg = make_border()
    wall_leg[40:160, 80] = True
    a_leg = math.atan2(0.67, 0.74)          # ~42° 贴墙斜射
    x0l = 58.0 + 18.0 * math.cos(a_leg)
    y0l = 80.0 + 18.0 * math.sin(a_leg)
    sol_leg = simulate_ray(wall_leg, x0l, y0l, a_leg, (45.0, 113.0),
                           tank=(58.0, 80.0, a_leg))
    ok &= check('short-leg reflect: b1, leg1<MIN_LEG, no self-hit',
                sol_leg is not None and sol_leg.bounces == 1
                and sol_leg.leg1 < MIN_LEG_PX and not sol_leg.hit_self,
                'leg1=%.1f' % (sol_leg.leg1 if sol_leg else -1))
    if sol_leg is not None:
        ok &= check('short-leg reflect: conf forced to 0',
                    shot_confidence(sol_leg, 9.0, 0.0, 2.0) == 0.0)

    # 16) 放宽后：foe 明确移动中（≥3 帧观测、差分>阈值）也开火（1 帧即射，减少
    #     "迟钝迟疑"）；foe 停住后正常节奏射击。
    wall_big2 = np.zeros((600, 600), dtype=bool)
    wall_big2[0, :] = True
    wall_big2[599, :] = True
    wall_big2[:, 0] = True
    wall_big2[:, 599] = True
    me_sw = dict(me_d, x=40.0, y=100.0, angle=0.0)
    foe_a = {'x': 250.0, 'y': 100.0, 'angle': 0.0, 'vx': 0.0, 'vy': 0.0}
    foe_b = {'x': 253.0, 'y': 100.0, 'angle': 0.0, 'vx': 0.0, 'vy': 0.0}
    foe_c = {'x': 256.0, 'y': 100.0, 'angle': 0.0, 'vx': 0.0, 'vy': 0.0}
    ctrlSW = CombatController()
    ctrlSW.reset()
    w1 = ctrlSW.attack(0.0, me_sw, foe_a, [], wall_big2)     # 观测第 1 帧（未知）→ 3 帧
    w2 = ctrlSW.attack(0.02, me_sw, foe_b, [], wall_big2)    # 观测第 2 帧（移动中但 <3 帧）
    w3 = ctrlSW.attack(0.04, me_sw, foe_c, [], wall_big2)    # ≥3 帧且仍在移动 → 1 帧即射
    ok &= check('moving foe: fires after 3 observed moving frames',
                w1['fire'] == 0 and w2['fire'] == 0 and w3['fire'] == 1,
                'w1=%s w2=%s w3=%s' % (w1, w2, w3))

    # 17) 贴脸近战（foe 距 me 25px，落在 MUZZLE_SAFE=36px 出膛盲区）：常规
    #     solve/can_hit 永远判不中（贴脸 foe 冲脸打不出的根源）→ 近战特判
    #     （子弹出生点即 foe 体内，游戏必命中）应直接给解并开火。
    ctrlNK = CombatController()
    ctrlNK.reset()
    me_nk = dict(me_d, angle=0.0)
    foe_nk = {'x': 65.0, 'y': 100.0, 'angle': 0.0, 'vx': 0.0, 'vy': 0.0}
    n1 = ctrlNK.attack(0.0, me_nk, foe_nk, [], raw)
    ok &= check('point-blank foe: near-kill fires immediately',
                n1['fire'] == 1,
                'act=%s' % n1)

    if not ok:
        print('SELFTEST FAILED')
        raise SystemExit(1)
    print('SELFTEST PASSED')


if __name__ == '__main__':
    _selftest()
