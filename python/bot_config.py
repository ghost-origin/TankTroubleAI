# -*- coding: utf-8 -*-
"""导航 bot 全部可调参数（集中配置：调参只改本文件）。

留在 navigation_bot.py 的结构性常量（非调参项，勿动）：
    MAP_ORIGIN_X / MAP_ORIGIN_Y / MAZE_SIZE / CSV_COLUMNS / PLANS_COLUMNS
导航几何类常量在 navigation_mvp.py（如 TUBE_MIN_CLEARANCE、WINDOW_TARGET_PX、
A_STAR_TOPOLOGY_RADIUS_PX、GRID_PX 等），调那些也去对应文件。
"""
import math

# ---------------- 路点与窗口 ----------------
WAYPOINT_REACHED_PX = 13.0              # 路点到达判定距离：距路点 ≤13px 即推进
WP_FIRST_MIN_PX = 30.0                  # 初始路点最小距离：不被车身覆盖（半长15.5+余量）
WP_SPACING_PX = 40.0                    # 路点间距（用户指定）
MIN_ACCEPT_PATH_PX = 100.0              # 只接受 ≥100px 的规划路径（短窗口导致频繁目标切换/抖动）

# ---------------- OODA / 规划节奏 ----------------
OODA_PERIOD_S = 0.50                    # 常规重规划 2Hz，减少高速时目标滞后
LOCAL_WINDOW_RETRY_S = 0.05             # 进入窗口尾段后 20Hz 快速续接
# 窗口尾段判据（弧长标记点法，用户方法）：
# 在轨迹 WINDOW_REPLAN_FRACTION（默认 0.70 = 已走 70% 弧长）处取标记点 A，
# 其前 WINDOW_REPLAN_REF_BACK 弧长处取参考点 B；v1=B−A（指向轨迹后方），
# v2=车中心−A；cos(v1,v2) ≤ 0 ⟺ 车已越过 A 点 → 进入后段 → 快速续接。
WINDOW_REPLAN_FRACTION = 0.70           # 弧长标记点分数（越过此点进入窗口后段）
WINDOW_REPLAN_REF_BACK = 0.03           # 标记点后向参考弧长（v1 方向近似轨迹切线）
WINDOW_REPLAN_TAIL_FRACTION = 0.95      # 兜底标记点分数：车偏离轨迹导致 0.7 判据
                                        # 失效时，越过 0.95（距终点 5%）必然几何触发
WINDOW_REPLAN_TAIL_REF_BACK = 0.02      # 兜底标记点后向参考弧长（0.95 → 0.93）
WINDOW_REPLAN_TAIL_REMAINING_PX = 45.0  # 兜底判据：剩余路径 ≤45px 也视为尾段
                                        # （车被弹开/偏离轨迹时弧长判据不可靠）
DEFAULT_PREDICTION_HORIZON_S = 0.0      # 敌方卡尔曼预测时域（0=关）

# ---------------- 转向控制（Stanley 横向控制器，借鉴斯坦福 DARPA 挑战赛） ----------------
# 控制律：ω_des = FF·v·κ(s) + Kp·θe + Kd·dθe/dt − clamp(Kct·e_ct, ±CT_MAX)
#   κ(s) = 路径在最近点处的有符号曲率（前馈，消除弯道内切）
#   θe   = 路径切向与车头夹角；e_ct = 有符号横向偏移（px，右正）
# 注：原版 Stanley 的横向项是 atan(k·e/v)（÷v 是因为汽车转向能力随车速变化）；
#     坦克/独轮车转向角速度恒定(3.8 rad/s)，去掉 ÷v，用线性+限幅，收敛更快。
LOOKAHEAD_PX = 25.0                     # 纯追踪前瞻弧长（备用模式）
PID_KP_TH = 5.0                         # 航向误差 P 增益（用户指定）
PID_KI_TH = 0.0                         # 航向误差 I 增益（不用积分）
PID_KD_TH = 0.0                         # 航向误差 D 增益。仿真实测 Kd=1 在 PWM 转向下
                                        # 会把量化锯齿注入控制量（R=16 误差 8.4px vs 0 的 1.3px）；
                                        # D 项已按无噪声参考实现（切线旋转率−实际施加ω），
                                        # 想用建议 0.1~0.2 小增益。
STANLEY_K_CT = 0.20                     # 横向误差增益（rad/s per px）
STANLEY_CT_MAX = 2.0                    # 横向误差项上限（rad/s，防大偏移过冲）
STANLEY_FF_GAIN = 1.0                   # 曲率前馈增益（1.0 = 完全前馈，转弯零内切）
STANLEY_CORNER_MARGIN = 0.85            # 过弯速度余量：v=min_r·3.8·margin，给反馈留转向权威
TURN_ONLY_RAD = math.radians(18.0)      # 旧 PWM 转向分级（_steer_dir，备用）
TURN_DEADBAND_RAD = math.radians(10.0)  # 转向死区
TURN_FULL_RAD = math.radians(25.0)      # 满舵阈值：|err|>此值满舵；中间段 50% 占空比
STEER_FLIP_COOL_S = 0.35                # 转向切换冷却：换舵向后 0.35s 内不再换回
CROSS_GAIN = 0.02                       # cross-track 纠偏增益（rad/px，备用）
CROSS_CORR_MAX = 0.6                    # 纠偏上限（rad，备用）

# ---------------- 速度控制 ----------------
# 玩家坦克与敌方坦克使用相同的游戏物理上限。只要存在已验证路径，控制器持续按住
# 前进键，由游戏自身的 maxspeed=125px/s 限速，不再进行直线/弯道 PWM 降速。
GAME_MAX_SPEED_PX_S = 125.0
CRUISE_SPEED_HIGH = GAME_MAX_SPEED_PX_S # 兼容测试/旧工具；线上控制器不再以它松油
CRUISE_SPEED_LOW = GAME_MAX_SPEED_PX_S
PID_KP_V = 0.1                          # 速度 P（观测器外备用增益）
SPEED_HYST_PX_S = 3.0                   # 油门滞回死区：±3px/s 内不翻油门（防喘动）
SPEED_MODEL_ACCEL = 200.0               # 运动模型加速度（与游戏 Car accel/decel 一致）
SPEED_MEAS_CORR = 0.15                  # 测量慢修正系数：模型为主、差分速度为辅
CORNER_SPEED = GAME_MAX_SPEED_PX_S      # 兼容旧工具：弯道不再主动限速
CORNER_MIN_SPEED = GAME_MAX_SPEED_PX_S
CORNER_LOOKAHEAD_PX = 60.0              # 曲率前瞻窗口
STEER_SPEED_PX_S = 3.8                  # 游戏转向速率（rad/s，实测值）
TURN_BRAKE_RAD = 2.5                    # 急弯降速转向阈值（rad/s）：|w_des| 超过此值
                                        # 时打舵帧松开油门，让游戏速度 s 衰减，触发物理
                                        # "低速转向保底" → 转弯半径从 33px 缩到 ~20px 级；
                                        # 0 = 关闭（始终全速转向，R 恒 33px）
TURN_BRAKE_MIN_SPEED_PX_S = 40.0        # 起步保护：车速低于此值不压油门。
                                        # 否则"车头与路径差角大"（困角落的常态）时
                                        # |w_des| 恒超阈值 → 油门被压死 → v=0 原地转圈死锁

# ---------------- 原地旋转（无路线时） ----------------
SPIN_SIDE_SIN_THRESHOLD = 0.15          # 选向阈值：|sinθ|>0.15（≈8.6°）时朝敌所在侧转
SPIN_BEHIND_RAD = 0.15                  # 判定"敌在正后方(≈180°)"的死区：|θ|>π-此值 → 默认右转

# ---------------- 路径切换滞回 ----------------
PATH_HOLD_MAX_S = 8.0                   # 路径保持上限：超过才允许 180° 级换向
SIMILAR_TARGET_PX = 35.0                # 目标移动小于此距离视为同目标
NORMAL_SWITCH_RATIO = 1.15              # 普通换向允许的新旧路径比
STAGING_TO_ATTACK_RATIO = 1.35          # 由守转攻允许的新旧路径比
PATH_SWITCH_MIN_REMAINING_PX = 20.0     # 短程方向粘滞：旧路径剩余>此值才防换向

# ---------------- 轮次边界判定 ----------------
ROUND_GAP_S = 2.5                       # 帧间隔超过此值视为轮次间隙（配合位置跳变判定）
ROUND_JUMP_PX = 120.0                   # 位置跳变超过此值视为换轮（重生/瞬移护栏）

# ---------------- 迷宫锁图 ----------------
MAZE_MIN_WALL_CELLS = 20                # 有效迷宫至少 20 格墙（真实迷宫 99-100）
