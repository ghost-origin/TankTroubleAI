# -*- coding: utf-8 -*-
"""Online navigation bot for TankTroubleAI.

Bug-fixed round recorder/controller.

Minimal integration: reuse existing src/ai/ai-bridge.js and data log/run_match.js.
This process receives the same state JSON, sends key actions back, and writes
track_*.csv / maze_*.csv / plans_*.csv that belong to exactly ONE round.

Important invariants:
- The first valid maze snapshot is kept; it is never overwritten by a later round.
- Once a round end / respawn is detected, no more track rows are appended.
- A long state gap or implausible position discontinuity is treated as a round boundary.
"""
from __future__ import annotations
import argparse, csv, json, math, os, re, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))

from navigation_mvp import (
    A_STAR_TOPOLOGY_RADIUS_PX, DEFAULT_TACTICAL_MODE, MAP_PX,
    TACTICAL_MODE_REAR_ONLY, TACTICAL_MODE_V1, TACTICAL_MODE_CHASE,
    WINDOW_REPLAN_REMAINING_PX,
    TANK_HALF_LENGTH_PX, TANK_HALF_WIDTH_PX,
    build_maps, load_polys, plan, swept_rectangle_path_clear,
    tank_footprint_corners,
)
from kalman_path_predictor import ConstantVelocityKalman2D
from combat_ai import CombatController, aim_beam, MUZZLE_OFFSET_PX

# 全部可调参数集中在 bot_config.py（路点间距 / OODA / PID / 速度 / 旋转 / 滞回等）
from bot_config import *   # noqa: F401,F403

CSV_COLUMNS = [
    't','me_x','me_y','me_angle','me_vx','me_vy',
    'foe_x','foe_y','foe_angle','foe_vx','foe_vy','n_bullets','n_powerups',
    'cmd_up','cmd_down','cmd_left','cmd_right','controller_mode','wall'
]
# 开火判定日志（每帧 combat 判定摘要，排查"为什么打/为什么不打"）
FIRELOG_COLUMNS = ['t','round','aim_ok','aim_deg','conf','err_deg','turning',
                   'hit','bounces','miss','self_clr',
                   'pass_c','pass_a','pass_s','pass_d','lock','fire','why','wall',
                   'pred_x','pred_y','foe_x','foe_y']
# 双方子弹轨迹（每帧每颗子弹一行；owner: me/foe/? 按出生位置就近归属）
BULLETS_COLUMNS = ['t','round','uid','x','y','vx','vy','type','owner','wall']
PLANS_COLUMNS = [
    't','success','accepted','tactical_mode','target_type','relative_deg','target_x','target_y',
    'path_length_px','old_remaining_px','smoothness_rad','planning_ms','reason','path_points',
    'candidate_count','reachable_candidates','rear_candidates','rear_reachable_candidates',
    'reachable_candidate_rate','line_of_sight','open_directions','tactical_score','switch_cost',
    'path_source','path_validated','validation_reason','raw_path_points','executed_path_points',
    'global_path_length_px','window_path_length_px','window_target_length_px',
    'window_lookahead_length_px','window_goal_reached',
    'local_window_due','replan_period_s',
    'prediction_horizon_s','observed_foe_x','observed_foe_y',
    'predicted_foe_x','predicted_foe_y','predicted_foe_path','wall'
]
# 坐标/结构事实（非调参项）：tilemap 原点与迷宫尺寸
MAP_ORIGIN_X = 197.5
MAP_ORIGIN_Y = 31.0
MAZE_SIZE = 10


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Bot:
    def __init__(self, port, out_dir, repo_root, tactical_mode=DEFAULT_TACTICAL_MODE,
                 prediction_horizon_s=DEFAULT_PREDICTION_HORIZON_S):
        sys.path.insert(0, os.path.join(repo_root, 'data log'))
        from ws_server import WSServer
        self.WSServer = WSServer
        self.port = port
        self.out_dir = out_dir
        self.repo_root = repo_root
        self.tactical_mode = tactical_mode
        self.prediction_horizon_s = max(0.0, min(1.0, float(prediction_horizon_s)))
        os.makedirs(out_dir, exist_ok=True)

        # 网页模式数据管理（对齐无头"一局一组 CSV"的思路）：
        # 固定文件名，新局直接覆盖重写（从 0 开始），bot 常驻不杀进程。
        # 外部工具只读这三个最新文件即可判断当前局信息。
        # round_no = 会话内真实局号。递增只由两个客观"新局信号"驱动：
        #   ① 桥事件 me_respawn / layout_changed（新局 me 实例必重建）；
        #   ② save_map 迷宫签名实际变化（每局随机迷宫不同）。
        # 2s 防抖合并同一换局的多个信号；误触发的 finish/reset（同局爆炸位移、
        # foe 动画误判等）不再产生新号 —— 根治"round 与局数对不上"。
        self.round_no = 0
        self._last_round_inc_wall = 0.0   # 上次局号递增的墙钟（防抖）
        self._round_wall_start = None     # 本局开始的真实时间（rounds wall_start）
        self.track_path = os.path.join(out_dir, 'track.csv')
        self.maze_path = os.path.join(out_dir, 'maze.csv')
        self.plan_path = os.path.join(out_dir, 'plans.csv')
        self.firelog_path = os.path.join(out_dir, 'firelog.csv')   # 开火判定日志
        self.bullets_path = os.path.join(out_dir, 'bullets.csv')   # 双方子弹轨迹
        self.ff = None          # firelog 句柄（惰性创建）
        self.fw = None
        self.bf = None          # bullets 句柄（惰性创建）
        self.bw = None
        # 逻辑子弹跟踪（游戏实例 uid 会复用，不能作跟踪键）：
        # 跨帧最近邻匹配，给每颗"逻辑子弹"一个自增 tid。
        self._btrack = {}        # tid -> [x, y, vx, vy, type, owner, last_t]
        self._btrack_next = 1    # 下一个逻辑子弹 id
        self._bullet_miss = {}   # tid -> 连续未出现帧数
        self._bullet_t = None    # 上次跟踪帧时间（算 dt 用）

        # 惰性创建：第一帧到达才写文件，避免 0 行空文件噪声
        self.f = None
        self.w = None
        self.pf = None
        self.pw = None
        # 按局归档（每局独立、不覆盖）：track_roundNNN.csv / plans_roundNNN.csv
        # —— 与 track.csv/plans.csv（当前局，覆盖）并存，前几局数据不丢。
        self.f_rec = None
        self.w_rec = None
        self.pf_rec = None
        self.pw_rec = None
        self._arch_round = None       # 当前归档所属局号（幂等建文件的判据）
        self._open_round_files()

        self.polys = None
        self.raw_wall = None
        self.blocked = None
        self.blocked_turn = None
        self.free_dist = None
        self.maze_signature = None
        self.maze_written = False
        self.exit_on_round_end = False
        self.visualize = False

        self.last_plan_t = -1e9
        self._force_plan_accept = False
        self._blocked_since = None       # 前方受阻开始时刻（卡死旋转脱困判定）
        self._blocked_pos = None         # 受阻开始位置（判断是否"有进展"）
        self.path = []
        self.dense_path = []
        self.wp_idx = 0
        self._start_aligned = False      # 起步对准（一次性）跨轮清零
        self._align_started_at = None
        self.current_plan_reason = ''
        self.current_path_source = None       # 当前执行路线的来源（chase / chase_alt）
        self.current_relative_deg = None
        self.current_tactical_target = None
        self.current_window_goal_reached = False
        self.current_window_replan_remaining = WINDOW_REPLAN_REMAINING_PX
        self._tail_due_last = False          # 尾段上升沿记忆（进入尾段瞬间立即续接）
        self.controller_mode = 'STOP'
        self._init_accel_since = None
        self._last_steer_dir = 0
        self._steer_flip_t = None
        self._pwm_phase = 0
        # PID 状态
        self._pid_th_i = 0.0
        self._pid_th_prev = 0.0
        self._pid_lat_i = 0.0
        self._pid_lat_prev = 0.0
        self._pid_v_i = 0.0
        self._steer_accum = 0.0
        self._throttle_on = False
        self._v_smooth = None
        self._frame_dt = 1.0 / 66.0
        self._spin_dir = 'right'
        self._tangent_prev = None
        self._omega_applied = None
        self._last_action_t = None
        self.last_path_switch_t = -1e9
        self.client = None
        self.rows = 0
        self._plans_flush_cnt = 0        # plans 落盘节流计数（每 4 次 replan flush 一次）

        self.seen_both = False
        self.round_ended = False
        self.round_end_reason = ''
        self.last_state_t = None
        self.last_me_pos = None
        self.last_foe_pos = None
        self.me_predictor = ConstantVelocityKalman2D()
        self.foe_predictor = ConstantVelocityKalman2D()

        # 自动瞄准射击（独立模块，不修改导航逻辑）：
        # 每帧在导航 action 之后做一次射击判定——若存在能直瞄/反弹命中敌人
        # 的发射角，则暂停导航、进入射击模式（停车 + 转向瞄准 + 开火）。
        self.combat = CombatController()
        self.combat_enabled = True
        self.last_bullets = []

        # 文件覆盖信号：launcher 每次收到桥的新局事件就覆盖写 round_reset.csv，
        # bot 发现内容变化 → 立即清理本轮数据开始新轮（进程永不重启）。
        # 启动时读一次做基线：旧会话残留的旧信号不触发。
        self.round_sig_path = os.path.join(out_dir, 'round_reset.csv')
        self._last_sig = self._read_round_sig()

        print('track:', self.track_path, flush=True)
        print('maze:', self.maze_path, flush=True)
        self._watch_parent()

    def _read_round_sig(self):
        """读新局覆盖信号文件；不存在/读空返回 None（不触发）。"""
        try:
            with open(self.round_sig_path, 'r', encoding='utf-8') as f:
                return f.read().strip() or None
        except Exception:
            return None

    def _on_round_signal(self, sig):
        """文件被覆盖 = 新一轮已经开始：立即清理旧数据，不等 WS 事件帧。

        调用方保证在 save_map/事件处理之前执行，使本轮帧能直接锁新图。
        """
        print('== round signal from file: %s ==' % sig, flush=True)
        if self.seen_both and not self.round_ended:
            self.finish_round('file_signal(%s)' % sig)
        if self.round_ended or self.path:
            self.reset_for_new_round('file_signal(%s)' % sig)
        # 开局前（从未见过完整帧）收到信号：无数据可清，直接等首帧

    def _watch_parent(self):
        """孤儿自检：启动器（父进程）死亡后自动退出。

        旧行为：关闭启动器窗口时 bot 变孤儿继续监听端口，旧游戏标签页
        仍连着旧代码的 bot —— "改了代码实测没生效/环境还在跑旧 bot" 的根源。
        启动器正常退出路径已改为终止当前 bot；这里是硬关闭（X 掉控制台/
        taskkill）时的兜底：每 5s 检查父进程存活，父死则自尽。
        """
        if os.name != 'nt':
            return
        import threading
        try:
            import ctypes
        except Exception:
            return
        ppid = os.getppid()
        STILL_ACTIVE = 259

        def loop():
            while True:
                time.sleep(5.0)
                try:
                    k = ctypes.windll.kernel32
                    h = k.OpenProcess(0x1000, False, ppid)  # PROCESS_QUERY_LIMITED_INFORMATION
                    if not h:
                        break
                    code = ctypes.c_ulong()
                    ok = k.GetExitCodeProcess(h, ctypes.byref(code))
                    k.CloseHandle(h)
                    if ok and code.value != STILL_ACTIVE:
                        break
                except Exception:
                    continue
            print('parent process gone, bot exiting', flush=True)
            self.close()
            os._exit(0)

        threading.Thread(target=loop, daemon=True).start()
        print('parent-watch armed (ppid=%d, 5s cycle)' % os.getppid(), flush=True)

    def _open_round_files(self):
        """首帧到达时（惰性）创建本轮的 track/plans 文件 + "当前轮"latest 文件。"""
        if self.f is not None:
            return
        self.f = open(self.track_path, 'w', newline='', encoding='utf-8')
        self.w = csv.DictWriter(self.f, fieldnames=CSV_COLUMNS)
        self.w.writeheader()
        self.pf = open(self.plan_path, 'w', newline='', encoding='utf-8')
        self.pw = csv.DictWriter(self.pf, fieldnames=PLANS_COLUMNS)
        self.pw.writeheader()

        # "当前轮"固定名文件：每轮开始覆盖旧的（新内容 = 新一轮）。
        # 外部工具/用户只读 *_latest.csv 永远是最新轮的数据。
        self.lf = open(os.path.join(self.out_dir, 'track_latest.csv'), 'w', newline='', encoding='utf-8')
        self.lw = csv.DictWriter(self.lf, fieldnames=CSV_COLUMNS)
        self.lw.writeheader()
        self.lpf = open(os.path.join(self.out_dir, 'plans_latest.csv'), 'w', newline='', encoding='utf-8')
        self.lpw = csv.DictWriter(self.lpf, fieldnames=PLANS_COLUMNS)
        self.lpw.writeheader()

        # 开火判定日志 / 双方子弹轨迹：**追加累积**（round 列区分局次）。
        # 覆盖式会丢掉前几局（排查需要连续多局）；如需从零开始，删除这两个文件。
        self.ff = None
        self.fw = None
        self.bf = None
        self.bw = None
        if IS_DATA_LOG:
            # 调试数据（firelog/bullets）：IS_DATA_LOG=False 时全部不创建不写入
            self.ff = open(self.firelog_path, 'a', newline='', encoding='utf-8',
                           buffering=64 * 1024)   # 大缓冲：减少自动落盘次数
            self.fw = csv.DictWriter(self.ff, fieldnames=FIRELOG_COLUMNS)
            if os.path.getsize(self.firelog_path) == 0:
                self.fw.writeheader()
            self.bf = open(self.bullets_path, 'a', newline='', encoding='utf-8',
                           buffering=64 * 1024)
            self.bw = csv.DictWriter(self.bf, fieldnames=BULLETS_COLUMNS)
            if os.path.getsize(self.bullets_path) == 0:
                self.bw.writeheader()
        self._btrack = {}        # 换局清空跟踪（新局子弹全为新逻辑子弹）
        self._btrack_next = 1
        self._bullet_miss = {}
        self._bullet_t = None

        # 轮次/死亡取证数据
        self.round_t0 = None
        self.round_dist = 0.0
        self.round_frames = 0
        self._prev_rec_pos = None
        self.last_me = None
        self.last_foe = None
        self.last_foe_anim = ''
        self.round_rec_path = os.path.join(self.out_dir, 'rounds.csv')
        self._rec_tail = []

    @staticmethod
    def stop_action():
        return {'keys': {'up':0,'down':0,'left':0,'right':0}, 'fire':0, 'lock':0}

    def finish_round(self, reason):
        if self.round_ended:
            return
        self.round_ended = True
        self.round_end_reason = reason
        self.path = []
        self.dense_path = []
        self._tangent_prev = None
        self._omega_applied = None
        self.wp_idx = 0
        self.current_plan_reason = ''
        self.current_path_source = None       # 当前执行路线的来源（chase / chase_alt）
        self.current_relative_deg = None
        self.current_tactical_target = None
        self.current_window_goal_reached = False
        self.current_window_replan_remaining = WINDOW_REPLAN_REMAINING_PX
        self._tail_due_last = False          # 尾段上升沿记忆（进入尾段瞬间立即续接）
        self.me_predictor = ConstantVelocityKalman2D()
        self.foe_predictor = ConstantVelocityKalman2D()
        try:
            self.f.flush()
            self.pf.flush()
        except Exception:
            pass
        print('== round ended: %s | rows=%d ==' % (reason, self.rows), flush=True)
        self._write_round_record(reason)
        # 无头评测模式：一局结束即整场结束（一次比赛 = 一次结果）。
        # 写标志文件 + 主动退出，runner 检测到即收场，不再滚动下一局。
        if self.exit_on_round_end:
            try:
                with open(os.path.join(self.out_dir, 'round_end.flag'), 'w', encoding='utf-8') as f:
                    f.write(reason)
                self.f.flush()
                self.pf.flush()
            except Exception:
                pass
            print('exit_on_round_end: terminating bot after this round', flush=True)
            os._exit(0)

    def _los_clear_between(self, p1, p2, raw_wall):
        """me_end→foe_end 连线是否穿墙（两端各缩 22px 车体半径，2px 步长采样）。

        直线弹道击杀 → 连线不穿墙；穿墙 → 非直线弹道（反弹弹/其他死亡）。
        距离 < 44px 时无法判定（返回 None）。
        """
        if raw_wall is None or p1 is None or p2 is None:
            return None
        x1, y1 = p1; x2, y2 = p2
        d = math.hypot(x2 - x1, y2 - y1)
        if d < 44.0:
            return None
        ux, uy = (x2 - x1) / d, (y2 - y1) / d
        s0, s1 = 22.0, d - 22.0
        # 1px 步长采样（2px+取整会跳过窄墙，把"穿墙"误判为"不穿墙"）
        n = max(4, int(s1 - s0))
        h, w = raw_wall.shape
        for i in range(n + 1):
            s = s0 + (s1 - s0) * i / n
            x = int(x1 + ux * s); y = int(y1 + uy * s)
            if 0 <= y < h and 0 <= x < w and raw_wall[y, x]:
                return False
        return True

    def _write_round_record(self, reason):
        """每轮结束写一行取证记录（rounds.csv，见表头）。

        me_foe_los=1 → 死亡时 me_end 与 foe_end 连线不穿墙（疑似直线弹道击杀）；
        me_foe_los=0 → 连线穿墙（反弹弹/其他）；空 → 距离过近或地图缺失无法判定。
        """
        try:
            me = self.last_me or {}
            foe = self.last_foe or {}
            maze_wall = None
            if self.maze_signature is not None:
                maze_wall = sum(1 for row in self.maze_signature for v in row if v != 0 and v != -1)
            los = None
            me_x = me.get('x'); me_y = me.get('y')
            foe_x = foe.get('x'); foe_y = foe.get('y')
            if None not in (me_x, me_y, foe_x, foe_y):
                los = self._los_clear_between(
                    (me_x, me_y), (foe_x, foe_y), self.raw_wall)
            # 最后两帧速度估算
            last_speed = None
            if len(self._rec_tail) >= 2:
                (t0, x0, y0), (t1, x1, y1) = self._rec_tail[-2], self._rec_tail[-1]
                if t1 > t0:
                    last_speed = math.hypot(x1 - x0, y1 - y0) / (t1 - t0)
            header = ['round_id','t_end','t_start','reason','me_end_x','me_end_y','me_end_angle',
                      'foe_end_x','foe_end_y','foe_end_angle','foe_anim','me_round_dist_px',
                      'me_round_frames','me_last_speed_px_s','me_foe_los','maze_wall_count',
                      'track_file','round_no','wall_start','wall_end']
            new = not os.path.exists(self.round_rec_path)
            with open(self.round_rec_path, 'a', newline='', encoding='utf-8') as f:
                w = csv.writer(f)
                if new:
                    w.writerow(header)
                w.writerow([
                    os.path.basename(self.track_path), round(self.last_state_t or 0.0, 3),
                    round(self.round_t0 or 0.0, 3), reason,
                    round(me.get('x', 0.0), 1), round(me.get('y', 0.0), 1),
                    round(me.get('angle', 0.0), 3),
                    round(foe.get('x', 0.0), 1), round(foe.get('y', 0.0), 1),
                    round(foe.get('angle', 0.0), 3), self.last_foe_anim,
                    round(self.round_dist, 1), self.round_frames,
                    round(last_speed, 1) if last_speed is not None else '',
                    int(los) if los is not None else '',
                    maze_wall if maze_wall is not None else '',
                    os.path.basename(self.track_path),
                    self.round_no,
                    (round(self._round_wall_start, 2) if self._round_wall_start else '')
                    if IS_DATA_LOG else '',
                    round(time.time(), 2) if IS_DATA_LOG else '',
                ])
        except Exception:
            import traceback
            traceback.print_exc()

    def _inc_round(self):
        """开新局号（唯一入口）。1s 防抖：同一换局链的多个信号（me_respawn +
        maze_changed 常同帧/隔几帧到达）只算一局；真正的下一局（间隔 >1s）递增。"""
        now = time.time()
        if now - self._last_round_inc_wall >= 1.0:
            self.round_no += 1
            self._round_wall_start = now
        self._last_round_inc_wall = now

    def reset_for_new_round(self, sig_reason):
        """网页模式连续对局：一轮结束后自动复位，等待下一轮开始。

        复位状态但**保留** map 相关（raw_wall/blocked/maze_signature/polys），
        因为 save_map 只锁定"第一张有效迷宫"，下一局迷宫不同时会走
        maze_changed_new_round 分支重新处理（见 save_map）。
        """
        # 注意：round_no 不在这里递增！局号只由客观新局信号（me_respawn /
        # layout_changed / 迷宫变化）驱动，见 _inc_round。同局误触发的 reset
        # （爆炸位移/frame gap 等假 finish）只清状态，不产生新局号。
        self.round_ended = False
        self.round_end_reason = ''
        self.seen_both = False
        self.last_state_t = None
        self.last_me_pos = None
        self.last_foe_pos = None
        # 关键修复：跨轮复位必须清空上局**规划缓存**（raw_wall 等），否则下一局会
        # 用旧迷宫规划穿墙直行。但 maze_signature **保留**：save_map 用它比较新迷宫
        # 是否变化来判定"客观新局"（round 号递增的依据）；清掉它会导致新迷宫
        # 无法识别 → 多局共用一个 round 号（数据对不上局的根因）。
        self.raw_wall = None
        self.blocked = None
        self.blocked_turn = None
        self.free_dist = None
        self.maze_written = False
        self.path = []
        self.dense_path = []
        self.wp_idx = 0
        self._start_aligned = False      # 起步对准（一次性）跨轮清零
        self._align_started_at = None
        self.current_plan_reason = ''
        self.current_path_source = None       # 当前执行路线的来源（chase / chase_alt）
        self.current_relative_deg = None
        self.current_tactical_target = None
        self.current_window_goal_reached = False
        self.current_window_replan_remaining = WINDOW_REPLAN_REMAINING_PX
        self._tail_due_last = False          # 尾段上升沿记忆（进入尾段瞬间立即续接）
        self.me_predictor = ConstantVelocityKalman2D()
        self.foe_predictor = ConstantVelocityKalman2D()
        self.last_plan_t = -1e9
        self.last_path_switch_t = -1e9
        self._blocked_since = None       # 卡死旋转状态跨轮清零
        self._blocked_pos = None
        # 速度环状态清零：跨轮残留旧速度平滑值会延迟/错乱新局油门
        self._v_smooth = None
        self._throttle_on = False
        self._steer_accum = 0.0
        self._tangent_prev = None
        self._omega_applied = None
        # 轮次取证状态清零
        self.round_t0 = None
        self.round_dist = 0.0
        self.round_frames = 0
        self._prev_rec_pos = None
        self._rec_tail = []
        self.last_me = None
        self.last_foe = None
        self.last_foe_anim = ''
        # 自动瞄准状态跨轮复位：新局清掉旧瞄准角/开火冷却
        self.combat.reset()
        self.last_bullets = []
        # 固定文件名：新局覆盖重写（从 0 开始），bot 常驻不杀进程。
        # round_no 不在此递增（见 _inc_round：只由 me_respawn/迷宫变化驱动）
        self.track_path = os.path.join(self.out_dir, 'track.csv')
        self.plan_path = os.path.join(self.out_dir, 'plans.csv')
        self.maze_path = os.path.join(self.out_dir, 'maze.csv')
        try:
            self.f.close()
            self.pf.close()
            self.lf.close()
            self.lpf.close()
            if self.ff is not None:
                self.ff.close()
            if self.bf is not None:
                self.bf.close()
            if self.f_rec is not None:
                self.f_rec.close()
            if self.pf_rec is not None:
                self.pf_rec.close()
        except Exception:
            pass
        self.f = None
        self.pf = None
        self.w = None
        self.pw = None
        self.lf = None
        self.lpf = None
        self.lw = None
        self.lpw = None
        self.f_rec = None
        self.pf_rec = None
        self.w_rec = None
        self.pw_rec = None
        self._arch_round = None       # 换局后让 _ensure_round 按新局号重开归档
        self.ff = None
        self.fw = None
        self.bf = None
        self.bw = None
        self._btrack = {}        # 换局清空跟踪（新局子弹全为新逻辑子弹）
        self._btrack_next = 1
        self._bullet_miss = {}
        self._bullet_t = None
        print('== new round reset (%s): %s ==' % (sig_reason, self.track_path), flush=True)
    def map_coords(self, o):
        if not o:
            return None
        return dict(
            x=float(o['x']) - MAP_ORIGIN_X,
            y=float(o['y']) - MAP_ORIGIN_Y,
            angle=float(o.get('angle', 0.0)),
            vx=float(o.get('vx', 0.0)),
            vy=float(o.get('vy', 0.0)),
        )

    def on_client(self, c):
        self.client = c
        c.on_message = lambda txt: self.on_message(c, txt)
        c.on_close = lambda _c: self.handle_client_disconnect(_c)
        print('connected:', c.addr, flush=True)

    def handle_client_disconnect(self, c):
        if self.client is c:
            self.client = None
        # 只标记断开，绝不关闭 CSV（文件由轮次复位/进程退出管理），
        # 否则桥 2s 后重连时机器人已"哑火"。
        print('client disconnected (waiting for reconnect)', flush=True)

    def save_map(self, m):
        """Capture exactly the first valid maze for this round.

        A different maze signature means a new round has begun. Do not overwrite
        the maze that belongs to the already-recorded track.
        """
        grid = m.get('grid') or []
        if len(grid) < MAZE_SIZE:
            return
        maze = [[int(grid[y][x]) for x in range(MAZE_SIZE)] for y in range(MAZE_SIZE)]
        # 迷宫有效性校验：真实迷宫 99-100 格墙；全 -1/空白（未生成/垃圾帧）
        # 或墙数过少 → 拒绝锁图，等下一帧（桥现在每帧都带 grid）。
        wall_n = sum(1 for row in maze for v in row if v != 0 and v != -1)
        if wall_n < MAZE_MIN_WALL_CELLS:
            return
        sig = tuple(tuple(row) for row in maze)

        if self.maze_signature is not None and sig == self.maze_signature:
            if self.raw_wall is not None:
                return                       # 同迷宫且已锁：不变
            # reset 后重锁同迷宫（同局误判恢复）：不是新局，不递增，直接重锁
        elif self.maze_signature is not None:
            # 迷宫实际变化 = 客观新局信号：reset 规划缓存并开新局号（防抖合并
            # 同一次换局的事件与迷宫信号）。signature 保留到此处比较后才更新。
            self.reset_for_new_round('maze_changed')
            self._inc_round()
        # 走到这里 = 首次锁图 / 新迷宫 / reset 后需重锁

        self.maze_signature = sig
        self._ensure_round()   # 首局锁图时 round_no 还是 0 → 置为 1
        with open(self.maze_path, 'w', encoding='utf-8') as f:
            for row in maze:
                f.write(','.join(map(str, row)) + '\n')
        # "当前轮"迷宫：固定名覆盖写
        with open(os.path.join(self.out_dir, 'maze_latest.csv'), 'w', encoding='utf-8') as f:
            for row in maze:
                f.write(','.join(map(str, row)) + '\n')
        # 按局存档：maze_roundNNN.csv（与 firelog/bullets 的 round 列对应）。
        # 旧实现只有覆盖式 maze.csv，局结束即丢失，无法复现"模拟穿墙/假直射"。
        # 仅 IS_DATA_LOG=True 时存档（关闭日志时省一次磁盘写）
        if IS_DATA_LOG and self.round_no > 0:
            arch = os.path.join(self.out_dir, 'maze_round%03d.csv' % self.round_no)
            with open(arch, 'w', encoding='utf-8') as f:
                for row in maze:
                    f.write(','.join(map(str, row)) + '\n')

        if m.get('polys'):
            self.polys = m['polys']
        elif self.polys is None:
            self.polys = load_polys(os.path.join(self.repo_root, 'data log', 'tile_polys.json'))

        # 方向性 Minkowski + tube 管道：A* 用窄网格（10px 直行），
        # 转弯点用外接圆（18.45px）校验；free_dist 供 tube 车道校验。
        (self.raw_wall, _, self.blocked, self.blocked_turn, self.free_dist) = build_maps(
            maze, self.polys, 10, 5, turn_radius_px=18.45,
            straight_radius_px=A_STAR_TOPOLOGY_RADIUS_PX)
        self.maze_written = True
        print('maze snapshot locked for this round', flush=True)

    def _ensure_round(self):
        """确保当前局号 >= 1：首局（bot 启动后第一局）锁图/首帧时 round_no 还是 0。"""
        if self.round_no == 0:
            self.round_no = 1
            self._round_wall_start = time.time()
        self._open_round_archive()   # 建立/续写本局"按局归档"文件（每局独立、不覆盖）

    def _open_round_archive(self):
        """为当前 round_no 建/续写 按局归档 track_roundNNN.csv / plans_roundNNN.csv。

        与 track.csv（当前局，覆盖）并存，前几局完整轨迹不丢。按局号幂等：换局后
        (round_no 变) 自动换到新文件；同局内重复调用不会重复 open。round_no<1 不建。
        """
        if self._arch_round == self.round_no:
            return
        # 切换到本局的归档文件：先关旧句柄（换局时旧局写入至此结束）
        for h in (self.f_rec, self.pf_rec):
            if h is not None:
                try:
                    h.close()
                except Exception:
                    pass
        self._arch_round = self.round_no
        self.f_rec = None
        self.pf_rec = None
        self.w_rec = None
        self.pw_rec = None
        if self.round_no < 1:
            return
        self.f_rec = open(os.path.join(self.out_dir, 'track_round%03d.csv' % self.round_no),
                          'w', newline='', encoding='utf-8')
        self.w_rec = csv.DictWriter(self.f_rec, fieldnames=CSV_COLUMNS)
        self.w_rec.writeheader()
        self.pf_rec = open(os.path.join(self.out_dir, 'plans_round%03d.csv' % self.round_no),
                           'w', newline='', encoding='utf-8')
        self.pw_rec = csv.DictWriter(self.pf_rec, fieldnames=PLANS_COLUMNS)
        self.pw_rec.writeheader()

    def record(self, t, me, foe, msg, action):
        keys = action.get('keys') or {}
        self._ensure_round()          # 先确保局号>=1，_open_round_files 才能用局号命名归档文件
        self._open_round_files()
        if self.round_t0 is None:
            self.round_t0 = t
        if self._prev_rec_pos is not None:
            self.round_dist += math.hypot(me['x'] - self._prev_rec_pos[0],
                                          me['y'] - self._prev_rec_pos[1])
        self._prev_rec_pos = (me['x'], me['y'])
        self.round_frames += 1
        self._rec_tail.append((t, me['x'], me['y']))
        if len(self._rec_tail) > 4:
            self._rec_tail.pop(0)
        row = {
            't':round(t,2),
            'me_x':round(me['x'],1), 'me_y':round(me['y'],1), 'me_angle':round(me['angle'],3),
            'me_vx':round(me['vx'],1), 'me_vy':round(me['vy'],1),
            'foe_x':round(foe['x'],1), 'foe_y':round(foe['y'],1), 'foe_angle':round(foe['angle'],3),
            'foe_vx':round(foe['vx'],1), 'foe_vy':round(foe['vy'],1),
            'n_bullets':len(msg.get('bullets') or []),
            'n_powerups':len(msg.get('powerups') or []),
            'cmd_up':int(bool(keys.get('up'))),
            'cmd_down':int(bool(keys.get('down'))),
            'cmd_left':int(bool(keys.get('left'))),
            'cmd_right':int(bool(keys.get('right'))),
            'controller_mode':self.controller_mode,
            'wall': round(time.time(), 2) if IS_DATA_LOG else '',
        }
        self.w.writerow(row)
        self.lw.writerow(row)      # 同步写"当前轮"track_latest.csv
        if self.w_rec is not None:
            self.w_rec.writerow(row)   # 按局归档（不覆盖）
        self.rows += 1

        # ---- 开火判定日志（每帧 combat 判定摘要，排查开火问题）----
        fd = getattr(self.combat, 'last_firelog', None) or {}
        if fd and self.fw is not None:
            frow = {'t': round(t, 3), 'round': self.round_no,
                    'wall': round(time.time(), 2) if IS_DATA_LOG else ''}
            for k in FIRELOG_COLUMNS:
                if k not in frow:
                    frow[k] = fd.get(k, '')
            self.fw.writerow(frow)
        # ---- 双方子弹轨迹（逻辑 id 跨帧跟踪；owner 按出生位置就近归属一次）----
        if self.bw is not None:
            for tr in self._track_bullets(t, me, foe, msg.get('bullets') or []):
                self.bw.writerow({
                    't': round(t, 3), 'round': self.round_no, 'uid': 't%d' % tr['tid'],
                    'x': round(tr['x'], 1), 'y': round(tr['y'], 1),
                    'vx': round(tr['vx'], 1), 'vy': round(tr['vy'], 1),
                    'type': tr['btype'], 'owner': tr['owner'],
                    'wall': round(time.time(), 2) if IS_DATA_LOG else ''})

        # 磁盘 I/O 节流：每 60 帧才 flush 一次（30Hz 下 ≈ 每 2 秒 1 次）。
        # flush 是磁盘写：系统磁盘忙时（杀软/同步扫描大日志文件）会从 0.2ms
        # 飙到几十 ms，直接卡住单线程 bot → 决策延迟抖动 → 转向过冲放大。
        if self.rows % 60 == 0:
            self.f.flush()
            self.lf.flush()
            if self.f_rec is not None:
                self.f_rec.flush()
            if self.ff is not None:
                self.ff.flush()
            if self.bf is not None:
                self.bf.flush()

    def _track_bullets(self, t, me, foe, bl):
        """逻辑子弹跟踪：跨帧最近邻匹配（游戏实例 uid 会复用，不能作跟踪键）。

        每颗"逻辑子弹"分配自增 tid：用上一帧位置+速度外推预测本帧位置，
        与实测位置最近邻匹配（同 type、距离 ≤ BULLET_MATCH_PX）。owner 只在
        子弹首次出现时按出生位置就近归属 me/foe/?，之后沿用。
        返回本帧每颗子弹的记录（tid/x/y/vx/vy/type/owner），供 bullets.csv 落盘。
        """
        if self._bullet_t is None:
            self._bullet_t = t
        dt = max(0.0, t - self._bullet_t)
        self._bullet_t = t
        mz_me = (me['x'] + math.cos(me['angle']) * MUZZLE_OFFSET_PX,
                 me['y'] + math.sin(me['angle']) * MUZZLE_OFFSET_PX)
        mz_foe = (foe['x'] + math.cos(foe['angle']) * MUZZLE_OFFSET_PX,
                  foe['y'] + math.sin(foe['angle']) * MUZZLE_OFFSET_PX)
        rows = []
        used = set()
        for b in bl:
            try:
                x = float(b['x']) - MAP_ORIGIN_X
                y = float(b['y']) - MAP_ORIGIN_Y
                vx = float(b.get('vx', 0.0))
                vy = float(b.get('vy', 0.0))
                btype = str(b.get('type', ''))
            except Exception:
                continue
            best_tid, best_d = None, 1e18
            for tid, st in self._btrack.items():
                if tid in used or st[4] != btype:
                    continue
                px = st[0] + st[2] * dt      # 按速度外推的预测位置
                py = st[1] + st[3] * dt
                d = math.hypot(x - px, y - py)
                if d < best_d:
                    best_d, best_tid = d, tid
            if best_tid is not None and best_d <= BULLET_MATCH_PX:
                tid = best_tid
                st = self._btrack[tid]
                st[0], st[1], st[2], st[3], st[6] = x, y, vx, vy, t
                self._bullet_miss[tid] = 0
                owner = st[5]
            else:
                tid = self._btrack_next       # 新逻辑子弹
                self._btrack_next += 1
                d_me = math.hypot(x - mz_me[0], y - mz_me[1])
                d_foe = math.hypot(x - mz_foe[0], y - mz_foe[1])
                if d_me <= d_foe and d_me < 45.0:
                    owner = 'me'
                elif d_foe < 45.0:
                    owner = 'foe'
                else:
                    owner = '?'
                self._btrack[tid] = [x, y, vx, vy, btype, owner, t]
                self._bullet_miss[tid] = 0
            used.add(tid)
            rows.append({'tid': tid, 'x': x, 'y': y, 'vx': vx, 'vy': vy,
                         'btype': btype, 'owner': owner})
        # 本帧未出现的已跟踪子弹：连续 BULLET_LOST_FRAMES 帧未出现才判定消失
        for tid in list(self._btrack):
            if tid not in used:
                self._bullet_miss[tid] = self._bullet_miss.get(tid, 0) + 1
                if self._bullet_miss[tid] >= BULLET_LOST_FRAMES:
                    del self._btrack[tid]
                    del self._bullet_miss[tid]
        return rows

    def _dense_lookahead(self, me):
        """纯追踪前瞻点（备用模式）：密集路径上找距车最近的点，再沿路径前进
        LOOKAHEAD_PX 弧长取目标点。"""
        dense = getattr(self, 'dense_path', None)
        if not dense or len(dense) < 2:
            return None
        bx, by = me['x'], me['y']
        bi = 0
        bd = 1e18
        for i, p in enumerate(dense):
            d = (p[0] - bx) * (p[0] - bx) + (p[1] - by) * (p[1] - by)
            if d < bd:
                bd = d
                bi = i
        acc = 0.0
        for i in range(bi, len(dense) - 1):
            x1, y1 = dense[i]
            x2, y2 = dense[i + 1]
            seg = math.hypot(x2 - x1, y2 - y1)
            if acc + seg >= LOOKAHEAD_PX:
                u = (LOOKAHEAD_PX - acc) / seg if seg > 1e-6 else 0.0
                return (x1 + (x2 - x1) * u, y1 + (y2 - y1) * u)
            acc += seg
        return dense[-1]

    def _stanley_steer(self, me):
        """Stanley 横向控制器 + 曲率前馈（借鉴斯坦福 DARPA 挑战赛）：
        ω_des = FF·v·κ(s) + Kp·θe + Kd·dθe/dt − clamp(Kct·e_ct, ±CT_MAX)
        - κ(s)：最近密集点处的有符号局部曲率（16px 窗口三点圆拟合）。前馈让坦克
          在弯道处按路径曲率预打舵 —— 消除纯追踪的内切（L²/2R 级偏移）。
        - θe：路径切向与车头夹角（PD 项）；e_ct：到路径段的有符号横向距离（右正）。
        返回 ω_des（rad/s，>0 = 顺时针 = 右转）；密集路径缺失返回 None。"""
        dense = getattr(self, 'dense_path', None)
        if not dense or len(dense) < 2:
            return None
        n = len(dense)
        bx, by = me['x'], me['y']
        bi = 0
        bd = 1e18
        for i, p in enumerate(dense):
            d2 = (p[0] - bx) * (p[0] - bx) + (p[1] - by) * (p[1] - by)
            if d2 < bd:
                bd = d2
                bi = i
        # 切线：用 bi−1→bi+1 的 8px 跨度（最近点跳变时切线平滑，D 项不尖峰）
        j0 = max(0, bi - 1)
        j1 = min(n - 1, bi + 1)
        if j1 <= j0:
            return 0.0
        tx, ty = dense[j1][0] - dense[j0][0], dense[j1][1] - dense[j0][1]
        tl = math.hypot(tx, ty)
        if tl < 1e-6:
            return 0.0
        ux, uy = tx / tl, ty / tl
        tangent = math.atan2(ty, tx)
        # 航向误差 θe；D 项不对 θe 直接微分（PWM 量化的车头朝向会放大噪声）：
        # dθe/dt = 切线旋转率 − 上一帧实际施加的转向角速度（两者均无测量噪声）。
        th_e = wrap(tangent - me['angle'])
        tangent_rate = 0.0
        if self._tangent_prev is not None:
            tangent_rate = wrap(tangent - self._tangent_prev) / max(self._frame_dt, 1e-3)
        self._tangent_prev = tangent
        w_prev = self._omega_applied if self._omega_applied is not None else 0.0
        # 有符号横向误差：投影钳制到 8px 跨度段内，e_ct>0 = 右（顺时针）侧
        dx, dy = bx - dense[j0][0], by - dense[j0][1]
        proj = dx * ux + dy * uy
        proj = max(0.0, min(tl, proj))
        ex, ey = bx - (dense[j0][0] + ux * proj), by - (dense[j0][1] + uy * proj)
        e_ct = ux * ey - uy * ex
        # 局部有符号曲率：三点圆拟合（跨 16px），cross>0 = 顺时针弯 = κ>0
        kappa = 0.0
        i0, i1, i2 = bi, min(bi + 2, n - 1), min(bi + 4, n - 1)
        if i2 > i1 > i0:
            ax, ay = dense[i0]
            mx, my = dense[i1]
            cx, cy = dense[i2]
            abx, aby = mx - ax, my - ay
            bcx, bcy = cx - mx, cy - my
            ab = math.hypot(abx, aby)
            bc = math.hypot(bcx, bcy)
            ca = math.hypot(cx - ax, cy - ay)
            if min(ab, bc) > 0.5 and ab + bc + ca > 1e-6:
                cross = abx * bcy - aby * bcx
                area2 = abs(cross)
                if area2 > 1e-6:
                    r = ab * bc * ca / (2.0 * area2)
                    if 1.0 < r < 1000.0:
                        kappa = (1.0 / r) if cross > 0 else (-1.0 / r)
        v_est = self._v_smooth if self._v_smooth is not None else 0.0
        w_ff = STANLEY_FF_GAIN * v_est * kappa
        w_h = PID_KP_TH * th_e + PID_KD_TH * (tangent_rate - w_prev)
        # 坦克转向角速度恒定（不像汽车随车速变化）→ 横向项不做 ÷v 归一化，
        # 线性 + 限幅（原版 Stanley 的 atan(k·e/v) 在坦克上收敛过慢）。
        w_ct = max(-STANLEY_CT_MAX, min(STANLEY_CT_MAX, STANLEY_K_CT * e_ct))
        w_des = w_ff + w_h - w_ct
        return max(-STEER_SPEED_PX_S, min(STEER_SPEED_PX_S, w_des))

    def _corner_target_speed(self):
        """旧版曲率限速计算（保留给离线诊断，满速线上控制器不再调用）。

        沿密集路径（4px 采样，self.dense_path）在 60px 窗口内取
        min 曲率半径（8px 采样间隔降噪）→ 目标速度 v = min_r * STEER_SPEED
        （clamp 12..CORNER_SPEED）。直路返回 None。行驶转弯半径
        R=speed/STEER_SPEED，速度高于目标会外切。

        路点已稀疏到 50px：不能再拿 self.path 估曲率（60px 窗口内凑不齐
        3 个点，过弯永不减速）；必须用规划器输出的密集路径。"""
        dense = getattr(self, 'dense_path', None)
        if not dense or not self.path or self.wp_idx >= len(self.path) - 1:
            return None
        # 从当前目标路点对应的密集点开始（稀疏路点是密集路径的子集）
        tx, ty = self.path[self.wp_idx]
        start_i = 0
        best_d = 1e18
        for i, p in enumerate(dense):
            d = (p[0] - tx) * (p[0] - tx) + (p[1] - ty) * (p[1] - ty)
            if d < best_d:
                best_d = d
                start_i = i
        pts = [tuple(dense[start_i])]
        acc = 0.0
        for i in range(start_i + 2, len(dense), 2):   # 8px 间隔采样
            x, y = dense[i]
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
        # ×STANLEY_CORNER_MARGIN：留转向权威余量给反馈项（前馈 v·κ 不饱和）
        return max(CORNER_MIN_SPEED,
                   min(CORNER_SPEED, min_r * STEER_SPEED_PX_S * STANLEY_CORNER_MARGIN))

    def _signed_lat_error(self, me):
        """车到当前路径段的有符号横向距离（+ = 路径右侧）。"""
        i = max(self.wp_idx - 1, 0)
        j = min(i + 1, len(self.path) - 1)
        x1, y1 = self.path[i]
        x2, y2 = self.path[j]
        dx, dy = x2 - x1, y2 - y1
        seg = math.hypot(dx, dy)
        if seg < 1e-6:
            return 0.0
        ux, uy = dx / seg, dy / seg
        rx, ry = me['x'] - x1, me['y'] - y1
        return rx * uy - ry * ux

    def _steer_dir(self, err):
        """PWM 比例转向 + 切换冷却：返回 1=right / -1=left / 0=直行。

        分级：|err|>TURN_FULL_RAD 满舵；死区~满舵之间 50% 占空比（隔帧打舵，
        减少过冲与振荡）；死区内直行。换舵向后 0.35s 冷却期保持原方向。
        """
        a = abs(err)
        if a <= TURN_DEADBAND_RAD:
            return 0
        d = 1 if err > 0 else -1
        if self._last_steer_dir and d != self._last_steer_dir:
            if (self.last_state_t is not None and self._steer_flip_t is not None and
                    self.last_state_t - self._steer_flip_t < STEER_FLIP_COOL_S):
                d = self._last_steer_dir   # 冷却期保持原方向
            else:
                self._steer_flip_t = self.last_state_t
        # 中等误差：50% 占空比（隔帧打舵）
        if a <= TURN_FULL_RAD:
            self._pwm_phase = 1 - getattr(self, '_pwm_phase', 0)
            if self._pwm_phase == 1:
                return 0
        if d:
            self._last_steer_dir = d
        return d

    def _debug_overlay(self, me=None):
        """网页可视化：path / waypoint / 车体扫掠投影 + 瞄准光束预览（bot 坐标）。"""
        beam = []
        if IS_DATA_LOG and me is not None and self.raw_wall is not None:
            try:
                beam = [[round(px, 1), round(py, 1)] for px, py in
                        aim_beam(self.raw_wall, me)]
            except Exception:
                beam = []
        pts = list(self.path) if self.path else []
        if not pts:
            return {'path': [], 'waypoints': [], 'swept': [], 'aim_beam': beam}
        path_out = [[round(x, 1), round(y, 1)] for x, y in pts]
        # waypoint：直接画控制器实际追踪的路点（self.path 本身已按 WP_SPACING_PX 稀疏化）
        wps = [[round(x, 1), round(y, 1)] for x, y in pts]
        # swept：每 8px 采样点处车体四角（heading = 路径切线）
        swept = []
        for i in range(0, len(pts) - 1, 2):
            x, y = pts[i]
            j = min(i + 1, len(pts) - 1)
            h = math.atan2(pts[j][1] - y, pts[j][0] - x)
            corners = tank_footprint_corners(x, y, h)
            swept.append([[round(cx, 1), round(cy, 1)] for cx, cy in corners])
        return {'path': path_out, 'waypoints': wps, 'swept': swept, 'aim_beam': beam}

    def remaining_path_length(self, me):
        if not self.path or self.wp_idx >= len(self.path):
            return 0.0
        pts = [(me['x'], me['y'])] + list(self.path[self.wp_idx:])
        return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(pts, pts[1:]))

    def _point_at_arc(self, dense, target_len):
        """沿密集路径按弧长取点（线性插值）。"""
        acc = 0.0
        for a, b in zip(dense, dense[1:]):
            seg = math.hypot(b[0] - a[0], b[1] - a[1])
            if acc + seg >= target_len:
                u = (target_len - acc) / seg if seg > 0 else 0.0
                return (a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u)
            acc += seg
        return dense[-1]

    def _past_marker_at(self, dense, total, frac, ref_back, me):
        """单标记点判据：车是否已越过轨迹 frac 弧长标记点。

        标记点 A = 轨迹 frac 弧长处；参考点 B = A 之前 ref_back 弧长处；
        v1 = B−A（指向轨迹后方，近似 A 点反向切线）；v2 = 车中心 − A。
        cos(v1, v2) ≤ 0 ⟺ 夹角 ≥90° ⟺ 车在 A 点之后。退化时保守返回 True。
        """
        ax, ay = self._point_at_arc(dense, total * frac)
        bx, by = self._point_at_arc(dense, max(0.0, total * (frac - ref_back)))
        v1x, v1y = bx - ax, by - ay
        v2x, v2y = me['x'] - ax, me['y'] - ay
        denom = math.hypot(v1x, v1y) * math.hypot(v2x, v2y)
        if denom < 1e-9:
            return True
        return (v1x * v2x + v1y * v2y) / denom <= 0.0

    def past_window_marker(self, me):
        """弧长标记点判据（用户方法）：车是否已越过执行窗口轨迹的
        WINDOW_REPLAN_FRACTION（0.70）弧长标记点。

        主判据 = 70% 标记点夹角余弦 ≤0（车越过标记点 → 进入窗口后段）；
        若车偏离轨迹导致主判据失效（横向误差大时夹角判据失真），用 0.95
        兜底标记点 —— 车越过 0.95 弧长点（距终点只剩 5%）时几何上必然触发，
        确保窗口末端必然进入快速续接。剩余路径长度兜底判据在调用方另做。
        路径过短/标记点退化时保守返回 True（尽早进入快速续接）。
        """
        dense = self.dense_path
        if not dense or len(dense) < 2:
            return True
        total = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                    for a, b in zip(dense, dense[1:]))
        if total <= 0.0:
            return True
        if self._past_marker_at(dense, total,
                                WINDOW_REPLAN_FRACTION, WINDOW_REPLAN_REF_BACK, me):
            return True
        return self._past_marker_at(dense, total,
                                    WINDOW_REPLAN_TAIL_FRACTION,
                                    WINDOW_REPLAN_TAIL_REF_BACK, me)

    def replan(self, t, me, foe, local_window_due=False):
        if self.raw_wall is None or self.blocked is None or self.round_ended:
            return

        predicted_foe = (foe['x'], foe['y'])
        predicted_path = []
        if self.prediction_horizon_s > 0.0 and self.foe_predictor.initialized:
            predicted_path = self.foe_predictor.forecast_path(self.prediction_horizon_s)
            px, py = predicted_path[-1]
            predicted_foe = (
                min(MAP_PX - 2.0, max(2.0, px)),
                min(MAP_PX - 2.0, max(2.0, py)),
            )

        pr = plan(
            (me['x'], me['y']),
            predicted_foe,
            foe['angle'],
            self.raw_wall,
            self.blocked,
            preferred_rel_deg=self.current_relative_deg,
            preferred_target=self.current_tactical_target,
            blocked_turn=self.blocked_turn,
            free_dist=self.free_dist,
            tactical_mode=self.tactical_mode,
            me_heading=me['angle'],
        )
        self.last_plan_t = t

        active = bool(self.path and self.wp_idx < len(self.path))
        old_remaining = self.remaining_path_length(me) if active else 0.0
        accepted = False
        # 受阻强制换路标志：必须在外层门槛之前取出并生效 —— 车头被墙卡住时
        # 任何有效新路径都要接受（包括 40px 应急窗口），否则旧路径被钉死 → 死锁。
        force = self._force_plan_accept
        self._force_plan_accept = False

        if (pr.success and pr.path_validated and pr.path and
                (pr.path_length >= MIN_ACCEPT_PATH_PX or pr.window_goal_reached or force)):
            if force:
                accepted = True
            elif not active:
                accepted = True
            else:
                # PlanResult.target remains the global tactical target while
                # self.path[-1] is only the current receding-horizon endpoint.
                old_target = self.current_tactical_target or self.path[-1]
                target_shift = (math.hypot(pr.target[0]-old_target[0], pr.target[1]-old_target[1])
                                if pr.target is not None else 1e9)
                held = t - self.last_path_switch_t
                staging_to_attack = (self.current_plan_reason == 'staging_no_los' and
                                     pr.reason != 'staging_no_los')
                ratio = STAGING_TO_ATTACK_RATIO if staging_to_attack else NORMAL_SWITCH_RATIO

                # Similar target = harmless route refresh. Otherwise require a
                # competitive route, or eventually release a stale path.
                # 第二远策略（chase_alt）：目标相同（敌人在附近没跑）时——
                #   · 当前已在走绕行 → 只接受绕行（保持接触角优势，防 0.5s 级
                #     直连↔绕行来回切换）；
                #   · 当前在走直连 → 允许升级为绕行（新计划换更好接触角）；
                #   · 敌人位移超阈值（收不到 target_shift 分支）→ 绕行直接放行。
                if target_shift <= SIMILAR_TARGET_PX:
                    accepted = (self.current_path_source != 'chase_alt'
                                or pr.path_source == 'chase_alt')
                elif pr.path_source == 'chase_alt':
                    accepted = True
                elif pr.path_length <= max(35.0, old_remaining * ratio):
                    accepted = True
                elif held >= PATH_HOLD_MAX_S:
                    accepted = True
                elif local_window_due:
                    # 窗口尾段续接是保底动作：标记点触发后旧路径只剩 ~30%
                    # （48px≈0.38s），新窗口（160px）必然比剩余旧路径长，
                    # 按 ratio 比较永远被拒 → 车直走到窗口终点被迫停车。
                    # 尾段时接受任何有效新路径（接受后新窗口从车位置开始，
                    # 判据自然退出尾段，不会每周期换路径）。
                    accepted = True

            if accepted:
                # 短程方向粘滞：旧路径还有一小段可走时，拒绝与当前行进方向
                # 差 >90° 的新路径 —— 防路口等长双路 1s 级来回掉头。
                # 状态升级(staging→attack)与超时(PATH_HOLD_MAX_S)仍放行；
                # force 时跳过：受阻必须换向，粘滞会阻止掉头导致继续顶墙。
                # 窗口尾段续接（local_window_due）同样放行 —— 旧路径即将
                # 耗尽，掉头去新方向是续接的必然代价。
                if (not force and active and old_remaining > PATH_SWITCH_MIN_REMAINING_PX
                        and len(pr.path) > 1 and not staging_to_attack
                        and held < PATH_HOLD_MAX_S and not local_window_due):
                    cur_pt = self.path[self.wp_idx] if self.wp_idx < len(self.path) else None
                    if cur_pt is not None:
                        cur_dir = math.atan2(cur_pt[1] - me['y'], cur_pt[0] - me['x'])
                        new_dir = math.atan2(pr.path[1][1] - me['y'], pr.path[1][0] - me['x'])
                        if abs((new_dir - cur_dir + math.pi) % (2 * math.pi) - math.pi) > math.radians(90):
                            accepted = False
            if accepted:
                # 路点稀疏化：每 50px 保留一个路点（贪吃蛇目标间隔）
                sparse = [pr.path[0]]
                acc_sp = 0.0
                for p in pr.path[1:]:
                    acc_sp += math.hypot(p[0] - sparse[-1][0], p[1] - sparse[-1][1])
                    if acc_sp >= WP_SPACING_PX:
                        sparse.append(p)
                        acc_sp = 0.0
                if len(sparse) < 2 or sparse[-1] != pr.path[-1]:
                    sparse.append(pr.path[-1])
                self.path = sparse
                # 密集路径另存：Stanley 控制器用（曲率前馈/切向/横向误差）。
                self.dense_path = list(pr.path)
                self._tangent_prev = None
                self._omega_applied = None
                # 初始路点 = 第一个不被车身覆盖的路点（≥WP_FIRST_MIN_PX）：
                # 密集路点在车身上，转向时其方位角剧烈摆动 → err 不稳转圈
                self.wp_idx = 1 if len(self.path) > 1 else 0
                while (self.wp_idx < len(self.path) - 1 and
                       math.hypot(self.path[self.wp_idx][0] - me['x'],
                                  self.path[self.wp_idx][1] - me['y']) < WP_FIRST_MIN_PX):
                    self.wp_idx += 1
                self.current_plan_reason = pr.reason
                self.current_path_source = pr.path_source
                # Hold-position results have a continuous relative angle. Keep
                # the previous discrete tactical side as the switch preference.
                if pr.reason != 'hold_attack_position' and pr.relative_deg is not None:
                    self.current_relative_deg = pr.relative_deg
                if pr.target is not None:
                    self.current_tactical_target = pr.target
                self.current_window_goal_reached = pr.window_goal_reached
                # 兜底判据阈值：弧长标记点（70%，含 0.95 兜底）为主判据，剩余
                # 路径长度只在车偏离轨迹导致弧长判据不可靠时兜底。
                self.current_window_replan_remaining = WINDOW_REPLAN_TAIL_REMAINING_PX
                # 起步段弧长阈值（方案二，动态）：随执行窗口长度变化，
                # 长窗口起步段长、短窗口自动缩短（ALIGN 起步判定用）。
                self.start_align_arc_px = max(
                    START_ALIGN_ARC_MIN_PX,
                    pr.window_target_length * START_ALIGN_WINDOW_FRACTION,
                )
                # 新路径 = 新起步段：重置一次性对准状态
                self._start_aligned = False
                self._align_started_at = None
                self.last_path_switch_t = t
        else:
            # Do not throw away a still-useful path just because one 1 Hz OODA
            # update has no legal rear candidate. The next cycle may recover.
            if not active:
                self.path = []
                self.dense_path = []
                self._tangent_prev = None
                self._omega_applied = None
                self.wp_idx = 0
                self.current_plan_reason = ''
                self.current_path_source = None       # 当前执行路线的来源（chase / chase_alt）
                self.current_relative_deg = None
                self.current_tactical_target = None
                self.current_window_goal_reached = False
                self.current_window_replan_remaining = WINDOW_REPLAN_REMAINING_PX

        prow = {
            't':round(t,3),
            'success':int(pr.success),
            'accepted':int(accepted),
            'tactical_mode':pr.tactical_mode,
            'target_type':pr.target_type,
            'relative_deg':'' if pr.relative_deg is None else pr.relative_deg,
            'target_x':'' if pr.target is None else round(pr.target[0],2),
            'target_y':'' if pr.target is None else round(pr.target[1],2),
            'path_length_px':round(pr.path_length,3),
            'old_remaining_px':round(old_remaining,3),
            'smoothness_rad':round(pr.smoothness_rad,4),
            'planning_ms':round(pr.planning_ms,3),
            'reason':pr.reason,
            'path_points':json.dumps([[round(x,1),round(y,1)] for x,y in pr.path]),
            'candidate_count':pr.candidate_count,
            'reachable_candidates':pr.reachable_candidates,
            'rear_candidates':pr.rear_candidates,
            'rear_reachable_candidates':pr.rear_reachable_candidates,
            'reachable_candidate_rate':round(pr.reachable_candidates / max(1, pr.candidate_count), 4),
            'line_of_sight':int(pr.line_of_sight),
            'open_directions':pr.open_directions,
            'tactical_score':round(pr.tactical_score, 3),
            'switch_cost':round(pr.switch_cost, 3),
            'path_source':pr.path_source,
            'path_validated':int(pr.path_validated),
            'validation_reason':pr.validation_reason,
            'raw_path_points':pr.raw_path_points,
            'executed_path_points':pr.executed_path_points,
            'global_path_length_px':round(pr.global_path_length, 3),
            'window_path_length_px':round(pr.window_path_length, 3),
            'window_target_length_px':round(pr.window_target_length, 3),
            'window_lookahead_length_px':round(pr.window_lookahead_length, 3),
            'window_goal_reached':int(pr.window_goal_reached),
            'local_window_due':int(local_window_due),
            'replan_period_s':round(LOCAL_WINDOW_RETRY_S if local_window_due else OODA_PERIOD_S, 3),
            'prediction_horizon_s':round(self.prediction_horizon_s, 3),
            'observed_foe_x':round(foe['x'], 3),
            'observed_foe_y':round(foe['y'], 3),
            'predicted_foe_x':round(predicted_foe[0], 3),
            'predicted_foe_y':round(predicted_foe[1], 3),
            'predicted_foe_path':json.dumps(
                [[round(x, 2), round(y, 2)] for x, y in predicted_path]
            ),
            'wall': round(time.time(), 2) if IS_DATA_LOG else '',
        }
        self.pw.writerow(prow)
        self.lpw.writerow(prow)    # 同步写"当前轮"plans_latest.csv
        if self.pw_rec is not None:
            self.pw_rec.writerow(prow)   # 按局归档（不覆盖）
        # plans 落盘节流：原每次 replan(0.5s) 都 flush 2 个文件；改为每 4 次
        # （≈2s）一次，减少磁盘触碰频率（磁盘写是单线程 bot 的卡顿源）
        self._plans_flush_cnt += 1
        if self._plans_flush_cnt >= 4:
            self._plans_flush_cnt = 0
            self.pf.flush()
            self.lpf.flush()
            if self.pf_rec is not None:
                self.pf_rec.flush()

    def _spin_action(self, me):
        """原地旋转（无路线时）：方向先看我方朝向与"我方→敌方"连线的夹角 θ
        （θ = wrap(连线方位角 − 我方朝向)，与主转向通道同约定：θ>0 = 顺时针 = 右转）。
        sinθ > 阈值 → 敌在顺时针侧（右侧）→ 右转；sinθ < -阈值 → 左转；
        敌在正后方（|θ|≈180°）→ 默认右转；正对敌（|θ| 小）或敌方位置未知
        → 保持上次方向连续旋转。"""
        d = self._spin_dir
        foe = self.last_foe
        if foe is not None:
            bear = math.atan2(foe['y'] - me['y'], foe['x'] - me['x'])
            th = wrap(bear - me['angle'])
            if abs(th) > math.pi - SPIN_BEHIND_RAD:
                # 敌在正后方：左右距离相等，默认右转（用户指定）
                d = 'right'
            elif math.sin(th) > SPIN_SIDE_SIN_THRESHOLD:
                d = 'right'
            elif math.sin(th) < -SPIN_SIDE_SIN_THRESHOLD:
                d = 'left'
            # 正对敌：保持上次方向
        self._spin_dir = d
        k = {'up': 0, 'down': 0, 'left': 1 if d == 'left' else 0,
             'right': 1 if d == 'right' else 0}
        return {'keys': k, 'fire': 0}

    def _head_clear(self, me, ahead_px=None):
        """车头前方碰撞检测（防路径过期仍全速直行顶墙）。

        沿车头朝向，在「车头前沿(半长) → 半长+ahead_px」范围内、按车体横向
        半宽扫 5 列采样点，任一点落在墙内或出界即判定前方有墙。
        True=安全可直行；False=前方有墙应急停。
        """
        raw = self.raw_wall
        if raw is None:
            return True
        if ahead_px is None:
            ahead_px = HEAD_BLOCK_AHEAD_PX
        ca, sa = math.cos(me['angle']), math.sin(me['angle'])
        sx, sy = -sa, ca          # 车头横向单位向量
        h, w = raw.shape
        hl, hw = TANK_HALF_LENGTH_PX, TANK_HALF_WIDTH_PX
        d = hl + 2.0
        stop = hl + ahead_px
        while d <= stop + 1e-9:
            for lat in (-hw, -hw * 0.5, 0.0, hw * 0.5, hw):
                px = me['x'] + ca * d + sx * lat
                py = me['y'] + sa * d + sy * lat
                ix, iy = int(round(px)), int(round(py))
                if not (0 <= ix < w and 0 <= iy < h):
                    return False
                if raw[iy, ix]:
                    return False
            d += 4.0
        return True

    def action(self, me):
        k = {'up':0,'down':0,'left':0,'right':0}
        self.controller_mode = 'STOP'
        # 局间/无图保持停止；局内无合理路线才原地旋转
        if self.round_ended:
            return {'keys':k, 'fire':0}
        if not self.path:
            # 无合理路线：原地旋转观察（每个 OODA 周期重规划，路径出现即恢复行驶）。
            self.controller_mode = 'SPIN_NO_PATH'
            return self._spin_action(me)
        if self.wp_idx >= len(self.path):
            # 轨迹已执行完毕（到达规划终点）：停车等待续接，不盲目前进
            # （尾段判据会以 LOCAL_WINDOW_RETRY_S 快速续接）。
            self.controller_mode = 'PATH_DONE_STOP'
            return self.stop_action()

        while self.wp_idx < len(self.path):
            tx, ty = self.path[self.wp_idx]
            if math.hypot(tx - me['x'], ty - me['y']) <= WAYPOINT_REACHED_PX:
                self.wp_idx += 1
            else:
                break
        if self.wp_idx >= len(self.path):
            # 窗口走完、新规划未到：停车等待（尾段判据以快速周期续接），
            # 不盲目前进。replan 失败清空路径后自然转入 SPIN_NO_PATH 观察。
            self.controller_mode = 'PATH_DONE_STOP'
            return self.stop_action()

        # ---- 起步航向对准：车仍处于"起步段"（dense 上距起点弧长 ≤ 动态阈值）
        # 且近停时，先把车头原地转到与路径切线一致再踩油门 —— 避免车头歪着
        # 起步走弧线（开局直冲墙角/续接跑偏）。切线取密集路径上距车最近点的
        # 前方段（起步段内最近点≈入口，等价于"入口切线"）。旋转方向按最短角：
        # err = wrap(切线 − 车头)，err>0 → right（角度增大=顺时针），err<0 →
        # left —— 与"车头向量×切线向量叉积"符号判定等价（自动选方便侧）。
        # 起步段判定用"到路径起点的弧长"而不用 wp_idx：wp_idx 受路点稀疏/跳过
        # 逻辑影响会从 1 漂到 2+（开局短路径下 ALIGN 永不触发 → 开局直冲根因）。
        # 速度判定用位置差分（自维护上一帧位置），不用桥上报的 vx/vy —— 桥速度
        # 是位置差分、噪声可达几百 px/s，起步瞬间就把"近停"条件破坏。
        dense = getattr(self, 'dense_path', None)
        _spd = 0.0
        _prev = getattr(self, '_align_prev_pos', None)
        if _prev is not None and self._frame_dt:
            _spd = math.hypot(me['x'] - _prev[0], me['y'] - _prev[1]) / self._frame_dt
        self._align_prev_pos = (me['x'], me['y'])
        if dense and len(dense) >= 2 and _spd < START_ALIGN_SPEED_PX_S:
            bx, by = me['x'], me['y']
            # 最近点 + 该点距路径起点的弧长（单趟扫描，dense ~百点级）
            bi = 0
            bd = 1e18
            arc = 0.0
            arc_at_bi = 0.0
            pprev = dense[0]
            for i, p in enumerate(dense):
                if i > 0:
                    arc += math.hypot(p[0] - pprev[0], p[1] - pprev[1])
                d2 = (p[0] - bx) ** 2 + (p[1] - by) ** 2
                if d2 < bd:
                    bd = d2
                    bi = i
                    arc_at_bi = arc
                pprev = p
            if arc_at_bi <= getattr(self, 'start_align_arc_px', START_ALIGN_ARC_MIN_PX):
                j0 = max(0, bi - 1)
                j1 = min(len(dense) - 1, bi + 1)
                tang = math.atan2(dense[j1][1] - dense[j0][1], dense[j1][0] - dense[j0][0])
                err_a = wrap(tang - me['angle'])
                if abs(err_a) <= math.radians(START_ALIGN_DEG):
                    # 已对准：本段路径起步对准完成，之后不再回 ALIGN
                    self._start_aligned = True
                else:
                    # 一次性起步对准：转 START_ALIGN_MAX_S 仍未对准 → 放弃转正，
                    # 直接放行油门（Stanley/blocked 脱困接管），消灭无限瞎转。
                    t_now0 = self.last_state_t or 0.0
                    _ast = getattr(self, '_align_started_at', None)
                    if _ast is None:
                        _ast = t_now0
                        self._align_started_at = t_now0
                    if (not getattr(self, '_start_aligned', False)
                            and (t_now0 - _ast) < START_ALIGN_MAX_S):
                        if err_a > 0:
                            k['right'] = 1
                        else:
                            k['left'] = 1
                        k['up'] = 0
                        self.controller_mode = 'ALIGN_START'
                        return {'keys': k, 'fire': 0}

        # ---- 前方碰撞急停：车头快贴墙时松油门，避免顶墙直行 ----
        # 只松油门、不清路径、不强制重规划：路径由常规重规划自然纠正，
        # 避免频繁清路径导致可视化轨迹闪动 + 坦克愣住。
        # 同时标记下一次重规划强制接受新路径（覆盖方向粘滞，让车能掉头绕墙）。
        blocked_ahead = not self._head_clear(me)
        if blocked_ahead:
            self._force_plan_accept = True
            # ---- 卡死脱困：持续受阻且几乎无位移 → 顺时针原地旋转找出口 ----
            # 车头对墙时 Stanley 对（过期/朝墙的）路径输出≈0，车既不前进也不转，
            # 会无限死锁。受阻超过 BLOCKED_ROTATE_DELAY_S 且没挪动 → 持续按右转
            # （顺时针），车头扫过开口方向（_head_clear 变 True）后自然恢复正常。
            t_now = self.last_state_t or 0.0
            if self._blocked_since is None:
                self._blocked_since = t_now
                self._blocked_pos = (me['x'], me['y'])
            stuck_move = math.hypot(me['x'] - self._blocked_pos[0],
                                    me['y'] - self._blocked_pos[1])
            if (t_now - self._blocked_since >= BLOCKED_ROTATE_DELAY_S
                    and stuck_move <= BLOCKED_ROTATE_MOVE_PX):
                k['right'] = 1          # 顺时针（角度增大方向 = 右转）
                k['up'] = 0
                self.controller_mode = 'BLOCKED_ROTATE'
                return {'keys': k, 'fire': 0}
        else:
            self._blocked_since = None
            self._blocked_pos = None

        # Stanley 横向控制（曲率前馈 + 航向 PD + 横向误差 atan 项）→ ω_des（rad/s）。
        # 占空比 PWM：duty = |ω_des|/3.8（转向时间占比）；ω>0 = 顺时针 = 右转。
        w_des = self._stanley_steer(me)
        if w_des is None:
            # 密集路径缺失：退回瞄路点方位角（旧贪吃蛇）
            tx, ty = self.path[self.wp_idx]
            desired = math.atan2(ty - me['y'], tx - me['x'])
            w_des = PID_KP_TH * wrap(desired - me['angle'])
        duty = min(1.0, abs(w_des) / STEER_SPEED_PX_S)
        self._steer_accum = getattr(self, '_steer_accum', 0.0) + duty
        steer_on = False
        if self._steer_accum >= 1.0:
            self._steer_accum -= 1.0
            steer_on = True
        if steer_on:
            if w_des > 0:
                k['right'] = 1
            else:
                k['left'] = 1
        # 记录本帧实际施加的转向角速度（供 D 项参考微分用，无测量噪声）
        self._omega_applied = (STEER_SPEED_PX_S if steer_on else 0.0) * (1 if w_des > 0 else -1)

        # x 通道（纵向）：与敌方使用相同的游戏物理最高速度。
        # 有已验证路径时持续按住前进键，取消直路 65px/s 与弯道 12~60px/s
        # 的主动限速；实际速度由游戏 Car.maxspeed（约 125px/s）统一封顶。
        # 速度测量仍做平滑，仅供 Stanley 曲率前馈使用，不参与油门开关。
        speed_now = math.hypot(me.get('vx', 0.0), me.get('vy', 0.0))
        if getattr(self, '_v_smooth', None) is None:
            self._v_smooth = speed_now
        else:
            self._v_smooth += SPEED_MEAS_CORR * (speed_now - self._v_smooth)
        self._throttle_on = not blocked_ahead
        k['up'] = 0 if blocked_ahead else 1

        self.controller_mode = 'HEAD_BLOCKED_STOP' if blocked_ahead else 'FULL_SPEED_PATH'
        return {'keys':k, 'fire':0}

    def on_message(self, c, text):
        """统一异常护栏：任何单帧处理错误都不能杀死连接线程/关闭文件。"""
        try:
            return self._on_message(c, text)
        except Exception:
            import traceback
            traceback.print_exc()
            print('on_message crashed (frame dropped, connection kept alive)', flush=True)

    def _on_message(self, c, text):
        try:
            msg = json.loads(text)
        except Exception:
            return
        if not isinstance(msg, dict):
            return

        # 文件覆盖信号（launcher 收到桥的新局事件后覆盖写 round_reset.csv）：
        # 必须在 save_map/事件处理之前 —— 覆盖 = 新轮，先清理旧数据，
        # 本帧随后即可锁新图/重规划（进程永不重启）。
        sig = self._read_round_sig()
        if sig is not None and sig != self._last_sig:
            self._last_sig = sig
            self._on_round_signal(sig)

        # Keep first maze only. A later different maze is a hard round boundary.
        if msg.get('map'):
            self.save_map(msg['map'])
        if self.round_ended:
            # 网页模式连续对局：round 结束后若又收到完整 me+foe 状态帧
            # （新局已开始：同迷宫新局 / 回菜单再进），则自动复位继续导航。
            # 迷宫变化的新局已由 save_map 走 reset_for_new_round 处理。
            if msg.get('me') and msg.get('foe'):
                self.reset_for_new_round('state_resumed')
                # 同帧补锁新局地图：reset 清空了旧图；而上方 save_map 对
                # "同迷宫新局"会因签名相同提前返回 → 本帧地图被白白丢弃，
                # 下一帧才重新锁图。这里立即用本帧 map 锁图，消除新局首帧
                # 无图窗口（WAIT_MAP 停摆 / 短暂沿用旧图）。
                if msg.get('map'):
                    self.save_map(msg['map'])
            else:
                c.send_text(json.dumps(self.stop_action(), separators=(',',':')))
            return

        me = self.map_coords(msg.get('me'))
        foe = self.map_coords(msg.get('foe'))

        # Foe disappearance is visible to Python and means the current round ended.
        if not me or not foe:
            if self.seen_both:
                self.finish_round('tank_missing')
            c.send_text(json.dumps(self.stop_action(), separators=(',',':')))
            return

        t = float(msg.get('t', 0.0))

        # 桥端死亡/消失事件（显式信号，优先于动画名猜测；可逗号分隔多个）
        evts = set(str(msg.get('event') or '').split(','))
        # 新局同步信号：me 实例 uid 变化 / 布局切换 —— 立即结束旧局、
        # 清空旧地图与路径（否则新局用旧地图旧位置规划 → 乱走）。
        # round_no 在这里递增（客观新局信号：新局 me 实例必重建）；me_death /
        # foe_death 等只是旧局结束，不产生新号（防 finish 误判虚增）。
        if self.seen_both and ('me_respawn' in evts or 'layout_changed' in evts or 'me_teleport' in evts):
            self.finish_round('bridge_event(new_round)')
            self._inc_round()          # 开新局号（2s 防抖合并 maze_changed）
            c.send_text(json.dumps(self.stop_action(), separators=(',',':')))
            return
        if self.seen_both and ('me_death_anim' in evts or 'me_vanish' in evts):
            self.finish_round('bridge_event(me_death)')
            c.send_text(json.dumps(self.stop_action(), separators=(',',':')))
            return
        if self.seen_both and ('foe_death_anim' in evts or 'foe_vanish' in evts):
            self.finish_round('bridge_event(foe_death)')
            c.send_text(json.dumps(self.stop_action(), separators=(',',':')))
            return

        # 爆炸信号 = 本局即将结束（真实浏览器击杀必定触发坦克爆炸动画，
        # 随后下一局必然开始）。收到即结束本轮、清空规划与地图，等待新局。
        # 兼容多个爆炸/销毁相关动画名（explosion/boom/explode/destroy/death）。
        foe_anim = ''
        if msg.get('foe') and isinstance(msg['foe'], dict):
            foe_anim = str(msg['foe'].get('anim') or '').lower()
        # 精确匹配死亡动画：'deathray'(死亡射线武器) 只是敌人开火动画，
        # 不是死亡 —— 必须用词边界判断，防止误判结束本轮。
        EXPLODE_RE = re.compile(r'(death$|explo|boom|blast|destroy|^dead)', re.I)
        if foe_anim and EXPLODE_RE.search(foe_anim):
            self.finish_round('foe_explosion_anim(%s)' % foe_anim)
            c.send_text(json.dumps(self.stop_action(), separators=(',',':')))
            return

        # Player death can produce no bridge frames until the next respawn. The first
        # frame of the next round must NOT be appended to the old CSV.
        if self.last_state_t is not None:
            dt = t - self.last_state_t
            if dt > ROUND_GAP_S:
                # 帧间隔过大有两种成因：
                #  1) 真轮次切换（新局位置必然瞬移 → 位置跳变判定）
                #  2) 桥/渲染停顿（jsdom 无头 8x 下迷宫生成/加载会卡 5s+，
                #     位置连续 → 不结束轮次，错过期间直接续上，继续执行）
                if (self.last_me_pos is not None and self.last_foe_pos is not None):
                    dm = math.hypot(me['x'] - self.last_me_pos[0], me['y'] - self.last_me_pos[1])
                    df = math.hypot(foe['x'] - self.last_foe_pos[0], foe['y'] - self.last_foe_pos[1])
                    if dm > ROUND_JUMP_PX or df > ROUND_JUMP_PX:
                        self.finish_round('state_gap_%.2fs+jump me=%.1f foe=%.1f' % (dt, dm, df))
                        c.send_text(json.dumps(self.stop_action(), separators=(',',':')))
                        return
                # 位置连续：桥停顿，原地续上（不重置轮次）
                print('frame stall %.2fs (positions continuous), resuming' % dt, flush=True)
            elif dt > 0 and self.last_me_pos is not None and self.last_foe_pos is not None:
                dm = math.hypot(me['x'] - self.last_me_pos[0], me['y'] - self.last_me_pos[1])
                df = math.hypot(foe['x'] - self.last_foe_pos[0], foe['y'] - self.last_foe_pos[1])
                if dm > ROUND_JUMP_PX or df > ROUND_JUMP_PX:
                    self.finish_round('position_jump me=%.1f foe=%.1f' % (dm, df))
                    c.send_text(json.dumps(self.stop_action(), separators=(',',':')))
                    return

        self.seen_both = True
        self._frame_dt = (t - self.last_state_t) if self.last_state_t is not None else (1.0 / 66.0)
        self._frame_dt = max(0.005, min(0.1, self._frame_dt))
        self.last_state_t = t
        self.last_me_pos = (me['x'], me['y'])
        self.last_foe_pos = (foe['x'], foe['y'])
        self.last_me = dict(me)
        self.last_foe = dict(foe)
        self.last_foe_anim = foe_anim
        self.last_bullets = msg.get('bullets') or []
        self.me_predictor.update(t, me['x'], me['y'], me['vx'], me['vy'])
        self.foe_predictor.update(t, foe['x'], foe['y'], foe['vx'], foe['vy'])
        # 惰性文件：任何帧处理前确保本轮 track/plans 文件已创建
        # （replan 写 plans 先于 record，缺此会 pw=None 崩溃丢帧）
        self._open_round_files()

        # 地图未就绪时绝不移动：开局或跨轮复位后，save_map 可能尚未锁图
        # （桥的 map 每 3s 才重发一次）。在锁图前强制停止，避免"开局直冲、
        # 不管障碍"。锁图后首个 OODA 自然接管。
        if self.raw_wall is None or self.blocked is None or not self.maze_written:
            self.controller_mode = 'WAIT_MAP'
            action = self.stop_action()
            self.record(t, me, foe, msg, action)
            c.send_text(json.dumps(action, separators=(',',':')))
            return

        remaining = self.remaining_path_length(me)
        # 窗口尾段判据：弧长标记点法为主（车越过轨迹 70% 弧长标记点，0.95
        # 兜底），剩余路径长度兜底（车被弹开/偏离轨迹时弧长判据不可靠）。
        past_marker = self.past_window_marker(me)
        local_window_due = (not self.current_window_goal_reached and
                            (past_marker or remaining <= self.current_window_replan_remaining))
        replan_period = LOCAL_WINDOW_RETRY_S if local_window_due else OODA_PERIOD_S
        if t - self.last_plan_t >= replan_period:
            self.replan(t, me, foe, local_window_due=local_window_due)
        elif local_window_due and not self._tail_due_last:
            # 刚进入尾段（上升沿）：立即续接一次，不等周期 —— 标记点触发时
            # 剩余窗口只有 (1-0.7)*160≈48px≈0.38s 行程，必须抢占这段缓冲，
            # 否则车走到终点后新路径还没规划出来（停车等下一周期）。
            self.replan(t, me, foe, local_window_due=True)
        self._tail_due_last = bool(local_window_due)
        action = self.action(me)
        # ---- 自动瞄准射击判定层（独立模块，不修改上方任何导航逻辑）----
        # combat.attack() 每帧求解：从炮口出发的直瞄/反射弹道能否命中敌人
        # 预测位置。返回 lock=1（有可行射击角）或 lock=0（无）。
        #   lock=1 → 暂停导航，进入射击模式：停车（up/down=0）+ 按 combat
        #            的转向键瞄准 + 开火；桥端 lock=1 每帧强制同步键状态。
        #   lock=0 → 保持上方导航 action 原样（fire=0）。
        combat_act = self.combat.attack(t, me, foe, self.last_bullets, self.raw_wall) if self.combat_enabled else {'lock': 0}
        if combat_act.get('lock'):
            action = {
                'keys': {'up': 0, 'down': 0,
                         'left': combat_act['keys'].get('left', 0),
                         'right': combat_act['keys'].get('right', 0)},
                'fire': combat_act.get('fire', 0),
                'lock': 1,
            }
            self.controller_mode = 'COMBAT_LOCK'
        else:
            action = dict(action)
            action['lock'] = 0
        if self.visualize:
            action = dict(action)
            action['debug'] = self._debug_overlay(me)
        self.record(t, me, foe, msg, action)
        c.send_text(json.dumps(action, separators=(',',':')))

    def serve(self):
        ws = self.WSServer('127.0.0.1', self.port, self.on_client)
        print('navigation bot ws://127.0.0.1:%d/ai' % self.port, flush=True)
        try:
            ws.serve_forever()
        finally:
            self.close()

    def close(self):
        for obj in (getattr(self,'f',None), getattr(self,'pf',None),
                    getattr(self,'lf',None), getattr(self,'lpf',None),
                    getattr(self,'ff',None), getattr(self,'bf',None),
                    getattr(self,'f_rec',None), getattr(self,'pf_rec',None)):
            try:
                obj.flush()
                obj.close()
            except Exception:
                pass


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8766)
    ap.add_argument('--out-dir', default=os.path.join(HERE, 'nav_logs'))
    ap.add_argument('--repo', default=os.path.abspath(os.path.join(HERE, '..')))
    ap.add_argument('--tactical-mode', choices=[TACTICAL_MODE_REAR_ONLY, TACTICAL_MODE_V1, TACTICAL_MODE_CHASE],
                    default=DEFAULT_TACTICAL_MODE,
                    help='rear_only keeps the old baseline; tactical_v1 enables 360-degree scoring; chase targets foe position directly')
    ap.add_argument('--prediction-horizon-s', type=float, default=DEFAULT_PREDICTION_HORIZON_S,
                    help='constant-velocity Kalman foe forecast; set 0 for the P1E baseline')
    ap.add_argument('--exit-on-round-end', action='store_true',
                    help='terminate the bot process when a round ends (headless one-round-per-match mode)')
    ap.add_argument('--visualize', action='store_true',
                    help='send path/waypoint/swept-footprint overlay data to the bridge (web overlay)')
    ap.add_argument('--no-combat', action='store_true',
                    help='disable auto-aim shooting gate (pure navigation; used by headless benchmark)')
    a = ap.parse_args()
    bot = Bot(a.port, a.out_dir, a.repo, a.tactical_mode, a.prediction_horizon_s)
    bot.exit_on_round_end = a.exit_on_round_end
    bot.visualize = a.visualize
    bot.combat_enabled = not a.no_combat
    if bot.combat_enabled:
        print('auto-aim shooting gate ENABLED（导航 + 可行射击角时暂停导航瞄准开火）', flush=True)
    # 孤儿进程防护：bot 通常由 launcher.py spawn（带自动重启）。launcher 被
    # Ctrl+C 走 finally 会 terminate bot；但被直接关窗/强杀时 finally 不执行
    # → bot 成孤儿进程一直跑（僵尸损耗资源）。bot 自监控父进程，父消失即
    # 自杀 —— 无论 launcher 怎么死都兜底。直接命令行跑时父进程是控制台，
    # 控制台关闭 bot 同样跟随退出（合理）。
    try:
        import ctypes
        import threading as _th
        _k32 = ctypes.windll.kernel32
        _ppid = os.getppid()

        def _watch_parent():
            # PROCESS_QUERY_LIMITED_INFORMATION=0x1000：进程存在可打开；
            # OpenProcess 失败时 GetLastError=5(拒绝访问)=进程存在但权限不足
            # → 父活着；=87(参数无效)=pid 不存在 → 父已死 → 自杀。
            while True:
                h = _k32.OpenProcess(0x1000, False, _ppid)
                if not h:
                    if _k32.GetLastError() != 5:
                        os._exit(0)
                else:
                    _k32.CloseHandle(h)
                time.sleep(1.0)

        _th.Thread(target=_watch_parent, daemon=True).start()
    except Exception:
        pass   # 非 Windows/无 ctypes：跳过监控，不影响运行
    # 孤儿进程防护（主）：launcher.py 会在导航日志目录写 .launcher_hb 心跳文件
    # （每秒刷新）。launcher 被关窗/强杀时 atexit/finally 都不执行 → bot 靠
    # 心跳停更检测到 launcher 已死并自退（4 秒未更新即自杀）。Windows 的
    # OpenProcess 父进程探测在"父→子"场景不可靠（父进程对象被滞留，终止后
    # 长时间仍可打开）——心跳文件绕开该问题。直接命令行跑（无 launcher）时
    # 心跳文件不存在 → 不监控、不自杀。
    try:
        import threading as _th2
        # launcher 的心跳写在会话日志目录（= bot 的 out_dir）内
        _hb = os.path.join(os.path.abspath(a.out_dir), '.launcher_hb')

        def _hb_watch():
            while True:
                try:
                    stale = (os.path.exists(_hb)
                             and time.time() - os.path.getmtime(_hb) > 4.0)
                except Exception:
                    stale = False
                if stale:
                    try:
                        print('launcher heartbeat stale, bot exiting', flush=True)
                    except Exception:
                        pass   # launcher 已死 → stdout 管道可能已断，忽略
                    os._exit(0)   # 必须在 try 外：print 异常不能吞掉自杀
                time.sleep(1.0)

        _th2.Thread(target=_hb_watch, daemon=True).start()
    except Exception:
        pass
    bot.serve()
