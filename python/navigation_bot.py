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
    build_maps, load_polys, plan, swept_rectangle_path_clear,
    tank_footprint_corners,
)
from kalman_path_predictor import ConstantVelocityKalman2D

# 全部可调参数集中在 bot_config.py（路点间距 / OODA / PID / 速度 / 旋转 / 滞回等）
from bot_config import *   # noqa: F401,F403

CSV_COLUMNS = [
    't','me_x','me_y','me_angle','me_vx','me_vy',
    'foe_x','foe_y','foe_angle','foe_vx','foe_vy','n_bullets','n_powerups',
    'cmd_up','cmd_down','cmd_left','cmd_right','controller_mode'
]
PLANS_COLUMNS = [
    't','success','accepted','tactical_mode','target_type','relative_deg','target_x','target_y',
    'path_length_px','old_remaining_px','smoothness_rad','planning_ms','reason','path_points',
    'candidate_count','reachable_candidates','rear_candidates','rear_reachable_candidates',
    'reachable_candidate_rate','line_of_sight','open_directions','tactical_score','switch_cost',
    'path_source','path_validated','validation_reason','raw_path_points','executed_path_points',
    'global_path_length_px','window_path_length_px','window_target_length_px',
    'window_lookahead_length_px','window_goal_reached',
    'prediction_horizon_s','observed_foe_x','observed_foe_y',
    'predicted_foe_x','predicted_foe_y','predicted_foe_path'
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
        self.round_no = 1
        self.track_path = os.path.join(out_dir, 'track.csv')
        self.maze_path = os.path.join(out_dir, 'maze.csv')
        self.plan_path = os.path.join(out_dir, 'plans.csv')

        # 惰性创建：第一帧到达才写文件，避免 0 行空文件噪声
        self.f = None
        self.w = None
        self.pf = None
        self.pw = None
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
        self.path = []
        self.dense_path = []
        self.wp_idx = 0
        self.current_plan_reason = ''
        self.current_relative_deg = None
        self.current_tactical_target = None
        self.current_window_goal_reached = False
        self.current_window_replan_remaining = WINDOW_REPLAN_REMAINING_PX
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

        self.seen_both = False
        self.round_ended = False
        self.round_end_reason = ''
        self.last_state_t = None
        self.last_me_pos = None
        self.last_foe_pos = None
        self.me_predictor = ConstantVelocityKalman2D()
        self.foe_predictor = ConstantVelocityKalman2D()

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
        return {'keys': {'up':0,'down':0,'left':0,'right':0}, 'fire':0}

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
        self.current_relative_deg = None
        self.current_tactical_target = None
        self.current_window_goal_reached = False
        self.current_window_replan_remaining = WINDOW_REPLAN_REMAINING_PX
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
            los = self._los_clear_between(
                (me.get('x'), me.get('y')), (foe.get('x'), foe.get('y')), self.raw_wall)
            # 最后两帧速度估算
            last_speed = None
            if len(self._rec_tail) >= 2:
                (t0, x0, y0), (t1, x1, y1) = self._rec_tail[-2], self._rec_tail[-1]
                if t1 > t0:
                    last_speed = math.hypot(x1 - x0, y1 - y0) / (t1 - t0)
            header = ['round_id','t_end','t_start','reason','me_end_x','me_end_y','me_end_angle',
                      'foe_end_x','foe_end_y','foe_end_angle','foe_anim','me_round_dist_px',
                      'me_round_frames','me_last_speed_px_s','me_foe_los','maze_wall_count',
                      'track_file']
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
                ])
        except Exception:
            import traceback
            traceback.print_exc()

    def reset_for_new_round(self, sig_reason):
        """网页模式连续对局：一轮结束后自动复位，等待下一轮开始。

        复位状态但**保留** map 相关（raw_wall/blocked/maze_signature/polys），
        因为 save_map 只锁定"第一张有效迷宫"，下一局迷宫不同时会走
        maze_changed_new_round 分支重新处理（见 save_map）。
        """
        self.round_ended = False
        self.round_end_reason = ''
        self.seen_both = False
        self.last_state_t = None
        self.last_me_pos = None
        self.last_foe_pos = None
        # 关键修复：跨轮复位必须清空上局地图，否则下一局会用旧迷宫规划，
        # 产生"开局往前走、不管障碍"（旧墙不匹配新局 → 路径穿墙直行）。
        self.maze_signature = None
        self.raw_wall = None
        self.blocked = None
        self.blocked_turn = None
        self.free_dist = None
        self.maze_written = False
        self.path = []
        self.dense_path = []
        self.wp_idx = 0
        self.current_plan_reason = ''
        self.current_relative_deg = None
        self.current_tactical_target = None
        self.current_window_goal_reached = False
        self.current_window_replan_remaining = WINDOW_REPLAN_REMAINING_PX
        self.me_predictor = ConstantVelocityKalman2D()
        self.foe_predictor = ConstantVelocityKalman2D()
        self.last_plan_t = -1e9
        self.last_path_switch_t = -1e9
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
        # 固定文件名：新局覆盖重写（从 0 开始），bot 常驻不杀进程
        self.round_no += 1
        self.track_path = os.path.join(self.out_dir, 'track.csv')
        self.plan_path = os.path.join(self.out_dir, 'plans.csv')
        self.maze_path = os.path.join(self.out_dir, 'maze.csv')
        try:
            self.f.close()
            self.pf.close()
            self.lf.close()
            self.lpf.close()
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

        if self.maze_signature is not None:
            if sig != self.maze_signature:
                # 迷宫变化 = 新局（无论 round_ended 与否）：
                # 一律 reset 并重新锁图，绝不能保留旧地图继续规划
                # （否则开局的 bot 会沿上局旧墙的路径直冲，且局数越多累积越错）。
                self.reset_for_new_round('maze_changed')
                self.maze_signature = None   # 强制下面重新锁图
            else:
                return
        # 走到这里 = 首次锁图 或 新轮次 reset 后需要重新锁图

        self.maze_signature = sig
        with open(self.maze_path, 'w', encoding='utf-8') as f:
            for row in maze:
                f.write(','.join(map(str, row)) + '\n')
        # "当前轮"迷宫：固定名覆盖写
        with open(os.path.join(self.out_dir, 'maze_latest.csv'), 'w', encoding='utf-8') as f:
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

    def record(self, t, me, foe, msg, action):
        keys = action.get('keys') or {}
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
        }
        self.w.writerow(row)
        self.lw.writerow(row)      # 同步写"当前轮"track_latest.csv
        self.f.flush()
        self.lf.flush()
        self.rows += 1

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

    def _debug_overlay(self):
        """网页可视化：当前执行窗口的 path / waypoint / 车体扫掠投影（bot 坐标）。"""
        pts = list(self.path) if self.path else []
        if not pts:
            return {'path': [], 'waypoints': [], 'swept': []}
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
        return {'path': path_out, 'waypoints': wps, 'swept': swept}

    def remaining_path_length(self, me):
        if not self.path or self.wp_idx >= len(self.path):
            return 0.0
        pts = [(me['x'], me['y'])] + list(self.path[self.wp_idx:])
        return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(pts, pts[1:]))

    def replan(self, t, me, foe):
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

        if (pr.success and pr.path_validated and pr.path and
                (pr.path_length >= MIN_ACCEPT_PATH_PX or pr.window_goal_reached)):
            if not active:
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
                if target_shift <= SIMILAR_TARGET_PX:
                    accepted = True
                elif pr.path_length <= max(35.0, old_remaining * ratio):
                    accepted = True
                elif held >= PATH_HOLD_MAX_S:
                    accepted = True

            if accepted:
                # 短程方向粘滞：旧路径还有一小段可走时，拒绝与当前行进方向
                # 差 >90° 的新路径 —— 防路口等长双路 1s 级来回掉头。
                # 状态升级(staging→attack)与超时(PATH_HOLD_MAX_S)仍放行。
                if (active and old_remaining > PATH_SWITCH_MIN_REMAINING_PX
                        and len(pr.path) > 1 and not staging_to_attack
                        and held < PATH_HOLD_MAX_S):
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
                # Hold-position results have a continuous relative angle. Keep
                # the previous discrete tactical side as the switch preference.
                if pr.reason != 'hold_attack_position' and pr.relative_deg is not None:
                    self.current_relative_deg = pr.relative_deg
                if pr.target is not None:
                    self.current_tactical_target = pr.target
                self.current_window_goal_reached = pr.window_goal_reached
                self.current_window_replan_remaining = min(
                    WINDOW_REPLAN_REMAINING_PX,
                    max(20.0, pr.window_target_length * WINDOW_REPLAN_FRACTION),
                )
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
            'prediction_horizon_s':round(self.prediction_horizon_s, 3),
            'observed_foe_x':round(foe['x'], 3),
            'observed_foe_y':round(foe['y'], 3),
            'predicted_foe_x':round(predicted_foe[0], 3),
            'predicted_foe_y':round(predicted_foe[1], 3),
            'predicted_foe_path':json.dumps(
                [[round(x, 2), round(y, 2)] for x, y in predicted_path]
            ),
        }
        self.pw.writerow(prow)
        self.lpw.writerow(prow)    # 同步写"当前轮"plans_latest.csv
        self.pf.flush()
        self.lpf.flush()

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

    def action(self, me):
        k = {'up':0,'down':0,'left':0,'right':0}
        self.controller_mode = 'STOP'
        # 局间/无图保持停止；局内无合理路线才原地旋转
        if self.round_ended:
            return {'keys':k, 'fire':0}
        if not self.path or self.wp_idx >= len(self.path):
            # 无合理路线：原地旋转观察（每个 OODA 周期重规划，路径出现即恢复行驶）。
            self.controller_mode = 'SPIN_NO_PATH'
            return self._spin_action(me)

        while self.wp_idx < len(self.path):
            tx, ty = self.path[self.wp_idx]
            if math.hypot(tx - me['x'], ty - me['y']) <= WAYPOINT_REACHED_PX:
                self.wp_idx += 1
            else:
                break
        if self.wp_idx >= len(self.path):
            # 窗口走完、新规划未到：同样原地旋转等待
            self.controller_mode = 'SPIN_NO_PATH'
            return self._spin_action(me)

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
        self._throttle_on = True
        k['up'] = 1

        self.controller_mode = 'FULL_SPEED_PATH'
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
        if self.seen_both and ('me_respawn' in evts or 'layout_changed' in evts or 'me_teleport' in evts):
            self.finish_round('bridge_event(new_round)')
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
        local_window_due = (not self.current_window_goal_reached and
                            remaining <= self.current_window_replan_remaining)
        replan_period = LOCAL_WINDOW_RETRY_S if local_window_due else OODA_PERIOD_S
        if t - self.last_plan_t >= replan_period:
            self.replan(t, me, foe)
        action = self.action(me)
        if self.visualize:
            action = dict(action)
            action['debug'] = self._debug_overlay()
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
                    getattr(self,'lf',None), getattr(self,'lpf',None)):
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
    a = ap.parse_args()
    bot = Bot(a.port, a.out_dir, a.repo, a.tactical_mode, a.prediction_horizon_s)
    bot.exit_on_round_end = a.exit_on_round_end
    bot.visualize = a.visualize
    bot.serve()
