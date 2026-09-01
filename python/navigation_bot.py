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
import argparse, csv, json, math, os, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))

from navigation_mvp import build_maps, load_polys, plan

CSV_COLUMNS = [
    't','me_x','me_y','me_angle','me_vx','me_vy',
    'foe_x','foe_y','foe_angle','foe_vx','foe_vy','n_bullets','n_powerups'
]
MAP_ORIGIN_X = 197.5
MAP_ORIGIN_Y = 31.0
MAZE_SIZE = 10
WAYPOINT_REACHED_PX = 13.0
TURN_ONLY_RAD = math.radians(18.0)
TURN_DEADBAND_RAD = math.radians(5.0)
OODA_PERIOD_S = 1.0

# Minimal path-switch hysteresis. OODA still evaluates every second, but a
# transient target change is not allowed to force a huge U-turn.
PATH_HOLD_MAX_S = 3.0
SIMILAR_TARGET_PX = 35.0
NORMAL_SWITCH_RATIO = 1.15
STAGING_TO_ATTACK_RATIO = 1.35

# Headless runs showed ~5.1s silent gaps between rounds. 2.5s is comfortably
# above normal bridge jitter and below the inter-round gap.
ROUND_GAP_S = 2.5
# Defensive respawn/teleport guard for state streams that do not contain a gap.
ROUND_JUMP_PX = 120.0


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Bot:
    def __init__(self, port, out_dir, repo_root):
        sys.path.insert(0, os.path.join(repo_root, 'data log'))
        from ws_server import WSServer
        self.WSServer = WSServer
        self.port = port
        self.out_dir = out_dir
        self.repo_root = repo_root
        os.makedirs(out_dir, exist_ok=True)

        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.track_path = os.path.join(out_dir, 'track_%s.csv' % stamp)
        self.maze_path = os.path.join(out_dir, 'maze_%s.csv' % stamp)
        self.plan_path = os.path.join(out_dir, 'plans_%s.csv' % stamp)

        self.f = open(self.track_path, 'w', newline='', encoding='utf-8')
        self.w = csv.DictWriter(self.f, fieldnames=CSV_COLUMNS)
        self.w.writeheader()

        self.pf = open(self.plan_path, 'w', newline='', encoding='utf-8')
        self.pw = csv.DictWriter(
            self.pf,
            fieldnames=['t','success','accepted','relative_deg','target_x','target_y',
                        'path_length_px','old_remaining_px','smoothness_rad','planning_ms','reason','path_points']
        )
        self.pw.writeheader()

        self.polys = None
        self.raw_wall = None
        self.blocked = None
        self.blocked_turn = None
        self.free_dist = None
        self.maze_signature = None
        self.maze_written = False

        self.last_plan_t = -1e9
        self.path = []
        self.wp_idx = 0
        self.current_plan_reason = ''
        self.current_relative_deg = None
        self.last_path_switch_t = -1e9
        self.client = None
        self.rows = 0

        self.seen_both = False
        self.round_ended = False
        self.round_end_reason = ''
        self.last_state_t = None
        self.last_me_pos = None
        self.last_foe_pos = None

        print('track:', self.track_path, flush=True)
        print('maze:', self.maze_path, flush=True)

    @staticmethod
    def stop_action():
        return {'keys': {'up':0,'down':0,'left':0,'right':0}, 'fire':0}

    def finish_round(self, reason):
        if self.round_ended:
            return
        self.round_ended = True
        self.round_end_reason = reason
        self.path = []
        self.wp_idx = 0
        self.current_plan_reason = ''
        self.current_relative_deg = None
        try:
            self.f.flush()
            self.pf.flush()
        except Exception:
            pass
        print('== round ended: %s | rows=%d ==' % (reason, self.rows), flush=True)

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
        self.wp_idx = 0
        self.current_plan_reason = ''
        self.current_relative_deg = None
        self.last_plan_t = -1e9
        self.last_path_switch_t = -1e9
        # 每局独立日志文件（后续轮次不覆盖第一局）
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.track_path = os.path.join(self.out_dir, 'track_%s.csv' % stamp)
        self.plan_path = os.path.join(self.out_dir, 'plans_%s.csv' % stamp)
        self.maze_path = os.path.join(self.out_dir, 'maze_%s.csv' % stamp)
        try:
            self.f.close()
            self.pf.close()
        except Exception:
            pass
        self.f = open(self.track_path, 'w', newline='', encoding='utf-8')
        self.w = csv.DictWriter(self.f, fieldnames=CSV_COLUMNS)
        self.w.writeheader()
        self.pf = open(self.plan_path, 'w', newline='', encoding='utf-8')
        self.pw = csv.DictWriter(
            self.pf,
            fieldnames=['t','success','accepted','relative_deg','target_x','target_y',
                        'path_length_px','old_remaining_px','smoothness_rad','planning_ms','reason','path_points']
        )
        self.pw.writeheader()
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
        c.on_close = lambda _c: self.close()
        print('connected:', c.addr, flush=True)

    def save_map(self, m):
        """Capture exactly the first valid maze for this round.

        A different maze signature means a new round has begun. Do not overwrite
        the maze that belongs to the already-recorded track.
        """
        grid = m.get('grid') or []
        if len(grid) < MAZE_SIZE:
            return
        maze = [[int(grid[y][x]) for x in range(MAZE_SIZE)] for y in range(MAZE_SIZE)]
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

        if m.get('polys'):
            self.polys = m['polys']
        elif self.polys is None:
            self.polys = load_polys(os.path.join(self.repo_root, 'data log', 'tile_polys.json'))

        # 方向性 Minkowski + tube 管道：A* 用窄网格（10px 直行），
        # 转弯点用外接圆（18.45px）校验；free_dist 供 tube 车道校验。
        (self.raw_wall, _, self.blocked, self.blocked_turn, self.free_dist) = build_maps(
            maze, self.polys, 10, 5, turn_radius_px=18.45, straight_radius_px=10)
        self.maze_written = True
        print('maze snapshot locked for this round', flush=True)

    def record(self, t, me, foe, msg):
        self.w.writerow({
            't':round(t,2),
            'me_x':round(me['x'],1), 'me_y':round(me['y'],1), 'me_angle':round(me['angle'],3),
            'me_vx':round(me['vx'],1), 'me_vy':round(me['vy'],1),
            'foe_x':round(foe['x'],1), 'foe_y':round(foe['y'],1), 'foe_angle':round(foe['angle'],3),
            'foe_vx':round(foe['vx'],1), 'foe_vy':round(foe['vy'],1),
            'n_bullets':len(msg.get('bullets') or []),
            'n_powerups':len(msg.get('powerups') or []),
        })
        self.f.flush()
        self.rows += 1

    def remaining_path_length(self, me):
        if not self.path or self.wp_idx >= len(self.path):
            return 0.0
        pts = [(me['x'], me['y'])] + list(self.path[self.wp_idx:])
        return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(pts, pts[1:]))

    def replan(self, t, me, foe):
        if self.raw_wall is None or self.blocked is None or self.round_ended:
            return

        pr = plan(
            (me['x'], me['y']),
            (foe['x'], foe['y']),
            foe['angle'],
            self.raw_wall,
            self.blocked,
            preferred_rel_deg=self.current_relative_deg,
            blocked_turn=self.blocked_turn,
            free_dist=self.free_dist,
        )
        self.last_plan_t = t

        active = bool(self.path and self.wp_idx < len(self.path))
        old_remaining = self.remaining_path_length(me) if active else 0.0
        accepted = False

        if pr.success:
            if not active:
                accepted = True
            else:
                old_target = self.path[-1]
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
                self.path = pr.path
                self.wp_idx = 1 if len(self.path) > 1 else 0
                self.current_plan_reason = pr.reason
                # Hold-position results have a continuous relative angle. Keep
                # the previous discrete tactical side as the switch preference.
                if pr.reason != 'hold_attack_position' and pr.relative_deg is not None:
                    self.current_relative_deg = pr.relative_deg
                self.last_path_switch_t = t
        else:
            # Do not throw away a still-useful path just because one 1 Hz OODA
            # update has no legal rear candidate. The next cycle may recover.
            if not active:
                self.path = []
                self.wp_idx = 0
                self.current_plan_reason = ''
                self.current_relative_deg = None

        self.pw.writerow({
            't':round(t,3),
            'success':int(pr.success),
            'accepted':int(accepted),
            'relative_deg':'' if pr.relative_deg is None else pr.relative_deg,
            'target_x':'' if pr.target is None else round(pr.target[0],2),
            'target_y':'' if pr.target is None else round(pr.target[1],2),
            'path_length_px':round(pr.path_length,3),
            'old_remaining_px':round(old_remaining,3),
            'smoothness_rad':round(pr.smoothness_rad,4),
            'planning_ms':round(pr.planning_ms,3),
            'reason':pr.reason,
            'path_points':json.dumps([[round(x,1),round(y,1)] for x,y in pr.path]),
        })
        self.pf.flush()

    def action(self, me):
        k = {'up':0,'down':0,'left':0,'right':0}
        if self.round_ended or not self.path or self.wp_idx >= len(self.path):
            return {'keys':k, 'fire':0}

        while self.wp_idx < len(self.path):
            tx, ty = self.path[self.wp_idx]
            if math.hypot(tx - me['x'], ty - me['y']) <= WAYPOINT_REACHED_PX:
                self.wp_idx += 1
            else:
                break
        if self.wp_idx >= len(self.path):
            return {'keys':k, 'fire':0}

        tx, ty = self.path[self.wp_idx]
        desired = math.atan2(ty - me['y'], tx - me['x'])
        err = wrap(desired - me['angle'])

        # Car 行为运动学（实测）：steerSpeed ≈ 3.8 rad/s，maxspeed=125px/s。
        #   - 转向角速度 ω = steerSpeed × |s|/maxspeed → 速度=0 时转不动
        #   - 转弯半径 R = maxspeed/steerSpeed ≈ 33px 恒定
        # 策略：转向时保持油门（s>0 才能转），此处只分配方向键。
        if err > TURN_DEADBAND_RAD:
            k['right'] = 1
        elif err < -TURN_DEADBAND_RAD:
            k['left'] = 1
        k['up'] = 1    # 始终前进，转向靠 s>0
        return {'keys':k, 'fire':0}

    def on_message(self, c, text):
        try:
            msg = json.loads(text)
        except Exception:
            return
        if not isinstance(msg, dict):
            return

        # Keep first maze only. A later different maze is a hard round boundary.
        if msg.get('map'):
            self.save_map(msg['map'])
        if self.round_ended:
            # 网页模式连续对局：round 结束后若又收到完整 me+foe 状态帧
            # （新局已开始：同迷宫新局 / 回菜单再进），则自动复位继续导航。
            # 迷宫变化的新局已由 save_map 走 reset_for_new_round 处理。
            if msg.get('me') and msg.get('foe'):
                self.reset_for_new_round('state_resumed')
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

        # 爆炸信号 = 本局即将结束（真实浏览器击杀必定触发坦克爆炸动画，
        # 随后下一局必然开始）。收到即结束本轮、清空规划与地图，等待新局。
        # 兼容多个爆炸/销毁相关动画名（explosion/boom/explode/destroy/death）。
        foe_anim = ''
        if msg.get('foe') and isinstance(msg['foe'], dict):
            foe_anim = str(msg['foe'].get('anim') or '').lower()
        EXPLODE_KEYS = ('explo', 'boom', 'blast', 'destroy', 'death', 'dead', 'die')
        if foe_anim and any(k in foe_anim for k in EXPLODE_KEYS):
            self.finish_round('foe_explosion_anim(%s)' % foe_anim)
            c.send_text(json.dumps(self.stop_action(), separators=(',',':')))
            return

        # Player death can produce no bridge frames until the next respawn. The first
        # frame of the next round must NOT be appended to the old CSV.
        if self.last_state_t is not None:
            dt = t - self.last_state_t
            if dt > ROUND_GAP_S:
                self.finish_round('state_gap_%.2fs' % dt)
                c.send_text(json.dumps(self.stop_action(), separators=(',',':')))
                return

            if dt > 0 and self.last_me_pos is not None and self.last_foe_pos is not None:
                dm = math.hypot(me['x'] - self.last_me_pos[0], me['y'] - self.last_me_pos[1])
                df = math.hypot(foe['x'] - self.last_foe_pos[0], foe['y'] - self.last_foe_pos[1])
                if dm > ROUND_JUMP_PX or df > ROUND_JUMP_PX:
                    self.finish_round('position_jump me=%.1f foe=%.1f' % (dm, df))
                    c.send_text(json.dumps(self.stop_action(), separators=(',',':')))
                    return

        self.seen_both = True
        self.last_state_t = t
        self.last_me_pos = (me['x'], me['y'])
        self.last_foe_pos = (foe['x'], foe['y'])

        # 地图未就绪时绝不移动：开局或跨轮复位后，save_map 可能尚未锁图
        # （桥的 map 每 3s 才重发一次）。在锁图前强制停止，避免"开局直冲、
        # 不管障碍"。锁图后首个 OODA 自然接管。
        if self.raw_wall is None or self.blocked is None or not self.maze_written:
            c.send_text(json.dumps(self.stop_action(), separators=(',',':')))
            return

        self.record(t, me, foe, msg)
        if t - self.last_plan_t >= OODA_PERIOD_S:
            self.replan(t, me, foe)
        c.send_text(json.dumps(self.action(me), separators=(',',':')))

    def serve(self):
        ws = self.WSServer('127.0.0.1', self.port, self.on_client)
        print('navigation bot ws://127.0.0.1:%d/ai' % self.port, flush=True)
        try:
            ws.serve_forever()
        finally:
            self.close()

    def close(self):
        for obj in (getattr(self,'f',None), getattr(self,'pf',None)):
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
    a = ap.parse_args()
    Bot(a.port, a.out_dir, a.repo).serve()
