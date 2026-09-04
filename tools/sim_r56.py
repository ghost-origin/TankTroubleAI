# -*- coding: utf-8 -*-
"""Deterministic sim: real NavigationBot.action() + r56 real walls + simplified car physics.
Usage: python tools/sim_r56.py [--fix] [--seconds 25]
"""
import csv, json, math, os, sys, tempfile, argparse
import numpy as np

repo = r"C:\Users\jasongeorge\deepseek harness workspace\tanktrouble12"
session = os.path.join(repo, "web_nav_logs", "20260903_125144")
sys.path.insert(0, os.path.join(repo, "python"))
from navigation_mvp import load_maze, load_polys, build_maps, footprint_collides  # noqa: E402
from navigation_bot import Bot  # noqa: E402

DEG = math.pi / 180.0
STEER = 240.0 * DEG      # rad/s 全速转向
ACC = 400.0              # px/s^2 起步加速
MAXV = 125.0             # px/s 最高速
DEC = 600.0              # px/s^2 松油门减速
STEER_HOLD = 0.09        # s 转向键松开后的惯性保持（游戏转向缓冲）

def run(use_fix_flag, seconds=25.0):
    md = tempfile.mkdtemp(prefix="r56sim_")
    bot = Bot(port=0, out_dir=md, repo_root=repo, tactical_mode="chase",
              prediction_horizon_s=0.0)
    bot.round_ended = False
    bot.combat_enabled = False
    maze = load_maze(os.path.join(session, "maze.csv"))
    polys = load_polys(os.path.join(repo, "data log", "tile_polys.json"))
    raw, _, blocked, blocked_turn, free_dist = build_maps(maze, polys, 10, 5, turn_radius_px=18.45)
    bot.raw_wall = raw
    bot.blocked = blocked
    bot.blocked_turn = blocked_turn
    bot.free_dist = free_dist
    # r56 真实执行窗口（plans.csv 第一个 accepted 计划，从 (57,285) 向上）
    pl = list(csv.DictReader(open(os.path.join(session, "plans.csv"), encoding="utf-8")))
    pts = json.loads(pl[0]["path_points"])
    bot.path = list(pts)
    bot.dense_path = list(pts)
    bot.wp_idx = 0
    if not hasattr(bot, "start_align_arc_px"):
        bot.start_align_arc_px = 60.0
    bot._align_prev_pos = None
    bot._v_smooth = None
    bot._steer_accum = 0.0
    bot._omega_applied = None
    bot._tangent_prev = None
    bot._blocked_since = None
    bot._blocked_pos = None

    x, y, ang = 57.0, 285.0, 195.0 * DEG
    v = 0.0
    ang_vel = 0.0
    dt = 1.0 / 30.0
    rows = []
    fs_runs = 0
    fs_run_len = 0
    max_fs_run = 0
    for step in range(int(seconds / dt)):
        t = step * dt
        bot.last_state_t = t
        bot._frame_dt = dt if step > 0 else None
        me = {"x": x, "y": y, "angle": ang,
              "vx": v * math.cos(ang), "vy": v * math.sin(ang)}
        act = bot.action(me)
        keys = act["keys"]
        mode = bot.controller_mode
        # --- physics ---
        if keys["right"] and not keys["left"]:
            ang_vel = STEER
        elif keys["left"] and not keys["right"]:
            ang_vel = -STEER
        else:
            # 惯性保持后归零
            if abs(ang_vel) > 0:
                ang_vel *= max(0.0, 1.0 - dt / STEER_HOLD)
                if abs(ang_vel) < 1.0 * DEG:
                    ang_vel = 0.0
        ang += ang_vel * dt
        if keys["up"]:
            v = min(MAXV, v + ACC * dt)
        else:
            v = max(0.0, v - DEC * dt)
        if v > 0:
            nx, ny = x + math.cos(ang) * v * dt, y + math.sin(ang) * v * dt
            if footprint_collides(raw, nx, ny, ang, shrink=0.0):
                v = 0.0
            else:
                x, y = nx, ny
        rows.append((t, x, y, ang / DEG % 360.0, mode, keys["up"], keys["left"], keys["right"]))
        if mode == "FULL_SPEED_PATH":
            fs_run_len += 1
            max_fs_run = max(max_fs_run, fs_run_len)
        else:
            if fs_run_len:
                fs_runs += 1
            fs_run_len = 0
    if fs_run_len:
        fs_runs += 1
    return rows, fs_runs, max_fs_run, os.path.basename(md)

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=25.0)
    args = ap.parse_args()
    rows, fs_runs, max_fs_run, tag = run(False, args.seconds)
    print("=== baseline (current code) %s ===" % tag)
    print("FULL_SPEED 段数=%d 最长连续段=%.2fs (%.0f帧@30fps)" % (fs_runs, max_fs_run / 30.0, max_fs_run))
    print("t=0 位置 (57,285) -> t=%.0fs 位置 (%.1f,%.1f)" % (args.seconds, rows[-1][1], rows[-1][2]))
    print("向上净位移(y 减小): %.1f px" % (285.0 - rows[-1][2]))
    # 轨迹采样
    print("\n t     x     y    ang    mode")
    for r in rows[::15]:
        print("%5.2f %6.1f %6.1f %6.1f %s" % (r[0], r[1], r[2], r[3], r[4][:14]))
