# -*- coding: utf-8 -*-
"""Smoke test: ALIGN 一次性 + 超时放弃逻辑 (start_aligned / align_started_at)."""
import sys, os, tempfile, math
repo = r"C:\Users\jasongeorge\deepseek harness workspace\tanktrouble12"
sys.path.insert(0, os.path.join(repo, "python"))
from bot_config import START_ALIGN_MAX_S  # noqa: E402
print("START_ALIGN_MAX_S =", START_ALIGN_MAX_S)

from navigation_bot import Bot  # noqa: E402
md = tempfile.mkdtemp(prefix="smokelock2_")
b = Bot(port=0, out_dir=md, repo_root=repo, tactical_mode="chase", prediction_horizon_s=0.0)
b.round_ended = False
b.combat_enabled = False
b.path = [(57.0, 285.0), (57.0, 245.0)]
b.dense_path = [(57.0, 285.0), (57.0, 281.0), (57.0, 277.0), (57.0, 273.0), (57.0, 269.0), (57.0, 265.0)]
b.wp_idx = 0
b.start_align_arc_px = 60.0
b.last_state_t = 10.0
b._frame_dt = 1 / 30.0
b._align_prev_pos = (57.0, 285.0)
b._v_smooth = None
b._steer_accum = 0.0
b._omega_applied = None
b._tangent_prev = None
b._blocked_since = None
b._blocked_pos = None
b._start_aligned = False
b._align_started_at = None

def act(lbl, x, y, ang, px_prev=(57.0, 285.0), t=10.0):
    b.last_state_t = t
    b._align_prev_pos = px_prev
    me = {"x": x, "y": y, "angle": ang * math.pi / 180.0, "vx": 0.0, "vy": 0.0}
    a = b.action(me)
    print("%-40s -> mode=%-16s up=%d l=%d r=%d  align_done=%s" % (
        lbl, b.controller_mode, a["keys"]["up"], a["keys"]["left"], a["keys"]["right"], b._start_aligned))

# A: 静止 195° 首次 -> 应 ALIGN(up=0,r=1)
act("A 静止195deg 首次(未超时)", 57.0, 285.0, 195.0)
# B: 同时刻再一帧(0.1s后) 仍歪 -> 继续 ALIGN
act("B 0.1s后仍195deg", 57.0, 285.0, 195.0, t=10.1)
# C: 对准成功 280deg -> _start_aligned=True, 走 Stanley
act("C 280deg(对准)", 57.0, 285.0, 280.0, t=10.2)
# D: 对准成功后回到歪 195° -> 不再 ALIGN（放行）
act("D 已对准后乱回195deg", 57.0, 285.0, 195.0, t=10.3)
# E: 全新路径(replan)+超时场景: 重置对齐态, 静止195°, _align_started_at=8.0(2s前) -> 超时放行
b._start_aligned = False; b._align_started_at = 8.0
act("E 超时2s后仍195deg", 57.0, 285.0, 195.0, t=10.4)
print("OK")
