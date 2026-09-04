# -*- coding: utf-8 -*-
"""把某一局的 track/bullets/maze 渲染成 GIF 动画。
用法: py render_round_gif.py <会话目录> <局号>
输出: 会话目录/round_局号.gif
"""
import csv, math, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from matplotlib.patches import Rectangle, Polygon
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data log'))
import plot_tracks as PT

HERE = os.path.dirname(os.path.abspath(__file__))
TILE_PX = 57

def f(x):
    try:
        return float(x)
    except Exception:
        return None

def load_track(path):
    rows = []
    with open(path, newline='', encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            rows.append(r)
    return rows

def tank_corners(cx, cy, ang, half_l=15.5, half_w=10.0):
    """以 (cx,cy) 为中心、长 half_l*2 沿朝向 ang 的旋转矩形四角（世界坐标，y 向下）。"""
    ca, sa = math.cos(ang), math.sin(ang)
    out = []
    for (lx, ly) in ((half_l, half_w), (-half_l, half_w), (-half_l, -half_w), (half_l, -half_w)):
        out.append((cx + lx * ca - ly * sa, cy + lx * sa + ly * ca))
    return out

def main():
    sess = sys.argv[1]
    rid = int(sys.argv[2])
    tr = load_track(os.path.join(sess, 'track_round%03d.csv' % rid))
    # 墙线
    grid = []
    with open(os.path.join(sess, 'maze_round%03d.csv' % rid), encoding='utf-8') as fh:
        for line in fh:
            line = line.strip()
            if line:
                grid.append([int(v) for v in line.split(',')])
    walls = []
    for y, row in enumerate(grid):
        for x, v in enumerate(row):
            if v == 0 or v == -1:
                continue
            tid, fliph, flipv, flipd = PT.parse_tile(v)
            if tid > 0:
                walls.append((x, y, tid, fliph, flipv, flipd))
    polys = PT.load_polys()
    # 本局子弹
    buls = []
    with open(os.path.join(sess, 'bullets.csv'), encoding='utf-8-sig') as fh:
        for b in csv.DictReader(fh):
            if f(b['round']) != rid:
                continue
            t, x, y = f(b['t']), f(b['x']), f(b['y'])
            if t is not None and x is not None and y is not None:
                buls.append((t, x, y, b['owner']))
    buls.sort(key=lambda v: v[0])

    t0 = f(tr[0]['t']); t1 = f(tr[-1]['t']); n = len(tr)
    step = max(1, int(round(n / max(1.0, (t1 - t0) * 14))))
    idxs = list(range(0, n, step))
    if idxs[-1] != n - 1:
        idxs.append(n - 1)

    fig, ax = plt.subplots(figsize=(6.2, 6.2), dpi=100)
    ax.set_aspect('equal', adjustable='box')
    ax.invert_yaxis()
    ax.set_xlim(-15, 10 * TILE_PX + 15)
    ax.set_ylim(10 * TILE_PX + 15, -15)
    ax.set_xticks([]); ax.set_yticks([])
    ax.add_patch(Rectangle((0, 0), 10 * TILE_PX, 10 * TILE_PX, fill=False, edgecolor='#444', lw=1.2, zorder=1))
    PT.draw_maze(ax, walls, polys)

    me_ln, = ax.plot([], [], '-', color='#1f77b4', lw=1.3, alpha=0.9, zorder=3)
    foe_ln, = ax.plot([], [], '-', color='#d62728', lw=1.3, alpha=0.9, zorder=3)
    me_body = ax.add_patch(Polygon([(0, 0)] * 4, closed=True, facecolor='#1f77b4', edgecolor='k', lw=0.9, zorder=5))
    foe_body = ax.add_patch(Polygon([(0, 0)] * 4, closed=True, facecolor='#d62728', edgecolor='k', lw=0.9, zorder=5))
    me_bar, = ax.plot([], [], '-', color='#fff', lw=2.0, zorder=6)
    foe_bar, = ax.plot([], [], '-', color='#fff', lw=2.0, zorder=6)
    bullet_sc = ax.scatter([], [], s=26, zorder=7)
    title = ax.set_title('', fontsize=10)

    me_x = [f(r['me_x']) for r in tr]; me_y = [f(r['me_y']) for r in tr]
    foe_x = [f(r['foe_x']) for r in tr]; foe_y = [f(r['foe_y']) for r in tr]

    def upd(k):
        i = idxs[k]
        r = tr[i]; t = f(r['t'])
        upto = i + 1
        me_ln.set_data(me_x[:upto], me_y[:upto]); foe_ln.set_data(foe_x[:upto], foe_y[:upto])
        for body, bar, (cx, cy, ang) in (
                (me_body, me_bar, (f(r['me_x']), f(r['me_y']), f(r['me_angle']) or 0.0)),
                (foe_body, foe_bar, (f(r['foe_x']), f(r['foe_y']), f(r['foe_angle']) or 0.0))):
            body.set_xy(tank_corners(cx, cy, ang))
            bx, by = cx + math.cos(ang) * 26, cy + math.sin(ang) * 26
            bar.set_data([cx, bx], [cy, by])
        bpts = [(x, y, o) for (bt, x, y, o) in buls if abs(bt - t) < 0.07]
        if bpts:
            arr = np.array([[x, y] for (x, y, _) in bpts])
            col = ['#00bfff' if o == 'me' else ('#ff7f0e' if o == 'foe' else '#888') for (_, _, o) in bpts]
            bullet_sc.set_offsets(arr); bullet_sc.set_color(col)
        else:
            bullet_sc.set_offsets(np.empty((0, 2)))
        title.set_text('Round %02d   t=%.2fs / %.2fs   (%d frames, WIN)' % (rid, t, t1, len(idxs)))
        return (me_ln, foe_ln, me_body, foe_body, me_bar, foe_bar, bullet_sc, title)

    fig.tight_layout()
    ani = anim.FuncAnimation(fig, upd, frames=len(idxs), blit=False)
    out = os.path.join(sess, 'round%03d_win.gif' % rid)
    ani.save(out, writer='pillow', fps=14, dpi=100)
    print('saved', out, 'frames=', len(idxs), 'dur=%.2fs' % (t1 - t0))

if __name__ == '__main__':
    main()
