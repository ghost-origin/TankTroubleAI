# -*- coding: utf-8 -*-
"""地图生成管道演示动图：从 maze_roundNNN.csv 原始数值 → 解码/翻转/平移 → 最终迷宫墙线。

用法: py render_map_pipeline.py <会话目录> <局号>
输出: <会话目录>/map_pipeline.gif
复用 plot_tracks 的 tile 解码与 tile_polys。
"""
import csv, math, os, sys
import matplotlib
matplotlib.use("Agg")
for _f in ('Microsoft YaHei', 'SimHei', 'SimSun', 'Noto Sans CJK SC', 'WenQuanYi Micro Hei'):
    try:
        matplotlib.rcParams['font.sans-serif'] = [_f]
        matplotlib.rcParams['axes.unicode_minus'] = False
        # 验证能找到该字体（避免中文变方块）
        import matplotlib.font_manager as _fm
        if any(_f.lower() in (f.name or '').lower() for f in _fm.fontManager.ttflist):
            break
    except Exception:
        pass
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from matplotlib.patches import Rectangle, Polygon
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data log'))
import plot_tracks as PT

HERE = os.path.dirname(os.path.abspath(__file__))
TILE_PX = 57

def build_cell_geometry(cx, cy, tid, fliph, flipv, flipd, polys):
    """返回格内墙线折点（世界 px）—— 与 plot_tracks.draw_maze 完全一致的算法。"""
    poly = polys[tid] if tid < len(polys) and polys[tid] else None
    if not poly:
        return None
    pts = [(x * TILE_PX, y * TILE_PX) for x, y in zip(poly[0::2], poly[1::2])]
    if flipd:
        pts = [(y, x) for (x, y) in pts]
    if fliph:
        pts = [(TILE_PX - x, y) for (x, y) in pts]
    if flipv:
        pts = [(x, TILE_PX - y) for (x, y) in pts]
    off = (cx * TILE_PX, cy * TILE_PX)
    return [(off[0] + px, off[1] + py) for (px, py) in pts]

def main():
    sess = sys.argv[1]
    rid = int(sys.argv[2])
    grid = []
    with open(os.path.join(sess, 'maze_round%03d.csv' % rid), encoding='utf-8') as fh:
        for line in fh:
            line = line.strip()
            if line:
                grid.append([int(v) for v in line.split(',')])
    polys = PT.load_polys()
    cells = []
    for y, row in enumerate(grid):
        for x, v in enumerate(row):
            if v == 0 or v == -1:
                continue
            tid, fliph, flipv, flipd = PT.parse_tile(v)
            if tid > 0:
                cells.append((x, y, v, tid, fliph, flipv, flipd))
    ncells = len(cells)
    # 挑几种墙块做"解码演示"：每个 tile id 首次出现
    demo = {}
    for c in cells:
        if c[3] not in demo:
            demo[c[3]] = c
    demo_ids = sorted(demo.keys())
    demo_cells = [demo[t] for t in demo_ids]

    # 每个墙格的墙线折点（draw_maze 会逐边连线成环）
    polylines = {}   # (x,y) -> 折点列表
    for c in cells:
        g = build_cell_geometry(c[0], c[1], c[3], c[4], c[5], c[6], polys)
        if g:
            polylines[(c[0], c[1])] = g

    # 演示步骤（每个 demo 格）：说明文字（随子步递增）
    DEMO_STEPS = 8
    def demo_text(i0, step):
        x, y, v, tid, h, vv, dd = demo_cells[i0]
        u = v & 0xFFFFFFFF
        tids = u & 0x1FFFFFFF
        bits = []
        for name, m in (('H', 0x80000000), ('V', 0x40000000), ('D', 0x20000000)):
            bits.append(('1' if (u & m) else '0'))
        # 归一化多边形
        poly = polys[tid]
        norm_pts = [(px_, py_) for px_, py_ in zip(poly[0::2], poly[1::2])]
        scaled = [(px_ * TILE_PX, py_ * TILE_PX) for (px_, py_) in norm_pts]
        lines = [
            "单元格 (%d,%d)  原始值 = %d" % (x, y, v),
            "  位运算: 值 & 0xFFFFFFFF = 0x%08X" % u,
            "  tile_id = 值 & 0x1FFFFFFF = %d" % tids,
            "  方向标志: H=%s V=%s D=%s  (旋转→标志组合)" % (bits[0], bits[1], bits[2]),
        ]
        p = lines if step <= 2 else lines
        if step >= 3:
            p.append("归一化多边形 ×57px:")
            p.append("  " + ", ".join("(%.2f,%.2f)" % pt for pt in norm_pts[:4]) + " …")
        if step >= 4:
            p.append("缩放后(格内 px):")
            p.append("  " + ", ".join("(%.0f,%.0f)" % pt for pt in scaled[:4]) + " …")
        if step >= 5:
            fl = []
            fl.append("  翻转顺序 D→H→V")
            if dd: fl.append("   D: 交换 x,y")
            if h:  fl.append("   H: x → %d - x" % TILE_PX)
            if vv: fl.append("   V: y → %d - y" % TILE_PX)
            if not (dd or h or vv):
                fl.append("   (无翻转标志)")
            p += fl
        if step >= 6:
            p.append("平移到格 (%d*57, %d*57) → 世界坐标" % (x, y))
        if step >= 7:
            p.append("连接顶点成环 → 3px 墙线")
        return "\n".join(p)

    # ---- 帧计划 ----
    RAWG = 6          # 原始网格帧数
    DEMO_LEAD = 4     # 演示间隙帧
    STAMP_EVERY = 2   # 每 2 帧盖一格
    raw_frames = RAWG
    demo_frames = (len(demo_cells) * DEMO_STEPS + DEMO_LEAD * len(demo_cells))
    stamp_start = raw_frames + demo_frames
    stamp_frames = int(math.ceil(ncells / STAMP_EVERY)) + 3
    total = stamp_start + stamp_frames

    fig, ax = plt.subplots(figsize=(7.5, 6.6), dpi=100)
    ax.set_aspect('equal', adjustable='box'); ax.invert_yaxis()
    ax.set_xlim(-20, 10 * TILE_PX + 20); ax.set_ylim(10 * TILE_PX + 20, -20)
    ax.set_xticks([]); ax.set_yticks([])
    ax.add_patch(Rectangle((0, 0), 10 * TILE_PX, 10 * TILE_PX, fill=False, edgecolor='#444', lw=1.2))
    for i in range(11):
        ax.axvline(i * TILE_PX, color='#e0e0e0', lw=0.4); ax.axhline(i * TILE_PX, color='#e0e0e0', lw=0.4)

    # 原始数值文字（每个墙格中心）
    val_arts = []
    for c in cells:
        t = ax.text(c[0] * TILE_PX + TILE_PX / 2, c[1] * TILE_PX + TILE_PX / 2,
                    '%d' % c[2], ha='center', va='center', fontsize=5.5, color='#555', zorder=3)
        val_arts.append(t)
    # 墙线（初始隐藏）: 每格一个 Polygon 边线艺术家
    wall_arts = {}
    for c in cells:
        g = polylines.get((c[0], c[1]))
        if not g:
            continue
        art, = ax.plot([p[0] for p in g] + [g[0][0]], [p[1] for p in g] + [g[0][1]],
                       color='#333', lw=2.0, zorder=4, visible=False)
        wall_arts[(c[0], c[1])] = art
    # 演示格高亮框
    demo_box = ax.add_patch(Rectangle((0, 0), 1, 1, fill=True, facecolor='#ffd400', alpha=0.35,
                                      edgecolor='#e08000', lw=1.4, zorder=2, visible=False))
    info = ax.text(-20, -6, '', fontsize=8.2, color='#003366', va='top',
                   zorder=10, linespacing=1.25)
    title = ax.set_title('地图生成管道: maze_round%03d.csv  →  最终迷宫墙线' % rid, fontsize=10, color='#222')

    # 先隐藏所有墙线 & 值，直到相关阶段
    def show_vals(on):
        for t in val_arts:
            t.set_visible(on)
    def reveal_cell(k):
        # 第 k 个墙格出现（STAMP 阶段用）
        if k < ncells:
            c = cells[k]
            art = wall_arts.get((c[0], c[1]))
            if art:
                art.set_visible(True)

    # 主循环分帧状态
    stamped = 0
    def upd(i):
        nonlocal stamped
        # 阶段 1：原始网格（仅显示数值，墙线全隐藏）
        if i < raw_frames:
            show_vals(True)
            for a in wall_arts.values():
                a.set_visible(False)
            demo_box.set_visible(False)
            info.set_text('第 1 步：原始 maze CSV —— 每格一个"带符号整数"原始墙块值\n'
                          '（负数 = 最高位被当作符号位，是 H 翻转标志）')
            title.set_text('① 原始 CSV 数值（maze_round%03d.csv）' % rid)
            return
        # 阶段 2：演示逐个格子如何解码 + 翻转 + 平移
        cell_frames = DEMO_STEPS + DEMO_LEAD
        di = (i - raw_frames) // cell_frames
        within = (i - raw_frames) % cell_frames
        if di < len(demo_cells):
            c = demo_cells[di]
            show_vals(False)
            for k, cc in enumerate(cells):           # 只显示"已演示"的格的墙线
                w = wall_arts.get((cc[0], cc[1]))
                if w:
                    w.set_visible(k < di + 1)
            demo_box.set_xy((c[0] * TILE_PX, c[1] * TILE_PX))
            demo_box.set_width(TILE_PX); demo_box.set_height(TILE_PX)
            demo_box.set_visible(True)
            step = min(DEMO_STEPS - 1, int(within))
            info.set_text(demo_text(di, step))
            title.set_text('② 解码一个格子（%d/%d）: 数值 → 墙块+标志 → 翻转 → 平移'
                           % (di + 1, len(demo_cells)))
            if step >= DEMO_STEPS - 2:               # 最后两步点亮该格墙线
                w = wall_arts.get((c[0], c[1]))
                if w:
                    w.set_visible(True)
            return
        # 阶段 3：逐格盖完整个迷宫
        show_vals(True)
        demo_box.set_visible(False)
        k = (i - stamp_start) * STAMP_EVERY
        for j in range(stamped, min(ncells, k)):
            reveal_cell(j)
        stamped = min(ncells, k)
        info.set_text('③ 逐格盖墙线 → 最终迷宫\n'
                      '（墙是格内 3px 细线，格子的空白部分就是通道——所以迷宫不开洞也能走）')
        title.set_text('③ 最终地图: 3px 细墙线组成迷宫')
        if stamped >= ncells:
            info.set_text('完成！\n墙线 = 每格按 tile_id 取多边形 → 按 H/V/D 翻转变换\n'
                          '→ 放大 57px → 平移到格 → 连成细线；空白格 = 通道')
        return

    fig.tight_layout()
    ani = anim.FuncAnimation(fig, upd, frames=total, blit=False)
    out = os.path.join(sess, 'map_pipeline.gif')
    ani.save(out, writer='pillow', fps=8, dpi=100)
    print('saved', out, 'frames=', total, 'cells=', ncells)

if __name__ == '__main__':
    main()
