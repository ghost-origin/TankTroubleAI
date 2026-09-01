# -*- coding: utf-8 -*-
"""绘制一局双方坦克轨迹（含真实迷宫墙线）。

用法：
    conda run -n base python "data log\\plot_tracks.py" [track CSV文件]
    （不传参数 = 绘制 data log 里最新的 track_*.csv）

自动读取同名的 maze_*.csv（若有）：10 行 10 列，
每格 = 原始 tile 值（含翻转/旋转标志），结合 data log/tile_polys.json
画出真实的 3px 细墙线（墙是格内细线，格子空白处为通道），轨迹叠加其上。

输出：同名 .png（如 track_xxx.png），画在 data log/ 目录。
"""
import csv
import glob
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")   # 无界面后端，只存文件
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
TILE_PX = 57            # 每格像素
MAZE_SIZE = 10          # 迷宫 10×10
TILE_ID_MASK = 0x1FFFFFFF
FLAG_H, FLAG_V, FLAG_D = 0x80000000, 0x40000000, 0x20000000


def latest_csv():
    files = sorted(glob.glob(os.path.join(HERE, "track_*.csv")))
    return files[-1] if files else None


def load(csv_path):
    rows = []
    with open(csv_path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            out = {}
            for k, v in r.items():
                if not v:
                    continue
                try:
                    out[k] = float(v)
                except ValueError:
                    out[k] = v  # 非数值列（如 controller_mode）原样保留
            rows.append(out)
    return rows


def load_polys():
    p = os.path.join(HERE, "tile_polys.json")
    if not os.path.exists(p):
        return None
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def parse_tile(v):
    """原始 tile 值 → (tile_id, fliph, flipv, flipd)"""
    uv = int(v) & 0xFFFFFFFF
    tid = uv & TILE_ID_MASK
    return tid, bool(uv & FLAG_H), bool(uv & FLAG_V), bool(uv & FLAG_D)


def load_maze(csv_path):
    """读同名 maze CSV → (grid, walls)。walls: [(x格,y格,tile_id,...), ...]"""
    maze_path = os.path.splitext(csv_path)[0].replace("track_", "maze_") + ".csv"
    if not os.path.exists(maze_path):
        return None, []
    grid = []
    with open(maze_path, newline="", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            grid.append([int(v) for v in line.split(",")])
    walls = []
    for y, row in enumerate(grid):
        for x, v in enumerate(row):
            if v == 0 or v == -1:
                continue  # 路 / 空白
            tid, fliph, flipv, flipd = parse_tile(v)
            if tid <= 0:
                continue
            walls.append((x, y, tid, fliph, flipv, flipd))
    return grid, walls


def draw_maze(ax, walls, polys):
    """把每格的墙块多边形（细线）画出来，格子空白处即通道"""
    if not walls or not polys:
        return
    for (cx, cy, tid, fliph, flipv, flipd) in walls:
        poly = polys[tid] if tid < len(polys) and polys[tid] else None
        if not poly:
            continue
        # 归一化 → 格内像素
        pts = [(x * TILE_PX, y * TILE_PX) for x, y in zip(poly[0::2], poly[1::2])]
        # 翻转/旋转（与 C2 cacheTilePoly 一致）
        if flipd:
            pts = [(y, x) for (x, y) in pts]
        if fliph:
            pts = [(TILE_PX - x, y) for (x, y) in pts]
        if flipv:
            pts = [(x, TILE_PX - y) for (x, y) in pts]
        # 平移到格子
        off_x, off_y = cx * TILE_PX, cy * TILE_PX
        wpts = [(off_x + px, off_y + py) for (px, py) in pts]
        # 画多边形边线（墙线）
        n = len(wpts)
        for i in range(n):
            x1, y1 = wpts[i]
            x2, y2 = wpts[(i + 1) % n]
            ax.plot([x1, x2], [y1, y2], color="#333", lw=2.0, zorder=2)
    # 网格线（轻）
    for i in range(MAZE_SIZE + 1):
        ax.axvline(i * TILE_PX, color="#ccc", lw=0.4, zorder=1)
        ax.axhline(i * TILE_PX, color="#ccc", lw=0.4, zorder=1)


def plot(csv_path, rows):
    me_x = [r["me_x"] for r in rows]
    me_y = [r["me_y"] for r in rows]
    foe_x = [r["foe_x"] for r in rows]
    foe_y = [r["foe_y"] for r in rows]

    _, walls = load_maze(csv_path)
    polys = load_polys()

    fig, ax = plt.subplots(figsize=(9, 6.5))
    draw_maze(ax, walls, polys)   # 先画墙线（底层）

    if rows:
        # 我方轨迹（蓝）
        ax.plot(me_x, me_y, "-", color="#1f77b4", lw=1.4, alpha=0.9, label="Me (blue)", zorder=3)
        # 敌方轨迹（红）
        ax.plot(foe_x, foe_y, "-", color="#d62728", lw=1.4, alpha=0.9, label="Foe (red)", zorder=3)

        # 起点/终点标记
        ax.plot(me_x[0], me_y[0], "o", color="#1f77b4", ms=7, label="Me start", zorder=4)
        ax.plot(me_x[-1], me_y[-1], "s", color="#1f77b4", ms=7, label="Me end", zorder=4)
        ax.plot(foe_x[0], foe_y[0], "o", color="#d62728", ms=7, label="Foe start", zorder=4)
        ax.plot(foe_x[-1], foe_y[-1], "s", color="#d62728", ms=7, label="Foe end", zorder=4)

    # 迷宫区域边框
    ax.add_patch(plt.Rectangle((0, 0), MAZE_SIZE * TILE_PX, MAZE_SIZE * TILE_PX,
                               fill=False, edgecolor="#555", lw=1.4, zorder=2))

    ax.set_aspect("equal", adjustable="box")
    # 游戏坐标系 y 向下为正（屏幕坐标），matplotlib 默认向上 —— 反转 y 轴保持一致
    ax.invert_yaxis()
    ax.set_xlabel("x (px, map origin at top-left)")
    ax.set_ylabel("y (px, map origin at top-left)")
    maze_note = (" | 迷宫 %d 墙块" % len(walls)) if walls else " | 无迷宫数据"
    duration = rows[-1]["t"] - rows[0]["t"] if rows else 0.0
    ax.set_title("Tank tracks  %s\n%d frames, %.1fs%s" % (
        os.path.basename(csv_path), len(rows), duration, maze_note))
    if rows:
        ax.legend(loc="upper right", fontsize=8)
    ax.grid(False)

    # 限制显示范围：迷宫区 + 一点外边距
    ax.set_xlim(-30, MAZE_SIZE * TILE_PX + 30)
    ax.set_ylim(MAZE_SIZE * TILE_PX + 30, -30)

    out_png = os.path.splitext(csv_path)[0] + ".png"
    fig.savefig(out_png, dpi=110, bbox_inches="tight")
    print("已保存: %s" % out_png)


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else latest_csv()
    if not csv_path or not os.path.exists(csv_path):
        print("没找到 CSV，请指定路径: conda run -n base python data log\\plot_tracks.py <file.csv>")
        sys.exit(1)
    rows = load(csv_path)
    print("读取 %s: %d 帧" % (csv_path, len(rows)))
    plot(csv_path, rows)


if __name__ == "__main__":
    main()
