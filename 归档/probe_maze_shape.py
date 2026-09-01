# -*- coding: utf-8 -*-
"""验证迷宫生成机制：把 mazes.json 一张迷宫 + tile_poly_data 多边形，
精确画成世界坐标下的墙线（应用翻转/旋转），确认真实的墙长什么样。

用法（Py 侧绘到 PNG）：
    E:\\anaconda\\python.exe 归档\\probe_maze_shape.py <迷宫键 如 0>
"""
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAZES = os.path.join(ROOT, "assets", "mazes.json")

TILE_PX = 57
FLAG_H = 0x80000000
FLAG_V = 0x40000000
FLAG_D = 0x20000000
TILE_ID_MASK = 0x1FFFFFFF


def rot_to_flags(rot):
    """StateComboToFlags 等价：rotation 90/180/270 -> flags"""
    r = int(rot)
    if r == 0:
        return 0
    if r == 90:
        return FLAG_H | FLAG_D
    if r == 180:
        return FLAG_H | FLAG_V
    if r == 270:
        return FLAG_V | FLAG_D
    return 0


# tile_poly_data: 每个 tile id 的归一化 poly（从 type38 提取，前 6 个 + 常见边线）
POLYS = {
    0: [0.4737, 0.4737, 0.5263, 0.4737, 0.5263, 0.5263, 0.4737, 0.5263],
    1: [0.4737, 0, 0.5263, 0, 0.5263, 0.5263, 0.4737, 0.5263],
    2: [0.4737, 0, 0.5263, 0, 0.5263, 0.4737, 1, 0.4737, 1, 0.5263, 0.4737, 0.5263],
    3: [0.4737, 0.4737, 0.4737, 0, 0.5263, 0, 0.5263, 0.4737, 1, 0.4737, 1, 0.5263, 0, 0.5263, 0, 0.4737],
    4: [0.5263, 0, 0.5263, 0.4737, 1, 0.4737, 1, 0.5263, 0.5263, 0.5263, 0.5263, 1, 0.4737, 1, 0.4737, 0.5263, 0, 0.5263, 0, 0.4737, 0.4737, 0.4737, 0.4737, 0],
    5: [0.4737, 0, 0.5263, 0, 0.5263, 1, 0.4737, 1],
    9: [0, 0, 2.0/57, 0, 2.0/57, 1, 0, 1],
    10: [0, 0, 1, 0, 1, 2.0/57, 0, 2.0/57],
    12: [0, 55.0/57, 1, 55.0/57, 1, 1, 0, 1],
    13: [54.9/57, 0, 1, 0, 1, 1, 54.9/57, 0.999],
}


def transform(poly, fliph, flipv, flipd):
    pts = [(x * TILE_PX, y * TILE_PX) for x, y in zip(poly[0::2], poly[1::2])]
    if flipd:
        pts = [(y, x) for (x, y) in pts]
    if fliph:
        pts = [(TILE_PX - x, y) for (x, y) in pts]
    if flipv:
        pts = [(x, TILE_PX - y) for (x, y) in pts]
    return pts


def main():
    key = sys.argv[1] if len(sys.argv) > 1 else "0"
    m = json.load(open(MAZES, encoding="utf-8"))[key]
    arr = json.loads(m)
    data = arr["data"]

    segs = []
    for cy, row in enumerate(data):
        for cx, (tid, rot) in enumerate(row):
            tid, rot = int(tid), int(rot)
            if tid <= 0:
                continue
            poly = POLYS.get(tid)
            if not poly:
                continue
            flags = rot_to_flags(rot)
            fliph = bool(flags & FLAG_H)
            flipv = bool(flags & FLAG_V)
            flipd = bool(flags & FLAG_D)
            pts = transform(poly, fliph, flipv, flipd)
            off_x, off_y = cx * TILE_PX, cy * TILE_PX
            wpts = [(off_x + px, off_y + py) for (px, py) in pts]
            n = len(wpts)
            for i in range(n):
                x1, y1 = wpts[i]
                x2, y2 = wpts[(i + 1) % n]
                segs.append(((x1, y1), (x2, y2)))

    fig, ax = plt.subplots(figsize=(8, 8))
    for (a, b) in segs:
        ax.plot([a[0], b[0]], [a[1], b[1]], color="black", lw=2.5)
    # 网格线(格边界)
    for i in range(11):
        ax.axvline(i * TILE_PX, color="#ccc", lw=0.5)
        ax.axhline(i * TILE_PX, color="#ccc", lw=0.5)
    ax.set_aspect("equal")
    ax.invert_yaxis()
    ax.set_xlim(-5, 10 * TILE_PX + 5)
    ax.set_ylim(10 * TILE_PX + 5, -5)
    ax.set_title("Maze #%s - real wall lines (3px thin) from tile_polys" % key)
    out = os.path.join(ROOT, "归档", "maze_shape_%s.png" % key)
    fig.savefig(out, dpi=100, bbox_inches="tight")
    print("saved:", out)


if __name__ == "__main__":
    main()
