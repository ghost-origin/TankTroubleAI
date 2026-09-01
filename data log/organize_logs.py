# -*- coding: utf-8 -*-
"""按时间戳整理 data log 目录：把每局的数据（track/maze csv + 轨迹png）归入子文件夹。

规则：
  - 子文件夹名 = 时间戳（如 20260901_085921）
  - 放入该时间戳的 track_<ts>.csv、maze_<ts>.csv、track_<ts>.png（存在才移动）
  - 工具脚本（record_ws/plot_tracks/run_match/ws_server/tile_polys）不移动
  - 0 字节的空 track 文件（未进入对局）直接删除，不留空文件夹

用法（在 data log 目录或任意处）：
    E:\\anaconda\\python.exe "data log\\organize_logs.py"
"""
import glob
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
TS_RE = re.compile(r"(\d{8}_\d{6})")


def main():
    moved = 0
    removed = 0
    for path in sorted(glob.glob(os.path.join(HERE, "*.csv"))):
        name = os.path.basename(path)
        m = TS_RE.search(name)
        if not m:
            continue  # 不是带时间戳的数据文件
        ts = m.group(1)
        # 空文件（未进入对局）：删除
        if os.path.getsize(path) == 0:
            os.remove(path)
            print("删除空文件: %s" % name)
            removed += 1
            continue
        folder = os.path.join(HERE, ts)
        os.makedirs(folder, exist_ok=True)
        dst = os.path.join(folder, name)
        os.rename(path, dst)
        print("移动: %s -> %s\\" % (name, ts))
        moved += 1
    # 时间戳同名图片（track_<ts>.png）
    for path in sorted(glob.glob(os.path.join(HERE, "track_*.png"))):
        name = os.path.basename(path)
        m = TS_RE.search(name)
        if not m:
            continue
        ts = m.group(1)
        folder = os.path.join(HERE, ts)
        if os.path.isdir(folder):
            os.rename(path, os.path.join(folder, name))
            print("移动: %s -> %s\\" % (name, ts))
            moved += 1
    print("完成: 移动 %d 个文件, 删除 %d 个空文件" % (moved, removed))


if __name__ == "__main__":
    main()
