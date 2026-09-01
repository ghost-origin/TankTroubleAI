# -*- coding: utf-8 -*-
"""数据记录器：接收桥上传的游戏状态，解析成 GameState，写入 CSV。

用法：
    conda run -n base python "data log\\record_ws.py" [端口] [输出目录]
    （默认端口 8766，输出到脚本所在目录 = data log/）

输出文件（每次启动一套，时间戳一致）：
    track_<stamp>.csv   每帧一行（宽表），字段见 CSV_COLUMNS
    maze_<stamp>.csv    本局迷宫（10 行 10 列，值 = tile id：0=路, >0=墙）

CSV 说明（track_*.csv，每帧一行）：
    t          游戏时间（秒）
    me_x/me_y  我方坦克中心坐标（地图坐标系，左上角为原点）
    me_angle   我方坦克朝向（弧度，0=朝右，顺时针）
    me_vx/me_vy 我方坦克速度（像素/游戏秒）
    foe_*      敌方坦克同上
    n_bullets  场上子弹数
    n_powerups 场上道具数

对局结束判定：双方坦克都出现过（n_tanks == 2）后，
n_tanks 降到 1 = 一方被击杀，一局结束（见 game_state.py 的 n_tanks）。

控制命令（通过 WebSocket 发送）：
    {"cmd": "stop"}   结束记录并退出
"""
import json
import os
import sys
import time
import csv

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))          # ws_server
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "python"))  # game_state

from ws_server import WSServer
from game_state import GameState

CSV_COLUMNS = [
    "t",
    "me_x", "me_y", "me_angle", "me_vx", "me_vy",
    "foe_x", "foe_y", "foe_angle", "foe_vx", "foe_vy",
    "n_bullets", "n_powerups",
]

MAZE_SIZE = 10        # 迷宫 10×10（瓦片图画布左上角区域）
TILE_ID_MASK = 0x1FFFFFFF


def parse_tile(v):
    """桥上传的 tile 原始值 → tile id（0=路, >0=墙, -1=空白）"""
    if v is None:
        return -1
    if v == -1:
        return -1
    return (v & 0xFFFFFFFF) & TILE_ID_MASK


class Recorder:
    def __init__(self, port, out_dir):
        self.port = port
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(out_dir, "track_%s.csv" % stamp)
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=CSV_COLUMNS)
        self.writer.writeheader()
        self.rows = 0
        self.maze_path = os.path.join(out_dir, "maze_%s.csv" % stamp)
        self.maze_written = False
        self.seen_both = False   # 双方坦克是否都出现过（对局结束判定用）
        # 僵局/对局结束检测：双方连续多少游戏秒无位置变化即判定结束
        self.still_since = None            # 最近一次双方都在动/静止切换的 t
        self.last_me = None
        self.last_foe = None
        self.STILL_TIMEOUT = 8.0           # 双方静止超该游戏秒即判对局结束
        print("记录文件: %s" % self.csv_path, flush=True)
        print("迷宫文件: %s" % self.maze_path, flush=True)

    def on_client(self, client):
        client.on_message = lambda text: self.on_message(client, text)
        print("连接:", client.addr, flush=True)

    def on_message(self, client, text):
        try:
            msg = json.loads(text)
        except Exception:
            return
        # 控制命令
        if isinstance(msg, dict) and "cmd" in msg:
            if msg["cmd"] == "stop":
                print("收到 stop，结束记录", flush=True)
                self.close()
                os._exit(0)
            return
        # 迷宫（map 字段，桥在布局变化/3s 周期时发）
        if isinstance(msg, dict) and msg.get("map"):
            self._save_maze(msg["map"])
        # 游戏状态帧
        try:
            st = GameState.from_dict(msg)
        except Exception as e:
            print("解析失败:", e, flush=True)
            return
        if st.n_tanks < 2:
            if self.seen_both:
                # 双方出现过，现在只剩一辆 = 一方被击杀，对局结束
                print("对局结束：坦克数 %d -> %d (t=%.1f)" % (2, st.n_tanks, st.t), flush=True)
                self._end_round()
            return  # 开局过渡帧 / 对局结束帧都不记录
        self.seen_both = True
        me, foe = st.tanks[0], st.tanks[1]

        # 僵局检测：双方都无位置变化持续 STILL_TIMEOUT 游戏秒 → 对局结束
        self._check_still(st.t, me, foe)

        if getattr(self, "round_ended", False):
            return  # 对局已结束，不再记录后续帧

        self.writer.writerow({
            "t": round(st.t, 2),
            "me_x": round(me.x, 1), "me_y": round(me.y, 1),
            "me_angle": round(me.angle, 3),
            "me_vx": round(me.vx, 1), "me_vy": round(me.vy, 1),
            "foe_x": round(foe.x, 1), "foe_y": round(foe.y, 1),
            "foe_angle": round(foe.angle, 3),
            "foe_vx": round(foe.vx, 1), "foe_vy": round(foe.vy, 1),
            "n_bullets": len(st.bullets),
            "n_powerups": len(st.powerups),
        })
        self.csv_file.flush()   # 每帧落盘，随时可查
        self.rows += 1
        if self.rows % 200 == 0:
            print("已记录 %d 帧 (t=%.1f)" % (self.rows, st.t), flush=True)

    def _check_still(self, t, me, foe):
        """双方位置都无变化累计 STILL_TIMEOUT 游戏秒 → 判定对局结束。

        位置用保留1位小数的坐标比较（桥上报已 round 到 0.1px），
        坦克物理碰撞导致微小抖动会被忽略。
        """
        me_pos = (round(me.x, 1), round(me.y, 1))
        foe_pos = (round(foe.x, 1), round(foe.y, 1))
        if me_pos == self.last_me and foe_pos == self.last_foe:
            if self.still_since is None:
                self.still_since = t
                self.last_me = me_pos
                self.last_foe = foe_pos
            elif t - self.still_since >= self.STILL_TIMEOUT:
                print("对局结束：双方静止 %.0f 游戏秒 (t=%.1f)" % (self.STILL_TIMEOUT, t), flush=True)
                self._end_round()
                self.still_since = None   # 防止重复触发
        else:
            # 有移动，重置静止计时
            self.still_since = None
        self.last_me = me_pos
        self.last_foe = foe_pos

    def _end_round(self):
        """一局结束：标记已结束，停止再写帧（记录器进程由 launcher 管理，不退出）。"""
        if getattr(self, "round_ended", False):
            return
        self.round_ended = True
        print("== 对局结束，停止记录 (共 %d 帧) ==" % self.rows, flush=True)

    def _save_maze(self, mapinfo):
        """把桥传来的迷宫网格（左上角 10×10）写进 maze CSV（固定 10 行 10 列）。

        存每个格的【原始 tile 值】—— 含翻转/旋转标志位，以便绘图端还原墙线方向。
        值 = tile_id(低29位) | 标志(高3位)，如 -1610612734 = tile 2 + rot90。
        0 = 无墙线（路），-1 = 画布空白。
        """
        grid = mapinfo.get("grid") or []
        if len(grid) < MAZE_SIZE:
            return
        rows = []
        for y in range(MAZE_SIZE):
            row = grid[y]
            rows.append(",".join(str(int(row[x])) for x in range(MAZE_SIZE)))
        with open(self.maze_path, "w", encoding="utf-8") as f:
            f.write("\n".join(rows) + "\n")
        if not self.maze_written:
            self.maze_written = True
            print("迷宫已写入 %s" % self.maze_path, flush=True)

    def close(self):
        self.csv_file.close()
        print("完成: %s （%d 帧）" % (self.csv_path, self.rows), flush=True)


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8766
    out_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(os.path.abspath(__file__))
    rec = Recorder(port, out_dir)
    ws = WSServer("127.0.0.1", port, rec.on_client)
    print("记录服务监听 ws://127.0.0.1:%d/ai" % port, flush=True)
    try:
        ws.serve_forever()
    except KeyboardInterrupt:
        rec.close()


if __name__ == "__main__":
    main()
