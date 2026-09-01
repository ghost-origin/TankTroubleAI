# -*- coding: utf-8 -*-
"""游戏状态数据类 —— 桥（ai-bridge.js）上传数据的唯一载体。

为什么单独一个文件：
  桥采集到的每一帧数据都解析成 GameState 对象，Python 端（bot 等）
  只认这个类，不直接碰原始 JSON —— 看这个文件就知道"桥传了什么"。

坐标系约定：
  - 以「地图左上角」为原点（即游戏世界坐标 (197.5, 31) 处）
  - x 向右为正，y 向下为正，单位：像素
  - 角度：弧度，0 = 朝右（+x），顺时针增大（朝下为 +π/2）
  - 速度：像素 / 游戏秒

用法：
    state = GameState.from_dict(桥收到的JSON)
    for tank in state.tanks: ...
"""

from dataclasses import dataclass, field
from typing import List


# 地图左上角的世界坐标（迷宫 10×10 区域的左上角）
MAP_ORIGIN_X = 197.5
MAP_ORIGIN_Y = 31.0


@dataclass
class Tank:
    """一辆坦克（我方 / 敌方都在这）"""
    x: float      # 中心 X（地图坐标系）
    y: float      # 中心 Y（地图坐标系）
    angle: float  # 朝向（弧度，0=朝右，顺时针），炮塔方向 = 车体方向
    vx: float     # 速度 X（像素/游戏秒）
    vy: float     # 速度 Y（像素/游戏秒）


@dataclass
class Bullet:
    """一发子弹"""
    x: float    # 中心 X（地图坐标系）
    y: float    # 中心 Y（地图坐标系）
    vx: float   # 速度 X（像素/游戏秒）
    vy: float   # 速度 Y（像素/游戏秒）


@dataclass
class Powerup:
    """一个道具"""
    x: float    # 中心 X（地图坐标系）
    y: float    # 中心 Y（地图坐标系）


@dataclass
class GameState:
    """一帧完整的游戏状态（桥每 ~30Hz 上传一次）"""
    t: float                    # 游戏时间（秒）
    tanks: List[Tank]           # 场上所有坦克（我方 + 敌方）
    bullets: List[Bullet]       # 场上所有子弹
    powerups: List[Powerup]     # 场上所有道具

    @classmethod
    def from_dict(cls, d: dict) -> "GameState":
        """把桥上传的 JSON（dict）解析成 GameState。

        坐标统一换算：世界坐标 → 地图坐标系（减去地图左上角原点）。
        支持两种输入格式：
          - 新格式（桥改造后）：{"t": .., "tanks": [...], "bullets": [...], "powerups": [...]}
          - 旧格式（当前桥）：{"t": .., "me": {...}, "foe": {...}, "bullets": [...], "powerups": [...]}
        """
        def _tank(o: dict) -> Tank:
            return Tank(
                x=o["x"] - MAP_ORIGIN_X,
                y=o["y"] - MAP_ORIGIN_Y,
                angle=float(o.get("angle", 0.0)),
                vx=float(o.get("vx", 0.0)),
                vy=float(o.get("vy", 0.0)),
            )

        def _bullet(o: dict) -> Bullet:
            return Bullet(
                x=o["x"] - MAP_ORIGIN_X,
                y=o["y"] - MAP_ORIGIN_Y,
                vx=float(o.get("vx", 0.0)),
                vy=float(o.get("vy", 0.0)),
            )

        def _powerup(o: dict) -> Powerup:
            return Powerup(x=o["x"] - MAP_ORIGIN_X, y=o["y"] - MAP_ORIGIN_Y)

        # 新格式：tanks 数组
        tanks_raw = d.get("tanks")
        if isinstance(tanks_raw, list):
            tanks = [_tank(o) for o in tanks_raw if o]
        else:
            # 旧格式：me / foe 两个字段
            tanks = []
            for key in ("me", "foe"):
                o = d.get(key)
                if o:
                    tanks.append(_tank(o))

        return cls(
            t=float(d.get("t", 0.0)),
            tanks=tanks,
            bullets=[_bullet(o) for o in (d.get("bullets") or []) if o],
            powerups=[_powerup(o) for o in (d.get("powerups") or []) if o],
        )

    def describe(self) -> str:
        """一行摘要，方便看日志"""
        return ("t=%.1f 坦克%d 子弹%d 道具%d" % (
            self.t, len(self.tanks), len(self.bullets), len(self.powerups)))

    @property
    def n_tanks(self) -> int:
        """场上坦克数量。

        对局结束判定：双方坦克中一方被击杀（实例销毁）后，桥不再上报该坦克，
        n_tanks 会从 2 降到 1 —— 即「n_tanks < 2 且之前到过 2」= 一局结束。
        注意：我方坦克死亡时桥的 collect() 会直接返回 null（整帧不发），
        所以记录器侧通常看到的是敌方消失（foe 字段缺失）。
        """
        return len(self.tanks)
