# -*- coding: utf-8 -*-
"""
坦克动荡 3 本地版启动器
- 读取 config.json，按配置动态生成游戏数据 data.js（基准 = data.js.base 纯净原版）
- 修改 config.json 后刷新浏览器即生效，无需重启
- 自动寻找空闲端口，绑定 127.0.0.1（绕开 localhost 的 IPv6 坑）
- 自动打开浏览器；关闭本窗口或按 Ctrl+C 即停止
"""
import http.server
import json
import os
import socket
import socketserver
import subprocess
import sys
import time
import webbrowser

BASE_FILE = os.path.join("src", "tanktrouble", "data.js.base")  # 纯净原版游戏数据（不要改）
CONFIG_FILE = os.path.join("config", "config.json")  # 玩家配置文件（可以随便改）
OUT_FILE = os.path.join("src", "tanktrouble", "data.js")  # 生成的游戏数据（每次启动/配置变更时重新生成）
SRC_DIR = "src"      # 游戏运行文件所在目录（相对页面加载）
ASSETS_PREFIX = "../../assets/"  # 贴图/音效等资源目录（相对 src/tanktrouble/ 页面）
AI_SERVER = os.path.join("data log", "record_ws.py")  # 数据记录服务（桥 → GameState → CSV）

DEFAULT_CONFIG = {
    "子弹速度倍率": 1.0,      # 普通/大/小/直线子弹的发射冲量倍率
    "导弹速度倍率": 1.0,      # 追踪导弹/RC导弹速度倍率
    "爆炸碎片速度倍率": 1.0,  # 坦克爆炸碎片速度倍率
    "子弹存留时间倍率": 1.0,  # 所有子弹/导弹存活时间倍率
    "单人模式道具刷新": True, # 单人（人机）模式是否刷新道具
    "道具刷新间隔秒": 12,     # 道具每多少秒检查一次
    "道具场上数量上限": 3,    # 场上最多同时存在的道具数（含刚生成的）
}

# ---- 配置加载 ----
def load_config():
    cfg = dict(DEFAULT_CONFIG)
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8-sig") as f:
                user = json.load(f)
            for k in cfg:
                if k in user and user[k] is not None:
                    if isinstance(cfg[k], bool):
                        cfg[k] = bool(user[k])
                    else:
                        try:
                            v = float(user[k])
                            cfg[k] = v if isinstance(cfg[k], float) else int(v)
                        except (TypeError, ValueError):
                            pass
        except Exception as e:
            print("警告: 配置文件解析失败，使用默认值 (%s)" % e)
    # 数值合理性保护
    cfg["子弹速度倍率"] = max(0.01, cfg["子弹速度倍率"])
    cfg["导弹速度倍率"] = max(0.01, cfg["导弹速度倍率"])
    cfg["爆炸碎片速度倍率"] = max(0.01, cfg["爆炸碎片速度倍率"])
    cfg["子弹存留时间倍率"] = max(0.01, cfg["子弹存留时间倍率"])
    cfg["道具刷新间隔秒"] = max(1, int(cfg["道具刷新间隔秒"]))
    cfg["道具场上数量上限"] = max(1, int(cfg["道具场上数量上限"]))
    return cfg

# ---- 数据补丁 ----
def walk(obj, fn):
    """深度遍历 JSON 树，fn 收到每个数组（含它自己）"""
    if isinstance(obj, list):
        fn(obj)
        for el in obj:
            walk(el, fn)
    elif isinstance(obj, dict):
        for v in obj.values():
            walk(v, fn)

BULLET_TYPES = {44, 45, 50, 273}          # 物理子弹：普通/小/大/直线
MISSILE_TYPES = {49: 2, 53: 1}            # 导弹类型 -> Bullet 行为在类型中的序号
FRAGMENT_TYPES = {51: 0, 52: 0}           # 爆炸碎片 -> Bullet 行为序号
ALL_PROJECTILE = BULLET_TYPES | set(MISSILE_TYPES) | set(FRAGMENT_TYPES)

REF_IMPULSE = 228          # Physics.ApplyImpulseAtAngle
REF_COMPARE = 94           # System.Compare（用于 TotalTime 寿命阈值）
REF_EVERY = 123            # System.Every
REF_CREATE = 121           # System.CreateObject
REF_WAIT = 68              # System.Wait
REF_PATH_FAIL = 138        # Pathfinding.OnFailedToFindPath
REF_FINDPATH = 134         # Pathfinding.FindPath
POWERUP_TYPE = 40          # 道具对象类型

def patch_data(data, cfg):
    # 1) 物理子弹冲量（速度）倍率
    def patch_impulse(arr):
        if (len(arr) >= 6 and isinstance(arr[0], int) and arr[0] in BULLET_TYPES
                and arr[1] == REF_IMPULSE and arr[2] == "Physics"
                and isinstance(arr[5], list) and arr[5] and isinstance(arr[5][0], list)
                and isinstance(arr[5][0][1], list) and arr[5][0][1][0] in (0, 1)):
            arr[5][0][1][1] = round(arr[5][0][1][1] * cfg["子弹速度倍率"], 6)
    walk(data, patch_impulse)

    # 2) 导弹 / 3) 碎片：Bullet 行为默认速度（布局实例的属性数组）
    def patch_speed(arr):
        if (isinstance(arr, list) and len(arr) > 4 and isinstance(arr[1], int)
                and isinstance(arr[4], list)):
            if arr[1] in MISSILE_TYPES:
                bi = MISSILE_TYPES[arr[1]]
                if len(arr[4]) > bi and isinstance(arr[4][bi], list) and arr[4][bi]:
                    arr[4][bi][0] = round(arr[4][bi][0] * cfg["导弹速度倍率"], 6)
            elif arr[1] in FRAGMENT_TYPES:
                bi = FRAGMENT_TYPES[arr[1]]
                if len(arr[4]) > bi and isinstance(arr[4][bi], list) and arr[4][bi]:
                    arr[4][bi][0] = round(arr[4][bi][0] * cfg["爆炸碎片速度倍率"], 6)
    walk(data, patch_speed)

    # 4) 子弹/导弹存留时间倍率：真正的寿命机制是
    #    "ForEach 子弹 -> Compare Timer.TotalTime('time') >= 阈值 -> Destroy"
    #    即 Compare 条件里的阈值（不是 StartTimer 时长，那个只是启动计时器）
    def patch_lifetime(arr):
        if (len(arr) >= 10 and isinstance(arr, list) and arr[1] == REF_COMPARE
                and isinstance(arr[9], list) and len(arr[9]) >= 3
                and isinstance(arr[9][0], list) and isinstance(arr[9][0][1], list)
                and arr[9][0][1][0] == 22          # 行为表达式
                and arr[9][0][1][2] == "Timer"      # Timer 行为
                and arr[9][0][1][3] == 164          # TotalTime
                and isinstance(arr[9][0][1][1], int)
                and arr[9][0][1][1] in ALL_PROJECTILE  # 子弹/导弹类型
                and isinstance(arr[9][2], list) and isinstance(arr[9][2][1], list)
                and arr[9][2][1][0] in (0, 1)):
            arr[9][2][1][1] = round(arr[9][2][1][1] * cfg["子弹存留时间倍率"], 6)
    walk(data, patch_lifetime)

    # 5) 单人模式道具刷新：把 2 人事件表里的刷新事件克隆进 1 人事件表
    if cfg["单人模式道具刷新"]:
        sheet4 = data["project"][6][4]          # 1 Player Sheet
        sheet5 = data["project"][6][5]          # 2 Player Sheet
        src = sheet5[1][2][7][0][7][3][7][5]    # 2P 的刷新事件块
        if isinstance(src, list) and src[0] == 0:
            clone = json.loads(json.dumps(src))
            # 换全新事件 ID（确定性生成，避免与现有 ID 冲突）
            clone[4] = 9000000000000001
            for i, c in enumerate(clone[5]):
                c[7] = 9000000000000003 + i
            for i, a in enumerate(clone[6]):
                a[3] = 9000000000000007 + i
            ev2 = sheet4[1][2]
            if isinstance(ev2, list) and isinstance(ev2[7], list):
                ev2[7].append(clone)

    # 6) 道具刷新间隔 / 场上上限（1P~4P 所有刷新事件统一生效）
    def patch_respawn(arr):
        if (len(arr) >= 7 and arr[0] == 0 and isinstance(arr[5], list) and isinstance(arr[6], list)
                and len(arr[5]) >= 2
                and isinstance(arr[5][0], list) and arr[5][0][1] == REF_EVERY
                and isinstance(arr[5][1], list) and arr[5][1][1] == REF_COMPARE
                and any(isinstance(a, list) and a[1] == REF_CREATE
                        and a[5] and isinstance(a[5][0], list) and a[5][0][1] == POWERUP_TYPE
                        for a in arr[6])):
            try:
                arr[5][0][9][0][1][1] = cfg["道具刷新间隔秒"]   # Every X 秒（条件参数在 [9]）
                arr[5][1][9][2][1][1] = cfg["道具场上数量上限"]  # 数量 <= N
            except Exception:
                pass
    walk(data, patch_respawn)

    # 7) 修复游戏 BUG：OnFailedToFindPath -> FindPath 同步重试会无限递归爆栈
    #    （寻路持续失败时，失败回调里触发的事件又同步调 FindPath → 栈溢出）。
    #    在重试 FindPath 前插入 Wait(0.5) 动作，把重试变为异步。
    _wait_sid = 9000000000000010
    def patch_pathfail(arr):
        nonlocal _wait_sid
        if (len(arr) >= 7 and arr[0] == 0 and isinstance(arr[5], list) and isinstance(arr[6], list)
                and any(isinstance(c, list) and c[1] == REF_PATH_FAIL for c in arr[5])
                and any(isinstance(a, list) and a[1] == REF_FINDPATH for a in arr[6])):
            # 已有 Wait 则跳过（幂等）
            if any(isinstance(a, list) and a[1] == REF_WAIT for a in arr[6]):
                return
            idx = next(i for i, a in enumerate(arr[6]) if isinstance(a, list) and a[1] == REF_FINDPATH)
            _wait_sid += 1
            arr[6].insert(idx, [-1, REF_WAIT, None, _wait_sid, False, [[0, [0, 0.5]]]])
    walk(data, patch_pathfail)

    return data

def rewrite_media_paths(obj):
    """把 data.js 里的资源文件名统一加上 assets/ 前缀（贴图/迷宫数据已移到 assets 目录）。
    音频元数据（project[7]）不在这里加前缀——音频播放/预载路径由 files_subfolder（project[8]）统一处理。"""
    MEDIA_EXTS = ('.png', '.jpg', '.jpeg', '.gif', '.webp', '.m4a', '.ogg', '.mp3',
                  '.wav', '.ttf', '.woff', '.woff2', '.json')

    def walk(v):
        if isinstance(v, str):
            low = v.lower()
            if (low.endswith(MEDIA_EXTS)
                    and not low.startswith(('http://', 'https://', 'data:', '//', ASSETS_PREFIX))):
                return ASSETS_PREFIX + v
            return v
        if isinstance(v, list):
            for i in range(len(v)):
                v[i] = walk(v[i])
        elif isinstance(v, dict):
            for k in list(v.keys()):
                v[k] = walk(v[k])
        return v

    project = obj.get("project")
    if isinstance(project, list):
        for idx, section in enumerate(project):
            if idx == 7:      # 音频元数据表：交给 files_subfolder，避免双重前缀
                continue
            walk(section)
    else:
        walk(obj)

def build_data(cfg):
    with open(BASE_FILE, "r", encoding="utf-8-sig") as f:
        text = f.read()
    bom = "\ufeff" if text.startswith("\ufeff") else ""
    data = json.loads(text.lstrip("\ufeff"))
    data = patch_data(data, cfg)
    # files_subfolder：引擎播放/预载音频时拼路径用（file 基名 + ".ogg"/".m4a"），
    # 原版为空字符串（媒体在页面根目录）；重组后音频在 assets/，这里指向它
    try:
        data["project"][8] = "../../assets/"
    except Exception:
        pass
    rewrite_media_paths(data)
    out = bom + json.dumps(data, ensure_ascii=False, separators=(",", ":"))
    return out.encode("utf-8")

# ---- 缓存：config 变了才重新生成 ----
_cache = {"mtime": 0, "blob": None}

def get_patched_data():
    try:
        mtime = os.stat(CONFIG_FILE).st_mtime
    except OSError:
        mtime = 0
    if _cache["mtime"] != mtime or _cache["blob"] is None:
        cfg = load_config()
        blob = build_data(cfg)
        _cache["mtime"] = mtime
        _cache["blob"] = blob
        # 同步写一份到磁盘，保证文件与配置一致
        try:
            with open(OUT_FILE, "wb") as f:
                f.write(blob)
        except OSError:
            pass
    return _cache["blob"]

# ---- HTTP 服务 ----
_AI_PORT = None  # 由 main() 设置，供 /ai-port 接口返回

class GameHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        sys.stdout.write("[%s] %s\n" % (self.log_date_time_string(), format % args))
        sys.stdout.flush()

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", ""):
            # 根路径 -> 游戏入口
            self.send_response(302)
            self.send_header("Location", "/src/tanktrouble/")
            self.end_headers()
            return
        if path == "/ai-port":
            # 页面桥接脚本通过这里获取 AI 服务端口
            blob = json.dumps({"port": _AI_PORT}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(blob)))
            self.end_headers()
            self.wfile.write(blob)
            return
        if path.endswith("/data.js"):
            blob = get_patched_data()
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Content-Length", str(len(blob)))
            self.end_headers()
            self.wfile.write(blob)
        else:
            super().do_GET()

def find_free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]

def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    cfg = load_config()
    port = find_free_port()

    # 启动 Python AI 桥接服务（WebSocket），端口通过 /ai-port 暴露给页面
    global _AI_PORT
    ai_proc = None
    if os.path.exists(AI_SERVER):
        try:
            _AI_PORT = find_free_port()
            ai_proc = subprocess.Popen(
                [sys.executable, AI_SERVER, str(_AI_PORT)],
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            )
            print("  数据记录服务已启动: ws://127.0.0.1:%d/ai（写入 data log/）" % _AI_PORT)
        except Exception as e:
            print("  AI 桥接服务启动失败（游戏仍可玩，AI 功能不可用）: %s" % e)
            _AI_PORT = None

    httpd = socketserver.TCPServer(("127.0.0.1", port), GameHandler)
    url = "http://127.0.0.1:%d/src/tanktrouble/" % port
    print("=" * 54)
    print("  坦克动荡 3 本地版已启动")
    print("  游戏地址: %s" % url)
    print("  当前配置（config.json，改后刷新浏览器即生效）：")
    for k, v in cfg.items():
        print("    %s = %s" % (k, v))
    print("  关闭本窗口或按 Ctrl+C 即停止游戏服务")
    print("=" * 54)
    try:
        webbrowser.open(url)
    except Exception:
        pass
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
        if ai_proc:
            try:
                ai_proc.terminate()
            except Exception:
                pass

if __name__ == "__main__":
    main()
