/* ============================================================
 * ai-bridge.js —— 游戏 ↔ Python AI 的 WebSocket 桥
 *
 * 职责：
 *   1. 从 window.__rt 采集游戏状态（坦克/炮塔/弹丸/道具/迷宫网格），序列化为 JSON
 *   2. 通过 WebSocket 以 ~30Hz 发送给 Python 端
 *   3. 接收 Python 的动作指令（4 方向键 + 开火 + 鼠标瞄准点），
 *      合成真实键盘/鼠标事件注入游戏（走游戏自己的输入链路）
 *
 * 动作协议（Python -> 游戏）：
 *   { "keys": {"up":0/1, "down":0/1, "left":0/1, "right":0/1},
 *     "fire": 0/1, "mx": 像素x, "my": 像素y }
 * ============================================================ */
(function () {
    'use strict';

    var WS_URL = window.__AI_WS || null;   // 无头评估器可直接注入
    var ws = null;
    var connected = false;
    var lastSentAt = 0;
    var MIN_SEND_MS = 15;                   // 状态发送节流（真实毫秒）
    var lastPos = {};                       // 位置差分算速度
    var lastT = null;
    var lastMapKey = '';
    var lastMapSentAt = 0;
    var statusEl = null;

    // 玩家坦克类型 0；敌方 AI 坦克类型 9；炮塔类型集合（容器兄弟对象）
    var ME_TYPE = 0, FOE_TYPE = 9;
    var TURRET_TYPES = { 17: 1, 18: 1, 19: 1, 20: 1, 30: 1 };
    var BULLET_TYPES = { 44: 1, 45: 1, 50: 1, 273: 1, 49: 1, 53: 1, 172: 1, 276: 1 };
    var POWERUP_TYPE = 40;

    function rt() { return window.__rt || null; }

    function firstInst(typeIdx) {
        var t = rt() && rt().types_by_index[typeIdx];
        return t && t.instances.length ? t.instances[0] : null;
    }
    function allInst(typeIdx) {
        var t = rt() && rt().types_by_index[typeIdx];
        return t ? t.instances : [];
    }

    // 收集 Tilemap 类型的墙块碰撞多边形（归一化 0~1 坐标）
    // 来源：类型定义里的 tile_polys（每 tile id 一个点列 [x1,y1,x2,y2,...]）
    function collectPolys(tm) {
        var t = tm && tm.type;
        var src = t && t.tile_polys;
        if (!src || !src.length) return null;
        var out = [];
        for (var i = 0; i < src.length; i++) {
            var p = src[i];
            if (!p || !p.poly) { out.push(null); continue; }
            out.push(p.poly.slice());
        }
        return out;
    }

    // ---------- 状态采集 ----------
    function collect() {
        var r = rt();
        if (!r || !r.running_layout) return null;
        var t = r.kahanTime ? r.kahanTime.sum : 0;
        var me = firstInst(ME_TYPE);
        if (!me) return null;
        var foe = firstInst(FOE_TYPE);

        // 玩家炮塔 = 容器的兄弟对象
        var aim = null;
        if (me.siblings) {
            for (var i = 0; i < me.siblings.length; i++) {
                var s = me.siblings[i];
                if (s && TURRET_TYPES[s.type.index] === 1) { aim = s.angle; break; }
            }
        }
        if (aim === null) aim = me.angle;

        var out = { t: +t.toFixed(2), layout: r.running_layout.name };
        out.me = {
            x: +me.x.toFixed(1), y: +me.y.toFixed(1),
            angle: +me.angle.toFixed(3), aim: +aim.toFixed(3),
            vx: 0, vy: 0
        };
        if (foe) {
            out.foe = {
                x: +foe.x.toFixed(1), y: +foe.y.toFixed(1),
                angle: +foe.angle.toFixed(3), aim: +foe.angle.toFixed(3),
                vx: 0, vy: 0
            };
        }
        // 速度：位置差分（游戏时间）
        var dt = (lastT !== null && t > lastT) ? (t - lastT) : 0;
        for (var key in out) {
            if (key === 'me' || key === 'foe') {
                var o = out[key];
                if (!o) continue;
                var lp = lastPos[key];
                if (lp && dt > 0) {
                    o.vx = +((o.x - lp.x) / dt).toFixed(1);
                    o.vy = +((o.y - lp.y) / dt).toFixed(1);
                }
                lastPos[key] = { x: o.x, y: o.y };
            }
        }
        lastT = t;

        // 弹丸
        out.bullets = [];
        for (var bt in BULLET_TYPES) {
            var insts = allInst(+bt);
            for (var i = 0; i < insts.length; i++) {
                var b = insts[i];
                var bp = lastPos['b' + b.uid];
                var vx = 0, vy = 0;
                if (bp && dt > 0) {
                    vx = +((b.x - bp.x) / dt).toFixed(1);
                    vy = +((b.y - bp.y) / dt).toFixed(1);
                }
                lastPos['b' + b.uid] = { x: b.x, y: b.y };
                out.bullets.push({ x: +b.x.toFixed(1), y: +b.y.toFixed(1), vx: vx, vy: vy, type: +bt });
            }
        }
        // 道具
        out.powerups = [];
        var pus = allInst(POWERUP_TYPE);
        for (var j = 0; j < pus.length; j++) {
            out.powerups.push({ x: +pus[j].x.toFixed(1), y: +pus[j].y.toFixed(1) });
        }
        // 迷宫网格（布局变了才重发）：用 Tilemap 插件的 getTileAt 读取
        var tm = firstInst(38);
        if (tm && tm.getTileAt) {
            var w = tm.mapwidth || tm.tilemap_width || 0;
            var h = tm.mapheight || tm.tilemap_height || 0;
            var mkey = out.layout + '|' + w + 'x' + h;
            if (mkey !== lastMapKey || (Date.now() - (lastMapSentAt || 0)) > 3000) {
                // 周期性重发（3s）：布局变化发一次 + 定时刷新，
                // 避免迷宫生成前的垃圾帧被缓存后永远不更新
                lastMapSentAt = Date.now();
                var grid = [];
                var ready = w > 0 && h > 0;
                for (var yy = 0; yy < h; yy++) {
                    var row = [];
                    for (var xx = 0; xx < w; xx++) {
                        // 原始值（含翻转/旋转标志位）—— Python 端按位解析
                        row.push(tm.getTileAt(xx, yy));
                    }
                    grid.push(row);
                }
                // 就绪判定：至少 2 行非空。合法值：-1(空)/0(路)/正数或带标志位负数(墙)
                if (ready && h >= 2) {
                    var ok = 0;
                    for (var gi = 0; gi < Math.min(h, 3); gi++) {
                        var r0 = grid[gi];
                        var hasReal = false;
                        for (var gj = 0; gj < w; gj++) {
                            if (r0[gj] !== -1 && r0[gj] !== 0) { hasReal = true; break; }
                        }
                        if (hasReal) ok++;
                    }
                    ready = ok >= 2;
                } else ready = false;
                if (ready) lastMapKey = mkey;
                out.map = {
                    w: w, h: h,
                    tileW: tm.tilewidth || 0, tileH: tm.tileheight || 0,
                    x: tm.x || 0, y: tm.y || 0,
                    ox: tm.tilexoffset || 0, oy: tm.tileyoffset || 0,
                    grid: grid,
                    // 墙块碰撞多边形（归一化 0~1 坐标，每元素 = tile id 的点列 [x1,y1,x2,y2,...]）
                    polys: collectPolys(tm)
                };
            }
        }
        return out;
    }

    // ---------- 输入注入（合成键盘/鼠标事件） ----------
    var KEY_MAP = {
        up: { key: 'ArrowUp', code: 38 },
        down: { key: 'ArrowDown', code: 40 },
        left: { key: 'ArrowLeft', code: 37 },
        right: { key: 'ArrowRight', code: 39 }
    };
    var held = { up: 0, down: 0, left: 0, right: 0 };
    var firing = false;

    function sendKey(name, down) {
        var spec = KEY_MAP[name];
        var ev = new KeyboardEvent(down ? 'keydown' : 'keyup', {
            key: spec.key, code: spec.key, keyCode: spec.code, which: spec.code,
            bubbles: true, cancelable: true
        });
        try { Object.defineProperty(ev, 'keyCode', { get: function () { return spec.code; } }); } catch (e) {}
        document.dispatchEvent(ev);
    }
    function sendMouse(type, x, y) {
        var ev = new MouseEvent(type, {
            clientX: x, clientY: y, bubbles: true, cancelable: true, button: 0
        });
        document.dispatchEvent(ev);
    }

    function applyAction(a) {
        if (!a || !a.keys) return;
        var k = a.keys;
        ['up', 'down', 'left', 'right'].forEach(function (name) {
            var want = k[name] ? 1 : 0;
            if (want !== held[name]) {
                sendKey(name, want === 1);
                held[name] = want;
            }
        });
        if (typeof a.mx === 'number' && typeof a.my === 'number') {
            sendMouse('mousemove', a.mx, a.my);
        }
        var wantFire = a.fire ? 1 : 0;
        if (wantFire !== firing) {
            sendMouse(wantFire ? 'mousedown' : 'mouseup', a.mx || 0, a.my || 0);
            firing = wantFire;
        }
    }

    // ---------- WebSocket 连接 ----------
    function status(msg, color) {
        if (!statusEl) {
            statusEl = document.createElement('div');
            statusEl.style.cssText = 'position:fixed;bottom:0;left:0;right:0;z-index:9999;' +
                'background:' + (color || '#333') + ';color:#fff;font:12px monospace;padding:2px 8px;opacity:.85';
            document.body.appendChild(statusEl);
        }
        statusEl.textContent = msg;
        statusEl.style.background = color || '#333';
    }

    function connect() {
        if (ws && (ws.readyState === 0 || ws.readyState === 1)) return;
        var target = WS_URL;
        if (!target) {
            // 通过 /ai-port 从启动器获取端口
            fetch('/ai-port').then(function (r) { return r.json(); }).then(function (d) {
                if (d && d.port) { WS_URL = 'ws://127.0.0.1:' + d.port + '/ai'; connect(); }
                else { status('AI 服务未启动，重试中…', '#a33'); setTimeout(connect, 2000); }
            }).catch(function () {
                status('AI 服务未启动，重试中…', '#a33');
                setTimeout(connect, 2000);
            });
            return;
        }
        try {
            ws = new WebSocket(target);
        } catch (e) {
            status('WebSocket 连接失败: ' + e.message, '#a33');
            setTimeout(connect, 2000);
            return;
        }
        ws.onopen = function () {
            connected = true;
            status('AI 已连接 (' + target + ')', '#183');
        };
        ws.onmessage = function (e) {
            try { applyAction(JSON.parse(e.data)); } catch (err) {}
        };
        ws.onclose = function () {
            connected = false;
            status('AI 连接断开，重连中…', '#a33');
            setTimeout(connect, 2000);
        };
        ws.onerror = function () {
            try { ws.close(); } catch (e) {}
        };
    }

    // ---------- 主循环 ----------
    function tick() {
        var r = rt();
        if (connected) {
            var now = Date.now();
            if (now - lastSentAt >= MIN_SEND_MS && r && r.running_layout) {
                var st = collect();
                if (st && ws && ws.readyState === 1) {
                    try { ws.send(JSON.stringify(st)); lastSentAt = now; } catch (e) {}
                }
            }
        }
        setTimeout(tick, 4);
    }

    connect();
    tick();
})();
