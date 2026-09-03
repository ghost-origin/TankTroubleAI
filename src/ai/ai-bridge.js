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
    var lastGridSig = '';                   // 迷宫签名变化 → 清空可视化残留
    var lastDebug = null;                   // 最近一次 bot 调试数据（tick 重画用）
    var tilemapOriginLogged = false;        // tilemap 世界坐标一次性日志
    var identityLogged = false;             // 坦克身份一次性日志
    var lastIdentityWarnT = 0;
    var lastMeUid = -999;                   // me 实例 uid（变化 = 新局信号）
    var lastLayoutName = '';                // 布局名（变化 = 对局切换信号）
    var lastMePosW = null;                  // me 世界位置（瞬移检测）
    var lastEventLog = '';                  // 事件日志去重
    var debugRxLogged = false;              // debug 接收日志（一次）
    var lastT = null;
    var lastActionAt = 0;                   // 最近一次收到 bot 动作的时间（键位看门狗）
    var statusEl = null;

    // 玩家坦克类型 0；敌方 AI 坦克类型 9；炮塔类型集合（容器兄弟对象）
    var ME_TYPE = 0, FOE_TYPE = 9;
    var TURRET_TYPES = { 17: 1, 18: 1, 19: 1, 20: 1, 30: 1 };
    var BULLET_TYPES = { 44: 1, 45: 1, 50: 1, 273: 1, 49: 1, 53: 1, 172: 1, 276: 1 };
    var POWERUP_TYPE = 40;

    function rt() { return window.__rt || null; }

    // 新局信号 → 覆盖写 round_reset.csv（launcher 端），bot 发现覆盖即重置。
    // 与 WS 事件帧双保险：即使状态帧在间隙被吞，文件信号也能让 bot 换轮。
    var resetSeq = 0;
    function signalRoundReset() {
        resetSeq += 1;
        try {
            fetch('/round-reset?seq=' + resetSeq, { cache: 'no-store' }).catch(function () {});
        } catch (e) {}
    }

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
    var DEATH_ANIM_RE = /(death$|explo|boom|blast|destroy|^dead)/i;
    var foeWasDead = false;
    var meWasDead = false;

    function readGrid(tm, w, h) {
        var grid = [];
        for (var yy = 0; yy < h; yy++) {
            var row = [];
            for (var xx = 0; xx < w; xx++) row.push(tm.getTileAt(xx, yy));
            grid.push(row);
        }
        return grid;
    }

    function gridReady(grid, w, h) {
        if (!(w > 0 && h >= 2)) return false;
        var ok = 0;
        for (var gi = 0; gi < Math.min(h, 3); gi++) {
            var r0 = grid[gi];
            for (var gj = 0; gj < w; gj++) {
                if (r0[gj] !== -1 && r0[gj] !== 0) { ok++; break; }
            }
        }
        return ok >= 2;
    }

    function collect() {
        var r = rt();
        if (!r) return null;
        var t = r.kahanTime ? r.kahanTime.sum : 0;
        var out = { t: +t.toFixed(2), layout: r.running_layout ? r.running_layout.name : '' };
        var me = firstInst(ME_TYPE);
        var foe = firstInst(FOE_TYPE);
        out.me_present = !!me;
        out.foe_present = !!foe;

        // 死亡/消失事件（从有到无/动画死亡序列）—— 必须在任何 early return 之前：
        // 局间 running_layout 可能为 null，若提前返回则 uid/瞬移/布局信号全断
        out.event = '';
        var meUid = me ? me.uid : -1;
        if (lastMeUid !== -999 && lastMeUid !== meUid) {
            out.event = meUid === -1 ? 'me_vanish' : 'me_respawn';
        }
        lastMeUid = meUid;
        // 保险丝：me 位置瞬移 >250px（uid 不变的重置型刷新）；跨死亡保留位置
        if (me) {
            if (lastMePosW && Math.hypot(me.x - lastMePosW.x, me.y - lastMePosW.y) > 250) {
                out.event = (out.event ? out.event + ',' : '') + 'me_teleport';
            }
            lastMePosW = { x: me.x, y: me.y };
        }
        // 布局切换信号（对局结束回菜单/再开始）
        var layName = r.running_layout ? r.running_layout.name : '';
        if (lastLayoutName !== '' && lastLayoutName !== layName) {
            out.event = (out.event ? out.event + ',' : '') + 'layout_changed';
        }
        lastLayoutName = layName;
        // 事件日志（变化时打印）：无头/网页 F12 均可读
        if (out.event && out.event !== lastEventLog) {
            lastEventLog = out.event;
            console.log('EVENT ' + out.event + ' meUid=' + meUid + ' lay=' + layName);
            signalRoundReset();   // 轮次边界信号：launcher 覆盖 round_reset.csv
        }

        // 身份日志（第一次 me 出现时）：type 0/9 的实例数与类型名 —— 验证桥
        // 读取的"我方"是否就是画布上的 Player 1 绿坦克
        if (me && !identityLogged) {
            identityLogged = true;
            try {
                var t0 = r.types_by_index[0], t9 = r.types_by_index[9];
                console.log('IDENTITY meType=' + (me.type.name + '#uid' + me.uid) +
                    ' meCount=' + (t0 ? t0.instances.length : -1) +
                    ' mePos=' + me.x.toFixed(1) + ',' + me.y.toFixed(1) +
                    ' foeCount=' + (t9 ? t9.instances.length : -1) +
                    ' foePos=' + (foe ? foe.x.toFixed(1) + ',' + foe.y.toFixed(1) : '?'));
                // 全类型实例清单：哪些 type 有实例（确认灰/绿坦克各自动物）
                for (var ti = 0; ti < r.types_by_index.length; ti++) {
                    var ty = r.types_by_index[ti];
                    if (ty && ty.instances && ty.instances.length) {
                        var i0 = ty.instances[0];
                        console.log('INST ' + ti + ' name=' + ty.name + ' n=' + ty.instances.length +
                            ' pos=' + i0.x.toFixed(1) + ',' + i0.y.toFixed(1) +
                            ' anim=' + (i0.cur_animation ? i0.cur_animation.name : ''));
                    }
                }
            } catch (e) { console.log('IDENTITY err ' + e.message); }
        } else if (!me && !identityLogged && lastIdentityWarnT !== null && (Date.now() - lastIdentityWarnT) > 15000) {
            lastIdentityWarnT = Date.now();
            console.log('IDENTITY waiting for me…');
        }

        // 无 layout（菜单/对局间隙）：普通状态帧不发；但事件帧（击杀/重生/瞬移）
        // 必须送达 bot，否则间隙中的死亡信号被吞 → bot 不知已换局。
        if (!r.running_layout) {
            lastDebug = null;      // 缓存也清 —— 否则 tick 用 lastDebug 重画旧路径
            clearDebugOverlay();
            return out.event ? out : null;
        }
        if (me) {
            var meAnim = me.cur_animation ? me.cur_animation.name : '';
            var foeAnim = foe && foe.cur_animation ? foe.cur_animation.name : '';
            // 我方死亡动画（4ms tick 必捕获，即使死亡后同帧重生位置跳变也不漏）
            if (!meWasDead && DEATH_ANIM_RE.test(meAnim)) {
                out.event = 'me_death_anim';
                meWasDead = true;
            }
            if (meAnim !== '' && !DEATH_ANIM_RE.test(meAnim)) meWasDead = false;
            if (foe && !foeWasDead && DEATH_ANIM_RE.test(foeAnim)) {
                out.event = (out.event ? out.event + ',' : '') + 'foe_death_anim';
                foeWasDead = true;
            } else if (!foe && foeWasDead) {
                // 死亡动画后坦克从场消失
                out.event = (out.event ? out.event + ',' : '') + 'foe_vanish';
            }
            if (foe) foeWasDead = false;
        }

        if (!me) {
            // 缺席也发帧（presence 显式上报），避免 bot 靠超时猜测；
            // 同时清空可视化缓存与画布 —— 对局间隙不残留上一局的路径绘制
            // （只 clear 画布不够：tick 会立刻用 lastDebug 重画回来）
            lastDebug = null;
            clearDebugOverlay();
            return out;
        }

        // 玩家炮塔 = 容器的兄弟对象
        var aim = null;
        if (me.siblings) {
            for (var i = 0; i < me.siblings.length; i++) {
                var s = me.siblings[i];
                if (s && TURRET_TYPES[s.type.index] === 1) { aim = s.angle; break; }
            }
        }
        if (aim === null) aim = me.angle;

        // 速度：位置差分（游戏时间，原始全精度值）
        var dt = (lastT !== null && t > lastT) ? (t - lastT) : 0;
        var lpMe = lastPos['me'];
        var meVx = 0, meVy = 0;
        if (lpMe && dt > 0) {
            meVx = (me.x - lpMe.x) / dt;
            meVy = (me.y - lpMe.y) / dt;
        }
        lastPos['me'] = { x: me.x, y: me.y };
        out.me = {
            x: +me.x.toFixed(1), y: +me.y.toFixed(1),
            angle: +me.angle.toFixed(3), aim: +aim.toFixed(3),
            vx: +meVx.toFixed(1), vy: +meVy.toFixed(1),
            anim: me.cur_animation ? me.cur_animation.name : ''
        };
        if (foe) {
            var lpFoe = lastPos['foe'];
            var foeVx = 0, foeVy = 0;
            if (lpFoe && dt > 0) {
                foeVx = (foe.x - lpFoe.x) / dt;
                foeVy = (foe.y - lpFoe.y) / dt;
            }
            lastPos['foe'] = { x: foe.x, y: foe.y };
            out.foe = {
                x: +foe.x.toFixed(1), y: +foe.y.toFixed(1),
                angle: +foe.angle.toFixed(3), aim: +foe.angle.toFixed(3),
                vx: +foeVx.toFixed(1), vy: +foeVy.toFixed(1),
                anim: foe.cur_animation ? foe.cur_animation.name : ''
            };
        }
        lastT = t;

        // 弹丸
        out.bullets = [];
        for (var bt in BULLET_TYPES) {
            var insts = allInst(+bt);
            for (var i2 = 0; i2 < insts.length; i2++) {
                var b = insts[i2];
                var bp = lastPos['b' + b.uid];
                var vx = 0, vy = 0;
                if (bp && dt > 0) {
                    vx = (b.x - bp.x) / dt;
                    vy = (b.y - bp.y) / dt;
                }
                lastPos['b' + b.uid] = { x: b.x, y: b.y };
                out.bullets.push({ x: +b.x.toFixed(1), y: +b.y.toFixed(1),
                                   vx: +vx.toFixed(1), vy: +vy.toFixed(1),
                                   type: +bt, uid: b.uid });   // uid：Python 端跟踪单颗子弹轨迹用
            }
        }
        // 道具
        out.powerups = [];
        var pus = allInst(POWERUP_TYPE);
        for (var j = 0; j < pus.length; j++) {
            out.powerups.push({ x: +pus[j].x.toFixed(1), y: +pus[j].y.toFixed(1) });
        }
        // 迷宫网格：每帧全量附带（100 格，开销可忽略）——消除"3s 地图刷新窗口"。
        // 轮次 reset 后下一帧即可锁图；未就绪时 ready=false 且 grid 可能全 -1。
        var tm = firstInst(38);
        if (tm && tm.getTileAt) {
            var w = tm.mapwidth || tm.tilemap_width || 0;
            var h = tm.mapheight || tm.tilemap_height || 0;
            // 坐标系根基验证：tilemap 世界位置必须与 MAP_ORIGIN(197.5,31) 一致
            if (!tilemapOriginLogged) {
                tilemapOriginLogged = true;
                var lay = r.running_layout;
                var layer0 = lay && lay.layers && lay.layers.length ? lay.layers[0] : null;
                console.log('TILEMAP origin x=' + tm.x + ' y=' + tm.y +
                    ' tileW=' + (tm.tilewidth || 0) + ' tileH=' + (tm.tileheight || 0) +
                    ' mapW=' + w + ' mapH=' + h +
                    ' scrollX=' + (lay ? lay.scrollX : '?') +
                    ' scrollY=' + (lay ? lay.scrollY : '?') +
                    ' layoutScale=' + (lay ? lay.scale : '?') +
                    ' layerScale=' + (layer0 ? layer0.scale : '?') +
                    ' layerParallax=' + (layer0 ? layer0.parallaxX + ',' + layer0.parallaxY : '?') +
                    ' layerZoom=' + (layer0 ? layer0.zoomRate : '?') +
                    ' drawW=' + r.draw_width + ' drawH=' + r.draw_height +
                    ' layoutW=' + (lay ? lay.originalWidth : '?') + ' layoutH=' + (lay ? lay.originalHeight : '?') +
                    ' gameCanvas=' + (typeof c2canvasW !== 'undefined' ? c2canvasW : '?'));
            }
            var grid = (w > 0 && h > 0) ? readGrid(tm, w, h) : [];
            var sig = grid.length ? (grid[0] || []).join(',') : '';
            if (lastGridSig && sig !== lastGridSig) clearDebugOverlay();
            if (sig) lastGridSig = sig;
            out.map = {
                w: w, h: h,
                tileW: tm.tilewidth || 0, tileH: tm.tileheight || 0,
                x: tm.x || 0, y: tm.y || 0,
                ox: tm.tilexoffset || 0, oy: tm.tileyoffset || 0,
                ready: gridReady(grid, w, h),
                grid: grid,
                // 墙块碰撞多边形（归一化 0~1 坐标，每元素 = tile id 的点列 [x1,y1,x2,y2,...]）
                polys: collectPolys(tm)
            };
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
    // TankTrouble P1 默认开火键是 M（keyCode 77）。鼠标 mousedown/up 事件
    // 在本导出版本中不能可靠触发 Construct 的 Keyboard 条件（v3 实战验证），
    // 开火必须走原生键盘绑定。
    var FIRE_KEY = { key: 'm', domCode: 'KeyM', code: 77 };
    var held = { up: 0, down: 0, left: 0, right: 0 };
    var firing = false;

    function sendKey(name, down) {
        var spec = KEY_MAP[name];
        var ev = new KeyboardEvent(down ? 'keydown' : 'keyup', {
            key: spec.key, code: spec.key, keyCode: spec.code, which: spec.code,
            bubbles: true, cancelable: true
        });
        // 关键：KeyboardEvent 构造函数不接收 keyCode/which，必须强制重定义。
        // 游戏引擎（c2runtime）用 e.which 检测按键——之前只定义了 keyCode、
        // 漏了 which，导致方向键的 which 恒为 0、转向键完全无效（「愣住」根因）。
        try { Object.defineProperty(ev, 'keyCode', { get: function () { return spec.code; } }); } catch (e) {}
        try { Object.defineProperty(ev, 'which', { get: function () { return spec.code; } }); } catch (e) {}
        document.dispatchEvent(ev);
    }
    function sendFireKey(down) {
        var ev = new KeyboardEvent(down ? 'keydown' : 'keyup', {
            key: FIRE_KEY.key, code: FIRE_KEY.domCode,
            keyCode: FIRE_KEY.code, which: FIRE_KEY.code,
            bubbles: true, cancelable: true
        });
        try { Object.defineProperty(ev, 'keyCode', { get: function () { return FIRE_KEY.code; } }); } catch (e) {}
        try { Object.defineProperty(ev, 'which', { get: function () { return FIRE_KEY.code; } }); } catch (e) {}
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
        var locked = !!a.lock;
        ['up', 'down', 'left', 'right'].forEach(function (name) {
            var want = k[name] ? 1 : 0;
            if (locked) {
                // 接管（僵直）：每帧强制同步到 AI 期望状态。
                // want=1 → 每帧发 keydown，want=0 → 每帧发 keyup，用比玩家
                // 键盘重复（~30ms）更高的频率（~15ms）覆盖玩家真实输入，
                // 实现「玩家 ⬆⬇⬅➡ 均无法操作」。
                sendKey(name, want === 1);
                held[name] = want;
            } else if (want !== held[name]) {
                sendKey(name, want === 1);
                held[name] = want;
            }
        });
        if (typeof a.mx === 'number' && typeof a.my === 'number') {
            sendMouse('mousemove', a.mx, a.my);
        }
        if (a.debug) { lastDebug = a.debug; drawDebugOverlay(a.debug);
            if (!debugRxLogged) { debugRxLogged = true; console.log('DEBUG rx ok, path len=' + a.debug.path.length); } }
        else { lastDebug = null; clearDebugOverlay();
            if (!debugRxLogged) { debugRxLogged = true; console.log('DEBUG rx: action without debug (visualize off?)'); } }
        var wantFire = a.fire ? 1 : 0;
        if (wantFire !== firing) {
            // 开火走游戏原生 P1 键盘绑定（M 键），鼠标事件不可靠（v3 实测）
            sendFireKey(wantFire === 1);
            firing = wantFire;
        }
    }

    // ---------- 导航可视化：path / waypoint / 车体扫掠投影 ----------
    var OVERLAY_ORIGIN_X = 197.5, OVERLAY_ORIGIN_Y = 31.0;
    function clearDebugOverlay() {
        var canvas = window.__aiOverlay;
        if (!canvas) return;
        var ctx2 = canvas.getContext('2d');
        if (ctx2) ctx2.clearRect(0, 0, canvas.width, canvas.height);
    }
    function drawDebugOverlay(dbg) {
        var gc = document.getElementById('c2canvas');
        if (!gc) return;
        var canvas = window.__aiOverlay;
        if (!canvas) {
            canvas = document.createElement('canvas');
            canvas.style.cssText = 'position:fixed;pointer-events:none;z-index:10;background:transparent;';
            document.body.appendChild(canvas);
            window.__aiOverlay = canvas;
        }
        // 对齐游戏画布的真实渲染矩形（含 CSS 缩放/偏移），每帧同步；
        // fixed + 视口坐标，避免继承父元素 transform 造成双重缩放
        var r = gc.getBoundingClientRect();
        canvas.style.left = r.left + 'px';
        canvas.style.top = r.top + 'px';
        canvas.style.width = r.width + 'px';
        canvas.style.height = r.height + 'px';
        canvas.width = Math.max(1, Math.round(r.width));
        canvas.height = Math.max(1, Math.round(r.height));
        var ctx2 = canvas.getContext('2d');
        if (!ctx2) return;
        ctx2.clearRect(0, 0, canvas.width, canvas.height);
        // 探针：overlay 存活标记（右上角红方块）—— 看到=绘制活着
        ctx2.fillStyle = '#f00';
        ctx2.fillRect(canvas.width - 14, 6, 8, 8);
        ctx2.font = '10px monospace';
        ctx2.fillStyle = '#c00';
        ctx2.textBaseline = 'top';
        ctx2.fillText('OVL' + (dbg ? (' path=' + dbg.path.length) : ' null'), canvas.width - 60, 6);
        if (!dbg) return;
        // 世界坐标 → 显示像素（drawW/layoutW/scroll 推导，TILEMAP 日志验证）：
        //   wscale = drawW / layoutWidth；渲染表面 = (world−scroll)×wscale+drawW/2
        //   显示像素 = 渲染表面 × (CSS显示宽 / drawW)
        var rtx = rt();
        var scrollX = 0, scrollY = 0, drawW = gc.width || 965, drawH = gc.height || 644, layW = 965, layH = 644;
        if (rtx && rtx.running_layout) {
            scrollX = rtx.running_layout.scrollX || 0;
            scrollY = rtx.running_layout.scrollY || 0;
            layW = rtx.running_layout.originalWidth || layW;
            layH = rtx.running_layout.originalHeight || layH;
        }
        if (rtx) {
            drawW = rtx.draw_width || drawW;
            drawH = rtx.draw_height || drawH;
        }
        var wscale = drawW / layW, hscale = drawH / layH;
        var sxs = r.width / drawW, sys = r.height / drawH;
        function X(x) { return ((x + OVERLAY_ORIGIN_X - scrollX) * wscale + drawW / 2) * sxs; }
        function Y(y) { return ((y + OVERLAY_ORIGIN_Y - scrollY) * hscale + drawH / 2) * sys; }
        var calibInfo = 'draw=' + drawW + 'x' + drawH + ' wscale=' + wscale.toFixed(3);
        // 扫掠投影：每个采样位姿的车体矩形（蓝，半透明）
        if (dbg.swept && dbg.swept.length) {
            ctx2.strokeStyle = 'rgba(40,140,255,0.65)';
            ctx2.fillStyle = 'rgba(40,140,255,0.16)';
            ctx2.lineWidth = 1;
            for (var i = 0; i < dbg.swept.length; i++) {
                var c = dbg.swept[i];
                if (!c || c.length < 3) continue;
                ctx2.beginPath();
                ctx2.moveTo(X(c[0][0]), Y(c[0][1]));
                for (var j = 1; j < c.length; j++) ctx2.lineTo(X(c[j][0]), Y(c[j][1]));
                ctx2.closePath();
                ctx2.fill();
                ctx2.stroke();
            }
        }
        // 规划路径（橙线）：起点锚定到坦克实时中心（bot 的 path[0] 是
        // 上一次 replan 时的位置，坦克已前进 —— 视觉上必须接在车身上）
        if (dbg.path && dbg.path.length > 1) {
            var rtx2 = rt();
            var meInst = rtx2 && rtx2.types_by_index[0] && rtx2.types_by_index[0].instances.length
                ? rtx2.types_by_index[0].instances[0] : null;
            ctx2.strokeStyle = '#ff8c00';
            ctx2.lineWidth = 2;
            ctx2.beginPath();
            if (meInst) {
                ctx2.moveTo(X(meInst.x - OVERLAY_ORIGIN_X), Y(meInst.y - OVERLAY_ORIGIN_Y));
            } else {
                ctx2.moveTo(X(dbg.path[0][0]), Y(dbg.path[0][1]));
            }
            var startI = 1;
            for (var i = startI; i < dbg.path.length; i++) ctx2.lineTo(X(dbg.path[i][0]), Y(dbg.path[i][1]));
            ctx2.stroke();
            // 起点标记（坦克中心绿点）
            if (meInst) {
                ctx2.fillStyle = '#2e8';
                ctx2.beginPath();
                ctx2.arc(X(meInst.x - OVERLAY_ORIGIN_X), Y(meInst.y - OVERLAY_ORIGIN_Y), 4, 0, 6.2832);
                ctx2.fill();
            }
        }
        // 瞄准光束预览：炮口方向弹道（含反射）前 90px，实时更新，单色
        if (dbg.aim_beam && dbg.aim_beam.length > 1) {
            ctx2.strokeStyle = '#00e5cc';
            ctx2.lineWidth = 2;
            ctx2.beginPath();
            ctx2.moveTo(X(dbg.aim_beam[0][0]), Y(dbg.aim_beam[0][1]));
            for (var bi = 1; bi < dbg.aim_beam.length; bi++) {
                ctx2.lineTo(X(dbg.aim_beam[bi][0]), Y(dbg.aim_beam[bi][1]));
            }
            ctx2.stroke();
        }
        // waypoint（红点）
        if (dbg.waypoints && dbg.waypoints.length) {
            ctx2.fillStyle = '#e33';
            for (var i = 0; i < dbg.waypoints.length; i++) {
                ctx2.beginPath();
                ctx2.arc(X(dbg.waypoints[i][0]), Y(dbg.waypoints[i][1]), 2.5, 0, 6.2832);
                ctx2.fill();
            }
        }
        // 诊断文字：每个候选实体的实时位置（对比截图判定哪个是绿坦克）
        var lines = [];
        try {
            var rtx3 = rt();
            if (rtx3) {
                ctx2.font = '11px monospace';
                ctx2.fillStyle = '#111';
                ctx2.textBaseline = 'top';
                for (var ti2 = 0; ti2 < rtx3.types_by_index.length; ti2++) {
                    var ty2 = rtx3.types_by_index[ti2];
                    if (ty2 && ty2.instances && ty2.instances.length && [0, 9, 10, 11, 12, 13, 14, 15, 16, 17, 20, 30].indexOf(ti2) >= 0) {
                        var i2 = ty2.instances[0];
                        lines.push('type' + ti2 + '(' + ty2.name + ')#' + i2.uid + ': ' +
                            i2.x.toFixed(0) + ',' + i2.y.toFixed(0) +
                            (ty2.instances.length > 1 ? ' x' + ty2.instances.length : ''));
                    }
                }
                for (var li = 0; li < lines.length; li++) {
                    ctx2.fillText(lines[li], 6, 6 + li * 13);
                }
            }
        } catch (e) {}
        // CALIB 参数小字（诊断用，不再画品红框）
        try {
            ctx2.fillStyle = '#f0f';
            ctx2.font = '11px monospace';
            ctx2.textBaseline = 'top';
            ctx2.fillText('CALIB ' + calibInfo, 6, 6 + lines.length * 13 + 4);
        } catch (e) {}
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
                else { status('AI 服务未启动，重试中…', '#a33'); setTimeout(connect, 300); }
            }).catch(function () {
                status('AI 服务未启动，重试中…', '#a33');
                setTimeout(connect, 300);
            });
            return;
        }
        try {
            ws = new WebSocket(target);
        } catch (e) {
            status('WebSocket 连接失败: ' + e.message, '#a33');
            setTimeout(connect, 300);
            return;
        }
        ws.onopen = function () {
            connected = true;
            status('AI 已连接 (' + target + ')', '#183');
        };
        ws.onmessage = function (e) {
            try { applyAction(JSON.parse(e.data)); lastActionAt = Date.now(); } catch (err) {}
        };
        ws.onclose = function () {
            connected = false;
            status('AI 连接断开，重连中…', '#a33');
            setTimeout(connect, 300);
        };
        ws.onerror = function () {
            try { ws.close(); } catch (e) {}
        };
    }

    // ---------- 主循环 ----------
    var lastGTLog = -1;
    var lastDrawAt = 0;
    function releaseAllKeys() {
        ['up', 'down', 'left', 'right'].forEach(function (n) {
            if (held[n]) { sendKey(n, false); }
            held[n] = 0;
        });
        if (firing) { sendFireKey(false); firing = false; }
    }
    function tick() {
        var r = rt();
        try {
            // 可视化重画 20Hz（50ms 节流）：相机跟随平滑、CPU 开销可控
            var nowD = Date.now();
            if (nowD - lastDrawAt >= 50) {
                lastDrawAt = nowD;
                if (lastDebug) drawDebugOverlay(lastDebug);
                else clearDebugOverlay();
            }
        } catch (e) {}
        try {
            var gt = r && r.kahanTime ? Math.floor(r.kahanTime.sum / 5) : -1;
            if (gt !== lastGTLog) {
                lastGTLog = gt;
                var meN = r && r.types_by_index[0] ? r.types_by_index[0].instances.length : -1;
                console.log('bridge t=' + (gt * 5) + ' ws=' + (ws ? ws.readyState : -1) +
                    ' conn=' + connected + ' me=' + meN);
            }
        } catch (e) {}
        // 键位看门狗：250ms 未收到 bot 动作（对局切换间隙桥停发帧 / WS 抖动 /
        // bot 重启）→ 强制释放全部按键与开火。防止上一局的油门/方向键卡住，
        // 新局坦克一出生就沿旧指令乱跑（"击杀后不立即停"的兜底保证）。
        if (lastActionAt && Date.now() - lastActionAt > 250) {
            lastActionAt = 0;
            releaseAllKeys();
        }
        if (connected) {
            var now = Date.now();
            // collect() 内部已处理 layout 为空的场景：普通帧不发、事件帧必发
            if (now - lastSentAt >= MIN_SEND_MS) {
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
