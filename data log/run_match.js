/* ============================================================
 * run_match.js —— 无头跑一局游戏，把桥采集的数据发给记录器
 *
 * 结束条件：双方坦克都出现后，场上只剩一辆坦克（一方被击杀）
 *          即对局结束；[游戏秒数] 仅为兜底上限（默认 240）。
 *
 * 用法：
 *   set JSDOM_PATH=...\tt3jsdom
 *   node "data log\run_match.js" [记录器端口] [兜底秒数]
 *   （默认端口 8766）
 * ============================================================ */
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const { JSDOM, VirtualConsole } = require(process.env.JSDOM_PATH + '/node_modules/jsdom');

const REC_PORT = parseInt(process.argv[2] || '8766', 10);
const DURATION = parseFloat(process.argv[3] || '90');
const DIAG = !!process.env.DIAG;

const ROOT = path.join(__dirname, '..');
const GAME_DIR = path.join(ROOT, 'src', 'tanktrouble');
const BRIDGE = fs.readFileSync(path.join(ROOT, 'src', 'ai', 'ai-bridge.js'), 'utf8');

function bootGame() {
  const vc = new VirtualConsole();
  vc.on('log', function () { console.log('[js]', Array.prototype.join.call(arguments, ' ')); });
  vc.on('error', () => {});
  const dom = new JSDOM(
    '<!DOCTYPE html><html><head><script></script></head><body><div id="c2canvasdiv"><canvas id="c2canvas" width="965" height="644"></canvas></div></body></html>',
    { url: 'http://localhost:8137/src/tanktrouble/', runScripts: 'outside-only', virtualConsole: vc, pretendToBeVisual: true }
  );
  const { window } = dom;
  function uf() { return up; }
  const up = new Proxy(uf, { get(t, p) { if (p === Symbol.toPrimitive) return () => 0; return up; }, set() { return true; }, apply() { return up; } });
  class XHR {
    constructor() { this.responseType = ''; this.readyState = 0; this._s = 0; this._t = ''; }
    open(m, u) { this._u = u; }
    setRequestHeader() {}
    get response() { return this.responseType === 'json' && this._s === 200 ? JSON.parse(this._t.replace(/^\uFEFF/, '')) : this._t; }
    get responseText() { return this._t; }
    send() {
      const p = path.join(GAME_DIR, decodeURIComponent(this._u.split('?')[0].replace(/^\/+/, '')));
      setTimeout(() => {
        this.readyState = 4;
        this._s = this.status = fs.existsSync(p) ? 200 : 404;
        this._t = this._s === 200 ? fs.readFileSync(p, 'utf8') : '';
        if (this.onreadystatechange) this.onreadystatechange();
        if (this.onload) this.onload();
      }, 2);
    }
  }
  window.XMLHttpRequest = XHR;
  window.Image = class {
    constructor() { this._l = {}; this.naturalWidth = 32; this.naturalHeight = 32; this.complete = false; this.loaded = false; this.src = ''; }
    addEventListener(e, f) { (this._l[e] = this._l[e] || []).push(f); }
    removeEventListener(e, f) { this._l[e] = (this._l[e] || []).filter(g => g !== f); }
    set src(v) { this._src = v; setTimeout(() => { this.complete = true; this.loaded = true; (this._l.load || []).forEach(f => f()); if (this.onload) this.onload(); }, 1); }
    get src() { return this._src; }
  };
  Object.defineProperty(window.HTMLCanvasElement.prototype, 'getContext', { value: () => up, writable: true, configurable: true });
  const oce = window.document.createElement.bind(window.document);
  window.document.createElement = t => String(t).toLowerCase() === 'img' ? new window.Image() : oce(t);
  window.requestAnimationFrame = cb => setTimeout(() => cb(Date.now() + 250), 4);   // 8x 加速
  window.cancelAnimationFrame = id => clearTimeout(id);
  window.matchMedia = () => ({ matches: false, addListener() {}, removeListener() {}, addEventListener() {}, removeEventListener() {} });
  Object.defineProperty(window, 'AudioContext', { value: class { constructor() { return up; } }, writable: true, configurable: true });
  Object.defineProperty(window, 'webkitAudioContext', { value: window.AudioContext, writable: true, configurable: true });
  window.URL.createObjectURL = () => 'blob:fake';

  const ctx = dom.getInternalVMContext();
  // jsdom 不实现 WebSocket —— 注入 Node 全局实现，桥才能连上记录器
  ctx.WebSocket = WebSocket;
  for (const s of ['jquery-2.1.1.min.js', 'ga_init.js', 'firebase.js', 'pathfind.js', 'c2runtime.js']) {
    vm.runInContext(fs.readFileSync(path.join(GAME_DIR, s), 'utf8'), ctx, { filename: s });
  }
  const rt = vm.runInContext('cr_createRuntime("c2canvas")', ctx, { filename: 'startup' });
  window.__rt = rt;
  // 注入 AI 桥（指定 WebSocket 地址，跳过 /ai-port 探测）
  vm.runInContext(
    'window.__AI_WS = "ws://127.0.0.1:' + REC_PORT + '/ai";\n' + BRIDGE,
    ctx, { filename: 'ai-bridge.js' }
  );
  const oc = rt.areAllTexturesAndSoundsLoaded.bind(rt);
  rt.areAllTexturesAndSoundsLoaded = function () { const sv = this.preloadSounds; this.preloadSounds = false; const r = oc(); this.preloadSounds = sv; return r; };
  // 时间倍率：8x 时物理 dt=8/60，坦克每 tick 位移 ~16.7px，薄墙(4-6px)会被
  // 离散碰撞直接跨越（隧道穿墙）。降到 2x：步长 ~4.2px < 墙厚，穿墙消失。
  const TIME_SCALE = 2;
  const oa = rt.kahanTime.add.bind(rt.kahanTime);
  rt.kahanTime.add = dt => oa(dt * TIME_SCALE);
  const og = rt.getDt ? rt.getDt.bind(rt) : null;
  if (og) rt.getDt = inst => og(inst) * TIME_SCALE;
  return { dom, window, rt, ctx };
}

function diagLog(rt, ctx) {
  const me0 = rt.types_by_index[0] && rt.types_by_index[0].instances.length
    ? rt.types_by_index[0].instances[0] : null;
  const foe0 = rt.types_by_index[9] && rt.types_by_index[9].instances.length
    ? rt.types_by_index[9].instances[0] : null;
  const t = rt.kahanTime ? rt.kahanTime.sum : 0;
  if (!me0) { console.log('DIAG t=' + t.toFixed(2) + ' noMe'); return; }
  const car = me0.behavior_insts.find(b => b.steerSpeed !== undefined);
  let solid = null;
  try { solid = rt.testOverlapSolid(me0); } catch (e) {}
  const c = car ? ('en=' + car.enabled + ' ig=' + car.ignoreInput + ' s=' + car.s.toFixed(2) +
    ' a=' + car.a.toFixed(3) + ' m=' + car.m.toFixed(3)) : 'noCar';
  const sname = solid ? (solid.type && solid.type.name) + '#' + solid.uid : 'none';
  let km = '?';
  try {
    const crols = vm.runInContext('cr', ctx);
    for (let ti = 0; ti < rt.types_by_index.length; ti++) {
      const ty = rt.types_by_index[ti];
      if (ty && ty.plugin === crols.plugins_.Keyboard && ty.instances.length) {
        const m = ty.instances[0].keyMap;
        km = '';
        for (let kc = 37; kc <= 40; kc++) km += (m[kc] ? '1' : '0');
        km += '|sp=' + (m[32] ? '1' : '0');
        break;
      }
    }
  } catch (e) { km = 'err:' + e.message; }
  console.log('DIAG t=' + t.toFixed(2) + ' lay=' + rt.running_layout.name +
    ' me=(' + me0.x.toFixed(1) + ',' + me0.y.toFixed(1) + ')' +
    ' ang=' + me0.angle.toFixed(3) + ' [' + c + '] solid=' + sname + ' km=' + km +
    ' foe=(' + (foe0 ? foe0.x.toFixed(1) + ',' + foe0.y.toFixed(1) : '?') + ')');
}

(async () => {
  console.log('=== 无头跑一局（记录数据）===');
  console.log('记录器端口:', REC_PORT, '| 兜底上限:', DURATION + 's（正常由"只剩一辆坦克"结束）');
  const { rt, ctx } = bootGame();
  if (DIAG) setInterval(() => { try { diagLog(rt, ctx); } catch (e) { console.log('DIAG err:', e.message); } }, 600);
  await new Promise(r => setTimeout(r, 2500));
  try {
    rt.groups_by_name['gamep1'].setGroupActive(true);
    rt.groups_by_name['firing'].setGroupActive(true);
    rt.changelayout = rt.layouts_by_index[4];
  } catch (e) { console.log('进入对局失败:', e.message); }
  // 等对局开始 + 桥连上记录器
  await new Promise(r => setTimeout(r, 4000));

  // 坦克数量读取（类型 0 = 我方，9 = 敌方）
  function tankCounts() {
    const meN = rt.types_by_index[0] ? rt.types_by_index[0].instances.length : 0;
    const foeN = rt.types_by_index[9] ? rt.types_by_index[9].instances.length : 0;
    return { meN, foeN, total: meN + foeN };
  }

  // 等双方坦克都出现（开局过渡期可能只有一辆）
  let seenBoth = false;
  const waitStart = Date.now();
  while (!seenBoth && Date.now() - waitStart < 30000) {
    await new Promise(r => setTimeout(r, 200));
    seenBoth = tankCounts().total >= 2;
  }
  if (!seenBoth) console.log('警告: 30s 内未等到双方坦克，继续运行');

  // ---- 复现注入：TT_MAZE_CSV 指向迷宫 CSV（如 r56 局），注入真实 tilemap 并
  // 把坦克挪到指定位置/朝向（TT_SPAWN_X/Y/ANG 度），敌方挪远 —— 用于固定地图评测。
  // bot 端锁图靠"地图签名变化"自动重锁（首帧已锁旧图时注入后必触发 reset）。
  if (process.env.TT_MAZE_CSV) {
    try {
      const rows = fs.readFileSync(process.env.TT_MAZE_CSV, 'utf8').trim().split('\n')
        .map(l => l.split(',').map(v => parseInt(v, 10)));
      const tm = rt.types_by_index[38] && rt.types_by_index[38].instances.length
        ? rt.types_by_index[38].instances[0] : null;
      if (tm && tm.setTileAt) {
        for (let yy = 0; yy < 20; yy++) for (let xx = 0; xx < 30; xx++) tm.setTileAt(xx, yy, -1);
        for (let yy = 0; yy < rows.length; yy++) {
          for (let xx = 0; xx < rows[yy].length; xx++) tm.setTileAt(xx, yy, rows[yy][xx]);
        }
        console.log('[inj] maze injected rows=' + rows.length);
      } else {
        console.log('[inj] WARN tilemap not ready');
      }
      const me0 = rt.types_by_index[0] && rt.types_by_index[0].instances.length
        ? rt.types_by_index[0].instances[0] : null;
      if (me0) {
        const sx = parseFloat(process.env.TT_SPAWN_X || '57');
        const sy = parseFloat(process.env.TT_SPAWN_Y || '285');
        const sa = parseFloat(process.env.TT_SPAWN_ANG || '195');
        me0.x = 197.5 + sx; me0.y = 31 + sy; me0.angle = sa * Math.PI / 180;
        console.log('[inj] me -> map(' + sx + ',' + sy + ') ang=' + sa);
      }
      const foeTy = rt.types_by_index[9];
      if (foeTy && foeTy.instances.length) {
        const fo0 = foeTy.instances[0];
        fo0.x = 197.5 + 300; fo0.y = 31 + 300; fo0.angle = 0;
        console.log('[inj] foe moved away');
      }
    } catch (e) { console.log('[inj] fail: ' + e.message); }
  }

  const start = rt.kahanTime.sum;
  const SAFETY_CEIL = DURATION;   // 兜底上限 = 标准局时长（无头下游戏 60s 计时不触发）
  let lastT = start;
  let roundStart = null;          // 本局开始（我方坦克出现）时刻，兜底从此刻起算
  let reason = '';
  let diagLast = 0;
  let lastMePos = null;
  function meFoe() {
    const me0 = rt.types_by_index[0] && rt.types_by_index[0].instances[0];
    const foe0 = rt.types_by_index[9] && rt.types_by_index[9].instances[0];
    return { me0, foe0 };
  }
  while (true) {
    await new Promise(r => setTimeout(r, 100));
    const t = rt.kahanTime.sum;
    if (t - lastT >= 10) {
      lastT = t;
      const c = tankCounts();
      console.log('游戏时间 %.0f s | 坦克 %d (我%d 敌%d)' .replace('%.0f', t.toFixed(0)), c.total, c.meN, c.foeN);
    }
    // 对局结束判定：双方都出现过之后，场上只剩一辆坦克 = 一方被击杀
    const c = tankCounts();
    if (seenBoth && c.total < 2) { reason = '一方被击杀，场上仅剩 ' + c.total + ' 辆坦克'; break; }
    // 我方坦克消失/重生（被击杀即场终）：一次测试 = 一局比赛，
    // 不在同一场内滚动多局（否则一场出现多份 maze/track，评测口径混乱）。
    // meN 在死亡瞬间可能被立即 respawn 掩盖，用多重信号：
    const { me0, foe0 } = meFoe();
    const meAnim = me0 && me0.cur_animation ? me0.cur_animation.name : '';
    const ME_DEAD_RE = /(death$|explo|boom|blast|destroy|^dead)/i;
    if (seenBoth && !me0) { reason = '我方坦克消失（被击杀）'; break; }
    if (seenBoth && me0 && ME_DEAD_RE.test(meAnim)) { reason = '我方坦克死亡动画(' + meAnim + ')'; break; }
    if (seenBoth && me0) {
      if (lastMePos) {
        const dj = Math.hypot(me0.x - lastMePos.x, me0.y - lastMePos.y);
        if (dj > 250) { reason = '我方坦克重生跳变 ' + dj.toFixed(0) + 'px'; break; }
      }
      lastMePos = { x: me0.x, y: me0.y };
    } else if (me0) lastMePos = { x: me0.x, y: me0.y };
    // 回到菜单/切布局
    if (rt.running_layout.name !== '1 Player') { reason = '离开对局布局'; break; }
    // 兜底：超时上限从本局开始（我方坦克出现）起算 —— 严格对齐标准局时长
    const { me0: meNow } = meFoe();
    if (!roundStart && meNow) roundStart = t;
    if (roundStart && t - roundStart > SAFETY_CEIL) { reason = '超时兜底 (' + SAFETY_CEIL + 's)'; break; }
  }
  console.log('对局结束（%s），实际游戏时长: ' + (rt.kahanTime.sum - start).toFixed(1) + 's', reason);
  console.log('数据已由记录器写入 CSV，等待 1s 落盘…');
  setTimeout(() => process.exit(0), 1000);
})().catch(e => { console.error('失败:', e.message); process.exit(1); });
