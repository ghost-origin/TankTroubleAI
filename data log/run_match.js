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

const ROOT = path.join(__dirname, '..');
const GAME_DIR = path.join(ROOT, 'src', 'tanktrouble');
const BRIDGE = fs.readFileSync(path.join(ROOT, 'src', 'ai', 'ai-bridge.js'), 'utf8');

function bootGame() {
  const vc = new VirtualConsole();
  vc.on('log', () => {});
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
  const oa = rt.kahanTime.add.bind(rt.kahanTime);
  rt.kahanTime.add = dt => oa(dt * 8);
  const og = rt.getDt ? rt.getDt.bind(rt) : null;
  if (og) rt.getDt = inst => og(inst) * 8;
  return { dom, window, rt };
}

(async () => {
  console.log('=== 无头跑一局（记录数据）===');
  console.log('记录器端口:', REC_PORT, '| 兜底上限:', DURATION + 's（正常由"只剩一辆坦克"结束）');
  const { rt } = bootGame();
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

  // 先等双方坦克都出现（开局过渡期可能只有一辆）
  let seenBoth = false;
  const waitStart = Date.now();
  while (!seenBoth && Date.now() - waitStart < 30000) {
    await new Promise(r => setTimeout(r, 200));
    seenBoth = tankCounts().total >= 2;
  }
  if (!seenBoth) console.log('警告: 30s 内未等到双方坦克，继续运行');

  const start = rt.kahanTime.sum;
  const SAFETY_CEIL = Math.max(DURATION, 240);   // 兜底上限（游戏秒），正常由坦克数决定结束
  let lastT = start;
  let reason = '';
  // 双方静止检测（与记录器呼应）：双方位置连续 STILL_SEC 游戏秒不变即结束
  const STILL_SEC = 8.0;
  let lastMeInst = null, lastFoeInst = null, stillSince = null;
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
    // 回到菜单/切布局
    if (rt.running_layout.name !== '1 Player') { reason = '离开对局布局'; break; }
    // 双方静止检测：位置都不变持续 STILL_SEC 游戏秒 → 对局已结束/僵局
    const { me0, foe0 } = meFoe();
    if (me0 && foe0) {
      const mePos = me0.x.toFixed(1) + ',' + me0.y.toFixed(1);
      const foePos = foe0.x.toFixed(1) + ',' + foe0.y.toFixed(1);
      if (mePos === lastMeInst && foePos === lastFoeInst) {
        if (stillSince === null) stillSince = t;
        else if (t - stillSince >= STILL_SEC) { reason = '双方静止 ' + STILL_SEC + 's（对局已结束）'; break; }
      } else stillSince = null;
      lastMeInst = mePos; lastFoeInst = foePos;
    }
    // 兜底：超时上限（正常情况下不会触发）
    if (t - start > SAFETY_CEIL) { reason = '超时兜底 (' + SAFETY_CEIL + 's)'; break; }
  }
  console.log('对局结束（%s），实际游戏时长: ' + (rt.kahanTime.sum - start).toFixed(1) + 's', reason);
  console.log('数据已由记录器写入 CSV，等待 1s 落盘…');
  setTimeout(() => process.exit(0), 1000);
})().catch(e => { console.error('失败:', e.message); process.exit(1); });
