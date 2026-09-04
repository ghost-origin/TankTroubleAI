/* probe_runtime3.js —— 在真实游戏物理里重现卡死场景：
   把车放到 (46,286)（地图坐标，世界 +197.5/31），角度 279°，按 up 看是否前进；
   再放 99°（反向）对比；再测原地转动。 */
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const { JSDOM, VirtualConsole } = require(process.env.JSDOM_PATH + '/node_modules/jsdom');

const ROOT = path.join(__dirname, '..');
const GAME_DIR = path.join(ROOT, 'src', 'tanktrouble');

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
  window.requestAnimationFrame = cb => setTimeout(() => cb(Date.now() + 250), 4);
  window.cancelAnimationFrame = id => clearTimeout(id);
  window.matchMedia = () => ({ matches: false, addListener() {}, removeListener() {}, addEventListener() {}, removeEventListener() {} });
  Object.defineProperty(window, 'AudioContext', { value: class { constructor() { return up; } }, writable: true, configurable: true });
  Object.defineProperty(window, 'webkitAudioContext', { value: window.AudioContext, writable: true, configurable: true });
  window.URL.createObjectURL = () => 'blob:fake';
  const ctx = dom.getInternalVMContext();
  ctx.WebSocket = WebSocket;
  for (const s of ['jquery-2.1.1.min.js', 'ga_init.js', 'firebase.js', 'pathfind.js', 'c2runtime.js']) {
    vm.runInContext(fs.readFileSync(path.join(GAME_DIR, s), 'utf8'), ctx, { filename: s });
  }
  const rt = vm.runInContext('cr_createRuntime("c2canvas")', ctx, { filename: 'startup' });
  window.__rt = rt;
  const oc = rt.areAllTexturesAndSoundsLoaded.bind(rt);
  rt.areAllTexturesAndSoundsLoaded = function () { const sv = this.preloadSounds; this.preloadSounds = false; const r = oc(); this.preloadSounds = sv; return r; };
  const TIME_SCALE = 1;
  const oa = rt.kahanTime.add.bind(rt.kahanTime);
  rt.kahanTime.add = dt => oa(dt * TIME_SCALE);
  const og = rt.getDt ? rt.getDt.bind(rt) : null;
  if (og) rt.getDt = inst => og(inst) * TIME_SCALE;
  return { dom, window, rt, ctx };
}

function press(w, key, code, down) {
  const ev = new w.KeyboardEvent(down ? 'keydown' : 'keyup', {
    key: key, code: key, keyCode: code, which: code, bubbles: true, cancelable: true
  });
  try { Object.defineProperty(ev, 'keyCode', { get: () => code }); } catch (e) {}
  try { Object.defineProperty(ev, 'which', { get: () => code }); } catch (e) {}
  w.document.dispatchEvent(ev);
}

(async () => {
  const { rt, ctx, window } = bootGame();
  await new Promise(r => setTimeout(r, 2500));
  try {
    rt.groups_by_name['gamep1'].setGroupActive(true);
    rt.groups_by_name['firing'].setGroupActive(true);
    rt.changelayout = rt.layouts_by_index[4];
  } catch (e) { console.log('enter layout fail:', e.message); }
  await new Promise(r => setTimeout(r, 5000));
  let me0 = null;
  const t0w = Date.now();
  while (!me0 && Date.now() - t0w < 25000) {
    if (rt.running_layout && rt.running_layout.name !== 'Menu') {
      me0 = rt.types_by_index[0] && rt.types_by_index[0].instances.length ? rt.types_by_index[0].instances[0] : null;
    }
    if (!me0) {
      try {
        rt.groups_by_name['gamep1'].setGroupActive(true);
        rt.groups_by_name['firing'].setGroupActive(true);
        rt.changelayout = rt.layouts_by_index[4];
      } catch (e) {}
      await new Promise(r => setTimeout(r, 500));
    }
  }
  if (!me0) { console.log('no me after wait'); process.exit(1); }
  console.log('initial me=(' + me0.x.toFixed(1) + ',' + me0.y.toFixed(1) + ') a=' +
    (me0.angle * 180 / Math.PI).toFixed(1) + 'deg');
  const car = me0.behavior_insts.find(b => b.steerSpeed !== undefined);
  console.log('car keys:', car ? Object.keys(car).filter(k => /speed|enabled|angle|input|steer/i.test(k)).join(',') : 'none');

  async function testDrive(label, angleDeg, ms) {
    me0.x = 197.5 + 46; me0.y = 31 + 286;
    me0.angle = angleDeg * Math.PI / 180;
    if (car) { car.s = 0; car.a = 0; if (car.m !== undefined) car.m = 0; }
    await new Promise(r => setTimeout(r, 80));
    const x0 = me0.x, y0 = me0.y;
    press(window, 'ArrowUp', 38, true);
    await new Promise(r => setTimeout(r, ms));
    press(window, 'ArrowUp', 38, false);
    await new Promise(r => setTimeout(r, 120));
    const dx = me0.x - x0, dy = me0.y - y0;
    const dist = Math.hypot(dx, dy);
    const dir = dist > 0.5 ? ((Math.atan2(dy, dx) * 180 / Math.PI + 360) % 360).toFixed(1) + 'deg' : 'NO MOVE';
    console.log(label + ': start(46,286)a=' + angleDeg + ' -> end=(' + (me0.x - 197.5).toFixed(1) + ',' +
      (me0.y - 31).toFixed(1) + ') moved=' + dist.toFixed(2) + 'px dir=' + dir + ' angle now=' +
      (me0.angle * 180 / Math.PI).toFixed(1));
  }

  await testDrive('A 279deg', 279, 500);
  await testDrive('B 99deg', 99, 500);
  await testDrive('C 90deg', 90, 500);
  // 原地转右 1s
  me0.x = 197.5 + 46; me0.y = 31 + 286; me0.angle = 279 * Math.PI / 180;
  press(window, 'ArrowRight', 39, true);
  await new Promise(r => setTimeout(r, 1000));
  press(window, 'ArrowRight', 39, false);
  console.log('D rotate-right 1s: angle=' + (me0.angle * 180 / Math.PI).toFixed(1) + ' pos=(' +
    (me0.x - 197.5).toFixed(1) + ',' + (me0.y - 31).toFixed(1) + ')');
  process.exit(0);
})().catch(e => { console.error('FAIL', e.stack || e.message); process.exit(1); });
