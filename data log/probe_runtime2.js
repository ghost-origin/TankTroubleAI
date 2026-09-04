/* probe_runtime2.js —— 从运行时 type38 提取真实墙数据，对比 Python 建模 */
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
  const TIME_SCALE = 2;
  const oa = rt.kahanTime.add.bind(rt.kahanTime);
  rt.kahanTime.add = dt => oa(dt * TIME_SCALE);
  const og = rt.getDt ? rt.getDt.bind(rt) : null;
  if (og) rt.getDt = inst => og(inst) * TIME_SCALE;
  return { dom, window, rt, ctx };
}

(async () => {
  const { rt, ctx } = bootGame();
  await new Promise(r => setTimeout(r, 2500));
  try {
    rt.groups_by_name['gamep1'].setGroupActive(true);
    rt.groups_by_name['firing'].setGroupActive(true);
    rt.changelayout = rt.layouts_by_index[4];
  } catch (e) { console.log('enter layout fail:', e.message); }
  await new Promise(r => setTimeout(r, 5000));

  const ty = rt.types_by_index[38];
  console.log('type38 exists=', !!ty, 'name=', ty && ty.name);
  if (ty) {
    const tpolys = ty.tile_polys;
    console.log('tile_polys_len=', tpolys ? tpolys.length : 'none',
      'tileW=', ty.tileW, 'tileH=', ty.tileH,
      'polysKeys=', Object.keys(ty).filter(k => /poly|tile/i.test(k)).join(','));
    if (tpolys) {
      fs.writeFileSync(path.join(__dirname, 'runtime_tile_polys.json'), JSON.stringify(tpolys));
      console.log('saved runtime_tile_polys.json; tid0=' + JSON.stringify(tpolys[0]) +
        ' tid1=' + JSON.stringify(tpolys[1]) + ' tid2=' + JSON.stringify(tpolys[2]));
    }
    const tm = ty.instances && ty.instances.length ? ty.instances[0] : null;
    if (tm) {
      console.log('TM inst x=' + tm.x + ' y=' + tm.y + ' tilewidth=' + tm.tilewidth +
        ' tileheight=' + tm.tileheight + ' mapwidth=' + tm.mapwidth + ' mapheight=' + tm.mapheight +
        ' tilexoffset=' + tm.tilexoffset + ' tileyoffset=' + tm.tileyoffset +
        ' scroll=' + tm.scrollFactorX + ',' + tm.scrollFactorY);
      const w = tm.mapwidth || tm.tilemap_width || 0, h = tm.mapheight || tm.tilemap_height || 0;
      const rows = [];
      for (let yy = 0; yy < h; yy++) {
        const row = [];
        for (let xx = 0; xx < w; xx++) row.push(tm.getTileAt(xx, yy));
        rows.push(row);
      }
      fs.writeFileSync(path.join(__dirname, 'runtime_maze.csv'), rows.map(r => r.join(',')).join('\n'));
      console.log('saved runtime_maze.csv (' + w + 'x' + h + ')');
      console.log('grid sample row0=' + JSON.stringify(rows[0]));
    }
  }
  process.exit(0);
})().catch(e => { console.error('FAIL', e.stack || e.message); process.exit(1); });
