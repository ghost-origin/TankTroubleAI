/* probe_runtime.js —— 从游戏运行时提取真实墙数据，与 Python 端建模对比 */
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

  // ---- dump all type names ----
  const names = [];
  for (let ti = 0; ti < rt.types_by_index.length; ti++) {
    const ty = rt.types_by_index[ti];
    if (ty) names.push(ti + ':' + ty.name + (ty.plugin ? '|' + (ty.plugin.name || ty.plugin) : ''));
  }
  console.log('TYPES(' + names.length + ') ' + JSON.stringify(names));

  // ---- find tilemap types ----
  const cr = vm.runInContext('cr', ctx);
  const tmTypes = [];
  for (let ti = 0; ti < rt.types_by_index.length; ti++) {
    const ty = rt.types_by_index[ti];
    if (ty && ty.plugin && ty.plugin === cr.plugins_.TiledBackground) tmTypes.push(ti);
  }
  for (let ti = 0; ti < rt.types_by_index.length; ti++) {
    const ty = rt.types_by_index[ti];
    if (ty && ty.plugin && ty.plugin === cr.plugins_.Tilemap) tmTypes.push(ti);
  }
  console.log('TILEMAP_TYPE_IDX ' + JSON.stringify(tmTypes));
  for (const ti of tmTypes) {
    const ty = rt.types_by_index[ti];
    let tpolys = null;
    try { tpolys = ty.tile_polys; } catch (e) {}
    const keys = Object.keys(ty).filter(k => /poly|tile|coll/i.test(k));
    console.log('TM' + ti + ' name=' + ty.name + ' keys=' + JSON.stringify(keys) +
      ' tileW=' + ty.tileW + ' tileH=' + ty.tileH +
      ' tile_polys_len=' + (tpolys ? tpolys.length : 'none'));
    if (tpolys && tpolys.length > 2) {
      // save full dump
      fs.writeFileSync(path.join(__dirname, 'runtime_tile_polys.json'), JSON.stringify(tpolys));
      console.log('saved runtime_tile_polys.json, sample tid1=' + JSON.stringify(tpolys[1] || null));
    }
    // instances
    if (ty.instances && ty.instances.length) {
      const tm = ty.instances[0];
      console.log('TM' + ti + ' inst x=' + tm.x + ' y=' + tm.y + ' width=' + tm.width + ' height=' + tm.height +
        ' tilewidth=' + tm.tilewidth + ' tileheight=' + tm.tileheight +
        ' tilexoffset=' + tm.tilexoffset + ' tileyoffset=' + tm.tileyoffset +
        ' scrollFactor=' + tm.scrollFactorX + ',' + tm.scrollFactorY);
      // grid
      try {
        const rows = [];
        for (let yy = 0; yy < tm.height / tm.tileheight; yy++) {
          const row = [];
          for (let xx = 0; xx < tm.width / tm.tilewidth; xx++) row.push(tm.getTileAt(xx, yy));
          rows.push(row);
        }
        const csv = rows.map(r => r.join(',')).join('\n');
        fs.writeFileSync(path.join(__dirname, 'runtime_maze.csv'), csv);
        console.log('saved runtime_maze.csv');
        console.log('grid rows=' + rows.length + ' sample row0=' + JSON.stringify(rows[0]));
      } catch (e) { console.log('grid read err ' + e.message); }
    }
  }

  // ---- me tank geometry ----
  const me0 = rt.types_by_index[0] && rt.types_by_index[0].instances.length ? rt.types_by_index[0].instances[0] : null;
  if (me0) {
    console.log('ME uid=' + me0.uid + ' x=' + me0.x.toFixed(1) + ' y=' + me0.y.toFixed(1) +
      ' angle=' + me0.angle.toFixed(3) + ' width=' + me0.width + ' height=' + me0.height +
      ' imageW=' + (me0.cur_anim ? me0.cur_anim.frames[0].width : '?') +
      ' scale=' + (me0.scaleX || 1) + ',' + (me0.scaleY || 1) +
      ' collisionPoly=' + JSON.stringify(me0.collision_poly || me0.collision_polys || null));
    const car = me0.behavior_insts.find(b => b.steerSpeed !== undefined);
    if (car) console.log('ME car enabled=' + car.enabled + ' steerSpeed=' + car.steerSpeed +
      ' maxAngle?' + JSON.stringify(Object.keys(car).filter(k => /angle|turn|speed/i.test(k))));
    // collision poly from behavior
    const solids = me0.collision_insts || [];
    console.log('ME collisions=' + JSON.stringify(solids.map(s => s ? s.type.name : null)));
  }
  process.exit(0);
})().catch(e => { console.error('FAIL', e.stack || e.message); process.exit(1); });
