// probe-bot.js —— 实证：游戏运行时对外暴露哪些内部数据（供人机 AI 使用）
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const { JSDOM, VirtualConsole } = require(process.env.JSDOM_PATH + '/node_modules/jsdom');
process.on('uncaughtException', e => {
  console.log('UNCAUGHT:', e && e.stack ? e.stack.split('\n').slice(0, 8).join('\n') : e);
  process.exit(1);
});
process.on('unhandledRejection', e => {
  console.log('UNHANDLED:', e && e.stack ? e.stack.split('\n').slice(0, 8).join('\n') : e);
  process.exit(1);
});
const dir = path.join(process.argv[2] || process.cwd(), 'src');

const vc = new VirtualConsole();
vc.on('log', () => {});
vc.on('error', () => {});
const dom = new JSDOM(
  '<!DOCTYPE html><html><head><script></script></head><body><div id="c2canvasdiv"><canvas id="c2canvas" width="965" height="644"></canvas></div></body></html>',
  { url: 'http://localhost:8137/', runScripts: 'outside-only', virtualConsole: vc, pretendToBeVisual: true }
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
    const p = path.join(dir, decodeURIComponent(this._u.split('?')[0].replace(/^\/+/, '')));
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
for (const s of ['jquery-2.1.1.min.js', 'ga_init.js', 'firebase.js', 'pathfind.js', 'c2runtime.js']) {
  vm.runInContext(fs.readFileSync(path.join(dir, s), 'utf8'), ctx, { filename: s });
}
const rt = vm.runInContext('cr_createRuntime("c2canvas")', ctx, { filename: 'startup' });
const oc = rt.areAllTexturesAndSoundsLoaded.bind(rt);
rt.areAllTexturesAndSoundsLoaded = function () { const sv = this.preloadSounds; this.preloadSounds = false; const r = oc(); this.preloadSounds = sv; return r; };

setTimeout(() => {
  // 进入 1P 游戏布局 + 激活游戏组
  try {
    rt.groups_by_name['gamep1'].setGroupActive(true);
    rt.groups_by_name['firing'].setGroupActive(true);
    rt.changelayout = rt.layouts_by_index[4];
  } catch (e) { console.log('enter:', e.message); }
  setTimeout(() => {
    console.log('=== 布局:', rt.running_layout.name, '| 游戏时间:', rt.kahanTime.sum.toFixed(1) + 's ===');
    // 1) 坦克实例（类型 0/9/10/11 = P1~P4）
    for (const ti of [0, 9, 10, 11]) {
      const t = rt.types_by_index[ti];
      if (!t) continue;
      t.instances.slice(0, 2).forEach(inst => {
        const behs = inst.behavior_insts.map(b => {
          const o = {};
          try {
            if (b.type && b.type.name) o.beh = b.type.name;
            if (b.properties) o.props = b.properties.slice(0, 6);
            if (typeof b.speed === 'number') o.speed = b.speed;
            if (typeof b.angle === 'number') o.ang = b.angle;
          } catch (e) {}
          return o;
        });
        console.log('坦克 type#' + ti, JSON.stringify({
          uid: inst.uid, x: +inst.x.toFixed(1), y: +inst.y.toFixed(1),
          angle: +(inst.angle * 180 / Math.PI).toFixed(1) + '°',
          behaviors: behs
        }));
      });
    }
    // 2) 迷宫 Tilemap（类型 38）
    const tm = rt.types_by_index[38];
    if (tm && tm.instances.length) {
      const inst = tm.instances[0];
      const tilemap = inst._tilemap || inst.tilemap || {};
      console.log('迷宫 Tilemap: 实例数=' + tm.instances.length,
        '类型字段:', Object.keys(inst).filter(k => /tile|map|array/i.test(k)).slice(0, 8).join(','));
    }
    // 3) 内置 AI 组（游戏自带的 ComputerAI）
    console.log('内置 AI 事件组:', ['computerai', 'ai2', 'traceangle', 'aistate'].map(g =>
      g + '=' + (rt.groups_by_name[g] ? rt.groups_by_name[g].group_active : '无')).join(' '));
    // 4) 全局变量示例
    console.log('全局变量示例:', Object.keys(rt.all_global_vars || {}).slice(0, 6).map(k => {
      const v = rt.all_global_vars[k];
      return k + '=' + (v && v.getValue ? v.getValue() : v);
    }).join(' '));
    // 5) 事件系统钩子（可在任意事件动作执行前拦截）
    console.log('事件块(可挂钩):', Object.keys(rt.blocksBySid || {}).length, '| 条件:', Object.keys(rt.cndsBySid || {}).length,
      '| 动作:', Object.keys(rt.actsBySid || {}).length);
    process.exit(0);
  }, 6000);
}, 2500);
