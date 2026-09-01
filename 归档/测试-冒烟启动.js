// 测试-冒烟启动.js —— 在模拟浏览器环境（jsdom）里启动游戏，验证游戏数据/脚本没有损坏
// 用法（在游戏根目录执行）：
//   npm install jsdom --prefix "%TEMP%\tt3jsdom"          （只需装一次）
//   set JSDOM_PATH=%TEMP%\tt3jsdom
//   node 归档\测试-冒烟启动.js
// 输出 "errors: 0" 且 runtime created 即通过。
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const { JSDOM, VirtualConsole } = require(process.env.JSDOM_PATH + '/node_modules/jsdom');
const dir = path.join(process.cwd(), 'src', 'tanktrouble');   // 游戏运行文件在 src/ 下

const vc = new VirtualConsole();
const msgs = [];
vc.on('log', (...a) => msgs.push(a.join(' ')));
vc.on('error', (...a) => msgs.push('console.error: ' + a.join(' ')));

const dom = new JSDOM(
  '<!DOCTYPE html><html><head><script></script></head><body><div id="c2canvasdiv"><canvas id="c2canvas" width="965" height="644"></canvas></div></body></html>',
  { url: 'http://localhost:8137/', runScripts: 'outside-only', virtualConsole: vc, pretendToBeVisual: true }
);
const { window } = dom;
const errors = [];
window.addEventListener('error', e => errors.push(e.message || String(e.error)));

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
  try { vm.runInContext(fs.readFileSync(path.join(dir, s), 'utf8'), ctx, { filename: s }); }
  catch (e) { errors.push('EVAL ' + s + ': ' + e.message); }
}
let rt = null;
try { rt = vm.runInContext('cr_createRuntime("c2canvas")', ctx, { filename: 'startup' }); }
catch (e) { errors.push('START: ' + e.message); }
if (rt) {
  const oc = rt.areAllTexturesAndSoundsLoaded.bind(rt);
  rt.areAllTexturesAndSoundsLoaded = function () { const sv = this.preloadSounds; this.preloadSounds = false; const r = oc(); this.preloadSounds = sv; return r; };
}
setTimeout(() => {
  console.log('=== 冒烟测试结果 ===');
  console.log('runtime:', rt ? 'created' : 'NULL', '| isloading:', rt && rt.isloading, '| progress:', rt && rt.progress);
  console.log('errors (' + errors.length + '):');
  errors.forEach(e => console.log('  ' + e));
  console.log('console messages:', msgs.length);
  msgs.slice(0, 5).forEach(m => console.log('  ' + m.slice(0, 120)));
  process.exit(errors.length ? 1 : 0);
}, 6000);
