// 1) mazes.json 结构解剖
const mazes = JSON.parse(require('fs').readFileSync('C:/Users/77665/Desktop/TankTrouble/assets/mazes.json', 'utf8'));
console.log('=== mazes.json 顶层类型:', Array.isArray(mazes) ? '数组 len=' + mazes.length : typeof mazes);
if (Array.isArray(mazes)) {
  console.log('第一项:', JSON.stringify(mazes[0]).slice(0, 300));
  if (mazes.length > 1) console.log('第二项:', JSON.stringify(mazes[1]).slice(0, 300));
}

// 2) data.js 里的默认迷宫 c2array（WALLSDEFAULT / MAZEFULLARRAY）
const raw = require('fs').readFileSync('C:/Users/77665/Desktop/TankTrouble/src/data.js.base', 'utf8').replace(/^\uFEFF/, '');
const data = JSON.parse(raw);
const hits = [];
(function walk(v) {
  if (typeof v === 'string') {
    if (v.includes('c2array') && v.includes('WALLS') || v.includes('c2array') && v.includes('MAZE')) hits.push(v);
  } else if (Array.isArray(v)) v.forEach(walk);
  else if (v && typeof v === 'object') Object.values(v).forEach(walk);
})(data);
console.log('\n=== data.js 中 c2array 迷宫字符串数:', hits.length);
if (hits.length) {
  const parsed = JSON.parse(hits[0]);
  console.log('WALLSDEFAULT 结构: size=' + JSON.stringify(parsed.size), 'data 行数=' + parsed.data.length,
    '首行=' + JSON.stringify(parsed.data[0]));
  const mf = hits.find(h => h.includes('MAZEFULLARRAY'));
  if (mf) {
    const p = JSON.parse(mf);
    console.log('MAZEFULLARRAY: size=' + JSON.stringify(p.size), '首格=' + JSON.stringify(p.data[0][0]));
  }
}

// 3) Tilemap 类型(38)的行为（有没有 Solid）
const t38 = data.project[3][38];
console.log('\n=== type#38 行为:', Array.isArray(t38[8]) ? t38[8].map(b => b[0]).join(',') : '?');
