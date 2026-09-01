const mazes = JSON.parse(require('fs').readFileSync('C:/Users/77665/Desktop/TankTrouble/assets/mazes.json', 'utf8'));
console.log('=== mazes.json keys:', Object.keys(mazes).slice(0, 10));
for (const k of Object.keys(mazes).slice(0, 3)) {
  const v = mazes[k];
  console.log('key=' + k, Array.isArray(v) ? 'array len=' + v.length : typeof v,
    Array.isArray(v) ? '首项=' + JSON.stringify(v[0]).slice(0, 200) : JSON.stringify(v).slice(0, 200));
}
const s = require('fs').readFileSync('C:/Users/77665/Desktop/TankTrouble/src/data.js.base', 'utf8').replace(/^\uFEFF/, '');
const data = JSON.parse(s);
let found = 0;
(function walk(v) {
  if (typeof v === 'string') {
    if (v.includes('c2array')) {
      found++;
      if (found <= 2) {
        const p = JSON.parse(v);
        console.log('=== c2array #' + found, 'size=' + JSON.stringify(p.size), '首格=' + JSON.stringify(p.data[0][0]));
      }
    }
  } else if (Array.isArray(v)) v.forEach(walk);
  else if (v && typeof v === 'object') Object.values(v).forEach(walk);
})(data);
console.log('c2array 字符串总数:', found);
