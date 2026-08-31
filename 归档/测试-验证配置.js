// 测试-验证配置.js —— 请求服务器实际生成的 data.js，核对各配置项是否按预期生效
// 用法：
//   node 归档\测试-验证配置.js http://127.0.0.1:端口
// 输出各武器的 冲量(速度)/导弹速度/寿命阈值/道具刷新参数，人工核对即可。
const base = process.argv[2] || 'http://127.0.0.1:8000';
(async () => {
  const r = await fetch(base + '/src/data.js');
  const text = await r.text();
  console.log('status', r.status, 'cache-control:', r.headers.get('cache-control'), 'size:', text.length);
  const data = JSON.parse(text.replace(/^\uFEFF/, ''));

  // 1) 物理子弹冲量（速度）
  const impulses = new Set();
  // 2) 导弹/碎片 Bullet 行为速度
  const speeds = {};
  // 3) 寿命阈值 Compare(Timer.TotalTime >= N)
  const thresholds = {};
  // 4) 道具刷新参数
  const respawns = [];
  (function walk(v) {
    if (Array.isArray(v)) {
      if (v.length >= 6 && [44, 45, 50, 273].includes(v[0]) && v[1] === 228 && v[2] === 'Physics' && Array.isArray(v[5]) && v[5][0] && Array.isArray(v[5][0][1])) {
        impulses.add(v[5][0][1][1]);
      }
      if (v.length >= 10 && v[1] === 94 && Array.isArray(v[9]) && Array.isArray(v[9][0]) && Array.isArray(v[9][0][1])) {
        const n = v[9][0][1];
        if (n[0] === 22 && n[2] === 'Timer' && n[3] === 164 && typeof n[1] === 'number' && v[9][2] && v[9][2][1]) {
          thresholds[n[1]] = v[9][2][1][1];
        }
      }
      if (v.length >= 7 && v[0] === 0 && Array.isArray(v[5]) && Array.isArray(v[6]) && v[5].length >= 2 &&
          v[5][0] && v[5][0][1] === 123 && v[5][1] && v[5][1][1] === 94 &&
          v[6].some(a => Array.isArray(a) && a[1] === 121 && a[5] && a[5][0] && a[5][0][1] === 40)) {
        respawns.push({ every: v[5][0][9][0][1][1], cap: v[5][1][9][2][1][1] });
      }
      v.forEach(walk);
    } else if (v && typeof v === 'object') Object.values(v).forEach(walk);
  })(data);

  console.log('子弹冲量(44/45/50/273):', [...impulses].sort((a, b) => a - b).join(', '));
  console.log('寿命阈值 类型->秒:', JSON.stringify(thresholds));
  console.log('道具刷新 [间隔,上限]:', respawns.map(x => '[' + x.every + ',' + x.cap + ']').join(' '));

  // 单人模式刷新事件是否注入（9000000000000001 开头的新事件）
  const sheet4kids = data.project[6][4][1][2][7] || [];
  const has1pRespawn = sheet4kids.some(b => Array.isArray(b) && b[4] === 9000000000000001);
  console.log('单人模式道具刷新事件:', has1pRespawn ? '已注入' : '未注入');
})().catch(e => { console.log('ERR', e.message); process.exit(1); });
