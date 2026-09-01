# 项目目录结构

```text
TankTroubleAI_clean/
├─ assets/                 游戏素材
├─ src/
│  ├─ tanktrouble/         完整 Construct 2 游戏运行时代码
│  └─ ai/ai-bridge.js      游戏 ↔ Python AI WebSocket 接口
├─ python/
│  ├─ navigation_mvp.py    A* / Virtual Tube / 平滑规划
│  ├─ navigation_bot.py    在线导航控制器
│  ├─ score_navigation.py  Combat/导航评分
│  └─ game_state.py        游戏状态解析
├─ data log/               WebSocket 服务、记录、绘图、真实 headless runner
├─ config/config.json      运行参数
├─ test/                   所有 Benchmark / 测试环境 / 基线数据
├─ docs/                   所有项目文档与开发 Roadmap
├─ launcher.py             网页游戏启动器
├─ start_web_game.bat      网页原游戏
├─ start_web_navigation.bat 网页 + 导航 AI
├─ start_web_no_ai.bat     网页无 AI 模式
├─ run_nav_tests.bat       真实无头导航回归
└─ run_nav_combat.bat      Combat 测试入口
```

## 设计原则

- 游戏源码、导航代码和 AI 接口保持在根目录核心代码区。
- 测试相关文件统一进入 `test/`。
- 文档统一进入 `docs/`。
- `test/results/baseline_20260901/` 是当前固定基线，不应被普通回归测试覆盖。
- 新回归默认输出到 `test/results/current/`。
