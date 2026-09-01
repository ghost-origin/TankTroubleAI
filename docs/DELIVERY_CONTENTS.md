# 本次完整交付内容

该压缩包是完整工程，不是单独 Patch。包含：

- 完整 TankTrouble 游戏运行时代码与素材：`src/tanktrouble/`、`assets/`；
- AI WebSocket 接口：`src/ai/ai-bridge.js`；
- 导航代码：`python/navigation_mvp.py`、`python/navigation_bot.py`；
- 导航评价代码：`python/score_navigation.py`；
- 游戏状态适配：`python/game_state.py`；
- 网页启动器与网页/导航入口；
- 真实无头/Combat 测试入口；
- 全新的独立 Turn Benchmark 与 Navigation Arena：`test/`；
- 本次固定基线运行数据：`test/results/baseline_20260901/`；
- 全部项目文档、Benchmark 设计与 Roadmap：`docs/`。

测试时新结果默认写入 `test/results/current/`，不会覆盖基线数据。
