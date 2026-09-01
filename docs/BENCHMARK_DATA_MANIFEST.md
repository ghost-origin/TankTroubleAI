# Benchmark 数据包说明

## 1. 文件性质

本数据包是本次 TankTrouble 导航 Benchmark 的**运行/测试数据**，不是机器学习模型训练集。

数据主要用于：

- Virtual Tube 调参；
- follower A/B 对比；
- Turn regression；
- Navigation Arena regression；
- 轨迹绘图；
- 失败案例复盘。

---

## 2. Turn Benchmark 数据

目录：

`test/results/baseline_20260901/turn/`

包含：

- 每个场景的逐帧 CSV；
- 每个场景的 PNG 轨迹图；
- `turn_summary.csv`；
- `turn_summary.json`；
- `run.log`。

固定场景包括：

- T30 left/right
- T45 left/right
- T60 left/right
- T90 left/right
- T120 left/right
- T135 left/right
- T90 tight left/right
- T90 outer wall left/right
- S curve left-right/right-left

当前基线：18 cases，14 success，success rate 77.78%。

---

## 3. Navigation Arena 数据

目录：

`test/results/baseline_20260901/arena/`

核心文件：

### `goal_sequence.json`

固定 seed 生成的目标序列。用于以后复现同一组导航任务。

### `arena_segments.csv`

每个目标段一行，包含：

- start / goal
- distance band
- route complexity
- turn count
- success
- collision
- strict plan success
- execution mode
- plan reason
- planned path distance
- actual distance
- map efficiency
- time
- average speed
- moving speed
- idle ratio
- smoothness
- max cross-track
- min wall clearance
- steer flips
- min turn radius

### `arena_track.csv`

连续逐帧轨迹数据，用于绘图和控制器分析。

### `arena_track.png`

Arena 轨迹总图。不同 goal segment 已分段绘制，不会用假长线连接 reset 点。

### `arena_summary.json`

本次 Arena 的总体汇总。

当前基线：20 goals，2 completed，strict planner 0/20，diagnostic fallback collisions 18/20。

---

## 4. 总报告

- `test/results/baseline_20260901/benchmark_report.json`
- `test/results/baseline_20260901/benchmark_report.md`

Turn Benchmark 是独立回归指标，不并入 Arena 或 Combat 综合得分。

---

## 5. 建议使用方式

每次修改导航代码后：

1. 先保持相同 `turn_cases.json`；
2. 保持相同 Arena `seed` 和 `goal_sequence`；
3. 运行新版；
4. 将新版 `turn_summary.csv` 与当前基线对比；
5. 将新版 `arena_segments.csv` 与当前基线逐目标配对比较；
6. 只有 Turn 和 Arena 都没有退化，才进入 Combat Benchmark。

这样可以避免不同随机地图/目标造成的假提升。
