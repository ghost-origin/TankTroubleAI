# TankTrouble Navigation Test Platform

本目录新增两套与 Combat Benchmark 独立的快速导航测试。

## 1. Turn Benchmark（独立转弯基准）

入口：`test/run_turn_benchmark.bat`

目的：不测试 A*、敌人、射击，只测试当前 Virtual Tube 路径 + follower 的过弯执行质量。

标准场景：
- 30/45/60/90/120/135° 左转、右转：12 个基础场景；
- 90° tight corridor 左/右：2 个；
- 90° outer-wall regression 左/右：2 个；
- S 弯 left-right / right-left：2 个。

核心输出：
- `success / collision`
- `overshoot_deg`：方向超调；
- `heading_exit_error_deg`：出弯车头误差；
- `max_cross_track_px`：实际轨迹偏离规划中心线最大值；
- `min_wall_clearance_px`；
- `steer_flip_count`；
- `min_turn_radius_px`；
- `total_distance_px / average_speed_px_s / moving_speed_px_s`。

每个场景还会输出 CSV 和 PNG，因此可以直接检查“规划线 vs 实际车中心轨迹”。

## 2. Navigation Arena（整体导航基准）

入口：`test/run_navigation_arena.bat`

没有敌人，不包含攻击目标选择。使用实际 maze 数据建立固定场地；坦克到达一个 goal 后，激活下一个 goal，类似贪吃蛇目标刷新。

默认：
- maze index = 0；
- seed = 1001；
- goals = 20；
- 同一个 `maze + seed` 会预生成完全相同的目标序列，适合算法 A/B 测试。

目标生成规则：
- 位于可行区域；
- 墙距 >= 28 px；
- 按 shortest-map-distance 分成 short / medium / long；
- 目标序列在测试开始前一次性生成，不依赖算法最终走到哪里，因此不同版本公平。

核心输出：
- `goals_completed / goal_success_rate / goals_per_minute`；
- `total_distance_px`；
- `average_speed_px_s / moving_speed_px_s`；
- `map_path_efficiency = sum(A* shortest)/sum(actual)`；
- `smooth_rad_per_100px`；
- `collision / min_wall_clearance`；
- `max_cross_track_px / steer_flip_count`；
- Straight / 1-turn / 2-turn / Complex 分类成功率。

## 3. 与真实游戏 Benchmark 的关系

测试金字塔：

1. Turn Benchmark：最快，专门拦截过弯/超调回归；
2. Navigation Arena：无敌人，测完整导航吞吐、距离、速度和路径效率；
3. 原 `run_nav_tests.bat` / Combat Benchmark：真实游戏最终验证。

Turn/Arena 使用当前 Car 行为参数建立快速运动学仿真：maxspeed=125px/s、acc/dec=200px/s²、steer≈3.8rad/s、drift recover=200deg/s。它用于快速迭代，不替代真实游戏碰撞验证。

## 4. 现有 score_navigation.py 更新

现有 Combat 导航评分现在也显式输出：
- `total_distance_px`
- `duration_s`
- `average_speed_px_s`
- `moving_speed_px_s`
- `idle_ratio`

核心 Path Score 仍是 map-aware shortest/actual；平均速度目前作为独立诊断分数 `average_speed_score`，暂时不重复塞入原 Composite，等新 benchmark 数据稳定后再定权重。

## 5. 当前基线（2026-09-01）

以下结果用于暴露当前 Virtual Tube / follower 的问题，不代表最终算法能力。

### Turn Benchmark

18 个固定场景实际结果：

- 通过 14/18，成功率 77.8%；
- 本轮仿真没有检测到车体与测试墙直接重叠；
- 中位方向超调 11.31°；
- 中位出弯方向误差 6.79°；
- 中位最大横向跟踪误差 1.27 px；
- 30°/45°/60°/90° 左右转均通过；
- 120° 左右转失败：出现持续转向/回环，方向超调约 59.94°，出弯误差约 153.52°；
- 135° 左右转失败：同样出现回环，方向超调约 44.86°，出弯误差约 140.70°；
- S 弯当前通过。

因此当前首要 turn regression 不是 A*，而是大角度路径执行：现有“追下一个 waypoint + 始终油门”在 120°/135° 场景中不能稳定退出转弯。

### Navigation Arena

固定 `maze=0, seed=1001, goals=20`：

- 严格 Virtual Tube planner 成功 0/20；
- 严格失败原因：`turn_clearance_failed` 18 次，`tube_invalid` 2 次；
- Benchmark 为了继续诊断，会在严格失败后使用 **diagnostic spline fallback** 执行 A* 简化折线的 Catmull-Rom 曲线；此 fallback 不会修改线上导航行为；
- diagnostic fallback 共 20 段，其中 18 段碰撞；
- 最终完成 2/20 个 goal，均来自 1-turn 场景；
- 1-turn：2/2；2-turn：0/1；complex：0/17；
- 总实际路程 1506.4 px；
- 平均速度 86.6 px/s；运动时平均速度 88.0 px/s。

这组结果同时说明两个问题：

1. 当前 Virtual Tube / turn-clearance 硬判定对一般点到点任务过于保守，规划覆盖率接近 0；
2. 如果绕过硬判定直接执行当前 Catmull-Rom 平滑，复杂路线又大量发生碰撞，因此不能简单删除安全判定。

下一步应分别优化 **转弯可执行轨迹** 与 **Virtual Tube 可行性判定**，而不是通过放松一个阈值同时解决两者。

### 使用建议

每次修改转弯算法时按以下顺序回归：

1. `test/run_turn_benchmark.bat`：要求所有基础角度先稳定通过，重点观察 90° overshoot 与 120°/135° 是否回环；
2. `test/run_navigation_arena.bat`：观察 strict plan coverage、goal success、collision、total distance、average speed；
3. 基础测试改善后再运行真实 Combat benchmark。
