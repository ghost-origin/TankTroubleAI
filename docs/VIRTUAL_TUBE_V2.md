# Virtual Tube V2 / Swept Rectangle Footprint

日期：2026-09-01

## 本次目标

这次只升级转弯可行性与局部平滑轨迹，不修改 A* 主体、Combat 逻辑或 JS/Python 接口。

核心变化：

- 旧的固定圆 / 统一 tube 不再作为精确硬判定。
- A* 仍使用原来的 8-neighbour 网格搜索和 LOS 简化。
- A* 输出折线后，局部转角改用 C1 cubic Bezier 候选。
- Bezier 起点切线匹配入弯方向，终点切线匹配出弯方向。
- 控制臂长度与切角长度进行候选搜索。
- 精确可行性改为矩形坦克 footprint 沿候选曲线的扫掠碰撞检查。

## 精确碰撞判定

坦克碰撞盒按 31 x 20 px 处理：

- 前进方向半长：15.5 px
- 横向半宽：10 px

每个采样姿态都会生成矩形 footprint，并检查：

- 四个角点是否越界或落墙；
- 四条边是否穿墙；
- 相邻姿态之间按中心位移和角度变化自适应补采样，近似 swept area。

这意味着当前硬约束是：

- collision-free；
- 最小转弯半径不低于 `MIN_TURN_RADIUS_PX = 33.0`。

以下指标只进入 soft cost，不再直接否决：

- 路径长度；
- footprint clearance；
- 总转角平滑度；
- 曲率变化。

旧 Virtual Tube 的 distance field 只保留为粗过滤：中心线不能贴墙或越界；最终是否可行以 swept rectangle 为准。

## 测试环境更新

为了让这台机器可以直接运行测试，Python 距离场现在优先使用 SciPy；如果环境没有 SciPy，会自动退回项目内置的 8-neighbour 距离场实现。

以下入口也会优先使用 Codex bundled Python，找不到时再回退到系统 Python：

- `test/run_turn_benchmark.bat`
- `test/run_navigation_arena.bat`
- `test/run_all_benchmarks.bat`
- `run_nav_tests.bat`
- `run_nav_combat.bat`

## 回归结果

基线（2026-09-01）：

- Turn Benchmark：14/18，120° / 135° 左右转失败；
- Navigation Arena：strict Virtual Tube 0/20；
- diagnostic fallback collision：18/20；
- 平均速度约 86.6 px/s。

本次 V2：

- Turn Benchmark：18/18；
- 120° / 135° 左右转全部通过；
- Turn 碰撞：0；
- Turn 中位 overshoot：0.72°；
- Turn 中位 heading exit error：0.72°；
- Navigation Arena strict plan：3/20；
- Navigation Arena completed：4/20；
- diagnostic fallback collision：15/17；
- 平均速度约 97.9 px/s。

## 结论

V2 已经解决了独立大角度转弯中的持续回环问题，并且把“规划可行性”从固定圆 tube 升级为真实矩形车身扫掠判定。

Arena 仍然显示一个更深的问题：很多复杂路线由窄网格 A* 给出的中心线过于贴近墙角；在保持 33px 最小转弯半径、始终油门、无倒车/停车转向的执行模型下，矩形 footprint 会正确拒绝这些路线。下一步更适合做“路径中心线重定位 / 走廊中轴线化 / 局部多段绕角搜索”，而不是继续放宽碰撞判定。
