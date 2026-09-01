# TankTrouble 导航开发方向与阶段目标

## 1. 文档目的

本文档定义当前导航系统从“能够规划并运行”进一步发展到“稳定、可执行、可量化验证”的开发路线。

当前测试体系明确拆分为三个层级：

1. **Turn Benchmark**：独立测试过弯能力，不计入整体导航综合分。
2. **Navigation Arena**：无敌人、无射击，通过连续目标点测试完整导航系统。
3. **Combat Benchmark**：最后回到真实游戏环境，验证导航对战术和对战结果的实际贡献。

开发顺序固定为：

**Turn Benchmark → Navigation Arena → Combat Benchmark**。

---

## 2. 当前基线结果（2026-09-01）

### 2.1 Turn Benchmark

本次固定测试共 18 个场景。

- 测试场景数：18
- 成功：14
- 失败：4
- 成功率：77.78%
- 测试记录中的碰撞数：0
- 中位方向超调：11.31°
- 中位出弯方向误差：6.79°
- 中位最大横向偏差：1.27 px

当前最明显的失败边界是：

- 30°、45°、60°、90° 基础弯可以完成；
- **120° / 135° 大角度转弯失败**；
- 失败表现为持续转向、回环、无法正确完成出弯方向收敛。

因此当前主要问题已经不是 A* 能否找到几何路径，而是：

> **平滑后的 Virtual Tube / 多项式路径是否满足坦克的实际运动学，以及 follower 是否能够及时结束转向并稳定出弯。**

---

### 2.2 Navigation Arena

当前 Arena 固定配置：

- maze index：0
- seed：1001
- 目标数：20

实际结果：

- goals attempted：20
- goals completed：2
- goal success rate：10%
- strict planner success：0/20
- strict plan success rate：0%
- strict planner failure：20
- diagnostic fallback segments：20
- fallback collisions：18
- 总实际路程：1506.36 px
- 总有效时间：17.40 s
- 平均速度：86.57 px/s
- 运动期间平均速度：88.02 px/s
- steer flip count：9

严格 Virtual Tube 失败原因：

- `turn_clearance_failed`：18
- `tube_invalid`：2

诊断 fallback 的结果说明：

- 严格规则过于保守，导致 0/20 能通过；
- 但如果直接取消严格安全检查，18/20 会碰撞；
- 因而**不能简单通过缩小 clearance 或关闭 tube 检查解决问题**。

按路线复杂度：

- 1 turn：2/2 成功
- 2 turn：0/1
- complex：0/17

这进一步说明：

> **单弯能力已有基础，多弯和大角度连续转向才是当前主要瓶颈。**

---

## 3. 核心开发判断

### 3.1 暂不更换 A*

现阶段没有证据表明 A* 是主要瓶颈。A* 继续负责全局拓扑路径。

优先优化：

1. 转弯可执行性；
2. Virtual Tube 可行性判断；
3. follower 出弯稳定性；
4. 连续弯之间的轨迹连续性。

### 3.2 多项式平滑不能等同于车辆可执行轨迹

当前多项式曲线可以让几何路径平滑，但必须进一步验证：

- 局部曲率是否满足最小转弯半径；
- 新规划起始切线是否与坦克当前 heading 连续；
- 连续弯中是否出现过大的曲率变化；
- 出弯以后 follower 是否仍持续打方向。

下一阶段需要把“看起来平滑”升级为“运动学可执行”。

---

## 4. 第一优先级：Turn Benchmark 改进

### 4.1 当前目标

第一阶段不追求更快，而是先做到：

- 120° 左/右通过；
- 135° 左/右通过；
- 所有标准场景无碰撞；
- 大角度弯不再形成回环。

### 4.2 建议重点研究的控制量

每个 Turn Case 应持续记录：

- planned centerline
- Virtual Tube 边界
- 实际坦克中心轨迹
- tank heading
- desired heading
- steering input
- cross-track error
- wall clearance
- local curvature / turn radius
- OODA / replan 时刻

这样可以区分三类错误：

1. **轨迹本身不可执行**：计划曲率太大；
2. **follower 过度转向**：轨迹可行，但实际轨迹离开 tube；
3. **重规划不连续**：偏差恰好发生在新的路径替换之后。

### 4.3 阶段验收目标

以下是下一阶段的工程目标，不是当前已达到结果：

#### Turn V1

- Turn success rate：≥ 90%
- 120° / 135°：全部不回环
- collision：0
- median heading exit error：≤ 8°
- median overshoot：≤ 10°

#### Turn V2

- Turn success rate：100%
- collision：0
- median heading exit error：≤ 5°
- 90° / 120° / 135° 的最大 overshoot 显著受控
- Tight / Outer Wall 场景保持安全余量

---

## 5. 第二优先级：Virtual Tube 可行性

当前 strict coverage = 0/20，这是必须优先解决的问题。

但禁止采用以下“假修复”：

- 直接关闭 `turn_clearance`；
- 直接关闭 tube validity；
- 单纯大幅减小安全半径；
- 为提高通过率牺牲碰撞安全。

### 建议方向

1. 将当前纯几何多项式转弯与坦克最小转弯半径关联；
2. 考虑使用“直线 + 定半径圆弧 + 直线”的 turn primitive；
3. 多项式可以保留用于圆弧入口/出口的短 transition；
4. 新 tube 的起始 tangent 应与当前 tank heading 对齐；
5. 对连续弯增加 curvature continuity 检查；
6. strict feasibility 应验证真实 swept envelope，而不是仅验证中心线。

### 阶段验收目标

#### Tube V1

- strict plan success rate：≥ 60%
- diagnostic fallback collision rate：显著低于当前 90%
- 不允许以降低墙体安全为代价换取 coverage

#### Tube V2

- strict plan success rate：≥ 90%
- collision rate：接近 0
- complex path 能稳定通过

---

## 6. 第三优先级：Navigation Arena

Turn 和 Tube 达到阶段目标后，再把完整导航作为主测试对象。

### 6.1 Arena 测试规则

采用固定 seed，以便 A/B 对比：

- 相同 maze
- 相同 start
- 相同 goal sequence
- 相同测试时间/目标数

目标点按 short / medium / long 分层，避免随机样本偏置。

### 6.2 核心指标

#### 成功与吞吐

- goals attempted
- goals completed
- goal success rate
- goals per minute
- average goal completion time

#### 路程与效率

- total distance
- actual distance per goal
- A* shortest feasible distance per goal
- map path efficiency

建议整体效率：

`E_map = Σ shortest_feasible_distance / Σ actual_distance`

#### 速度

- average speed = total distance / total benchmark time
- moving speed
- idle ratio

平均速度必须和成功率、安全性、路径效率一起看，不能单独作为越大越好的指标。

#### 平滑与控制稳定性

- smooth rad / 100 px
- steer flip count
- cross-track error
- replan count
- path-switch count

#### 安全

- collision count
- min clearance
- near-wall ratio
- tube violation count

### 6.3 Arena 阶段验收目标

#### Arena V1

- strict planner coverage ≥ 60%
- goal success ≥ 60%
- collision rate ≤ 5%
- 1-turn 场景保持接近 100%
- 2-turn / complex 不再全部失败

#### Arena V2

- strict planner coverage ≥ 90%
- goal success ≥ 90%
- collision rate 接近 0
- map path efficiency ≥ 0.90
- 速度指标在不降低安全和成功率的前提下提升

---

## 7. 第四阶段：Combat Benchmark

只有当 Turn Benchmark 和 Navigation Arena 稳定后，才回到 Combat Benchmark。

Combat Benchmark 主要回答：

> 导航能力的提升是否真正转化为更好的追击、规避和战斗位置选择？

不能使用 Combat 数据替代 Turn/Arena 的专项诊断。

---

## 8. 开发循环

今后每次修改 Virtual Tube、转弯算法或 follower，都按以下顺序：

### 快速回归

1. 跑 Turn Benchmark；
2. 120° / 135° 若失败，停止，不进入 Arena；
3. Turn 通过后跑 5 个固定 Arena seed；
4. 指标没有退化再跑 20 seed 正式 Arena；
5. 最后才跑 Combat Benchmark。

建议把它视为测试金字塔：

`Turn → Arena quick → Arena full → Combat`

---

## 9. 下一次代码修改的明确目标

下一轮代码开发只围绕以下问题进行：

### P0：修复大角度转弯回环

目标：120° / 135° 不再持续转向形成圆圈。

需要优先验证：

- lookahead 是否太短；
- waypoint 是否导致“追点而非追切线”；
- 出弯是否缺少 heading settle / steering release；
- 曲线局部半径是否小于坦克可实现转弯半径；
- 新路径第一切线是否与当前 heading 不连续。

### P1：提高 strict Virtual Tube coverage

目标：从当前 0/20 提升，而不是依赖 diagnostic fallback。

### P2：提升 complex route 完成率

目标：让 2-turn 和 complex 路线从当前 0% 逐步恢复。

### P3：在安全稳定以后优化速度

速度优化放在最后。任何提高平均速度的修改，如果导致：

- collision 增加；
- Turn overshoot 增加；
- goal success 降低；

都视为退化。

---

## 10. 当前版本的阶段结论

当前系统已经具备：

- 全局路径搜索能力；
- Virtual Tube / 多项式平滑框架；
- 基础单弯执行能力；
- 独立 Turn Benchmark；
- 独立 Navigation Arena；
- Combat Benchmark。

下一阶段的核心不是增加更复杂的全局规划算法，而是把：

**几何平滑路径 → 运动学可执行轨迹 → 稳定 follower → 可量化通过 Turn/Arena**

这一链路打通。
