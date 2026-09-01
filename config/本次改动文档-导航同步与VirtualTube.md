# 本次改动文档 —— 导航同步与 Virtual Tube 演进

> 日期：2026-09-01
> 范围：从下载版 `TankTroubleAI_navigation_webui_latest` 同步导航核心到工作区，
>       以及此前围绕"拐角转圈 / 过度转弯 / 开局直冲"的导航迭代。

---

## 一、本次做了什么

### 1. 同步下载版导航代码到工作区

将下载版（`TankTroubleAI_navigation_webui_latest`）的导航核心同步到工作区
（`C:\Users\77665\Desktop\TankTrouble`），只同步必要部分：

| 文件 | 说明 | 工作区旧版 |
|---|---|---|
| `python/navigation_mvp.py` | 规划核心（含 Virtual Tube 管道） | 18.5KB → **31.0KB** |
| `python/navigation_bot.py` | WebSocket 控制器（爆炸检测/跨轮复位） | 11.9KB → **19.0KB** |
| `python/score_navigation.py` | 评分器 | 7.5KB → **14.5KB** |
| `launcher.py` | 启动器（新增 `--ai-mode`） | 15.1KB → **19.2KB** |
| `run_navigation_headless.py` | 无头批量测试 | 2.6KB → **5.5KB** |
| `run_nav_tests.bat` / `run_nav_combat.bat` | 无头基准 / 战斗测试 | 新增 |
| `data log/plot_tracks.py` | 绘图（含多轮断档保护） | 6.3KB → **7.2KB** |
| `data log/run_match.js` | 无头跑局（含 respawn/UID 保护） | 8.6KB → **9.8KB** |
| `src/ai/ai-bridge.js` | 桥（新增 `anim` 采集） | 已一致 |

工作区旧版三个导航文件已备份至 **`归档/旧版导航-回退基线/`**（可回退）。

### 2. 三个启动文件改英文名

| 新文件名 | 模式 | 原中文名 |
|---|---|---|
| `start_web_game.bat` | 网页游戏 + 数据记录（`--ai-mode record`） | 启动游戏.bat |
| `start_web_navigation.bat` | 网页游戏 + 导航 AI（`--ai-mode navigation`） | 启动网页导航AI.bat |
| `start_web_no_ai.bat` | 纯网页无 AI（`--ai-mode none`） | （新增） |

中文 `启动游戏.bat` 已移除。

---

## 二、导航迭代过程（本阶段）

### 背景问题（用户反馈）
1. **开局直冲**：开局坦克不管有没有障碍直接往前走一段，且对局次数越多越明显
2. **拐角转圈**：直角转弯处原地打转
3. **过度转弯**：tube 太长时转弯过度，车头方向未与道路平行

### 关键认知（实测）
- **Car 行为转向速率 ∝ 速度**：`ω = steerSpeed × |s|/maxspeed` —— 速度=0 时转不动
  （旧版"误差>18° 只按转向键"是拐角转圈根因）
- **Car 最小转弯半径 R = maxspeed/steerSpeed ≈ 33px 恒定**（与速度无关）
- **运动包线 ≠ 外接圆**：直行段包线半宽 = 车宽/2 = 10px（外接圆 18.45px 是浪费）

### 迭代记录
| 版本 | 改动 | 结果 |
|---|---|---|
| v1 基础 | tube 管道（Catmull-Rom 光滑中心线 + 包线距墙≥7px 校验） | 开局直冲修复、能持续移动 |
| 尝试：大转向低速 | 误差>30° 时 up+down（滑行减速） | ❌ 失败：Car 低速 s→0 转不动、触发倒车镜像转向反（偏差 176°），已撤回 |
| 尝试：前瞻点 + 曲率校验 | 转向目标改前方 55px 点 + tube 曲率≥33px 校验 | 偏差 33-36°→18-26°，但复杂化 |
| 尝试：转弯惩罚 | 候选代价 + 总转角×25px | 偏差稳定 18-26°，速度 38-42px/s |
| **最终：回退 v1** | 撤销前瞻/曲率/惩罚，回到第一版 tube | 用户选定基线：持续移动、开局正常、无失控 |

### 最终保留的改动（工作区当前状态）
- **Virtual Tube 管道**：A* 折线 → Catmull-Rom 光滑中心线 → 逐点校验
  `free_dist ≥ 包线半径(直行10px/转弯18.45px) + clearance(7px)`，规划不出 tube 走 fallback_chase
- **action()**：转向保持油门（`k['up']=1`）+ 死区 5° + 最近 waypoint 转向
- **爆炸信号轮次检测**：桥采集 `foe.anim`，动画名含 explosion/boom/death 等即判定本局结束，
  清空规划与地图，下一局自动复位（修复"对局次数越多越卡"的跨轮地图污染）
- **跨轮复位**：`reset_for_new_round()` 清空地图/路径/规划状态，每局独立日志文件
- **地图未锁停止**：`raw_wall` 未就绪时强制 stop（开局不直冲）

---

## 三、涉及的文件清单

### 新增
```
start_web_game.bat
start_web_navigation.bat
start_web_no_ai.bat
run_nav_combat.bat
导航评价指标设计.html          （ChatGPT 分享页另存，评价指标设计参考）
导航评价指标设计_files/        （上述页面资源）
README_ONLINE.md              （下载版附带）
归档/旧版导航-回退基线/        （工作区旧版导航备份）
```

### 修改
```
python/navigation_mvp.py
python/navigation_bot.py
python/score_navigation.py
launcher.py
run_navigation_headless.py
run_nav_tests.bat
data log/plot_tracks.py
data log/run_match.js
src/ai/ai-bridge.js            （anim 采集）
```

### 删除
```
启动游戏.bat                   （改为 start_web_game.bat 等英文名）
```

### 未同步 / 建议忽略
```
TankTrouble_nav_bugfix/        （另一份 bug 修复副本，与工作区主文件重复）
nav_headless_results/          （无头测试产物）
data log/20260901_*/           （对局数据，按需保留）
```

---

## 四、验证结果

| 验证项 | 结果 |
|---|---|
| 语法校验（py_compile） | ✅ 全部通过 |
| tube 规划成功率 | ✅ 98.8%（244/247，失败均走 fallback_chase） |
| 无头 3 局实测 | ✅ 持续移动（391~1750px）、开局转角 0-33°（无原地旋转）、无失控 |
| 转弯表现 | ✅ 转向率 ≤0.3 rad/s（无 3+ rad/s 疯狂转圈） |
| 爆炸检测 / 跨轮复位 | ✅ 单元测试 PASS（爆炸→停止→新局复位→重新锁图） |

---

## 五、使用说明

```bat
rem 网页游戏 + 数据记录（CSV 写入 data log/）
start_web_game.bat

rem 网页游戏 + 导航 AI 控制绿色坦克（日志写入 web_nav_logs/）
start_web_navigation.bat

rem 纯网页，无 AI
start_web_no_ai.bat

rem 无头基准测试（10 局，导航-only，原 AI=1.0）
run_nav_tests.bat

rem 无头战斗测试
run_nav_combat.bat
```

> 依赖：导航需要 Python + numpy（推荐 conda）。launcher 会自动探测带 numpy 的解释器。
> 无头测试需要 jsdom（`JSDOM_PATH` 环境变量指向安装目录）。
