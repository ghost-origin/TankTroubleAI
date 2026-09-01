# Test / Benchmark

所有导航测试环境统一放在本目录。测试代码不会替代真实游戏；Combat/真实游戏仍作为最终集成验证。

## 目录

- `scripts/`：Benchmark 实现。
- `config/`：Turn/Arena 固定测试配置。
- `results/baseline_20260901/`：本次交付的固定基线结果。
- `results/current/`：运行 `.bat` 后生成的新结果。
- `data/recorded/`：历史真实游戏轨迹数据。
- `legacy/`：早期探测/冒烟测试脚本，仅供追溯。

## 快速运行

- `run_turn_benchmark.bat`：18 个独立过弯场景。
- `run_navigation_arena.bat`：maze=0、seed=1001、20 个连续目标。
- `run_navigation_arena_long.bat`：同一地图的长程 Arena，只生成 A* 实际路径长度至少 400px 的目标。
- `run_all_benchmarks.bat`：依次执行 Turn + Arena 并生成汇总报告。

正式开发回归顺序：Turn Benchmark → Navigation Arena → Combat Benchmark。
