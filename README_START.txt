TankTrouble 绿色坦克「自动开火助手」版
========================================

1. 解压整个文件夹到普通目录。
2. 双击 START_AI.bat（或“一键安装并启动自主AI.bat”，后者会自动装依赖）。
3. 启动器自动检查 Python 依赖（numpy / Pillow / matplotlib），
   缺少时自动安装；然后先启动 AI 服务，就绪后再打开游戏。
4. 玩法：
   - 绿色坦克的移动 / 转向完全由你控制（方向键），AI 绝不碰任何移动键；
   - AI 每帧沿“当前炮口朝向”模拟一发炮弹的弹道（直线飞行 + 墙面镜面
     反射，最多 4 次反射），只要直瞄或反弹后的弹道能命中黑色 AI 坦克，
     就立刻自动开火；你边移动、转向边打，条件一满足 AI 就开枪。
5. 一局结束、双方复活后自动开始下一局，无需任何操作。

数据与排错：
- 每局轨迹记录在 data log\autonomous_nav\<时间戳>\ 下（track_*.csv）。
- 启动失败请看 startup_error.log。
- 开火判定逻辑在 python\combat_ai.py（simulate_ray 为弹道模拟，
  CombatController.attack 为每帧决策）；墙体掩码构建在 python\navigation_mvp.py。
