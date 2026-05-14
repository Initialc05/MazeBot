# MazeBot 导航算法

车端跑 Flood-Fill 走迷宫。三阶段：纯仿真 → 仿真雷达感知 → 真车控制器。

## 目录

| 目录 | 说明 |
|---|---|
| `flood_fill_sim/` | 阶段 1：纯软件仿真，验证 Flood-Fill 在迷宫拓扑上能稳定到终点 |
| `phase2_sensing/` | 阶段 2：在仿真雷达数据上跑感墙 + Flood-Fill 端到端 |
| `phase3_robot/` | 阶段 3：通过蓝牙把指令发给真车，跑真实迷宫 |

## 快速上手

```bash
# 阶段 1：仿真
cd Host/navigation/flood_fill_sim
python visualize.py competition_5x5 --prior

# 阶段 2：感墙压测
cd Host/navigation/phase2_sensing
python stress_test.py

# 阶段 3：真车（蓝牙 COM 口按本机配对实测改）
cd Host/navigation/phase3_robot
python drive_maze_snapped.py --start 4 0 --goal 0 4 --heading S \
  --prior competition_5x5 --log run.log
```

## 依赖

- Python 3.10+
- `numpy`、`matplotlib`（仿真用）
- `pyserial`（仅阶段 3 真车用）

详见各子目录 README。
