# 阶段 3：真车控制器（明天开放迷宫版）

目标：让车**在真实迷宫里跑 Flood-Fill**。今天晚上写的两个脚本：

| 脚本 | 干啥 |
|---|---|
| `sense_one_cell.py` | 读一圈真实雷达数据，告诉你"这一格四面是否有墙"——**最小验证** |
| `drive_maze.py` | 感知 → 决策 → 发指令 → 等回执，循环直到终点 |

---

## 明天的使用顺序（强烈建议按这个顺序）

### Step 0：出发前（在家）
- `git clone` 仓库到带去调试的电脑
- 或者用 git/U 盘/云盘把整个 `Host/navigation/` 目录带过去

### Step 1：到场后基础连接（10 分钟）
1. **不用 `lidar_visualizer.py`**（它会占用蓝牙串口）。但可以**先跑一次**确认蓝牙通，再关掉
2. 关 `lidar_visualizer.py` 后：
   ```bash
   cd Host/navigation/phase3_robot
   python sense_one_cell.py
   ```
3. 期待输出：
   ```
   Connected.
   --- scan 1 ---
     got ~360 beams in ~0.1s
     odom: ...
     dir  wall?    median  beams
     N    YES     0.27 m     ...
     E    no      0.72 m     ...
     ...
   ```

### Step 2：单格感墙验证（15 分钟，**最重要**）

**把车搬进迷宫的一格里**，大致居中、面朝北。

```bash
python sense_one_cell.py --loop
```

会不停打扫描结果。**手动搬车去 3–5 个不同格子**，每次看感墙是否和真实墙一致。

判断标准：
- 每格四面墙**至少 3 个判对**就算可用
- 特别留意**开口那边**（没有墙）是否被误报成有墙
- 如果大面积误报：`python sense_one_cell.py --loop --threshold 0.40`（减小阈值）
- 如果开口被当墙：阈值偏大，减到 0.35 或更小

Ctrl+C 退出。**这一步过不去就别进 Step 3**，先调阈值或改 `wall_sense.py` 的 `DEFAULT_WALL_THRESHOLD`。

### Step 3：Dry-run 走一遍（5 分钟）

不实际动车，先看算法打算怎么走：
```bash
python drive_maze.py --start 4 0 --goal 2 4 --prior z_island_5x5 --dry-run
```

输出每步的"turning: R90 / moving: F70"打印。如果路径明显不对（比如头两步就撞墙），**停下修 `build_z_island_maze()` 里的墙**再试。

### Step 4：真车有图直冲（第一次跑！）

**把车摆进起点格 `(4, 0)`，面朝北（+y 方向）**。起点和朝向必须对。

```bash
python drive_maze.py --start 4 0 --goal 2 4 --prior z_island_5x5
```

**手放在物理 E-STOP 按钮旁边**。如果看起来要撞墙，按 E-STOP。

每步控制台会打印：
```
--- cell 0: at (4, 0) heading=N ---
  sensed walls: [W]
  plan: go E (dist at cur=6, at next=5)
  turning: R90
  moving: F70
```

### Step 5（可选）：无图探索

如果 Step 4 跑通了，挑战无图：

```bash
python drive_maze.py --start 4 0 --goal 2 4 --rows 5 --cols 5
```

这次车不知道迷宫长啥样，每格感知一次后决定下一步。理论上会走一样的最短路径（因为 Flood-Fill 的乐观假设在这类迷宫上就够）。

---

## 命令行参数速查

```
--port COM4           蓝牙串口（配对后从设备管理器看）
--prior z_island_5x5  用预置迷宫做先验地图（有图模式）
--rows 5 --cols 5     无图模式的迷宫尺寸
--start R C           起点行列
--goal  R C           终点行列
--cell-cm 70          一格尺寸 cm，F指令用这个值
--wall-thresh 0.50    墙判定距离阈值（米）
--dry-run             不发指令，只打印
--once                感知一次后停
```

---

## 如果出问题的排查清单

| 现象 | 原因 | 怎么办 |
|---|---|---|
| `Serial error: could not open port` | lidar_visualizer 没关 / 串口号错 | 关掉可视化，确认 COM 号 |
| `no scan within 3.0s` | 蓝牙连上但车没发数据 | 重启车电源；检查车是否真在转雷达（听声音） |
| 感墙准确率低 | 阈值不对 / 车不在格子中心 | `--threshold 0.40` 或 `0.45` 试试 |
| 车转完 90° 没回 `TURN_DONE` | 下位机转弯 PID 不稳 | 这是 `bt_cmd.c` 的问题，下位机同学修 |
| 车走了 F70 实际走 90cm → 撞下一格 | 下位机距离标定偏 | 下位机同学修；或临时 `--cell-cm 55` |
| 车走 F70 走曲线偏离 | 下位机直行 PID 不稳 | 这个我们处理不了，记下偏多少给下位机 |

---

## 我最担心的三件事

1. **车走 F70 走曲线** → 到下一格时车身不正 → 下次感墙误判 → 雪崩
   - 如果发生，转向 **Step 2 单格感墙反复做**，把 `wall_sense` 的扇区 halfwidth 加宽（改为 40°），容忍更大偏航

2. **下位机 `F` 命令的 cm 标定偏** → 走 F70 实际走 90cm → 跨到下下格
   - 解决：在现场量一下 F70 走多远，用 `--cell-cm` 反推。比如实际走 91cm，一格 70cm，那让车每次只发 F54 (70*70/91=54)

3. **感墙阈值对不上** → 算法觉得到处都是墙 / 到处都没墙
   - 解决：Step 2 `--loop` 调阈值

---

## 文件清单

```
phase3_robot/
├── bt_protocol.py       蓝牙包解析（0xAA55 融合 / 0xBB55 odom / ACK行）
├── sense_one_cell.py    单格感墙测试
├── drive_maze.py        主控制器
└── README.md            本文件
```

---

## 今晚我做完了，你不用再做什么

明天早上出门前：
- 把 `Host/navigation/` 带上（或直接 git clone）
- 确认蓝牙还能配对
- 充满车的电池
- 带卷尺和笔（记错误数据）

出问题第一时间发聊天记录/截图给我，一起看。
