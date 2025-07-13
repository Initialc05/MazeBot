# MazeBot 迷宫车下位机说明文档（2025 V3.0）

> 本文档基于 **V3.0** 分支最新代码自动生成，覆盖了硬件接线、任务划分、蓝牙指令、PID 控制与各功能模块实现等内容。此前版本的日志与分工保留在 _docs/history.md_，不再追踪于主 README。

---

## 1. 项目目标

- 通过 **STM32F446RE** 主控、IMU900 惯导、RPLIDAR-C1 激光雷达与编码器融合，完成迷宫自主导航。
- 使用 **FreeRTOS** 提供实时调度，在多任务下实现 IMU 解算、雷达采集、电机闭环控制与蓝牙通信。
- 支持 **蓝牙/串口** 双向数据流：上位机下发运动指令，下位机回传里程计、姿态角与激光点云。

---

## 2. 代码结构

```text
MazeBot/
├── MazeBot.ino          // 主程序 & 任务调度
├── MotorControl.h       // 电机驱动 + 双环 PID + 指令解析
├── EncoderModule.h      // 四相编码器中断计数
├── LidarModule.h        // RPLIDAR-C1 串口协议解析
├── im948_CMD.h          // IMU900 指令声明 & 全局姿态变量
└── im948_CMD.ino        // IMU900 指令实现 & 数据解析
```

> ⚠️  旧版 `TestModule/*` 已删除；如需独立测试，请使用运行时开关（见下文 `DEBUG` 宏）。

---

## 3. FreeRTOS 任务拓扑

```mermaid
graph TD
  A[IMU900Task 2 ms] -->|Angle / Odom| D[MotorControlTask 20 ms]
  B[LidarTask 10 ms] -->|点云| E[BTSerial]
  C[CommandTask 20 ms] --> D
  C -->|指令| E
  subgraph Drivers
    F[IMU900Serial@115200]
    G[LidarSerial@460800]
    H[BTSerial@9600]
  end
  F --> A
  G --> B
  H --> C
```

各任务创建于 `setup()` 后立即启动；`loop()` 保持空函数，由调度器接管调度。

---

## 4. 功能模块

### 4.1 MotorControl（核心更新）

1. **双环闭环 PID**
   - 外环：航向角 → 目标左右轮速度差（`headingPID`）。
   - 内环：速度差 → PWM 占空比差（`velDiffPID`）。
   - 采样周期 20 ms，与任务周期同步。
2. **自动转向**  
   - `startTurnTo(targetDeg)` 支持绝对航向锁定，误差 ≤ 2° 自动刹车。
3. **连续运动模式**  
   - 大写 `W/S/A/D` 进入不受 200 ms 超时保护的持续运动。
4. **超时保护**  
   - 普通指令 200 ms 内未刷新自动 `hardBrake()`。

### 4.2 EncoderModule

- 四相编码器在 `PC6 / PB5 / PA8 / PA9` 触发中断。
- `getLeftTicks()` / `getRightTicks()` 线程安全地返回累计 tick，用于速度计算。

### 4.3 IMU900 模块

- 封装官方 IM948 指令集，串口 115200。
- 上电流程：`Cmd_03 → Cmd_12 → Cmd_13 → Cmd_05 → Cmd_19`。
- 回调 `Cmd_RxUnpack` 更新全局 `AngleX/Y/Z` 及 `OffsetX/Y/Z`，并置 `isNewData=1`。

### 4.4 LidarModule

- 采用官方 A5 协议 (0xA5 0x20)。
- 波特率 460 800，逐字节解析 6-byte 帧。
- 默认仅 `Serial` 输出点云；取消注释即可同步经 `BTSerial` 上报。

---

## 5. 蓝牙控制协议

| 指令 | 说明           | 备注                                   |
|------|----------------|----------------------------------------|
| w    | 前进 (保持)    | 超时保护，需连续刷新                   |
| s    | 后退 (保持)    | 同上                                   |
| a    | 左转 (保持)    | 同上                                   |
| d    | 右转 (保持)    | 同上                                   |
| W/S/A/D | 连续模式     | 不受超时保护，需显式发送 `x` 刹车      |
| x    | 抱死刹车       | 停止所有 PWM 输出                      |
| a±N  | 相对左转 N°    | 例如 `a90` → 左转 90° 并自动刹车       |
| zN   | 绝对转向 N°    | 例如 `z-45` → 航向 -45°                |

---

> **2025-07-04 更新**  
> 文档自动同步至 `V3.0`。如发现描述与代码不符，请在 Issue 区提交反馈。 