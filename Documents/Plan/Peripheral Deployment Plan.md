# 计划：添加 OLED + 按钮 + 电位器三个模块

## Context

Brief (EBU6475) Section 3d 要求实现嵌入式用户界面：OLED 显示状态、按钮控制启停/模式、电位器实时调参。Section 3f 要求紧急停止功能。当前项目已完成 UART/Motor/Encoder/PID/IMU/LIDAR/BtCmd 全部驱动和 FreeRTOS 任务，但缺少 I2C、ADC、GPIO 按钮相关的硬件模块。

## 引脚分配


| 功能                | 引脚   | 外设                 |
| ----------------- | ---- | ------------------ |
| OLED SCL          | PB8  | I2C1 (AF4)         |
| OLED SDA          | PB9  | I2C1 (AF4)         |
| E-STOP 按钮         | PC13 | EXTI13 (Nucleo B1) |
| START 按钮          | PB4  | GPIO Input Pull-up |
| RETURN 按钮         | PB7  | GPIO Input Pull-up |
| MODE 按钮           | PB2  | GPIO Input Pull-up |
| POT1 (Kp)         | PA4  | ADC1_IN4           |
| POT2 (Base Speed) | PA5  | ADC1_IN5           |
| POT3 (Turn Speed) | PA6  | ADC1_IN6           |


所有引脚已确认空闲，不与现有外设冲突。

## 新建文件 (10个)


| 文件                         | 用途                            |
| -------------------------- | ----------------------------- |
| `Core/Inc/robot_state.h`   | 机器人状态枚举 + E-STOP 标志           |
| `Core/Src/robot_state.c`   | 状态管理、E-STOP 锁存/复位             |
| `Core/Inc/button.h`        | 按钮 API + 消抖                   |
| `Core/Src/button.c`        | GPIO 初始化、EXTI 配置、轮询消抖         |
| `Core/Inc/potentiometer.h` | ADC 电位器 API                   |
| `Core/Src/potentiometer.c` | ADC+DMA 初始化、EMA 滤波、参数映射       |
| `Core/Inc/ssd1306.h`       | SSD1306 I2C 驱动头文件             |
| `Core/Src/ssd1306.c`       | SSD1306 驱动 + 6x8 字体 + 帧缓冲     |
| `Core/Inc/ui_task.h`       | UITask 原型                     |
| `Core/Src/ui_task.c`       | 显示刷新任务 (200ms) + Pot_Update() |


## 修改现有文件 (7个)


| 文件                     | 改动                                                                                      |
| ---------------------- | --------------------------------------------------------------------------------------- |
| `stm32f4xx_hal_conf.h` | 取消注释 `HAL_I2C_MODULE_ENABLED` 和 `HAL_ADC_MODULE_ENABLED`                                |
| `main.c`               | 添加 MX_I2C1_Init()、MX_ADC1_Init()、Pot_Init()、RobotState_Init() 调用；注册 ButtonTask + UITask |
| `main.h`               | 添加 `extern I2C_HandleTypeDef hi2c1; extern ADC_HandleTypeDef hadc1;`                    |
| `stm32f4xx_hal_msp.c`  | 添加 I2C1 MSP init (PB8/PB9 AF4) 和 ADC1 MSP init (PA4/PA5/PA6 Analog + DMA)               |
| `stm32f4xx_it.c`       | 添加 EXTI15_10_IRQHandler (E-STOP) 和 DMA2_Stream0_IRQHandler (ADC DMA)                    |
| `freertos.c`           | 添加 ButtonTask 和 UITask 的 include 和函数原型                                                  |
| `bt_cmd.c`             | 顶部加 E-STOP 守卫；用电位器值替换编译时常量 (MOTOR_BASE_DUTY, KH_P 等)                                    |


## 实现顺序

### Step 1: robot_state 基础设施

- 创建 `robot_state.h/c`：RobotState 枚举 (IDLE/EXPLORING/NAVIGATING/RETURNING/ESTOP/FAULT)、`volatile bool g_estop_latched`
- 这是其他模块的依赖基础

### Step 2: 按钮模块 (安全优先)

- 创建 `button.h/c`
- E-STOP: PC13 EXTI 下降沿中断，NVIC 优先级 2（高于 FreeRTOS 阈值），ISR 直接清零 TIM2 PWM 并置 g_estop_latched
  - ⚠️ 优先级 2 < configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5)，ISR 内禁止调用任何 FreeRTOS API
- START/RETURN/MODE: PB4/PB7/PB2 GPIO 输入上拉，ButtonTask 20ms 轮询消抖
- E-STOP 锁存：需 START+MODE 同时按下才能复位
- 修改 `stm32f4xx_it.c` 添加 EXTI15_10_IRQHandler
- 修改 `bt_cmd.c`：在 BtCmd_UpdateMotorControl() 顶部加 `if (g_estop_latched) return;`，同时在 BtCmd_ProcessByte() 的运动命令入口也加 E-STOP 检查，防止 E-STOP 期间缓存命令解锁后立即执行
- ButtonTask: osPriorityNormal, 256 words stack, 20ms 周期

### Step 3: 电位器模块

- 创建 `potentiometer.h/c`
- ADC1 扫描模式 + DMA 循环传输，3 通道 (IN4/IN5/IN6)
- EMA 滤波 (α=0.1)，映射：POT1→Kp(0.5-5.0), POT2→base_duty(20-80), POT3→turn_duty(15-50)
- Pot_Init() 中先启动一次 ADC 转换，用首次读数初始化 EMA 状态，避免从 0 开始的上升过渡
- 修改 `stm32f4xx_hal_conf.h` 启用 ADC
- 修改 `stm32f4xx_hal_msp.c` 添加 ADC1 MSP init
- 修改 `stm32f4xx_it.c` 添加 DMA2_Stream0_IRQHandler
- Pot_Update() 由 UITask 每 200ms 调用，无需独立任务
- 修改 `bt_cmd.c`：headingPID.kp = pv.kp_heading; base = pv.base_duty;

### Step 4: OLED 显示模块

- 创建 `ssd1306.h/c`：最小 I2C 驱动，1024B 帧缓冲，6x8 字体 (576B flash)
- I2C1 400kHz Fast Mode，阻塞发送（1KB 刷新 ~2.5ms）
- 创建 `ui_task.h/c`：UITask 读取共享全局变量显示 4 行信息
- 修改 `stm32f4xx_hal_conf.h` 启用 I2C
- 修改 `stm32f4xx_hal_msp.c` 添加 I2C1 MSP init
- UITask: osPriorityBelowNormal, 384 words stack, 200ms 周期（snprintf + HAL I2C 调用需要较大栈空间）
- 显示内容：状态 / 位姿(x,y) / 航向+速度 / 电位器参数值

### Step 5: main.c 整合

- 添加 MX_I2C1_Init()、MX_ADC1_Init() 外设初始化
- 添加 Pot_Init()、RobotState_Init() 调用
- 注册 ButtonTask 和 UITask
- 更新 main.h extern 声明

### Step 6: PIN_MAP.md 更新

- 添加新增的 9 个引脚到引脚清单

## FreeRTOS 资源预算

新增 2 个任务，总堆使用约 10.8KB / 24KB，剩余 ~13.2KB 余量充足。
SSD1306 帧缓冲 1024B 静态 RAM，ADC DMA 缓冲 6B，总新增静态 RAM ~1.1KB。

## 验证方法

1. E-STOP: 电机运转时按 PC13，确认 PWM 立即归零，蓝牙收到 "ESTOP\r\n"，再次按 START 无反应（锁存），START+MODE 同时按下后恢复
2. 按钮: 按 START 进入 EXPLORING 状态，按 RETURN 进入 RETURNING 状态，按 MODE 切换模式
3. 电位器: 转动旋钮，OLED 第 4 行实时显示参数变化，电机控制行为随之改变
4. OLED: 确认 200ms 刷新不影响 MotorControlTask 5ms 周期（用 GPIO toggle + 逻辑分析仪验证）
5. 整体: 所有 6 个 FreeRTOS 任务正常调度，无堆栈溢出

