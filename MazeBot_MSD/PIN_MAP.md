# MazeBot_MSD 引脚连接清单

MCU: STM32F446RET6 (Nucleo-F446RE)
时钟: HSE BYPASS 8MHz → PLL 180MHz, APB1=45MHz, APB2=90MHz

## UART

TX/RX 均为 MCU 侧引脚定义（MCU TX → 外设 RX，MCU RX → 外设 TX）


| 功能           | 外设     | TX   | RX   | 波特率    | DMA RX       | 备注          |
| ------------ | ------ | ---- | ---- | ------ | ------------ | ----------- |
| IMU IM900    | USART1 | PB6  | PA10 | 115200 | DMA2_Stream2 | IDLE中断      |
| Debug/printf | USART2 | PA2  | PA3  | 115200 | DMA1_Stream5 | ST-Link VCP |
| 蓝牙 HC-04     | UART4  | PA0  | PA1  | 921600 | DMA1_Stream2 | IDLE中断      |
| RPLIDAR C1   | UART5  | PC12 | PD2  | 460800 | DMA1_Stream0 | IDLE中断      |


## 电机 (AT8236驱动)


| 功能      | 引脚   | 外设/模式       | 备注                      |
| ------- | ---- | ----------- | ----------------------- |
| 右电机 PWM | PB3  | TIM2_CH2    | PSC=349, ARR=255, ~1kHz |
| 左电机 PWM | PB10 | TIM2_CH3    | 同上                      |
| 右电机方向   | PC7  | GPIO OUT PP | MOTOR_R_DIR_Pin         |
| 左电机方向   | PC8  | GPIO OUT PP | MOTOR_L_DIR_Pin         |


## 编码器


| 功能   | CH1 | CH2 | 外设                | 备注                  |
| ---- | --- | --- | ----------------- | ------------------- |
| 右编码器 | PA8 | PA9 | TIM1 Encoder TI12 | ARR=65535, Filter=5 |
| 左编码器 | PC6 | PB5 | TIM3 Encoder TI12 | ARR=65535, Filter=5 |


## 系统


| 引脚   | 功能             | 备注       |
| ---- | -------------- | -------- |
| PA13 | SYS_JTMS-SWDIO | SWD 调试   |
| PA14 | SYS_JTCK-SWCLK | SWD 调试   |
| PH0  | RCC_OSC_IN     | HSE 8MHz |
| PH1  | RCC_OSC_OUT    | HSE      |


## 引脚汇总 (按端口排列)


| 引脚   | 功能          | 用途       |
| ---- | ----------- | -------- |
| PA0  | UART4_TX    | 蓝牙 TX    |
| PA1  | UART4_RX    | 蓝牙 RX    |
| PA2  | USART2_TX   | Debug TX |
| PA3  | USART2_RX   | Debug RX |
| PA8  | TIM1_CH1    | 右编码器 A   |
| PA9  | TIM1_CH2    | 右编码器 B   |
| PA10 | USART1_RX   | IMU RX   |
| PA13 | SYS_SWDIO   | SWD 调试   |
| PA14 | SYS_SWCLK   | SWD 调试   |
| PB3  | TIM2_CH2    | 右电机 PWM  |
| PB5  | TIM3_CH2    | 左编码器 B   |
| PB6  | USART1_TX   | IMU TX   |
| PB10 | TIM2_CH3    | 左电机 PWM  |
| PC6  | TIM3_CH1    | 左编码器 A   |
| PC7  | GPIO OUT    | 右电机方向    |
| PC8  | GPIO OUT    | 左电机方向    |
| PC12 | UART5_TX    | LIDAR TX |
| PD2  | UART5_RX    | LIDAR RX |
| PH0  | RCC_OSC_IN  | HSE      |
| PH1  | RCC_OSC_OUT | HSE      |


