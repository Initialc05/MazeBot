/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "uart_device.h"
#include "motor.h"
#include "encoder.h"
#include "im948.h"
#include "bt_cmd.h"
#include "lidar.h"
#include "robot_state.h"
#include "button.h"
#include "ui_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void CommandTask(void const *argument);
void MotorControlTask(void const *argument);
void IMU900Task(void const *argument);
void LidarTask(void const *argument);
/* USER CODE END FunctionPrototypes */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief 蓝牙命令接收任务 (1ms周期, 优先级4)
 *        从BT DMA缓冲区读取字节, 逐字节喂给BtCmd_ProcessByte
 */
void CommandTask(void const *argument)
{
    (void)argument;
    for (;;) {
        uint8_t tmp[32];
        uint16_t n = BT_ReadDMA(tmp, sizeof(tmp));
        for (uint16_t i = 0; i < n; i++) {
            BtCmd_ProcessByte((char)tmp[i]);
        }
        osDelay(1);
    }
}

/**
 * @brief 电机控制任务 (5ms周期, 优先级4)
 *        采样编码器 + 级联PID运动控制
 */
void MotorControlTask(void const *argument)
{
    (void)argument;
    const float dt_s = 0.005f;  /* 5ms */
    for (;;) {
        Encoder_Update();
        BtCmd_UpdateMotorControl(dt_s);
        osDelay(5);
    }
}

/**
 * @brief IMU数据处理任务 (2ms周期, 优先级3)
 *        解析IMU数据包, 更新姿态角和里程计
 */
void IMU900Task(void const *argument)
{
    (void)argument;
    IMU_Init();
    for (;;) {
        IMU_Update();
        osDelay(2);
    }
}

/**
 * @brief LIDAR数据处理任务 (2ms周期, 优先级3)
 *        解析RPLIDAR扫描数据, 融合里程计后蓝牙发送
 */
void LidarTask(void const *argument)
{
    (void)argument;
    Lidar_Init();
    for (;;) {
        Lidar_Update();
        osDelay(2);
    }
}

/* USER CODE END Application */
