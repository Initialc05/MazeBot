#include <Arduino.h>
#include <STM32FreeRTOS.h>

HardwareSerial BTSerial(PA1, PA0);       // 蓝牙串口
HardwareSerial IMU900Serial(PA10, PB6);  // IMU900串口
HardwareSerial LidarSerial(PD2, PC12);   // 雷达串口

// 功能定义模块
#include "im948_CMD.h"
#include "LidarModule.h"  
#include "EncoderModule.h"
#include "MotorControl.h"

// 调试输出宏（根据 LidarModule.h 中的配置）
#if DEBUG_PRINT_ENABLED
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINT(x) Serial.print(x)
#else
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT(x)
#endif

// IMU900任务（高频更新以提高里程计精度）
void IMU900Task(void *pvParameters) {
  DEBUG_PRINTLN("IMU900任务启动");

  while (1) {
    updateIMU900();
    vTaskDelay(pdMS_TO_TICKS(2));  // 2ms = 500Hz，提供更精确的里程计数据
  }
}

// 雷达任务（高频处理以降低延迟）
void LidarTask(void *pvParameters) {
  DEBUG_PRINTLN("雷达任务启动");

  while (1) {
    readAndSendLidar();   // 在此函数中设置是否通过蓝牙打印雷达扫描到的数据
    vTaskDelay(pdMS_TO_TICKS(2));  // 2ms延迟，提高雷达数据处理实时性
  }
}

// 蓝牙控制任务（最高优先级，快速响应）
void CommandTask(void *pvParameters) {
  DEBUG_PRINTLN("蓝牙控制任务启动");

  while (1) {
    // 检查是否有新的蓝牙指令（高频轮询以确保最快响应）
    if (BTSerial.available()) {
      char cmd = BTSerial.read();
      // DEBUG_PRINT("收到蓝牙指令: ");
      // DEBUG_PRINTLN(cmd);
      processBluetoothCommand(cmd);
    }

    vTaskDelay(pdMS_TO_TICKS(1));  // 1ms延迟，确保极速响应操控指令
  }
}

// 电机控制任务
void MotorControlTask(void *pvParameters) {
  DEBUG_PRINTLN("电机控制任务启动");

  while (1) {
    updateMotorControl();  // 使用PID控制
    // updateMotorControlWithoutPID();   // 开环控制
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void setup() {
  Serial.begin(115200);
  IMU900Serial.begin(115200);
  BTSerial.begin(921600); 
  
  // 系统初始化
  DEBUG_PRINTLN("MazeBot初始化开始...");
  DEBUG_PRINTLN("========================================");
  #if USE_BLUETOOTH_MODE
  DEBUG_PRINTLN("模式: 蓝牙模式 (BTSerial @ 921600)");
  #else
  DEBUG_PRINTLN("模式: USB串口调试模式 (Serial @ 115200)");
  DEBUG_PRINTLN("⚠️ 注意：二进制数据将通过USB串口输出");
  #endif
  DEBUG_PRINTLN("========================================");
  
  initIMU900();
  initLidar();
  initMotors();
  initEncoders();
  initPID();

  // ==================== 启动FreeRTOS任务 ====================
  // 优先级说明：
  //   4 = 最高（操控指令响应，确保实时性）
  //   3 = 高（数据采集和处理）
  //   2 = 中
  //   1 = 低
  // 
  // 任务周期优化：
  //   - IMU900Task: 2ms (500Hz) - 高精度里程计
  //   - LidarTask:  2ms (500Hz) - 低延迟雷达处理
  //   - CommandTask: 1ms (1kHz) - 极速响应操控
  //   - MotorControlTask: 5ms (200Hz) - 与PID周期匹配
  
  xTaskCreate(IMU900Task, "IMU900", 2048, NULL, 3, NULL);           // 高优先级，高频更新
  xTaskCreate(LidarTask, "LIDAR", 2048, NULL, 3, NULL);             // 高优先级，低延迟
  xTaskCreate(CommandTask, "CMD", 512, NULL, 4, NULL);              // 最高优先级，极速响应
  xTaskCreate(MotorControlTask, "MotorCtrl", 1024, NULL, 4, NULL);  // 最高优先级，实时控制

  // 启动FreeRTOS任务调度
  vTaskStartScheduler();
}

void loop() {
  // 使用FreeRTOS时留空
}
