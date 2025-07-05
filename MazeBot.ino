#include <Arduino.h>
#include <Wire.h>
#include <STM32FreeRTOS.h>

HardwareSerial BTSerial(PA1, PA0);  // 蓝牙串口

// 功能定义模块
#include "IMUFilterModule.h"
#include "LidarModule.h"
#include "MotorControl.h"
#include "EncoderModule.h"

// 测试测试模块（测试模块的函数定义在自己的头文件里）
#include "MotorTestTask.h"
#include "EncoderTestTask.h"

// IMU任务
void IMUTask(void *pvParameters) {
  Serial.println("IMU Task Start");
  while (1) {
    updateIMUWithFilter();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// 雷达任务
void LidarTask(void *pvParameters) {
  while (1) {
    readAndSendLidar();
    vTaskDelay(pdMS_TO_TICKS(500));  // 控制雷达任务频率
  }
}

// 蓝牙控制任务
void CommandTask(void *pvParameters) {
  Serial.println("蓝牙控制任务启动");

  while (1) {
    // 检查是否有新的蓝牙指令
    if (BTSerial.available()) {
      char cmd = BTSerial.read();
      Serial.print("收到蓝牙指令: ");
      Serial.println(cmd);
      processBluetoothCommand(cmd);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// 电机控制任务
void MotorControlTask(void *pvParameters) {
  while (1) {
    // updateMotorControl();   // 使用PID控制
    updateMotorControlWithoutPID(); // 开环控制

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  Wire.begin(PB8, PB9);  // I2C用于MPU
  Serial.begin(115200);  // 调试串口

  // 初始化蓝牙串口
  BTSerial.begin(9600);

  // 初始化功能模块
  initLidar();
  initIMU();
  initMotors();
  initEncoders();

  // 启动任务
  xTaskCreate(IMUTask, "IMU", 4096, NULL, 1, NULL);
  xTaskCreate(LidarTask, "LIDAR", 2048, NULL, 3, NULL);
  xTaskCreate(OdomPrintTask, "Odom1D", 4096, NULL, 1, NULL);         
  // xTaskCreate(PIDParamTask, "PIDParam", 1024, NULL, 1, NULL);    // 使用PID控制
  xTaskCreate(CommandTask, "CMD", 512, NULL, 1, NULL);              // 启用蓝牙控制任务
  xTaskCreate(MotorControlTask, "MotorCtrl", 1024, NULL, 2, NULL);  // 启用电机控制任务

  // 启动测试任务（验收时烧录的程序所有 Test 模块要注释掉）
  //xTaskCreate(EncoderTestTask, "EncoderTest", 512, NULL, 1, NULL);
  //xTaskCreate(MotorTestTask, "MotorTest", 1024, NULL, 1, NULL);

  vTaskStartScheduler();
}

void loop() {
  // FreeRTOS控制下，不使用loop但要定义和留空
}