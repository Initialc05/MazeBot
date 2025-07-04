#include <Arduino.h>
#include <Wire.h>
#include <STM32FreeRTOS.h>

// 功能定义模块
#include "IMUFilterModule.h"  // 包含BTSerial定义
#include "LidarModule.h"
#include "MotorControl.h"
#include "EncoderModule.h"

// 测试测试模块（测试模块的函数定义在自己的头文件里）
#include "MotorTestTask.h"
#include "EncoderTestTask.h"

// 滤波任务
extern double filtered_x;
extern double filtered_y;
extern double theta;
void OdometryTask(void *pvParameters){
  while(1){
    Serial.print("[ODOM] x=");
    Serial.print(filtered_x, 3);
    Serial.print(" y=");
    Serial.print(filtered_y, 3);
    Serial.print(" theta=");
    Serial.println(theta, 3);

    BTSerial.print("[ODOM] x=");
    BTSerial.print(filtered_x, 3);
    BTSerial.print(" y=");
    BTSerial.print(filtered_y, 3);
    BTSerial.print(" theta=");
    BTSerial.println(theta, 3);

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

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

void MotorControlTask(void *pvParameters) {
  while (1) {
    // updateMotorControl();   // 使用PID控制
    updateMotorControlWithoutPID();

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

  Serial.println("🚗 蓝牙控制启动");
  Serial.println("蓝牙指令：w=前进 s=后退 a=左转 d=右转 x=停止");
  Serial.println("控制逻辑：指令持续执行500ms，超时自动刹车");

  // 启动任务
  xTaskCreate(IMUTask, "IMU", 4096, NULL, 1, NULL);
  xTaskCreate(LidarTask, "LIDAR", 2048, NULL, 3, NULL);
  xTaskCreate(OdometryTask, "Odom1D", 4096, NULL, 1, NULL);
  // xTaskCreate(PIDParamTask, "PIDParam", 1024, NULL, 1, NULL);
  xTaskCreate(CommandTask, "CMD", 512, NULL, 1, NULL);  // 启用蓝牙控制任务
  xTaskCreate(MotorControlTask, "MotorCtrl", 1024, NULL, 2, NULL);  // 启用电机控制任务

  // 启动测试任务
  //xTaskCreate(EncoderTestTask, "EncoderTest", 512, NULL, 1, NULL);
  //xTaskCreate(MotorTestTask, "MotorTest", 1024, NULL, 1, NULL);

  vTaskStartScheduler();
}

void loop() {
  // FreeRTOS控制下，不使用loop
}