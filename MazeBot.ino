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

// IMU900任务
void IMU900Task(void *pvParam) {
  Serial.println("IMU900任务启动");

  while (1) {
    updateIMU900();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// 雷达任务
void LidarTask(void *pvParameters) {
  Serial.println("雷达任务启动");

  while (1) {
    readAndSendLidar();   // 在此函数中设置是否通过蓝牙打印雷达扫描到的数据
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// 蓝牙控制任务
void CommandTask(void *pvParameters) {
  Serial.println("蓝牙控制任务启动");

  while (1) {
    // 检查是否有新的蓝牙指令
    if (BTSerial.available()) {
      char ch = BTSerial.read();

      // 若第一字符为字母，尝试读取后续数字形成复合指令
      if (isalpha(ch)) {
        String numStr = "";
        // 允许可选正负号
        while (BTSerial.available()) {
          char c2 = BTSerial.peek();
          if (isdigit(c2) || c2 == '-' || c2 == '+') {
            numStr += BTSerial.read();
          } else {
            break;
          }
        }

        if (numStr.length() > 0) {
          float val = numStr.toFloat();

          switch (tolower(ch)) {
            case 'a': {  // 相对左转 val°
              Serial.print("Relative Left Turn "); Serial.println(val);
              startTurnTo(AngleZ - val);
              break;
            }
            case 'z': {  // 转到绝对角度 val°
              Serial.print("Abs Turn to "); Serial.println(val);
              startTurnTo(val);
              break;
            }
            default:
              // 未定义复合指令，回退到单字符处理
              processBluetoothCommand(ch);
          }
        } else {
          // 无数字部分，按单字符处理
          processBluetoothCommand(ch);
        }
      } else {
        // 非字母，按单字符处理
        processBluetoothCommand(ch);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// 电机控制任务
void MotorControlTask(void *pvParameters) {
  Serial.println("电机控制任务启动");

  while (1) {
    updateMotorControl();  // 使用PID控制
    // updateMotorControlWithoutPID();   // 开环控制
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void setup() {
  Serial.begin(115200);
  IMU900Serial.begin(115200);
  BTSerial.begin(9600);  // 目前会和IM900冲突,需要更改引脚

  // 系统初始化
  Serial.println("MazeBot初始化开始...");
  initIMU900();
  initLidar();
  initMotors();
  initEncoders();
  initPID();

  // 启动任务
  xTaskCreate(IMU900Task, "IMU900", 2048, NULL, 3, NULL);
  xTaskCreate(LidarTask, "LIDAR", 2048, NULL, 3, NULL);
  xTaskCreate(CommandTask, "CMD", 512, NULL, 1, NULL);
  xTaskCreate(MotorControlTask, "MotorCtrl", 1024, NULL, 2, NULL);

  // 启动FreeRTOS任务调度
  vTaskStartScheduler();
}

void loop() {
  // 使用FreeRTOS时留空
}
