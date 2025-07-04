#ifndef MOTOR_TEST_TASK_H
#define MOTOR_TEST_TASK_H

#include <Arduino.h>
#include "MotorControl.h"
#include <STM32FreeRTOS.h>

// void MotorTestTask(void *pvParameters) {
//   const TickType_t phaseTime = pdMS_TO_TICKS(5000); // 每种状态2秒
//   while (1) {
//     Serial.println("[Test] Forward");
//     setSpeed(80, 80);
//     vTaskDelay(phaseTime);

//     Serial.println("[Test] Backward");
//     setSpeed(-80, -80);
//     vTaskDelay(phaseTime);

//     Serial.println("[Test] Left");
//     setSpeed(-80, 80);
//     vTaskDelay(phaseTime);

//     Serial.println("[Test] Right");
//     setSpeed(80, -80);
//     vTaskDelay(phaseTime);

//     Serial.println("[Test] Stop");

//     // void brake();

//     setSpeed(0, 0); // 左反右正满速
//     vTaskDelay(phaseTime);
//   }
// }

#endif // MOTOR_SPEED_TEST_TASK_H 