#ifndef MOTOR_TEST_TASK_H
#define MOTOR_TEST_TASK_H

#include <Arduino.h>
#include "MotorControl.h"
#include <STM32FreeRTOS.h>

void MotorTestTask(void *pvParameters) {
  const TickType_t phaseTime = pdMS_TO_TICKS(5000); // 每种状态2秒
  while (1) {

    // 根据需要实现

    vTaskDelay(phaseTime);
  }
}

#endif
