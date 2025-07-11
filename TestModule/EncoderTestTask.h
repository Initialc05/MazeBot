#ifndef ENCODER_TEST_TASK_H
#define ENCODER_TEST_TASK_H

// 编码器测试任务
void EncoderTestTask(void *pvParameters) {
  while (1) {
    Serial.print("L_ticks: ");
    Serial.print(getLeftTicks());
    Serial.print(" | R_ticks: ");
    Serial.println(getRightTicks());
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

#endif
