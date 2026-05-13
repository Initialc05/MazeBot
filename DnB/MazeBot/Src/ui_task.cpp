#include "../Inc/ui_task.h"

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <math.h>
#include <stdio.h>

#include "../Inc/EncoderModule.h"
#include "../Inc/im948_CMD.h"
#include "../Inc/host_display_sync.h"
#include "../Inc/potentiometer.h"
#include "../Inc/robot_state.h"
#include "../Inc/ssd1306.h"

namespace {
const char *stateToString(RobotState state) {
  switch (state) {
    case ROBOT_IDLE: return "IDLE";
    case ROBOT_EXPLORING: return "EXPLORE";
    case ROBOT_RETURNING: return "RETURN";
    case ROBOT_ESTOP: return "ESTOP";
    case ROBOT_FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}

void formatFixed2(char *buffer, size_t size, long valueTimes100) {
  long integerPart = valueTimes100 / 100;
  long fractionalPart = labs(valueTimes100 % 100);
  snprintf(buffer, size, "%ld.%02ld", integerPart, fractionalPart);
}
}

void UITask(void *pvParameters) {
  (void)pvParameters;

  bool displayOk = SSD1306_Init();
  long lastLeftTicks = getLeftTicks();
  long lastRightTicks = getRightTicks();

  while (1) {
    Pot_Update();

    if (displayOk) {
      long leftTicks = getLeftTicks();
      long rightTicks = getRightTicks();
      long dLeft = leftTicks - lastLeftTicks;
      long dRight = rightTicks - lastRightTicks;
      lastLeftTicks = leftTicks;
      lastRightTicks = rightTicks;

      float avgTicksPerPeriod = static_cast<float>(dLeft + dRight) * 0.5f;
      PotValues pots = Pot_GetValues();
      HostDisplayData hostDisplay = HostDisplay_Get();

      char line0[22];
      char line1[22];
      char line2[22];
      char line3[22];
      char xBuffer[10];
      char yBuffer[10];
      char angleBuffer[10];
      char speedBuffer[10];
      char kpBuffer[10];

      formatFixed2(xBuffer, sizeof(xBuffer), lroundf(hostDisplay.valid ? hostDisplay.x * 100.0f : (-OffsetY) * 10000.0f));
      formatFixed2(yBuffer, sizeof(yBuffer), lroundf(hostDisplay.valid ? hostDisplay.y * 100.0f : OffsetX * 10000.0f));
      formatFixed2(angleBuffer, sizeof(angleBuffer), lroundf(hostDisplay.valid ? hostDisplay.heading * 100.0f : AngleZ * 100.0f));
      formatFixed2(speedBuffer, sizeof(speedBuffer), lroundf(avgTicksPerPeriod * 100.0f));
      formatFixed2(kpBuffer, sizeof(kpBuffer), lroundf(pots.kp_heading * 100.0f));

      snprintf(line0, sizeof(line0), "ST:%s", stateToString(RobotState_Get()));
      snprintf(line1, sizeof(line1), "X:%s Y:%s", xBuffer, yBuffer);
      snprintf(line2, sizeof(line2), "H:%s V:%s", angleBuffer, speedBuffer);
      snprintf(line3, sizeof(line3), "K:%s B:%d T:%d", kpBuffer, pots.base_duty, pots.turn_duty);

      SSD1306_Clear();
      SSD1306_WriteString(0, 0, line0);
      SSD1306_WriteString(0, 16, line1);
      SSD1306_WriteString(0, 32, line2);
      SSD1306_WriteString(0, 48, line3);
      SSD1306_Update();
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}
