#include "../Inc/button_input.h"

#include <STM32FreeRTOS.h>

#include "../Inc/robot_state.h"

namespace {
constexpr uint8_t BTN_ESTOP = PC13;
constexpr uint8_t BTN_START = PB4;
constexpr uint8_t BTN_RETURN = PB7;
constexpr uint8_t BTN_MODE = PB2;
constexpr uint8_t DEBOUNCE_COUNT = 3;

struct ButtonFilter {
  bool stable;
  bool previousStable;
  uint8_t count;
};

volatile bool g_estop_irq = false;
ButtonFilter g_startFilter = {false, false, 0};
ButtonFilter g_returnFilter = {false, false, 0};
ButtonFilter g_modeFilter = {false, false, 0};

bool readPressed(uint8_t pin) {
  return digitalRead(pin) == LOW;
}

bool updateFilter(ButtonFilter &filter, bool rawPressed) {
  if (rawPressed == filter.stable) {
    filter.count = 0;
  } else {
    if (filter.count < DEBOUNCE_COUNT) {
      ++filter.count;
    }
    if (filter.count >= DEBOUNCE_COUNT) {
      filter.previousStable = filter.stable;
      filter.stable = rawPressed;
      filter.count = 0;
      return filter.stable && !filter.previousStable;
    }
  }
  return false;
}

void estopISR() {
  g_estop_irq = true;
}
}

void initButtons() {
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_RETURN, INPUT_PULLUP);
  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(BTN_ESTOP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_ESTOP), estopISR, FALLING);
}

void ButtonTask(void *pvParameters) {
  (void)pvParameters;

  while (1) {
    if (g_estop_irq) {
      noInterrupts();
      g_estop_irq = false;
      interrupts();
      RobotState_LatchEstop();
    }

    bool startPressed = readPressed(BTN_START);
    bool returnPressed = readPressed(BTN_RETURN);
    bool modePressed = readPressed(BTN_MODE);

    bool startEdge = updateFilter(g_startFilter, startPressed);
    bool returnEdge = updateFilter(g_returnFilter, returnPressed);
    bool modeEdge = updateFilter(g_modeFilter, modePressed);

    if (RobotState_IsEstopLatched()) {
      RobotState_TryResetEstop(g_startFilter.stable, g_modeFilter.stable);
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    if (startEdge) {
      RobotState_Set(ROBOT_EXPLORING);
    }
    if (returnEdge) {
      RobotState_Set(ROBOT_RETURNING);
    }
    if (modeEdge) {
      RobotState current = RobotState_Get();
      if (current == ROBOT_IDLE) {
        RobotState_Set(ROBOT_EXPLORING);
      } else {
        RobotState_Set(ROBOT_IDLE);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
