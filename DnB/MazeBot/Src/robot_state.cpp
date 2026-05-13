#include "../Inc/robot_state.h"

static volatile RobotState g_robot_state = ROBOT_IDLE;
static volatile bool g_estop_latched = false;

void RobotState_Init() {
  g_robot_state = ROBOT_IDLE;
  g_estop_latched = false;
}

RobotState RobotState_Get() {
  if (g_estop_latched) {
    return ROBOT_ESTOP;
  }
  return g_robot_state;
}

void RobotState_Set(RobotState state) {
  if (g_estop_latched && state != ROBOT_ESTOP) {
    return;
  }
  g_robot_state = state;
}

void RobotState_LatchEstop() {
  g_estop_latched = true;
  g_robot_state = ROBOT_ESTOP;
}

bool RobotState_IsEstopLatched() {
  return g_estop_latched;
}

bool RobotState_TryResetEstop(bool startPressed, bool modePressed) {
  if (!g_estop_latched) {
    return true;
  }
  if (startPressed && modePressed) {
    g_estop_latched = false;
    g_robot_state = ROBOT_IDLE;
    return true;
  }
  return false;
}

