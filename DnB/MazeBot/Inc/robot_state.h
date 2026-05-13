#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <Arduino.h>

enum RobotState {
  ROBOT_IDLE = 0,
  ROBOT_EXPLORING,
  ROBOT_RETURNING,
  ROBOT_ESTOP,
  ROBOT_FAULT
};

void RobotState_Init();
RobotState RobotState_Get();
void RobotState_Set(RobotState state);
void RobotState_LatchEstop();
bool RobotState_IsEstopLatched();
bool RobotState_TryResetEstop(bool startPressed, bool modePressed);

#endif
