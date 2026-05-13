#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#include <Arduino.h>

struct PotValues {
  float kp_heading;
  int base_duty;
  int turn_duty;
};

void Pot_Init();
void Pot_Update();
PotValues Pot_GetValues();

#endif
