#ifndef ENCODER_MODULE_H
#define ENCODER_MODULE_H

#include <Arduino.h>

// 编码器引脚定义
#define LEFT_ENC_A PC6
#define LEFT_ENC_B PB5
#define RIGHT_ENC_A PA8
#define RIGHT_ENC_B PA9

extern volatile long leftEncoderTicks;
extern volatile long rightEncoderTicks;

void leftEncoderISR();
void rightEncoderISR();
void initEncoders();
long getLeftTicks();
long getRightTicks();

#endif
