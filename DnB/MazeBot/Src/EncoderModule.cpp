#include "../Inc/EncoderModule.h"

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

void leftEncoderISR() {
  bool A = digitalRead(LEFT_ENC_A);
  bool B = digitalRead(LEFT_ENC_B);
  if (A == B) {
    leftEncoderTicks--;
  } else {
    leftEncoderTicks++;
  }
}

void rightEncoderISR() {
  bool A = digitalRead(RIGHT_ENC_A);
  bool B = digitalRead(RIGHT_ENC_B);
  if (A == B) {
    rightEncoderTicks++;
  } else {
    rightEncoderTicks--;
  }
}

void initEncoders() {
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, CHANGE);

  Serial.println("编码器中断初始化完成");
}

long getLeftTicks() {
  noInterrupts();
  long val = leftEncoderTicks;
  interrupts();
  return val;
}

long getRightTicks() {
  noInterrupts();
  long val = rightEncoderTicks;
  interrupts();
  return val;
}
