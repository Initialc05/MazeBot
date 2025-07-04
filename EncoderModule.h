#ifndef ENCODER_MODULE_H
#define ENCODER_MODULE_H

#include <Arduino.h>

// 编码器引脚定义
#define LEFT_ENC_A   PC6
#define LEFT_ENC_B   PB5
#define RIGHT_ENC_A  PA8
#define RIGHT_ENC_B  PA9

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

// 左轮编码器中断
void leftEncoderISR() {
    bool A = digitalRead(LEFT_ENC_A);
    bool B = digitalRead(LEFT_ENC_B);
    if (A == B) {
        leftEncoderTicks--;
    } else {
        leftEncoderTicks++;
    }
}

// 右轮编码器中断
void rightEncoderISR() {
    bool A = digitalRead(RIGHT_ENC_A);
    bool B = digitalRead(RIGHT_ENC_B);
    if (A == B) {
        rightEncoderTicks++;
    } else {
        rightEncoderTicks--;
    }
}

// 初始化编码器
void initEncoders() {
    pinMode(LEFT_ENC_A, INPUT_PULLUP);
    pinMode(LEFT_ENC_B, INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);

    pinMode(LEFT_ENC_A, INPUT_PULLUP);
    pinMode(LEFT_ENC_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, CHANGE);

    Serial.println("编码器中断初始化完成");
}

// 获取当前tick计数
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

#endif
