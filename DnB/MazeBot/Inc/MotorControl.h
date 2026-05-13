#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "wiring_constants.h"
#include <PID_v1.h>
#include <math.h>
#include <ctype.h>
#include "EncoderModule.h"
#include "potentiometer.h"
#include "robot_state.h"

extern HardwareSerial BTSerial;

// 硬件引脚
#define AIN1 PC8
#define AIN2_PWM PB10
#define BIN1 PC7
#define BIN2_PWM PB3

// === 默认运行参数 ===
#define DEFAULT_KH_P 2.0
#define KH_I 0.0
#define KH_D 0.1

#define KV_P 3.0
#define KV_I 0.0
#define KV_D 0.1

#define KT_P 0.4
#define KT_I 0.0
#define KT_D 10.0

#define DEFAULT_BASE_DUTY 40
#define DEFAULT_TURN_DUTY 30

extern unsigned long lastCommandTime;
extern const unsigned long COMMAND_TIMEOUT;
extern bool commandActive;
extern char currentCommand;
extern int leftDuty;
extern int rightDuty;

extern float AngleZ;
extern float OffsetX;
extern float OffsetY;

void initMotors();
void setMotor(int l_pwm, int r_pwm, int l_dir, int r_dir);
void hardBrake();
void processBluetoothCommand(char cmd);
void initPID();
void updateMotorControlWithoutPID();
void updateMotorControl();

#endif
