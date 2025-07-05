#include "wiring_constants.h"
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// 左轮 控制信号
#define AIN1 PC8
#define AIN2_PWM PB10

// 右轮 控制信号
#define BIN1 PC7
#define BIN2_PWM PB3

// 蓝牙控制相关变量
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 200;  // 指令超时时间（ms）
bool commandActive = false;
char currentCommand = 'x';

// 电机控制变量
int leftDuty = 0;
int rightDuty = 0;
int leftDirection = -1;
int rightDirection = -1;

// 电机初始化
void initMotors() {
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2_PWM, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2_PWM, OUTPUT);

  Serial.println("电机初始化完成，蓝牙控制启动");
  Serial.println("蓝牙指令：w=前进 s=后退 a=左转 d=右转 x=停止");
  Serial.println("控制逻辑：指令持续执行500ms，超时自动刹车");
}

// 设置占空比和方向
void setMotor(int l_pwm, int r_pwm, int l_dir, int r_dir) {
  leftDuty = l_pwm;
  rightDuty = r_pwm;
  leftDirection = l_dir;
  rightDirection = r_dir;

  int l_out = map(leftDuty, 0, 100, 0, 255);
  int r_out = map(rightDuty, 0, 100, 0, 255);

  digitalWrite(AIN1, (leftDirection > 0) ? HIGH : LOW);
  analogWrite(AIN2_PWM, l_out);

  digitalWrite(BIN1, (rightDirection > 0) ? HIGH : LOW);
  analogWrite(BIN2_PWM, r_out);
}

// 处理蓝牙命令
void processBluetoothCommand(char cmd) {
  currentCommand = cmd;
  lastCommandTime = millis();
  commandActive = true;

  switch (cmd) {
    case 'w':
      Serial.println(F("⬆️ 前进"));
      break;
    case 's':
      Serial.println(F("⬇️ 后退"));
      break;
    case 'a':
      Serial.println(F("⬅️ 左转"));
      break;
    case 'd':
      Serial.println(F("➡️ 右转"));
      break;
    case 'x':
    default:
      Serial.println(F("⏹️ 停止"));
      commandActive = false;
      break;
  }
}

// 声明外部变量（在MazeBot.ino中使用）
extern unsigned long lastCommandTime;
extern const unsigned long COMMAND_TIMEOUT;
extern bool commandActive;
extern char currentCommand;
extern int leftDuty;
extern int rightDuty;

// 直接输出PWM，不做PID调节
void updateMotorControlWithoutPID() {
  // 检查指令是否超时（COMMAND_TIMEOUT时间窗内没有新指令）
  if (commandActive && (millis() - lastCommandTime >= COMMAND_TIMEOUT)) {
    setMotor(0, 0, -1, -1);  // 立即刹车
    commandActive = false;
    currentCommand = 'x';
    Serial.println("指令超时，立即刹车");
    return;
  }

  // 如果当前没有指令激活，不动
  if (!commandActive) {
    setMotor(0, 0, -1, -1);
    return;
  }

  // 按当前命令持续执行（直到超时或收到新指令）
  switch (currentCommand) {
    case 'w': setMotor(75, 75, 1, 1); break;    // 前进
    case 's': setMotor(50, 50, -1, -1); break;  // 后退
    case 'a': setMotor(30, 80, -1, 1); break;   // 左转
    case 'd': setMotor(80, 30, 1, -1); break;   // 右转
    case 'x': setMotor(0, 0, -1, -1); break;    // 刹车
    default: setMotor(0, 0, -1, -1); break;     // 停止
  }
}

#endif
