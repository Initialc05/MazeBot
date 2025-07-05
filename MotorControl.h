#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <Arduino.h>         
#include "wiring_constants.h"

#define AIN1      PC8   // 左轮方向
#define AIN2_PWM  PB10  // 左轮 PWM
#define BIN1      PC7   // 右轮方向
#define BIN2_PWM  PB3   // 右轮 PWM

unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 200;   
bool   commandActive   = false;
char   currentCommand  = 'x';

int leftDuty  = 0;     // 0-100
int rightDuty = 0;     // 0-100
int leftDir   = -1;    // 1 = 正转, -1 = 反转
int rightDir  = -1;

// 统一占空比 → 实际 PWM（0-255），确保同 duty 同功率
static inline int dutyToPwm(int duty, int dir)
{
  duty = constrain(duty, 0, 100);
  /* H-Bridge 接法导致正反转 PWM 极性相反：
     - 正转(IN1=H)：duty 越大 → PWM 越高
     - 反转(IN1=L)：duty 越大 → PWM 越高（同速要求 → PWM 同方向）
     解决：正转时需把 duty 倒过来，反转保持原样                     
  */
  return (dir > 0)
         ? map(100 - duty, 0, 100, 0, 255)  // 正转：duty 0=255(刹车), 100=0(满速)
         : map(       duty, 0, 100, 0, 255); // 反转：duty 0=0  (刹车), 100=255(满速)
}

// 电机初始化
void initMotors()
{
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT);  pinMode(AIN2_PWM, OUTPUT);
  pinMode(BIN1, OUTPUT);  pinMode(BIN2_PWM, OUTPUT);

  Serial.println(F("电机初始化完成，蓝牙控制启动"));
  Serial.println(F("指令: w=前 s=后 a=左 d=右 x=停"));
  Serial.println(F("占空比 0=刹车, 100=满速 (方向无关)"));
}

// 主输出接口：设定左右轮 duty 与 dir（1/-1） 
void setMotor(int l_pwm, int r_pwm, int l_dir, int r_dir)
{
  leftDuty   = constrain(l_pwm,  0, 100);
  rightDuty  = constrain(r_pwm, 0, 100);
  leftDir    = (l_dir >= 0)  ?  1 : -1;
  rightDir   = (r_dir >= 0)  ?  1 : -1;

  // 方向端：IN1 高 = 正; 低 = 反 
  digitalWrite(AIN1, (leftDir  > 0) ? HIGH : LOW);
  digitalWrite(BIN1, (rightDir > 0) ? HIGH : LOW);

  // PWM 端：统一占空比映射 
  analogWrite(AIN2_PWM, dutyToPwm(leftDuty,  leftDir));
  analogWrite(BIN2_PWM, dutyToPwm(rightDuty, rightDir));
}

// 蓝牙指令解析
void processBluetoothCommand(char cmd)
{
  currentCommand   = cmd;
  lastCommandTime  = millis();
  commandActive    = (cmd != 'x');

  switch (cmd) {
    case 'w': Serial.println(F("⬆️ 前进")); break;
    case 's': Serial.println(F("⬇️ 后退")); break;
    case 'a': Serial.println(F("⬅️ 左转")); break;
    case 'd': Serial.println(F("➡️ 右转")); break;
    case 'x': Serial.println(F("⏹️ 停止")); break;
    default : Serial.println(F("❓ 未知指令")); break;
  }
}

// *示范*：不做 PID，仅按照当前 cmd 固定 duty 输出 
void updateMotorControlWithoutPID()
{
  // 指令超时自动刹车 
  if (commandActive && (millis() - lastCommandTime >= COMMAND_TIMEOUT)) {
    setMotor(0, 0, leftDir, rightDir);  // 保持方向，停转
    commandActive  = false;
    currentCommand = 'x';
    Serial.println(F("⌛ 指令超时，已刹车"));
    return;
  }
  if (!commandActive) {                 // 空闲
    setMotor(0, 0, leftDir, rightDir);
    return;
  }

  // 持续执行当前指令
  switch (currentCommand) {
    case 'w': setMotor(40, 40,  1,  1);   break; // 前进
    case 's': setMotor(40, 40, -1, -1);   break; // 后退
    case 'a': setMotor(40, 40, -1,  1);   break; // 左转
    case 'd': setMotor(40, 40,  1, -1);   break; // 右转
    default : setMotor(0,  0,  leftDir, rightDir);
  }
}

// 在 MazeBot.ino 中引用的外部声明 
extern unsigned long lastCommandTime;
extern const unsigned long COMMAND_TIMEOUT;
extern bool   commandActive;
extern char   currentCommand;
extern int    leftDuty;
extern int    rightDuty;

#endif
