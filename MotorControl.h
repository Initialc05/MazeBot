#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <Arduino.h>
#include "wiring_constants.h"
#include <PID_v1.h>

// 硬件引脚
#define AIN1      PC8   // 左轮方向
#define AIN2_PWM  PB10  // 左轮 PWM
#define BIN1      PC7   // 右轮方向
#define BIN2_PWM  PB3   // 右轮 PWM

// 控制参数
#define BASE_DUTY    60      // 走直 / 转向的基础输出
#define TURN_ANGLE   90.0    // a/d 目标角
#define MAX_CORR     20      // 航向环最大修正 duty
#define LOOP_DT_MS   20      // 控制周期

// 速度环 PID
#define KP_L 1.2
#define KI_L 0.0
#define KD_L 0.05
#define KP_R 1.2
#define KI_R 0.0
#define KD_R 0.05

// 航向环 PID
#define KH_P 2.0
#define KH_I 0.0
#define KH_D 0.1

// 全局状态 /
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 200;
bool   commandActive   = false;
char   currentCommand  = 'x';

int leftDuty  = 0;     // 0-100
int rightDuty = 0;
int leftDir   = -1;    // 1 正转, -1 反转
int rightDir  = -1;

// 占空比 -> PWM 
static inline int dutyToPwm(int duty, int dir)
{
  duty = constrain(duty, 0, 100);
  // 正转(IN1=H) 时需反转 duty，反转(IN1=L) 保持原样 
  return (dir > 0)
           ? map(100 - duty, 0, 100, 0, 255)   // 正转: duty 0→255, 100→0
           : map(      duty, 0, 100, 0, 255);  // 反转: duty 0→0,   100→255
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

// 主输出接口 
void setMotor(int l_pwm, int r_pwm, int l_dir, int r_dir)
{
  leftDuty   = constrain(l_pwm,  0, 100);
  rightDuty  = constrain(r_pwm, 0, 100);
  leftDir    = (l_dir >= 0)  ?  1 : -1;
  rightDir   = (r_dir >= 0)  ?  1 : -1;

  // 方向端 
  digitalWrite(AIN1, (leftDir  > 0) ? HIGH : LOW);
  digitalWrite(BIN1, (rightDir > 0) ? HIGH : LOW);

  // PWM 端 
  analogWrite(AIN2_PWM, dutyToPwm(leftDuty,  leftDir));
  analogWrite(BIN2_PWM, dutyToPwm(rightDuty, rightDir));
}

// 抱死刹车 
inline void hardBrake()
{
  // 左轮 
  digitalWrite(AIN1, HIGH);
  analogWrite (AIN2_PWM, 255);
  // 右轮 
  digitalWrite(BIN1, HIGH);
  analogWrite (BIN2_PWM, 255);

  // 更新逻辑占空比状态（便于外部查询） 
  leftDuty = rightDuty = 0;
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
    case 'x': Serial.println(F("⏹️ 停止(抱死刹车)")); break;
    default : Serial.println(F("❓ 未知指令")); break;
  }
}

// PID 对象
static double inL = 0, outL = 0, setL = 0;
static double inR = 0, outR = 0, setR = 0;
static double inH = 0, outH = 0, setH = 0;
static PID pidL(&inL, &outL, &setL,  KP_L, KI_L, KD_L,  DIRECT);
static PID pidR(&inR, &outR, &setR,  KP_R, KI_R, KD_R,  DIRECT);
static PID pidH(&inH, &outH, &setH,  KH_P, KH_I, KH_D,  DIRECT);
static double headingTarget = 0;
static unsigned long lastUpdateTime = 0;
static long prevTickL = 0, prevTickR = 0;

void initPID()
{
  pidL.SetSampleTime(LOOP_DT_MS);
  pidR.SetSampleTime(LOOP_DT_MS);
  pidH.SetSampleTime(LOOP_DT_MS);
  pidL.SetOutputLimits(0, 100);
  pidR.SetOutputLimits(0, 100);
  pidH.SetOutputLimits(-MAX_CORR, MAX_CORR);
  pidL.SetMode(AUTOMATIC);
  pidR.SetMode(AUTOMATIC);
  pidH.SetMode(AUTOMATIC);

  headingTarget   = 0;
  lastUpdateTime  = 0;
  prevTickL = prevTickR = 0;
}

// 无闭环示范 /
void updateMotorControlWithoutPID()
{
  // —— 指令超时：抱死刹车 —— 
  if (commandActive && (millis() - lastCommandTime >= COMMAND_TIMEOUT)) {
    hardBrake();
    commandActive  = false;
    currentCommand = 'x';
    Serial.println(F("⌛ 指令超时，已抱死刹车"));
    return;
  }

  // —— 空闲：保持抱死 —— 
  if (!commandActive) {
    hardBrake();
    return;
  }

  // —— 执行指令前先抱死，再输出新方向 —— 
  switch (currentCommand) {
    case 'w':   // 前进
      hardBrake();
      setMotor(40, 40,  1,  1);
      break;
    case 's':   // 后退
      hardBrake();
      setMotor(40, 40, -1, -1);
      break;
    case 'a':   // 左转
      hardBrake();
      setMotor(40, 40, -1,  1);
      break;
    case 'd':   // 右转
      hardBrake();
      setMotor(40, 40,  1, -1);
      break;
    default:    // 任何未知情况，抱死
      hardBrake();
  }
}

// 向外部暴露的符号 /
extern unsigned long lastCommandTime;
extern const unsigned long COMMAND_TIMEOUT;
extern bool   commandActive;
extern char   currentCommand;
extern int    leftDuty;
extern int    rightDuty;

// 在 MazeBot.ino 中引用的外部声明 
extern unsigned long lastCommandTime;
extern const unsigned long COMMAND_TIMEOUT;
extern bool   commandActive;
extern char   currentCommand;
extern int    leftDuty;
extern int    rightDuty;

void updateMotorControl() {
  ;
}

// void updateMotorControl() {
//   // 固定周期执行
//   unsigned long now = millis();
//   if (now - lastUpdateTime < LOOP_DT_MS) return;
//   lastUpdateTime = now;

//   // 指令超时或未激活时直接抱死
//   if (!commandActive || (now - lastCommandTime >= COMMAND_TIMEOUT)) {
//     hardBrake();
//     commandActive  = false;
//     currentCommand = 'x';
//     headingTarget  = 0;
//     return;
//   }

//   // 读取编码器增量估算速度
//   long curTickL = getLeftTicks();
//   long curTickR = getRightTicks();
//   inL = curTickL - prevTickL;           // tick / 周期
//   inR = curTickR - prevTickR;
//   prevTickL = curTickL;
//   prevTickR = curTickR;

//   // 读取航向角
//   inH = getTheta();

//   // 记录指令变化，只在首次进入该指令时锁定 headingTarget
//   static char prevCmd = 'x';
//   if (currentCommand != prevCmd) {
//     if (currentCommand == 'w') {
//       headingTarget = inH;                          // 走直
//     } else if (currentCommand == 's') {
//       headingTarget = inH;                          // 后退保持直线
//     } else if (currentCommand == 'a') {
//       headingTarget = inH - TURN_ANGLE;             // 左转
//     } else if (currentCommand == 'd') {
//       headingTarget = inH + TURN_ANGLE;             // 右转
//     }
//     prevCmd = currentCommand;
//   }

//   // 航向环 PID
//   setH = headingTarget;
//   pidH.Compute();                                   // outH ∈ ±MAX_CORR

//   // 根据指令配置方向
//   if (currentCommand == 'w')      { leftDir =  1; rightDir =  1; }
//   else if (currentCommand == 's') { leftDir = -1; rightDir = -1; }
//   else if (currentCommand == 'a') { leftDir = -1; rightDir =  1; }
//   else if (currentCommand == 'd') { leftDir =  1; rightDir = -1; }

//   // 生成左右轮目标 duty
//   if (currentCommand == 'w' || currentCommand == 's') {
//     setL = constrain(BASE_DUTY + outH, 0, 100);
//     setR = constrain(BASE_DUTY - outH, 0, 100);
//   } else { // 原地旋转
//     setL = constrain(BASE_DUTY + outH, 0, 100);
//     setR = constrain(BASE_DUTY - outH, 0, 100);
//   }

//   // 速度环 PID
//   pidL.Compute();
//   pidR.Compute();
//   setMotor((int)outL, (int)outR, leftDir, rightDir);

//   // 原地旋转到位自动停止
//   if ((currentCommand == 'a' || currentCommand == 'd') &&
//       abs(inH - headingTarget) < 3.0) {
//     hardBrake();
//     commandActive  = false;
//     currentCommand = 'x';
//     headingTarget  = 0;
//   }
// }

#endif
