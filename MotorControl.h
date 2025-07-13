#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <PID_v1.h>
#include <math.h>
#include <ctype.h>

// 硬件引脚
#define AIN1 PC8       // 左轮方向
#define AIN2_PWM PB10  // 左轮 PWM
#define BIN1 PC7       // 右轮方向
#define BIN2_PWM PB3   // 右轮 PWM

// 航向环 PID
#define KH_P 2.0
#define KH_I 0.0
#define KH_D 0.1

// 速差环 PID
#define KV_P 0.5  
#define KV_I 0.0
#define KV_D 0.1

// 直线运动基础占空比
#define STRAIGHT_BASE_DUTY 50  // 0-100
#define ROTATE_BASE_DUTY 30    // 0-100

// 全局状态
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 200;
bool commandActive = false;
char currentCommand = 'x';

int leftDuty = 0;   
int rightDuty = 0;  
int leftDir = -1;   // 正转
int rightDir = -1;  // 反转

// 连续运动标志
static bool continuousCommand = false;  // 大写W/S/A/D触发，免超时

// 供 startTurnTo 使用
static inline float yawError(float current, float target);
void processBluetoothCommand(char cmd);

static bool autoTurnActive = false;
static float autoTurnTarget = 0.0f;       // 目标航向角(deg)
static const float TURN_THRESHOLD = 2.0f; // 允许误差
static float prevTurnErr = 999.0f;

// 开始自动转向到绝对角度(单位deg, -180 ~ 180)
inline void startTurnTo(float targetDeg) {
  // 归一化到[-180,180]
  while (targetDeg > 180.0f) targetDeg -= 360.0f;
  while (targetDeg < -180.0f) targetDeg += 360.0f;

  autoTurnTarget = targetDeg;
  autoTurnActive = true;
  prevTurnErr = yawError(AngleZ, autoTurnTarget);  // 初始化误差

  // 选择转向方向并发起持续转向指令
  float err = yawError(AngleZ, autoTurnTarget);
  if (err > 0) {
    processBluetoothCommand('A');  // 持续左转
  } else {
    processBluetoothCommand('D');  // 持续右转
  }
}

// PID相关全局变量
static long prevLeftTicks = 0;
static long prevRightTicks = 0;
static const double MAX_VEL_DIFF_TARGET = 200.0;   // outer环输出限制 (tick/s)
static const double MAX_DUTY_CORR = 40.0;          // inner环输出限制 (占空比差分)

// PID 库要求 double 类型
static double headingInput = 0, headingOutput = 0, headingSetpoint = 0;      // 外环：航向角误差->目标速度差
static double velDiffInput = 0, velDiffOutput = 0, velDiffSetpoint = 0;      // 内环：速度差 -> duty 差

static PID headingPID(&headingInput, &headingOutput, &headingSetpoint, KH_P, KH_I, KH_D, DIRECT);
static PID velDiffPID(&velDiffInput, &velDiffOutput, &velDiffSetpoint, KV_P, KV_I, KV_D, DIRECT);

// 目标航向角 (由指令开始时锁定)
static float targetYaw = 0.0f;

// 占空比 -> PWM
static inline int dutyToPwm(int duty, int dir) {
  duty = constrain(duty, 0, 100);
  // 正转(IN1=H) 时需反转 duty，反转(IN1=L) 保持原样
  return (dir > 0)
           ? map(100 - duty, 0, 100, 0, 255)  // 正转: duty 0→255, 100→0
           : map(duty, 0, 100, 0, 255);       // 反转: duty 0→0,   100→255
}

// 电机初始化
void initMotors() {
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2_PWM, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2_PWM, OUTPUT);

  Serial.println(F("电机初始化完成"));
  Serial.println(F("指令: w=前 s=后 a=左 d=右 x=停"));
  Serial.println(F("占空比 0=刹车, 100=满速 (方向无关)"));
}

// 主输出接口
void setMotor(int l_pwm, int r_pwm, int l_dir, int r_dir) {
  leftDuty = constrain(l_pwm, 0, 100);
  rightDuty = constrain(r_pwm, 0, 100);
  leftDir = (l_dir >= 0) ? 1 : -1;
  rightDir = (r_dir >= 0) ? 1 : -1;

  // 方向端
  digitalWrite(AIN1, (leftDir > 0) ? HIGH : LOW);
  digitalWrite(BIN1, (rightDir > 0) ? HIGH : LOW);

  // PWM 端
  analogWrite(AIN2_PWM, dutyToPwm(leftDuty, leftDir));
  analogWrite(BIN2_PWM, dutyToPwm(rightDuty, rightDir));
}

// 抱死刹车
inline void hardBrake() {
  // 左轮
  digitalWrite(AIN1, HIGH);
  analogWrite(AIN2_PWM, 255);
  // 右轮
  digitalWrite(BIN1, HIGH);
  analogWrite(BIN2_PWM, 255);

  // 更新逻辑占空比状态（便于外部查询）
  leftDuty = rightDuty = 0;
}

// 蓝牙指令解析
void processBluetoothCommand(char cmd) {
  currentCommand = cmd;
  lastCommandTime = millis();
  continuousCommand = (cmd == 'W' || cmd == 'S' || cmd == 'A' || cmd == 'D');
  commandActive = (cmd != 'x' && cmd != 'X');

  // 锁定目标航向角
  if (cmd == 'w' || cmd == 's' || cmd == 'W' || cmd == 'S') {
    targetYaw = AngleZ;  // 记录当前航向作为期望航向
  }

  switch (cmd) {
    case 'w': Serial.println(F("前进")); break;
    case 'W': Serial.println(F("持续前进")); break;
    case 's': Serial.println(F("后退")); break;
    case 'S': Serial.println(F("持续后退")); break;
    case 'a': Serial.println(F("左转")); break;
    case 'A': Serial.println(F("持续左转")); break;
    case 'd': Serial.println(F("右转")); break;
    case 'D': Serial.println(F("持续右转")); break;
    case 'x':
    case 'X': Serial.println(F("抱死刹车")); break;
    default: Serial.println(F("未知指令")); break;
  }
}

// 角度误差帮助函数
static inline float yawError(float current, float target) {
  float err = current - target;
  while (err > 180.0f) err -= 360.0f;
  while (err < -180.0f) err += 360.0f;
  return err;
}

// PID 初始化
inline void initPID() {
  headingSetpoint = 0;
  headingPID.SetMode(AUTOMATIC);
  headingPID.SetSampleTime(20);  // 与任务周期保持一致 (ms)
  headingPID.SetOutputLimits(-MAX_VEL_DIFF_TARGET, MAX_VEL_DIFF_TARGET);

  velDiffPID.SetMode(AUTOMATIC);
  velDiffPID.SetSampleTime(20);
  velDiffPID.SetOutputLimits(-MAX_DUTY_CORR, MAX_DUTY_CORR);
}

// 开环控制
void updateMotorControlWithoutPID() {
  // 指令超时抱死刹车
  if (commandActive && (millis() - lastCommandTime >= COMMAND_TIMEOUT)) {
    hardBrake();
    commandActive = false;
    currentCommand = 'x';
    Serial.println(F("指令超时，已抱死刹车"));
    return;
  }

  // 空闲保持抱死
  if (!commandActive) {
    hardBrake();
    return;
  }

  // 执行指令前先抱死，再输出新方向
  switch (currentCommand) {
    case 'w':  // 前进
      hardBrake();
      setMotor(STRAIGHT_BASE_DUTY, STRAIGHT_BASE_DUTY, 1, 1);
      break;
    case 's':  // 后退
      hardBrake();
      setMotor(STRAIGHT_BASE_DUTY, STRAIGHT_BASE_DUTY, -1, -1);
      break;
    case 'a':  // 左转
      hardBrake();
      setMotor(ROTATE_BASE_DUTY, ROTATE_BASE_DUTY, -1, 1);
      break;
    case 'd':  // 右转
      hardBrake();
      setMotor(ROTATE_BASE_DUTY, ROTATE_BASE_DUTY, 1, -1);
      break;
    default:  // 任何未知情况，抱死
      hardBrake();
  }
}

// 向外部暴露的符号
extern unsigned long lastCommandTime;
extern const unsigned long COMMAND_TIMEOUT;
extern bool commandActive;
extern char currentCommand;
extern int leftDuty;
extern int rightDuty;

// 声明来自 IMU900 模块的全局航向角
extern float AngleZ;

void updateMotorControl() {
  // 指令超时抱死刹车
  if (commandActive && !continuousCommand && (millis() - lastCommandTime >= COMMAND_TIMEOUT)) {
    hardBrake();
    commandActive = false;
    currentCommand = 'x';
    Serial.println(F("指令超时，已抱死刹车"));
    return;
  }

  // 空闲保持抱死
  if (!commandActive) {
    hardBrake();
    return;
  }

  // 小写化当前指令，便于统一判断
  char cmdLower = tolower(currentCommand);

  // 根据指令选择控制模式
  if (cmdLower == 'w' || cmdLower == 's') {
    // 直线模式
    int motionDir = (cmdLower == 'w') ? 1 : -1;  // 1前 -1后

    // 外环：航向角 -> 目标速度差
    headingInput = static_cast<double>(yawError(AngleZ, targetYaw));
    headingPID.Compute();  // 结果存入 headingOutput

    // 计算当前轮速差
    long curLeftTicks = getLeftTicks();
    long curRightTicks = getRightTicks();
    long dLeft = curLeftTicks - prevLeftTicks;
    long dRight = curRightTicks - prevRightTicks;
    prevLeftTicks = curLeftTicks;
    prevRightTicks = curRightTicks;

    velDiffInput = static_cast<double>(dRight - dLeft);
    velDiffSetpoint = headingOutput;               // 由外环给定
    velDiffPID.Compute();

    double dutyCorr = velDiffOutput * motionDir;   // 后退补偿方向

    double lDuty = STRAIGHT_BASE_DUTY - dutyCorr / 2.0;
    double rDuty = STRAIGHT_BASE_DUTY + dutyCorr / 2.0;
    lDuty = constrain(static_cast<int>(round(lDuty)), 0, 100);
    rDuty = constrain(static_cast<int>(round(rDuty)), 0, 100);

    setMotor(static_cast<int>(lDuty), static_cast<int>(rDuty), motionDir, motionDir);

    // 调试
    Serial.print(F("[直线] YawErr:"));
    Serial.print(headingInput, 2);
    Serial.print(F(" DutyCorr:"));
    Serial.print(dutyCorr, 2);
    Serial.print(F(" L:"));
    Serial.print(lDuty);
    Serial.print(F(" R:"));
    Serial.println(rDuty);
    return;
  }

  // 转向模式
  // 左转(a) = 左轮后退(-1) 右轮前进(+1)
  // 右转(d) = 左轮前进(+1) 右轮后退(-1)
  int turnDir = (cmdLower == 'a') ? -1 : 1;  // -1=左转, 1=右转 (符号即左轮方向)

  // 简单开环差速
  setMotor(ROTATE_BASE_DUTY, ROTATE_BASE_DUTY, turnDir, -turnDir);

  // 调试
  Serial.print(F("[转向] LeftDir:"));
  Serial.print(turnDir);
  Serial.print(F(" RightDir:"));
  Serial.print(-turnDir);
  Serial.print(F(" Duty:"));
  Serial.println(ROTATE_BASE_DUTY);

  // 自动转向判定
  if (autoTurnActive) {
    float e = yawError(AngleZ, autoTurnTarget);
    // 若已跨过目标或进入阈值
    if (fabs(e) <= TURN_THRESHOLD || (prevTurnErr * e < 0)) {
      hardBrake();
      autoTurnActive = false;
      continuousCommand = false;
      commandActive = false;
      currentCommand = 'x';
      Serial.println(F("AutoTurn reached target, brake"));
    }
    prevTurnErr = e;
  }

  return;  // 转向模式结束
}

#endif
