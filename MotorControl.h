#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <Arduino.h>
#include "wiring_constants.h"
#include <PID_v1.h>
#include <math.h>
#include <ctype.h>
#include "EncoderModule.h"

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
#define KV_P 3.0 
#define KV_I 0.0
#define KV_D 0.1

// 转向环 PID
#define KT_P 0.5   // 转向P：降低以减少超调
#define KT_I 0.0   // 转向I：消除稳态误差
#define KT_D 1.0   // 转向D：增大以抑制震荡

// === 运动速度参数 ===
#define BASE_DUTY 40        // 直线运动基础占空比 (0-100)
#define TURN_SPEED_MIN 20   // 转向最小占空比
#define TURN_SPEED_MAX 30   // 转向最大占空比

// 全局状态 /
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 200;
bool commandActive = false;
char currentCommand = 'x';

int leftDuty = 0;  // 0-100
int rightDuty = 0;
int leftDir = -1;  // 1 正转, -1 反转
int rightDir = -1;

// === 新增: 连续运动标志 ===
static bool continuousCommand = false;  // 大写W/S触发，免超时

// === 新增: PID相关全局 ===
static long prevLeftTicks = 0;
static long prevRightTicks = 0;
static const double MAX_VEL_DIFF_TARGET = 200.0;   // outer环输出限制 (tick/s)
static const double MAX_DUTY_CORR = 40.0;          // inner环输出限制 (占空比差分)

// PID 库要求 double 类型
static double headingInput = 0, headingOutput = 0, headingSetpoint = 0;      // 外环：航向角误差->目标速度差
static double velDiffInput = 0, velDiffOutput = 0, velDiffSetpoint = 0;      // 内环：速度差 -> duty 差
static double turnInput = 0, turnOutput = 0, turnSetpoint = 0;               // 转向环：角度误差 -> 占空比

static PID headingPID(&headingInput, &headingOutput, &headingSetpoint, KH_P, KH_I, KH_D, DIRECT);
static PID velDiffPID(&velDiffInput, &velDiffOutput, &velDiffSetpoint, KV_P, KV_I, KV_D, DIRECT);
static PID turnPID(&turnInput, &turnOutput, &turnSetpoint, KT_P, KT_I, KT_D, DIRECT);

// 目标航向角 (由指令开始时锁定)
static float targetYaw = 0.0f;

// === 转向闭环状态机 ===
enum TurnState {
  TURN_IDLE,        // 空闲
  TURN_ROTATING,    // 转向中
  TURN_REACHED      // 已到达
};

static TurnState turnState = TURN_IDLE;
static float targetTurnAngle = 0.0f;        // 目标绝对角度
static const float turnTolerance = 8.0f;    // 到达容差（±8度，增大以减少震荡）
static unsigned long turnStartTime = 0;
static const unsigned long turnTimeout = 5000;  // 5秒超时

// === 直线运动闭环状态机 ===
enum MoveState {
  MOVE_IDLE,        // 空闲
  MOVE_RUNNING,     // 运动中
  MOVE_REACHED      // 已到达
};

static MoveState moveState = MOVE_IDLE;
static float startOdomX = 0.0f;             // 起始X位置（米）
static float startOdomY = 0.0f;             // 起始Y位置（米）
static float targetDistance = 0.0f;         // 目标距离（米）
static int moveDirection = 1;               // 1=前进, -1=后退
static const float distTolerance = 0.03f;   // 距离容差（±3cm）
static unsigned long moveStartTime = 0;
static const unsigned long moveTimeout = 10000;  // 10秒超时

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

// === 角度误差辅助函数 ===
static inline float yawError(float current, float target) {
  float err = current - target;
  while (err > 180.0f) err -= 360.0f;
  while (err < -180.0f) err += 360.0f;
  return err;
}

// === 精准直线运动函数 ===
void executePreciseMove(float distanceCm, int direction) {
  // 记录起始位置
  startOdomX = OffsetX;
  startOdomY = OffsetY;
  
  // 设置目标距离（cm转m）
  targetDistance = distanceCm / 100.0f;
  moveDirection = direction;  // 1=前进, -1=后退
  
  // 🔧 延迟采样角度，等待IMU稳定（解决转向后立即前进的角度偏差问题）
  delay(50);  // 延迟50ms等待角度稳定
  targetYaw = AngleZ;  // 采样稳定后的角度
  
  moveState = MOVE_RUNNING;
  moveStartTime = millis();
  commandActive = true;
  currentCommand = 'm';  // 标记为精准移动模式
  
  // Serial.print(F("开始精准移动: "));
  // Serial.print(distanceCm);
  // Serial.print(F("cm, 方向: "));
  // Serial.print(direction > 0 ? F("前进") : F("后退"));
  // Serial.print(F(" | 锁定航向: "));
  // Serial.print(targetYaw, 2);
  // Serial.println(F("°"));
}

// 直线运动控制更新
void updateMoveControl() {
  if (moveState != MOVE_RUNNING) {
    return;
  }
  
  // 计算已移动距离（使用勾股定理）
  float dx = OffsetX - startOdomX;
  float dy = OffsetY - startOdomY;
  float movedDist = sqrt(dx * dx + dy * dy);
  
  // 检查是否到达目标
  if (movedDist >= targetDistance - distTolerance) {
    moveState = MOVE_REACHED;
    hardBrake();
    BTSerial.println(F("MOVE_DONE"));  // 通知上位机
    // Serial.print(F("✅ 移动完成! 实际距离: "));
    // Serial.print(movedDist * 100);
    // Serial.println(F("cm"));
    commandActive = false;
    return;
  }
  
  // 超时保护
  if (millis() - moveStartTime > moveTimeout) {
    moveState = MOVE_IDLE;
    hardBrake();
    // Serial.println(F("⚠️  移动超时!"));
    BTSerial.println(F("MOVE_TIMEOUT"));
    commandActive = false;
    return;
  }
  
  // 使用航向PID保持直线（复用直线模式的PID）
  headingInput = static_cast<double>(yawError(AngleZ, targetYaw));
  headingPID.Compute();
  
  // 计算速度差
  long curLeftTicks = getLeftTicks();
  long curRightTicks = getRightTicks();
  long dLeft = curLeftTicks - prevLeftTicks;
  long dRight = curRightTicks - prevRightTicks;
  prevLeftTicks = curLeftTicks;
  prevRightTicks = curRightTicks;
  
  velDiffInput = static_cast<double>(dRight - dLeft);
  velDiffSetpoint = headingOutput;
  velDiffPID.Compute();
  
  double dutyCorr = velDiffOutput * moveDirection;
  double lDuty = BASE_DUTY - dutyCorr / 2.0;
  double rDuty = BASE_DUTY + dutyCorr / 2.0;
  lDuty = constrain(static_cast<int>(round(lDuty)), 0, 100);
  rDuty = constrain(static_cast<int>(round(rDuty)), 0, 100);
  
  setMotor(static_cast<int>(lDuty), static_cast<int>(rDuty), moveDirection, moveDirection);
}

// === 精准转向函数 ===
void executePreciseTurn(float targetAngleDelta) {
  // 计算目标绝对角度
  targetTurnAngle = AngleZ + targetAngleDelta;  // 左转为正，右转为负
  
  // 角度归一化到 [-180, 180]
  while (targetTurnAngle > 180.0f) targetTurnAngle -= 360.0f;
  while (targetTurnAngle < -180.0f) targetTurnAngle += 360.0f;
  
  turnState = TURN_ROTATING;
  turnStartTime = millis();
  commandActive = true;
  currentCommand = 't';  // 标记为转向模式
  
  // Serial.print(F("开始精准转向，目标角度: "));
  // Serial.println(targetTurnAngle);
}

// 转向闭环控制更新
void updateTurnControl() {
  if (turnState != TURN_ROTATING) {
    return;
  }
  
  // 计算角度误差
  float angleError = yawError(targetTurnAngle, AngleZ);
  turnInput = static_cast<double>(angleError);
  
  // 检查是否到达目标
  if (fabs(angleError) < turnTolerance) {
    turnState = TURN_REACHED;
    hardBrake();
    BTSerial.println(F("TURN_DONE"));  // 通知上位机
    // Serial.print(F("✅ 转向完成! 误差: "));
    // Serial.print(angleError, 2);
    // Serial.println(F("°"));
    commandActive = false;
    return;
  }
  
  // 超时保护
  if (millis() - turnStartTime > turnTimeout) {
    turnState = TURN_IDLE;
    hardBrake();
    // Serial.println(F("⚠️  转向超时!"));
    BTSerial.println(F("TURN_TIMEOUT"));
    commandActive = false;
    return;
  }
  
  // PID计算
  turnPID.Compute();
  
  // 限制最小/最大转速（使用独立的转向速度参数）
  int turnDuty = constrain(static_cast<int>(fabs(turnOutput)), TURN_SPEED_MIN, TURN_SPEED_MAX);
  
  // 根据误差方向决定转向方向
  // 误差 > 0：需要增加角度（左转），左轮反转(-1)，右轮正转(+1)
  // 误差 < 0：需要减小角度（右转），左轮正转(+1)，右轮反转(-1)
  int turnDir = (angleError > 0) ? -1 : 1;
  
  // 原地转向：左右轮反向
  setMotor(turnDuty, turnDuty, turnDir, -turnDir);
}

// 蓝牙指令解析（支持精准转向）
static String commandBuffer = "";  // 用于接收多字符指令

void processBluetoothCommand(char cmd) {
  // === 精准转向指令解析 (L90, R45等) ===
  if (cmd == 'L' || cmd == 'R') {
    commandBuffer = String(cmd);
    return;
  }
  
  // === 精准移动指令解析 (F100=前进, B50=后退等) ===
  if (cmd == 'F' || cmd == 'B') {
    commandBuffer = String(cmd);
    return;
  }
  
  if (commandBuffer.length() > 0 && isdigit(cmd)) {
    commandBuffer += cmd;
    return;
  }
  
  if (commandBuffer.length() > 1 && (cmd == '\n' || cmd == '\r')) {
    char cmdType = commandBuffer[0];
    int value = commandBuffer.substring(1).toInt();
    
    // 处理转向指令 L/R
    if (cmdType == 'L' || cmdType == 'R') {
      if (value > 0 && value <= 360) {
        if (cmdType == 'L') {
          executePreciseTurn(value);   // 左转（正角度）
          // Serial.print(F("指令: 左转 "));
          // Serial.print(value);
          // Serial.println(F("°"));
        } else if (cmdType == 'R') {
          executePreciseTurn(-value);  // 右转（负角度）
          // Serial.print(F("指令: 右转 "));
          // Serial.print(value);
          // Serial.println(F("°"));
        }
      } else {
        // Serial.println(F("⚠️  角度范围: 1-360"));
      }
    }
    
    // 处理直线运动指令 F/B (Forward/Backward)
    else if (cmdType == 'F' || cmdType == 'B') {
      if (value > 0 && value <= 500) {  // 最大5米
        int direction = (cmdType == 'F') ? 1 : -1;
        executePreciseMove(value, direction);
        // Serial.print(F("指令: "));
        // Serial.print(cmdType == 'F' ? F("前进") : F("后退"));
        // Serial.print(F(" "));
        // Serial.print(value);
        // Serial.println(F("cm"));
      } else {
        // Serial.println(F("⚠️  距离范围: 1-500cm"));
      }
    }
    
    commandBuffer = "";
    return;
  }
  
  // 清除无效缓冲
  if (cmd == '\n' || cmd == '\r') {
    commandBuffer = "";
  }
  
  // === 原有指令处理 ===
  currentCommand = cmd;
  lastCommandTime = millis();
  // 大写WSAD：持续运动（按住执行，无超时）；小写wsad：临时运动（200ms超时）
  continuousCommand = (cmd == 'W' || cmd == 'S' || cmd == 'A' || cmd == 'D');
  commandActive = (cmd != 'x' && cmd != 'X');

  // 锁定目标航向角
  if (cmd == 'w' || cmd == 's' || cmd == 'W' || cmd == 'S') {
    targetYaw = AngleZ;
  }

  switch (cmd) {
    case 'w': /* Serial.println(F("前进")); */ break;
    case 'W': /* Serial.println(F("持续前进")); */ break;
    case 's': /* Serial.println(F("后退")); */ break;
    case 'S': /* Serial.println(F("持续后退")); */ break;
    case 'a': /* Serial.println(F("左转")); */ break;
    case 'A': /* Serial.println(F("持续左转")); */ break;
    case 'd': /* Serial.println(F("右转")); */ break;
    case 'D': /* Serial.println(F("持续右转")); */ break;
    case 'x':
    case 'X': 
      // Serial.println(F("抱死刹车"));
      turnState = TURN_IDLE;    // 取消精准转向
      moveState = MOVE_IDLE;    // 取消精准移动
      break;
    default: 
      if (!isdigit(cmd) && cmd != '\n' && cmd != '\r') {
        // Serial.println(F("未知指令"));
      }
      break;
  }
}

// === 新增: PID 初始化 ===
inline void initPID() {
  headingSetpoint = 0;
  headingPID.SetMode(AUTOMATIC);
  headingPID.SetSampleTime(5);  // 与任务周期保持一致 (5ms)
  headingPID.SetOutputLimits(-MAX_VEL_DIFF_TARGET, MAX_VEL_DIFF_TARGET);

  velDiffPID.SetMode(AUTOMATIC);
  velDiffPID.SetSampleTime(5);  // 与任务周期保持一致 (5ms)
  velDiffPID.SetOutputLimits(-MAX_DUTY_CORR, MAX_DUTY_CORR);
  
  // 转向PID初始化
  turnSetpoint = 0;  // 目标误差为0
  turnPID.SetMode(AUTOMATIC);
  turnPID.SetSampleTime(5);
  turnPID.SetOutputLimits(-100, 100);  // 输出占空比范围
  
  // Serial.println(F("PID控制器初始化完成（航向环+速差环+转向环）"));
}

// 无闭环示范 /
void updateMotorControlWithoutPID() {
  // —— 指令超时：抱死刹车 ——
  if (commandActive && (millis() - lastCommandTime >= COMMAND_TIMEOUT)) {
    hardBrake();
    commandActive = false;
    currentCommand = 'x';
    // Serial.println(F("指令超时，已抱死刹车"));
    return;
  }

  // —— 空闲：保持抱死 ——
  if (!commandActive) {
    hardBrake();
    return;
  }

  // —— 执行指令前先抱死，再输出新方向 ——
  switch (currentCommand) {
    case 'w':  // 前进
      hardBrake();
      setMotor(BASE_DUTY, BASE_DUTY, 1, 1);
      break;
    case 's':  // 后退
      hardBrake();
      setMotor(BASE_DUTY, BASE_DUTY, -1, -1);
      break;
    case 'a':  // 左转
      hardBrake();
      setMotor(BASE_DUTY, BASE_DUTY, -1, 1);
      break;
    case 'd':  // 右转
      hardBrake();
      setMotor(BASE_DUTY, BASE_DUTY, 1, -1);
      break;
    default:  // 任何未知情况，抱死
      hardBrake();
  }
}

// 向外部暴露的符号（供 MazeBot.ino 使用）
extern unsigned long lastCommandTime;
extern const unsigned long COMMAND_TIMEOUT;
extern bool commandActive;
extern char currentCommand;
extern int leftDuty;
extern int rightDuty;

// 声明来自 IMU 模块的全局变量（由 im948_CMD 提供）
extern float AngleZ;      // 航向角（度）
extern float OffsetX;     // X轴位移（米）
extern float OffsetY;     // Y轴位移（米）

void updateMotorControl() {
  // === 优先处理精准转向控制 ===
  if (turnState == TURN_ROTATING) {
    updateTurnControl();
    return;  // 转向模式下不执行其他控制
  }
  
  // === 优先处理精准移动控制 ===
  if (moveState == MOVE_RUNNING) {
    updateMoveControl();
    return;  // 精准移动模式下不执行其他控制
  }
  
  // —— 指令超时：抱死刹车 ——
  if (commandActive && !continuousCommand && (millis() - lastCommandTime >= COMMAND_TIMEOUT)) {
    hardBrake();
    commandActive = false;
    currentCommand = 'x';
    // Serial.println(F("指令超时，已抱死刹车"));
    return;
  }

  // —— 空闲：保持抱死 ——
  if (!commandActive) {
    hardBrake();
    return;
  }

  // 小写化当前指令，便于统一判断
  char cmdLower = tolower(currentCommand);

  // === 根据指令选择控制模式 ===
  if (cmdLower == 'w' || cmdLower == 's') {
    // --- 直线模式 ---
    int motionDir = (cmdLower == 'w') ? 1 : -1;  // 1前 -1后

    // ====== 外环：航向角 -> 目标速度差 ======
    headingInput = static_cast<double>(yawError(AngleZ, targetYaw));
    headingPID.Compute();  // 结果存入 headingOutput

    // ====== 计算当前轮速差 ======
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

    double lDuty = BASE_DUTY - dutyCorr / 2.0;
    double rDuty = BASE_DUTY + dutyCorr / 2.0;
    lDuty = constrain(static_cast<int>(round(lDuty)), 0, 100);
    rDuty = constrain(static_cast<int>(round(rDuty)), 0, 100);

    setMotor(static_cast<int>(lDuty), static_cast<int>(rDuty), motionDir, motionDir);

    // 调试打印（已注释，提升实时性）
    // Serial.print(F("[直线] YawErr:"));
    // Serial.print(headingInput, 2);
    // Serial.print(F(" DutyCorr:"));
    // Serial.print(dutyCorr, 2);
    // Serial.print(F(" L:"));
    // Serial.print(lDuty);
    // Serial.print(F(" R:"));
    // Serial.println(rDuty);
    return;
  }

  // --- 转向模式 (a/d) ---
  // 左转(a) = 左轮后退(-1) 右轮前进(+1)
  // 右转(d) = 左轮前进(+1) 右轮后退(-1)
  int turnDir = (cmdLower == 'a') ? -1 : 1;  // -1=左转, 1=右转 (符号即左轮方向)

  // 简单开环差速（使用转向速度参数）
  setMotor(TURN_SPEED_MAX, TURN_SPEED_MAX, turnDir, -turnDir);

  // 调试打印（已注释，提升实时性）
  // Serial.print(F("[转向] LeftDir:"));
  // Serial.print(turnDir);
  // Serial.print(F(" RightDir:"));
  // Serial.print(-turnDir);
  // Serial.print(F(" Duty:"));
  // Serial.println(BASE_DUTY);

  return;  // 转向模式结束
}

#endif
