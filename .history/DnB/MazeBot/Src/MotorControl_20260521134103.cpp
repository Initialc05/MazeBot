#include "../Inc/MotorControl.h"
#include "../Inc/host_display_sync.h"

unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 200;
bool commandActive = false;
char currentCommand = 'x';

int leftDuty = 0;
int rightDuty = 0;
int leftDir = -1;
int rightDir = -1;

static bool continuousCommand = false;

static long prevLeftTicks = 0;
static long prevRightTicks = 0;
static const double MAX_VEL_DIFF_TARGET = 200.0;
static const double MAX_DUTY_CORR = 40.0;

static double headingInput = 0, headingOutput = 0, headingSetpoint = 0;
static double velDiffInput = 0, velDiffOutput = 0, velDiffSetpoint = 0;
static double turnInput = 0, turnOutput = 0, turnSetpoint = 0;
static double headingKp = DEFAULT_KH_P;
static int runtimeBaseDuty = DEFAULT_BASE_DUTY;
static int runtimeTurnDuty = DEFAULT_TURN_DUTY;

static PID headingPID(&headingInput, &headingOutput, &headingSetpoint, DEFAULT_KH_P, KH_I, KH_D, DIRECT);
static PID velDiffPID(&velDiffInput, &velDiffOutput, &velDiffSetpoint, KV_P, KV_I, KV_D, DIRECT);
static PID turnPID(&turnInput, &turnOutput, &turnSetpoint, KT_P, KT_I, KT_D, DIRECT);

static float targetYaw = 0.0f;

namespace {
enum TurnState {
  TURN_IDLE,
  TURN_ROTATING,
  TURN_REACHED
};

enum MoveState {
  MOVE_IDLE,
  MOVE_RUNNING,
  MOVE_REACHED
};

TurnState turnState = TURN_IDLE;
float targetTurnAngle = 0.0f;
const float turnTolerance = 1.0f;
unsigned long turnStartTime = 0;
const unsigned long turnTimeout = 5000;

MoveState moveState = MOVE_IDLE;
float startOdomX = 0.0f;
float startOdomY = 0.0f;
float targetDistance = 0.0f;
int moveDirection = 1;
const float distTolerance = 0.03f;
unsigned long moveStartTime = 0;
const unsigned long moveTimeout = 10000;

String commandBuffer = "";

inline int dutyToPwm(int duty, int dir) {
  duty = constrain(duty, 0, 100);
  return (dir > 0)
           ? map(100 - duty, 0, 100, 0, 255)
           : map(duty, 0, 100, 0, 255);
}

inline float yawError(float current, float target) {
  float err = current - target;
  while (err > 180.0f) err -= 360.0f;
  while (err < -180.0f) err += 360.0f;
  return err;
}

void executePreciseMove(float distanceCm, int direction) {
  startOdomX = OffsetX;
  startOdomY = OffsetY;
  targetDistance = distanceCm / 100.0f;
  moveDirection = direction;

  delay(50);
  targetYaw = AngleZ;

  moveState = MOVE_RUNNING;
  moveStartTime = millis();
  commandActive = true;
  currentCommand = 'm';
}

void updateMoveControl() {
  if (moveState != MOVE_RUNNING) {
    return;
  }

  PotValues potValues = Pot_GetValues();
  headingKp = potValues.kp_heading;
  runtimeBaseDuty = potValues.base_duty;
  headingPID.SetTunings(headingKp, KH_I, KH_D);

  float dx = OffsetX - startOdomX;
  float dy = OffsetY - startOdomY;
  float movedDist = sqrt(dx * dx + dy * dy);

  if (movedDist >= targetDistance - distTolerance) {
    moveState = MOVE_REACHED;
    hardBrake();
    BTSerial.println(F("MOVE_DONE"));
    commandActive = false;
    return;
  }

  if (millis() - moveStartTime > moveTimeout) {
    moveState = MOVE_IDLE;
    hardBrake();
    BTSerial.println(F("MOVE_TIMEOUT"));
    commandActive = false;
    return;
  }

  headingInput = static_cast<double>(yawError(AngleZ, targetYaw));
  headingPID.Compute();

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
  double lDuty = runtimeBaseDuty - dutyCorr / 2.0;
  double rDuty = runtimeBaseDuty + dutyCorr / 2.0;
  lDuty = constrain(static_cast<int>(round(lDuty)), 0, 100);
  rDuty = constrain(static_cast<int>(round(rDuty)), 0, 100);

  setMotor(static_cast<int>(lDuty), static_cast<int>(rDuty), moveDirection, moveDirection);
}

void executePreciseTurn(float targetAngleDelta) {
  targetTurnAngle = AngleZ + targetAngleDelta;

  while (targetTurnAngle > 180.0f) targetTurnAngle -= 360.0f;
  while (targetTurnAngle < -180.0f) targetTurnAngle += 360.0f;

  turnState = TURN_ROTATING;
  turnStartTime = millis();
  commandActive = true;
  currentCommand = 't';
}

void updateTurnControl() {
  if (turnState != TURN_ROTATING) {
    return;
  }

  PotValues potValues = Pot_GetValues();
  runtimeTurnDuty = potValues.turn_duty;

  float angleError = yawError(targetTurnAngle, AngleZ);
  turnInput = static_cast<double>(angleError);

  if (fabs(angleError) < turnTolerance) {
    turnState = TURN_REACHED;
    hardBrake();
    BTSerial.println(F("TURN_DONE"));
    commandActive = false;
    return;
  }

  if (millis() - turnStartTime > turnTimeout) {
    turnState = TURN_IDLE;
    hardBrake();
    BTSerial.println(F("TURN_TIMEOUT"));
    commandActive = false;
    return;
  }

  turnPID.Compute();

  int turnDuty = runtimeTurnDuty;
  int turnDir = (angleError > 0) ? -1 : 1;
  setMotor(turnDuty, turnDuty, turnDir, -turnDir);
}
}

void initMotors() {
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2_PWM, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2_PWM, OUTPUT);
}

void setMotor(int l_pwm, int r_pwm, int l_dir, int r_dir) {
  leftDuty = constrain(l_pwm, 0, 100);
  rightDuty = constrain(r_pwm, 0, 100);
  leftDir = (l_dir >= 0) ? 1 : -1;
  rightDir = (r_dir >= 0) ? 1 : -1;

  digitalWrite(AIN1, (leftDir > 0) ? HIGH : LOW);
  digitalWrite(BIN1, (rightDir > 0) ? HIGH : LOW);

  analogWrite(AIN2_PWM, dutyToPwm(leftDuty, leftDir));
  analogWrite(BIN2_PWM, dutyToPwm(rightDuty, rightDir));
}

void hardBrake() {
  digitalWrite(AIN1, HIGH);
  analogWrite(AIN2_PWM, 255);
  digitalWrite(BIN1, HIGH);
  analogWrite(BIN2_PWM, 255);
  leftDuty = rightDuty = 0;
}

void processBluetoothCommand(char cmd) {
  if (HostDisplay_ProcessByte(cmd)) {
    return;
  }

  if (RobotState_IsEstopLatched() && cmd != 'x' && cmd != 'X') {
    return;
  }

  if (cmd == 'L' || cmd == 'R') {
    commandBuffer = String(cmd);
    return;
  }

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

    if (cmdType == 'L' || cmdType == 'R') {
      if (value > 0 && value <= 360) {
        if (cmdType == 'L') {
          executePreciseTurn(value);
        } else if (cmdType == 'R') {
          executePreciseTurn(-value);
        }
      }
    }
    else if (cmdType == 'F' || cmdType == 'B') {
      if (value > 0 && value <= 500) {
        int direction = (cmdType == 'F') ? 1 : -1;
        executePreciseMove(value, direction);
      }
    }

    commandBuffer = "";
    return;
  }

  if (cmd == '\n' || cmd == '\r') {
    commandBuffer = "";
  }

  currentCommand = cmd;
  lastCommandTime = millis();
  continuousCommand = (cmd == 'W' || cmd == 'S' || cmd == 'A' || cmd == 'D');
  commandActive = (cmd != 'x' && cmd != 'X');

  if (cmd == 'w' || cmd == 's' || cmd == 'W' || cmd == 'S') {
    targetYaw = AngleZ;
  }

  switch (cmd) {
    case 'x':
    case 'X':
      turnState = TURN_IDLE;
      moveState = MOVE_IDLE;
      break;
    default:
      break;
  }
}

void initPID() {
  headingSetpoint = 0;
  headingPID.SetMode(AUTOMATIC);
  headingPID.SetSampleTime(5);
  headingPID.SetOutputLimits(-MAX_VEL_DIFF_TARGET, MAX_VEL_DIFF_TARGET);

  velDiffPID.SetMode(AUTOMATIC);
  velDiffPID.SetSampleTime(5);
  velDiffPID.SetOutputLimits(-MAX_DUTY_CORR, MAX_DUTY_CORR);

  turnSetpoint = 0;
  turnPID.SetMode(AUTOMATIC);
  turnPID.SetSampleTime(5);
  turnPID.SetOutputLimits(-100, 100);
}

void updateMotorControlWithoutPID() {
  if (RobotState_IsEstopLatched()) {
    hardBrake();
    commandActive = false;
    currentCommand = 'x';
    return;
  }

  if (commandActive && (millis() - lastCommandTime >= COMMAND_TIMEOUT)) {
    hardBrake();
    commandActive = false;
    currentCommand = 'x';
    return;
  }

  if (!commandActive) {
    hardBrake();
    return;
  }

  switch (currentCommand) {
    case 'w':
      hardBrake();
      setMotor(runtimeBaseDuty, runtimeBaseDuty, 1, 1);
      break;
    case 's':
      hardBrake();
      setMotor(runtimeBaseDuty, runtimeBaseDuty, -1, -1);
      break;
    case 'a':
      hardBrake();
      setMotor(runtimeBaseDuty, runtimeBaseDuty, -1, 1);
      break;
    case 'd':
      hardBrake();
      setMotor(runtimeBaseDuty, runtimeBaseDuty, 1, -1);
      break;
    default:
      hardBrake();
  }
}

void updateMotorControl() {
  if (RobotState_IsEstopLatched()) {
    hardBrake();
    commandActive = false;
    currentCommand = 'x';
    return;
  }

  PotValues potValues = Pot_GetValues();
  headingKp = potValues.kp_heading;
  runtimeBaseDuty = potValues.base_duty;
  runtimeTurnDuty = potValues.turn_duty;
  headingPID.SetTunings(headingKp, KH_I, KH_D);

  if (turnState == TURN_ROTATING) {
    updateTurnControl();
    return;
  }

  if (moveState == MOVE_RUNNING) {
    updateMoveControl();
    return;
  }

  if (commandActive && !continuousCommand && (millis() - lastCommandTime >= COMMAND_TIMEOUT)) {
    hardBrake();
    commandActive = false;
    currentCommand = 'x';
    return;
  }

  if (!commandActive) {
    hardBrake();
    return;
  }

  char cmdLower = tolower(currentCommand);

  if (cmdLower == 'w' || cmdLower == 's') {
    int motionDir = (cmdLower == 'w') ? 1 : -1;

    headingInput = static_cast<double>(yawError(AngleZ, targetYaw));
    headingPID.Compute();

    long curLeftTicks = getLeftTicks();
    long curRightTicks = getRightTicks();
    long dLeft = curLeftTicks - prevLeftTicks;
    long dRight = curRightTicks - prevRightTicks;
    prevLeftTicks = curLeftTicks;
    prevRightTicks = curRightTicks;

    velDiffInput = static_cast<double>(dRight - dLeft);
    velDiffSetpoint = headingOutput;
    velDiffPID.Compute();

    double dutyCorr = velDiffOutput * motionDir;

    double lDuty = runtimeBaseDuty - dutyCorr / 2.0;
    double rDuty = runtimeBaseDuty + dutyCorr / 2.0;
    lDuty = constrain(static_cast<int>(round(lDuty)), 0, 100);
    rDuty = constrain(static_cast<int>(round(rDuty)), 0, 100);

    setMotor(static_cast<int>(lDuty), static_cast<int>(rDuty), motionDir, motionDir);
    return;
  }

  int turnDir = (cmdLower == 'a') ? -1 : 1;
  setMotor(runtimeTurnDuty, runtimeTurnDuty, turnDir, -turnDir);
}
