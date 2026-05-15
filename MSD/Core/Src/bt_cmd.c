/**
 * @file bt_cmd.c
 * @brief 蓝牙命令解析 + 运动状态机 + 级联PID控制
 */
#include "bt_cmd.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "im948.h"
#include "uart_device.h"
#include "robot_state.h"
#include "potentiometer.h"
#include <math.h>
#include <ctype.h>
#include <string.h>

/* ==================== PID参数 ==================== */
#define KH_P  2.0f    /* 航向环 */
#define KH_I  0.0f
#define KH_D  0.1f
#define KV_P  3.0f    /* 速差环 */
#define KV_I  0.0f
#define KV_D  0.1f
#define KT_P  0.4f    /* 转弯环 */
#define KT_I  0.0f
#define KT_D  10.0f

#define MAX_VEL_DIFF_TARGET  200.0f
#define MAX_DUTY_CORR         40.0f
#define TURN_SPEED_MIN        25
#define TURN_SPEED_MAX        30

/* ==================== 运动参数 ==================== */
#define DIST_TOLERANCE   0.03f    /* 距离容差 3cm */
#define TURN_TOLERANCE   5.0f     /* 角度容差 5度 */
#define MOVE_TIMEOUT_MS  10000
#define TURN_TIMEOUT_MS  5000

/* ==================== PID实例 ==================== */
static PID_t headingPID;
static PID_t velDiffPID;
static PID_t turnPID;

/* PID 绑定变量 */
static float headingInput, headingOutput, headingSetpoint;
static float velDiffInput, velDiffOutput, velDiffSetpoint;
static float turnInput,    turnOutput,    turnSetpoint;

/* ==================== 全局状态 ==================== */
TurnState turnState  = TURN_IDLE;
MoveState moveState  = MOVE_IDLE;

bool     cmd_active    = false;
bool     cmd_continuous = false;
char     cmd_current   = 'x';
uint32_t cmd_last_time = 0;
float    targetYaw     = 0.0f;

/* 转弯状态 */
static float    targetTurnAngle = 0.0f;
static uint32_t turnStartTime   = 0;

/* 直线状态 */
static float    startOdomX      = 0.0f;
static float    startOdomY      = 0.0f;
static float    targetDistance  = 0.0f;
static int      moveDirection   = 1;
static uint32_t moveStartTime   = 0;

/* 命令缓冲区 (多字符命令) */
static char cmdBuf[16];
static uint8_t cmdBufLen = 0;

/* ==================== 工具函数 ==================== */
static float yawError(float current, float target)
{
    float err = current - target;
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

/* ==================== 初始化 ==================== */
void BtCmd_Init(void)
{
    /* Heading PID */
    headingSetpoint = 0.0f;
    PID_InitSimple(&headingPID, &headingInput, &headingOutput, &headingSetpoint, KH_P, KH_I, KH_D);
    PID_SetOutputLimits(&headingPID, -MAX_VEL_DIFF_TARGET, MAX_VEL_DIFF_TARGET);
    PID_SetSampleTime(&headingPID, 0.005f);
    PID_SetMode(&headingPID, PID_AUTOMATIC);

    /* VelDiff PID */
    velDiffSetpoint = 0.0f;
    PID_InitSimple(&velDiffPID, &velDiffInput, &velDiffOutput, &velDiffSetpoint, KV_P, KV_I, KV_D);
    PID_SetOutputLimits(&velDiffPID, -MAX_DUTY_CORR, MAX_DUTY_CORR);
    PID_SetSampleTime(&velDiffPID, 0.005f);
    PID_SetMode(&velDiffPID, PID_AUTOMATIC);

    /* Turn PID */
    turnSetpoint = 0.0f;
    PID_InitSimple(&turnPID, &turnInput, &turnOutput, &turnSetpoint, KT_P, KT_I, KT_D);
    PID_SetOutputLimits(&turnPID, -(float)TURN_SPEED_MAX, (float)TURN_SPEED_MAX);
    PID_SetSampleTime(&turnPID, 0.005f);
    PID_SetMode(&turnPID, PID_AUTOMATIC);

    turnState  = TURN_IDLE;
    moveState  = MOVE_IDLE;
    cmd_active = false;
    cmd_current = 'x';
    cmdBufLen  = 0;
}

/* ==================== 精准直线运动 ==================== */
void BtCmd_ExecutePreciseMove(float distance_cm, int direction)
{
    startOdomX = odom_x;
    startOdomY = odom_y;
    targetDistance = distance_cm / 100.0f;
    moveDirection = direction;

    /* 短延迟等IMU角度稳定 */
    HAL_Delay(50);
    targetYaw = AngleZ;

    moveState = MOVE_RUNNING;
    moveStartTime = HAL_GetTick();
    cmd_active = true;
    cmd_current = 'm';

    PID_Reset(&headingPID);
    PID_Reset(&velDiffPID);
}

/* ==================== 精准转弯 ==================== */
void BtCmd_ExecutePreciseTurn(float angle_deg)
{
    targetTurnAngle = AngleZ + angle_deg;
    /* 归一化到 [-180, 180] */
    while (targetTurnAngle >  180.0f) targetTurnAngle -= 360.0f;
    while (targetTurnAngle < -180.0f) targetTurnAngle += 360.0f;

    turnState = TURN_ROTATING;
    turnStartTime = HAL_GetTick();
    cmd_active = true;
    cmd_current = 't';

    PID_Reset(&turnPID);
}

/* ==================== 命令字节解析 ==================== */
static int simple_atoi(const char *s, int len)
{
    int val = 0;
    for (int i = 0; i < len; i++) {
        if (s[i] >= '0' && s[i] <= '9')
            val = val * 10 + (s[i] - '0');
    }
    return val;
}

void BtCmd_ProcessByte(char cmd)
{
    /* E-STOP 守卫: 拒绝所有运动命令 */
    if (g_estop_latched) {
        if (cmd == '\n' || cmd == '\r') cmdBufLen = 0;
        return;
    }

    /* 多字符命令起始: F/B/L/R */
    if (cmd == 'F' || cmd == 'B' || cmd == 'L' || cmd == 'R') {
        cmdBuf[0] = cmd;
        cmdBufLen = 1;
        return;
    }

    /* 累积数字 */
    if (cmdBufLen > 0 && isdigit((unsigned char)cmd)) {
        if (cmdBufLen < sizeof(cmdBuf) - 1) {
            cmdBuf[cmdBufLen++] = cmd;
        }
        return;
    }

    /* 换行 → 处理完整多字符命令 */
    if (cmdBufLen > 1 && (cmd == '\n' || cmd == '\r')) {
        char type = cmdBuf[0];
        int value = simple_atoi(&cmdBuf[1], cmdBufLen - 1);

        if ((type == 'L' || type == 'R') && value > 0 && value <= 360) {
            float angle = (type == 'L') ? (float)value : -(float)value;
            BtCmd_ExecutePreciseTurn(angle);
        } else if ((type == 'F' || type == 'B') && value > 0 && value <= 500) {
            int dir = (type == 'F') ? 1 : -1;
            BtCmd_ExecutePreciseMove((float)value, dir);
        }
        cmdBufLen = 0;
        return;
    }

    /* 换行清空无效缓冲 */
    if (cmd == '\n' || cmd == '\r') {
        cmdBufLen = 0;
        return;
    }

    /* 单字符命令 */
    cmdBufLen = 0;
    cmd_current = cmd;
    cmd_last_time = HAL_GetTick();
    cmd_continuous = (cmd == 'W' || cmd == 'S' || cmd == 'A' || cmd == 'D');
    cmd_active = (cmd != 'x' && cmd != 'X');

    /* 直行锁定航向 */
    if (cmd == 'w' || cmd == 's' || cmd == 'W' || cmd == 'S') {
        targetYaw = AngleZ;
        PID_Reset(&headingPID);
        PID_Reset(&velDiffPID);
    }

    /* 停止 */
    if (cmd == 'x' || cmd == 'X') {
        turnState = TURN_IDLE;
        moveState = MOVE_IDLE;
        Motor_Brake();
    }
}

/* ==================== 转弯控制更新 ==================== */
static void updateTurnControl(float dt_s)
{
    if (turnState != TURN_ROTATING) return;

    float err = yawError(AngleZ, targetTurnAngle);

    /* 到达判定 */
    if (fabsf(err) < TURN_TOLERANCE) {
        turnState = TURN_REACHED;
        Motor_Brake();
        BT_SendString("TURN_DONE\r\n");
        cmd_active = false;
        return;
    }

    /* 超时 */
    if (HAL_GetTick() - turnStartTime > TURN_TIMEOUT_MS) {
        turnState = TURN_REACHED;
        Motor_Brake();
        BT_SendString("TURN_TIMEOUT\r\n");
        cmd_active = false;
        return;
    }

    /* PID计算 */
    turnSetpoint = 0.0f;
    turnInput = err;
    PID_Compute(&turnPID);
    float duty_f = turnOutput;
    int duty = (int)fabsf(duty_f);
    if (duty < TURN_SPEED_MIN) duty = TURN_SPEED_MIN;
    if (duty > TURN_SPEED_MAX) duty = TURN_SPEED_MAX;

    /* 差速转弯: err>0 需右转, err<0 需左转 */
    if (err > 0) {
        Motor_Set(-1, duty, 1, duty);  /* 左反右正 = 右转 */
    } else {
        Motor_Set(1, duty, -1, duty);  /* 左正右反 = 左转 */
    }
}

/* ==================== 直线控制更新 ==================== */
static void updateMoveControl(float dt_s)
{
    if (moveState != MOVE_RUNNING) return;

    /* 计算已移动距离 */
    float dx = odom_x - startOdomX;
    float dy = odom_y - startOdomY;
    float moved = sqrtf(dx * dx + dy * dy);

    /* 到达判定 */
    if (moved >= targetDistance - DIST_TOLERANCE) {
        moveState = MOVE_REACHED;
        Motor_Brake();
        BT_SendString("MOVE_DONE\r\n");
        cmd_active = false;
        return;
    }

    /* 超时 */
    if (HAL_GetTick() - moveStartTime > MOVE_TIMEOUT_MS) {
        moveState = MOVE_REACHED;
        Motor_Brake();
        BT_SendString("MOVE_TIMEOUT\r\n");
        cmd_active = false;
        return;
    }

    /* 外环: 航向误差 → 目标速度差 */
    float heading_err = yawError(AngleZ, targetYaw);
    headingSetpoint = 0.0f;
    headingInput = heading_err;
    PID_Compute(&headingPID);
    float vel_diff_target = headingOutput;

    /* 内环: 速度差 → 占空比修正 */
    float left_spd  = Encoder_GetLeftSpeed(dt_s);
    float right_spd = Encoder_GetRightSpeed(dt_s);
    float vel_diff  = right_spd - left_spd;
    velDiffSetpoint = vel_diff_target;
    velDiffInput = vel_diff;
    PID_Compute(&velDiffPID);
    float duty_corr = velDiffOutput * (float)moveDirection;

    /* 应用到电机 */
    int base = (int)g_pot_values.base_duty;
    int l_duty = (int)lroundf((float)base - duty_corr * 0.5f);
    int r_duty = (int)lroundf((float)base + duty_corr * 0.5f);
    if (l_duty < 0) l_duty = 0;
    if (r_duty < 0) r_duty = 0;
    if (l_duty > MOTOR_DUTY_MAX) l_duty = MOTOR_DUTY_MAX;
    if (r_duty > MOTOR_DUTY_MAX) r_duty = MOTOR_DUTY_MAX;

    Motor_Set(moveDirection, l_duty, moveDirection, r_duty);
}

/* ==================== 简单命令控制 ==================== */
static void updateSimpleControl(float dt_s)
{
    /* 超时检查 */
    if (!cmd_continuous && (HAL_GetTick() - cmd_last_time > MOTOR_CMD_TIMEOUT)) {
        cmd_active = false;
        Motor_Brake();
        return;
    }

    char c = cmd_current;

    if (c == 'w' || c == 'W' || c == 's' || c == 'S') {
        /* 直行: 航向PID保持方向 */
        float heading_err = yawError(AngleZ, targetYaw);
        headingSetpoint = 0.0f;
        headingInput = heading_err;
        PID_Compute(&headingPID);
        float vel_diff_target = headingOutput;

        float left_spd  = Encoder_GetLeftSpeed(dt_s);
        float right_spd = Encoder_GetRightSpeed(dt_s);
        velDiffSetpoint = vel_diff_target;
        velDiffInput = right_spd - left_spd;
        PID_Compute(&velDiffPID);
        int dir = (c == 'w' || c == 'W') ? 1 : -1;
        float duty_corr = velDiffOutput * (float)dir;

        int base = (int)g_pot_values.base_duty;
        int l_duty = (int)lroundf((float)base - duty_corr * 0.5f);
        int r_duty = (int)lroundf((float)base + duty_corr * 0.5f);
        if (l_duty < 0) l_duty = 0;
        if (l_duty > 100) l_duty = 100;
        if (r_duty < 0) r_duty = 0;
        if (r_duty > 100) r_duty = 100;
        Motor_Set(dir, l_duty, dir, r_duty);

    } else if (c == 'a' || c == 'A') {
        Motor_TurnLeft((int)g_pot_values.turn_duty);
    } else if (c == 'd' || c == 'D') {
        Motor_TurnRight((int)g_pot_values.turn_duty);
    }
}

/* ==================== 总控制入口 ==================== */
void BtCmd_UpdateMotorControl(float dt_s)
{
    /* E-STOP 守卫 */
    if (g_estop_latched) {
        Motor_Brake();
        return;
    }

    /* 优先级1: 精准转弯 */
    if (turnState == TURN_ROTATING) {
        updateTurnControl(dt_s);
        return;
    }

    /* 优先级2: 精准直线 */
    if (moveState == MOVE_RUNNING) {
        updateMoveControl(dt_s);
        return;
    }

    /* 优先级3: 简单命令 */
    if (cmd_active) {
        updateSimpleControl(dt_s);
        return;
    }
}
