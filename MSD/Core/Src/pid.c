/**
 * @file pid.c
 * @brief PID控制器实现 — 移植自 Arduino PID_v1 (Brett Beauregard)
 */
#include "pid.h"

/* ==================== 内部函数 ==================== */

/** 无冲击转移初始化 (从 MANUAL 切到 AUTOMATIC 时调用) */
static void PID_Initialize(PID_t *pid)
{
    pid->outputSum = *(pid->myOutput);
    pid->lastInput = *(pid->myInput);

    if (pid->outputSum > pid->outMax) pid->outputSum = pid->outMax;
    else if (pid->outputSum < pid->outMin) pid->outputSum = pid->outMin;
}

/* ==================== 核心 API ==================== */

void PID_Init(PID_t *pid, float *input, float *output, float *setpoint,
              float kp, float ki, float kd,
              PID_POn_t pOn, PID_Dir_t dir)
{
    pid->myInput    = input;
    pid->myOutput   = output;
    pid->mySetpoint = setpoint;

    pid->outputSum  = 0.0f;
    pid->lastInput  = 0.0f;

    pid->outMin = 0.0f;
    pid->outMax = 255.0f;   /* Arduino 默认, 会被 SetOutputLimits 覆盖 */

    pid->sampleTime_s = 0.1f;  /* 默认 100ms */
    pid->mode = PID_MANUAL;
    pid->direction = dir;

    PID_SetTunings(pid, kp, ki, kd, pOn);
}

void PID_InitSimple(PID_t *pid, float *input, float *output, float *setpoint,
                    float kp, float ki, float kd)
{
    PID_Init(pid, input, output, setpoint, kp, ki, kd, PID_P_ON_E, PID_DIRECT);
}

bool PID_Compute(PID_t *pid)
{
    if (pid->mode != PID_AUTOMATIC) return false;

    float input  = *(pid->myInput);
    float error  = *(pid->mySetpoint) - input;
    float dInput = input - pid->lastInput;

    /* 积分项: 直接累积到 outputSum */
    pid->outputSum += pid->ki * error;

    /* P_ON_M: 比例项也累积到 outputSum (抗超调) */
    if (!pid->pOnE) {
        pid->outputSum -= pid->kp * dInput;
    }

    /* 积分项限幅 (抗饱和) */
    if (pid->outputSum > pid->outMax) pid->outputSum = pid->outMax;
    else if (pid->outputSum < pid->outMin) pid->outputSum = pid->outMin;

    /* 比例项 */
    float output;
    if (pid->pOnE) {
        output = pid->kp * error;
    } else {
        output = 0.0f;
    }

    /* 完整输出 = P + I - D (微分基于输入变化, 抗微分冲击) */
    output += pid->outputSum - pid->kd * dInput;

    /* 输出限幅 */
    if (output > pid->outMax) output = pid->outMax;
    else if (output < pid->outMin) output = pid->outMin;

    *(pid->myOutput) = output;
    pid->lastInput = input;

    return true;
}

/* ==================== 配置 API ==================== */

void PID_SetTunings(PID_t *pid, float kp, float ki, float kd, PID_POn_t pOn)
{
    if (kp < 0.0f || ki < 0.0f || kd < 0.0f) return;

    pid->pOn  = pOn;
    pid->pOnE = (pOn == PID_P_ON_E);

    pid->dispKp = kp;
    pid->dispKi = ki;
    pid->dispKd = kd;

    /* 预计算: ki 乘以采样时间, kd 除以采样时间 */
    pid->kp = kp;
    pid->ki = ki * pid->sampleTime_s;
    pid->kd = (pid->sampleTime_s > 0.0f) ? kd / pid->sampleTime_s : 0.0f;

    if (pid->direction == PID_REVERSE) {
        pid->kp = -pid->kp;
        pid->ki = -pid->ki;
        pid->kd = -pid->kd;
    }
}

void PID_SetTuningsSimple(PID_t *pid, float kp, float ki, float kd)
{
    PID_SetTunings(pid, kp, ki, kd, pid->pOn);
}

void PID_SetSampleTime(PID_t *pid, float newSampleTime_s)
{
    if (newSampleTime_s <= 0.0f) return;

    float ratio = newSampleTime_s / pid->sampleTime_s;
    pid->ki *= ratio;
    pid->kd /= ratio;
    pid->sampleTime_s = newSampleTime_s;
}

void PID_SetOutputLimits(PID_t *pid, float min, float max)
{
    if (min >= max) return;

    pid->outMin = min;
    pid->outMax = max;

    if (pid->mode == PID_AUTOMATIC) {
        if (*(pid->myOutput) > max) *(pid->myOutput) = max;
        else if (*(pid->myOutput) < min) *(pid->myOutput) = min;

        if (pid->outputSum > max) pid->outputSum = max;
        else if (pid->outputSum < min) pid->outputSum = min;
    }
}

void PID_SetMode(PID_t *pid, PID_Mode_t mode)
{
    bool newAuto = (mode == PID_AUTOMATIC);
    bool wasAuto = (pid->mode == PID_AUTOMATIC);

    /* 从 MANUAL 切到 AUTOMATIC: 无冲击转移 */
    if (newAuto && !wasAuto) {
        PID_Initialize(pid);
    }

    pid->mode = mode;
}

void PID_SetDirection(PID_t *pid, PID_Dir_t dir)
{
    if (pid->mode == PID_AUTOMATIC && dir != pid->direction) {
        pid->kp = -pid->kp;
        pid->ki = -pid->ki;
        pid->kd = -pid->kd;
    }
    pid->direction = dir;
}

/* ==================== 兼容 API ==================== */

void PID_Reset(PID_t *pid)
{
    pid->outputSum = 0.0f;
    pid->lastInput = *(pid->myInput);
    *(pid->myOutput) = 0.0f;
}
