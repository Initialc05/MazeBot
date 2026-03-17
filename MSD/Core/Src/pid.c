/**
 * @file pid.c
 * @brief 通用PID控制器实现
 */
#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint   = 0.0f;
    pid->output     = 0.0f;
    pid->out_min    = out_min;
    pid->out_max    = out_max;
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->enabled    = true;
}

float PID_Compute(PID_t *pid, float input, float dt_s)
{
    if (!pid->enabled || dt_s <= 0.0f) {
        pid->output = 0.0f;
        return 0.0f;
    }

    float error = pid->setpoint - input;

    /* 积分项 (带抗饱和钳位) */
    pid->integral += error * dt_s;
    float i_term = pid->ki * pid->integral;
    if (i_term > pid->out_max) { i_term = pid->out_max; pid->integral = pid->out_max / pid->ki; }
    if (i_term < pid->out_min) { i_term = pid->out_min; pid->integral = pid->out_min / pid->ki; }

    /* 微分项 */
    float d_term = (dt_s > 0.0f) ? pid->kd * (error - pid->prev_error) / dt_s : 0.0f;
    pid->prev_error = error;

    /* 输出 */
    float out = pid->kp * error + i_term + d_term;
    if (out > pid->out_max) out = pid->out_max;
    if (out < pid->out_min) out = pid->out_min;

    pid->output = out;
    return out;
}

void PID_Reset(PID_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0.0f;
}
