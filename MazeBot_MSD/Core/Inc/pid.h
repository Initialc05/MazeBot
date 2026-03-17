/**
 * @file pid.h
 * @brief 通用PID控制器 (替代Arduino PID_v1库)
 *
 * 级联PID架构:
 *   外环 Heading PID:  Kp=2.0 Ki=0 Kd=0.1  → 目标速度差(ticks/s)
 *   内环 VelDiff PID:  Kp=3.0 Ki=0 Kd=0.1  → 占空比修正(%)
 *   转弯 Turn PID:     Kp=0.4 Ki=0 Kd=10.0 → 转弯占空比(%)
 */
#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float kp, ki, kd;
    float setpoint;
    float output;
    float out_min, out_max;
    float integral;
    float prev_error;
    bool  enabled;
} PID_t;

/** 初始化PID结构体 */
void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_min, float out_max);

/** 计算PID输出, dt_s为采样周期(秒) */
float PID_Compute(PID_t *pid, float input, float dt_s);

/** 重置积分项和上次误差 */
void PID_Reset(PID_t *pid);

/** 设置目标值 */
static inline void PID_SetTarget(PID_t *pid, float sp) { pid->setpoint = sp; }

/** 使能/禁用 */
static inline void PID_Enable(PID_t *pid, bool en) { pid->enabled = en; }

#endif /* PID_H */
