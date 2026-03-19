/**
 * @file pid.h
 * @brief PID控制器 — 移植自 Arduino PID_v1 (Brett Beauregard)
 *
 * 特性:
 *   - 微分项基于输入变化 (Derivative-on-Measurement, 抗微分冲击)
 *   - 积分项直接累积到输出 (天然避免 Ki=0 除零)
 *   - 双重限幅: 积分项 + 最终输出
 *   - 支持 P_ON_E / P_ON_M 两种比例项模式
 *   - 支持 DIRECT / REVERSE 控制方向
 *   - 无冲击转移 (MANUAL → AUTOMATIC 平滑切换)
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

/* ==================== 枚举定义 ==================== */

typedef enum {
    PID_P_ON_E = 0,   /* 比例项基于误差 (传统) */
    PID_P_ON_M = 1    /* 比例项基于测量值变化 (抗超调) */
} PID_POn_t;

typedef enum {
    PID_DIRECT  = 0,   /* 输出增大 → 输入增大 */
    PID_REVERSE = 1    /* 输出增大 → 输入减小 */
} PID_Dir_t;

typedef enum {
    PID_MANUAL    = 0,
    PID_AUTOMATIC = 1
} PID_Mode_t;

/* ==================== 结构体 ==================== */

typedef struct {
    /* 用户设定的原始增益 (用于查询) */
    float dispKp, dispKi, dispKd;

    /* 预计算增益 (已乘/除采样时间, 已处理方向) */
    float kp, ki, kd;

    /* 指针绑定 */
    float *myInput;
    float *myOutput;
    float *mySetpoint;

    /* 运行时状态 */
    float outputSum;    /* 积分累积项 */
    float lastInput;    /* 上次输入值 (用于 Derivative-on-Measurement) */

    /* 配置 */
    float outMin, outMax;
    float sampleTime_s; /* 采样周期 (秒) */

    PID_POn_t  pOn;
    PID_Dir_t  direction;
    PID_Mode_t mode;
    bool       pOnE;    /* 快速标志: pOn == PID_P_ON_E */
} PID_t;

/* ==================== 核心 API ==================== */

/** 初始化 (完整版) */
void PID_Init(PID_t *pid, float *input, float *output, float *setpoint,
              float kp, float ki, float kd,
              PID_POn_t pOn, PID_Dir_t dir);

/** 初始化 (简化版, 默认 P_ON_E + DIRECT) */
void PID_InitSimple(PID_t *pid, float *input, float *output, float *setpoint,
                    float kp, float ki, float kd);

/** PID 计算, 返回 true 表示已计算 */
bool PID_Compute(PID_t *pid);

/* ==================== 配置 API ==================== */

void PID_SetOutputLimits(PID_t *pid, float min, float max);
void PID_SetTunings(PID_t *pid, float kp, float ki, float kd, PID_POn_t pOn);
void PID_SetTuningsSimple(PID_t *pid, float kp, float ki, float kd);
void PID_SetSampleTime(PID_t *pid, float newSampleTime_s);
void PID_SetDirection(PID_t *pid, PID_Dir_t dir);
void PID_SetMode(PID_t *pid, PID_Mode_t mode);

/* ==================== 兼容 API ==================== */

/** 重置内部状态 (outputSum + lastInput) */
void PID_Reset(PID_t *pid);

/* ==================== 查询 API ==================== */

static inline float      PID_GetKp(const PID_t *pid)        { return pid->dispKp; }
static inline float      PID_GetKi(const PID_t *pid)        { return pid->dispKi; }
static inline float      PID_GetKd(const PID_t *pid)        { return pid->dispKd; }
static inline PID_Mode_t PID_GetMode(const PID_t *pid)      { return pid->mode; }
static inline PID_Dir_t  PID_GetDirection(const PID_t *pid)  { return pid->direction; }

#endif /* PID_H */
