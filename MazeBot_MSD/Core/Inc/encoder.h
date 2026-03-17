/**
 * @file encoder.h
 * @brief 硬件定时器编码器模块
 *
 * 左编码器: TIM3 (PC6/PB5)
 * 右编码器: TIM1 (PA8/PA9)
 *
 * 替代Arduino GPIO中断方案，使用硬件编码器模式，零CPU开销
 * 390 ticks/rev (13线 × 30减速比), TI12模式下 ×4 = 1560 counts/rev
 */
#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include <stdint.h>

/* ==================== 编码器物理参数 ==================== */
#define ENCODER_TICKS_PER_REV  1560    /* TI12四倍频: 13×30×4 */
#define WHEEL_DIAMETER_M       0.065f  /* 轮径 65mm */
#define WHEEL_BASE_M           0.100f  /* 轮距 100mm */
#define WHEEL_CIRCUMFERENCE    (WHEEL_DIAMETER_M * 3.14159265f)
#define METERS_PER_TICK        (WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REV)
#define ENCODER_SCALE_FACTOR   0.534f  /* 里程计缩放系数 */

/* ==================== 编码器状态 ==================== */
extern volatile int32_t encoder_left_ticks;   /* 累计tick (有符号,可溢出回绕) */
extern volatile int32_t encoder_right_ticks;
extern volatile int32_t encoder_left_delta;   /* 上次采样间隔的增量 */
extern volatile int32_t encoder_right_delta;

/* 里程计 */
extern float odom_x;      /* 位移X (米) */
extern float odom_y;      /* 位移Y (米) */
extern float odom_theta;  /* 朝向 (弧度) */

/* ==================== 接口函数 ==================== */

/** 启动编码器计数 */
void Encoder_Init(void);

/**
 * @brief 采样编码器增量并更新累计值
 *        在MotorControlTask中周期调用 (5ms)
 */
void Encoder_Update(void);

/** 获取左轮速度 (ticks/s), 基于delta和采样周期 */
float Encoder_GetLeftSpeed(float dt_s);

/** 获取右轮速度 (ticks/s) */
float Encoder_GetRightSpeed(float dt_s);

/**
 * @brief 更新里程计 (使用IMU航向角)
 * @param heading_rad 当前航向角(弧度), 来自IMU
 */
void Encoder_UpdateOdometry(float heading_rad);

/** 里程计清零 */
void Encoder_ResetOdometry(void);

/** 编码器计数清零 */
void Encoder_ResetTicks(void);

#endif /* ENCODER_H */
