/**
 * @file motor.h
 * @brief 电机控制模块 - PWM + 方向GPIO
 *
 * 左电机: PC8(方向) + TIM2_CH3/PB10(PWM)
 * 右电机: PC7(方向) + TIM2_CH2/PB3(PWM)
 *
 * PWM范围 0-255 对应 analogWrite 兼容
 * duty范围 0-100(%) 对应业务层
 */
#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== 电机参数 ==================== */
#define MOTOR_DUTY_MAX      100   /* 最大占空比 % */
#define MOTOR_PWM_MAX       255   /* PWM计数器最大值 (Period) */
#define MOTOR_BASE_DUTY      40   /* 直行基础占空比 % */
#define MOTOR_TURN_DUTY      25   /* 原地转弯占空比 % */
#define MOTOR_CMD_TIMEOUT   200   /* 单次命令超时 ms */

/* ==================== 电机状态 ==================== */
extern int      motor_left_duty;     /* 0-100 */
extern int      motor_right_duty;    /* 0-100 */
extern int      motor_left_dir;      /* 1=前进, -1=后退 */
extern int      motor_right_dir;     /* 1=前进, -1=后退 */
extern bool     motor_cmd_active;    /* 命令是否激活 */
extern bool     motor_continuous;    /* 持续命令标志 */
extern char     motor_current_cmd;   /* 当前命令字符 */
extern uint32_t motor_last_cmd_time; /* 上次命令时间 */

/* ==================== 接口函数 ==================== */

/** 初始化电机: 启动PWM输出, 占空比归零 */
void Motor_Init(void);

/** 设置左电机: dir(1/-1), duty(0-100) */
void Motor_SetLeft(int dir, int duty);

/** 设置右电机: dir(1/-1), duty(0-100) */
void Motor_SetRight(int dir, int duty);

/** 双电机同时设置 */
void Motor_Set(int l_dir, int l_duty, int r_dir, int r_duty);

/** 紧急刹车: 两轮PWM归零 */
void Motor_Brake(void);

/** 前进: 两轮同速正转 */
void Motor_Forward(int duty);

/** 后退: 两轮同速反转 */
void Motor_Backward(int duty);

/** 左转: 左轮反转右轮正转 */
void Motor_TurnLeft(int duty);

/** 右转: 左轮正转右轮反转 */
void Motor_TurnRight(int duty);

/** 命令超时检查 (在MotorControlTask中周期调用) */
void Motor_CheckTimeout(void);

#endif /* MOTOR_H */
