/**
 * @file bt_cmd.h
 * @brief 蓝牙命令解析 + 运动状态机 + 级联PID控制
 *
 * 命令协议:
 *   单字符: w/W(前进) s/S(后退) a/A(左转) d/D(右转) x/X(停止)
 *            小写=200ms超时, 大写=持续
 *   多字符: F<cm>\n(前进) B<cm>\n(后退) L<deg>\n(左转) R<deg>\n(右转)
 *
 * 级联PID架构:
 *   直行: 外环Heading(Kp=2.0,Ki=0,Kd=0.1) → 内环VelDiff(Kp=3.0,Ki=0,Kd=0.1)
 *   转弯: Turn PID(Kp=0.4,Ki=0,Kd=10.0)
 */
#ifndef BT_CMD_H
#define BT_CMD_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== 运动状态机 ==================== */
typedef enum { TURN_IDLE, TURN_ROTATING, TURN_REACHED } TurnState;
typedef enum { MOVE_IDLE, MOVE_RUNNING, MOVE_REACHED } MoveState;

extern TurnState turnState;
extern MoveState moveState;

/* ==================== 命令状态 ==================== */
extern bool     cmd_active;
extern bool     cmd_continuous;
extern char     cmd_current;
extern uint32_t cmd_last_time;
extern float    targetYaw;

/* ==================== 接口函数 ==================== */

/** 初始化PID控制器和状态机 */
void BtCmd_Init(void);

/** 处理蓝牙接收的单字节命令 (在CommandTask中逐字节调用) */
void BtCmd_ProcessByte(char cmd);

/**
 * 运动控制更新 (在MotorControlTask中5ms周期调用)
 * @param dt_s 采样周期(秒)
 */
void BtCmd_UpdateMotorControl(float dt_s);

/** 执行精准直线运动 */
void BtCmd_ExecutePreciseMove(float distance_cm, int direction);

/** 执行精准转弯 */
void BtCmd_ExecutePreciseTurn(float angle_deg);

#endif /* BT_CMD_H */
