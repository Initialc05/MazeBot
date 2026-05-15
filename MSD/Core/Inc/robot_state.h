/**
 * @file robot_state.h
 * @brief 机器人状态枚举 + E-STOP 标志
 */
#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ROBOT_IDLE = 0,
    ROBOT_EXPLORING,
    ROBOT_NAVIGATING,
    ROBOT_RETURNING,
    ROBOT_ESTOP,
    ROBOT_FAULT
} RobotState_t;

/* E-STOP 锁存标志 (ISR 中置位, 仅 START+MODE 同时按下可复位) */
extern volatile bool g_estop_latched;

/* 当前机器人状态 */
extern volatile RobotState_t g_robot_state;

void RobotState_Init(void);
void RobotState_Set(RobotState_t state);
RobotState_t RobotState_Get(void);

/* E-STOP 锁存 (由 EXTI ISR 调用) */
void RobotState_LatchEstop(void);

/* E-STOP 复位 (需 START+MODE 同时按下) */
bool RobotState_TryResetEstop(void);

#endif /* ROBOT_STATE_H */
