/**
 * @file robot_state.c
 * @brief 状态管理、E-STOP 锁存/复位
 */
#include "robot_state.h"
#include "main.h"

volatile bool g_estop_latched = false;
volatile RobotState_t g_robot_state = ROBOT_IDLE;

void RobotState_Init(void)
{
    g_estop_latched = false;
    g_robot_state = ROBOT_IDLE;
}

void RobotState_Set(RobotState_t state)
{
    if (!g_estop_latched) {
        g_robot_state = state;
    }
}

RobotState_t RobotState_Get(void)
{
    return g_robot_state;
}

void RobotState_LatchEstop(void)
{
    g_estop_latched = true;
    g_robot_state = ROBOT_ESTOP;
    /* PWM 立即归零 — 直接操作寄存器, 不依赖 FreeRTOS */
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
}

bool RobotState_TryResetEstop(void)
{
    if (!g_estop_latched) return false;
    g_estop_latched = false;
    g_robot_state = ROBOT_IDLE;
    return true;
}
