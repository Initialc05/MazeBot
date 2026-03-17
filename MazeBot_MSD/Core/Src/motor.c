/**
 * @file motor.c
 * @brief 电机控制模块实现
 */
#include "motor.h"

/* ==================== 外部HAL句柄 ==================== */
extern TIM_HandleTypeDef htim2;

/* ==================== 全局状态 ==================== */
int      motor_left_duty     = 0;
int      motor_right_duty    = 0;
int      motor_left_dir      = 1;
int      motor_right_dir     = 1;
bool     motor_cmd_active    = false;
bool     motor_continuous    = false;
char     motor_current_cmd   = 'x';
uint32_t motor_last_cmd_time = 0;

/* ==================== 内部工具 ==================== */
static inline int clamp(int val, int lo, int hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/* duty(0-100) → PWM compare值(0-255) */
static inline uint16_t duty_to_pwm(int duty)
{
    return (uint16_t)(clamp(duty, 0, MOTOR_DUTY_MAX) * MOTOR_PWM_MAX / MOTOR_DUTY_MAX);
}

/* ==================== 接口实现 ==================== */
void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  /* 右电机 PB3 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);  /* 左电机 PB10 */
    Motor_Brake();
}

void Motor_SetLeft(int dir, int duty)
{
    motor_left_dir  = dir;
    motor_left_duty = clamp(duty, 0, MOTOR_DUTY_MAX);
    /* 方向: PC8 */
    HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin,
                      (dir >= 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty_to_pwm(motor_left_duty));
}

void Motor_SetRight(int dir, int duty)
{
    motor_right_dir  = dir;
    motor_right_duty = clamp(duty, 0, MOTOR_DUTY_MAX);
    /* 方向: PC7 */
    HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin,
                      (dir >= 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty_to_pwm(motor_right_duty));
}

void Motor_Set(int l_dir, int l_duty, int r_dir, int r_duty)
{
    Motor_SetLeft(l_dir, l_duty);
    Motor_SetRight(r_dir, r_duty);
}

void Motor_Brake(void)
{
    Motor_Set(1, 0, 1, 0);
    motor_cmd_active  = false;
    motor_current_cmd = 'x';
}

void Motor_Forward(int duty)
{
    Motor_Set(1, duty, 1, duty);
}

void Motor_Backward(int duty)
{
    Motor_Set(-1, duty, -1, duty);
}

void Motor_TurnLeft(int duty)
{
    Motor_Set(-1, duty, 1, duty);
}

void Motor_TurnRight(int duty)
{
    Motor_Set(1, duty, -1, duty);
}

void Motor_CheckTimeout(void)
{
    if (!motor_cmd_active || motor_continuous) return;
    if (HAL_GetTick() - motor_last_cmd_time > MOTOR_CMD_TIMEOUT) {
        Motor_Brake();
    }
}
