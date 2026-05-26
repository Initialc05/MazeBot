/**
 * @file encoder.c
 * @brief 硬件定时器编码器实现
 */
#include "encoder.h"
#include <math.h>

/* ==================== 外部HAL句柄 ==================== */
extern TIM_HandleTypeDef htim1;  /* 右编码器 */
extern TIM_HandleTypeDef htim3;  /* 左编码器 */

/* ==================== 全局状态 ==================== */
volatile int32_t encoder_left_ticks  = 0;
volatile int32_t encoder_right_ticks = 0;
volatile int32_t encoder_left_delta  = 0;
volatile int32_t encoder_right_delta = 0;
volatile int32_t encoder_left_odom_pending  = 0;
volatile int32_t encoder_right_odom_pending = 0;

float odom_x     = 0.0f;
float odom_y     = 0.0f;
float odom_theta = 0.0f;

/* 上次计数器原始值 */
static uint16_t last_cnt_left  = 0;
static uint16_t last_cnt_right = 0;

/* ==================== 接口实现 ==================== */
void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  /* 左 */
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);  /* 右 */
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    last_cnt_left  = 0;
    last_cnt_right = 0;
}

void Encoder_Update(void)
{
    uint16_t cnt_l = __HAL_TIM_GET_COUNTER(&htim3);
    uint16_t cnt_r = __HAL_TIM_GET_COUNTER(&htim1);

    /* 16位有符号差值，自动处理上溢/下溢回绕 */
    encoder_left_delta  = (int16_t)(cnt_l - last_cnt_left);
    encoder_right_delta = (int16_t)(cnt_r - last_cnt_right);

    encoder_left_ticks  += encoder_left_delta;
    encoder_right_ticks += encoder_right_delta;
    encoder_left_odom_pending  += encoder_left_delta;
    encoder_right_odom_pending += encoder_right_delta;

    last_cnt_left  = cnt_l;
    last_cnt_right = cnt_r;
}

float Encoder_GetLeftSpeed(float dt_s)
{
    if (dt_s <= 0.0f) return 0.0f;
    return (float)encoder_left_delta / dt_s;
}

float Encoder_GetRightSpeed(float dt_s)
{
    if (dt_s <= 0.0f) return 0.0f;
    return (float)encoder_right_delta / dt_s;
}

void Encoder_UpdateOdometry(float heading_rad)
{
    int32_t left_ticks;
    int32_t right_ticks;

    __disable_irq();
    left_ticks = encoder_left_odom_pending;
    right_ticks = encoder_right_odom_pending;
    encoder_left_odom_pending = 0;
    encoder_right_odom_pending = 0;
    __enable_irq();

    float dl = (float)left_ticks  * METERS_PER_TICK * ENCODER_SCALE_FACTOR;
    float dr = (float)right_ticks * METERS_PER_TICK * ENCODER_SCALE_FACTOR;
    float dc = (dl + dr) * 0.5f;

    odom_x += dc * cosf(heading_rad);
    odom_y += dc * sinf(heading_rad);
    odom_theta = heading_rad;
}

void Encoder_ResetOdometry(void)
{
    __disable_irq();
    encoder_left_odom_pending = 0;
    encoder_right_odom_pending = 0;
    __enable_irq();

    odom_x = 0.0f;
    odom_y = 0.0f;
    odom_theta = 0.0f;
}

void Encoder_ResetTicks(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    last_cnt_left  = 0;
    last_cnt_right = 0;
    encoder_left_ticks  = 0;
    encoder_right_ticks = 0;
    encoder_left_delta  = 0;
    encoder_right_delta = 0;
    encoder_left_odom_pending  = 0;
    encoder_right_odom_pending = 0;
}
