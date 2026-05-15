/**
 * @file potentiometer.c
 * @brief ADC1 DMA 循环读取, EMA 滤波, 参数映射
 *        ADC1 + DMA 由 CubeMX 生成的 MX_ADC1_Init() 初始化
 *        本模块只负责启动 DMA、滤波和映射
 */
#include "potentiometer.h"
#include "main.h"
#include <stdbool.h>

/* ==================== 配置 ==================== */
#define POT_NUM_CH      3
#define EMA_ALPHA       0.1f

/* 映射范围 */
#define POT1_MIN  0.5f
#define POT1_MAX  5.0f
#define POT2_MIN  20.0f
#define POT2_MAX  80.0f
#define POT3_MIN  15.0f
#define POT3_MAX  50.0f

/* ==================== 变量 ==================== */
volatile PotValues_t g_pot_values = { 2.0f, 40.0f, 25.0f };

/* 使用 CubeMX 生成的全局 hadc1 */
extern ADC_HandleTypeDef hadc1;

static volatile uint16_t adc_dma_buf[POT_NUM_CH];
static float ema[POT_NUM_CH];
static bool ema_initialized = false;

/* ==================== 内部: 线性映射 ==================== */
static float map_f(float val, float in_min, float in_max, float out_min, float out_max)
{
    if (val < in_min) val = in_min;
    if (val > in_max) val = in_max;
    return out_min + (val - in_min) * (out_max - out_min) / (in_max - in_min);
}

/* ==================== 初始化 ==================== */
void Pot_Init(void)
{
    /* ADC1 已由 MX_ADC1_Init() 初始化, 这里只启动 DMA */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, POT_NUM_CH);

    /* 等待首次转换完成, 用首次读数初始化 EMA */
    HAL_Delay(5);
    for (int i = 0; i < POT_NUM_CH; i++) {
        ema[i] = (float)adc_dma_buf[i];
    }
    ema_initialized = true;
}

/* ==================== EMA 滤波 + 映射 ==================== */
void Pot_Update(void)
{
    if (!ema_initialized) return;

    for (int i = 0; i < POT_NUM_CH; i++) {
        float raw = (float)adc_dma_buf[i];
        ema[i] = EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * ema[i];
    }

    g_pot_values.kp_heading = map_f(ema[0], 0.0f, 4095.0f, POT1_MIN, POT1_MAX);
    g_pot_values.base_duty  = map_f(ema[1], 0.0f, 4095.0f, POT2_MIN, POT2_MAX);
    g_pot_values.turn_duty  = map_f(ema[2], 0.0f, 4095.0f, POT3_MIN, POT3_MAX);
}

uint16_t Pot_GetRaw(uint8_t ch)
{
    if (ch >= POT_NUM_CH) return 0;
    return adc_dma_buf[ch];
}
