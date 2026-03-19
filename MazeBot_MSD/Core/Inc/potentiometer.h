/**
 * @file potentiometer.h
 * @brief ADC 电位器 API: 3通道 DMA + EMA 滤波 + 参数映射
 */
#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#include <stdint.h>

/* 映射后的参数值 */
typedef struct {
    float kp_heading;   /* POT1: 0.5 ~ 5.0 */
    float base_duty;    /* POT2: 20 ~ 80 */
    float turn_duty;    /* POT3: 15 ~ 50 */
} PotValues_t;

/* 全局可读的电位器参数 */
extern volatile PotValues_t g_pot_values;

/* 初始化 ADC1 + DMA, 启动循环转换, 用首次读数初始化 EMA */
void Pot_Init(void);

/* EMA 滤波 + 参数映射, 由 UITask 每 200ms 调用 */
void Pot_Update(void);

/* 获取原始 ADC 值 (12-bit, 0~4095), 用于 OLED 显示 */
uint16_t Pot_GetRaw(uint8_t ch);

#endif /* POTENTIOMETER_H */
