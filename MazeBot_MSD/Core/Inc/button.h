/**
 * @file button.h
 * @brief 按钮 API: E-STOP (EXTI) + START/RETURN/MODE (轮询消抖)
 */
#ifndef BUTTON_H
#define BUTTON_H

#include <stdbool.h>
#include <stdint.h>

/* 引脚定义 */
#define BTN_ESTOP_Pin       GPIO_PIN_13
#define BTN_ESTOP_Port      GPIOC
#define BTN_START_Pin       GPIO_PIN_4
#define BTN_START_Port      GPIOB
#define BTN_RETURN_Pin      GPIO_PIN_7
#define BTN_RETURN_Port     GPIOB
#define BTN_MODE_Pin        GPIO_PIN_2
#define BTN_MODE_Port       GPIOB

typedef enum {
    BTN_START = 0,
    BTN_RETURN,
    BTN_MODE,
    BTN_COUNT
} ButtonId_t;

typedef enum {
    BTN_EVENT_NONE = 0,
    BTN_EVENT_PRESSED,
    BTN_EVENT_RELEASED
} ButtonEvent_t;

/* 初始化 GPIO + EXTI */
void Button_Init(void);

/* 20ms 轮询消抖, 返回按钮事件 */
ButtonEvent_t Button_Poll(ButtonId_t id);

/* 检查按钮当前是否按下 (消抖后) */
bool Button_IsPressed(ButtonId_t id);

/* ButtonTask 入口 (FreeRTOS) */
void ButtonTask(void const *argument);

#endif /* BUTTON_H */
