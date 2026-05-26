/**
 * @file button.c
 * @brief GPIO 初始化、EXTI E-STOP、轮询消抖、ButtonTask
 */
#include "button.h"
#include "autonav.h"
#include "robot_state.h"
#include "main.h"
#include "cmsis_os.h"

/* ==================== 消抖状态 ==================== */
#define DEBOUNCE_COUNT  3  /* 3次连续一致 = 60ms @ 20ms轮询 */

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       count;
    bool          stable;   /* 消抖后的稳定状态 (true=按下) */
    bool          prev;     /* 上一次稳定状态 */
} BtnState_t;

static BtnState_t btns[BTN_COUNT];

/* ==================== 初始化 ==================== */
void Button_Init(void)
{
    /*
     * GPIO 已由 CubeMX 生成的 MX_GPIO_Init() 配置:
     *   PC13 = EXTI Falling + Pull-up
     *   PB2/PB4/PB7 = Input + Pull-up
     *
     * 这里只需要启用 EXTI NVIC 和初始化消抖状态
     */

    /*
     * E-STOP NVIC 优先级 2 — 高于 FreeRTOS 阈值(5)
     * ⚠️ ISR 内禁止调用任何 FreeRTOS API
     */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* 初始化消抖状态 */
    btns[BTN_START].port  = BTN_START_Port;
    btns[BTN_START].pin   = BTN_START_Pin;
    btns[BTN_RETURN].port = BTN_RETURN_Port;
    btns[BTN_RETURN].pin  = BTN_RETURN_Pin;
    btns[BTN_MODE].port   = BTN_MODE_Port;
    btns[BTN_MODE].pin    = BTN_MODE_Pin;

    for (int i = 0; i < BTN_COUNT; i++) {
        btns[i].count  = 0;
        btns[i].stable = false;
        btns[i].prev   = false;
    }
}

/* ==================== 轮询消抖 ==================== */
ButtonEvent_t Button_Poll(ButtonId_t id)
{
    if (id >= BTN_COUNT) return BTN_EVENT_NONE;

    BtnState_t *b = &btns[id];
    bool raw = (HAL_GPIO_ReadPin(b->port, b->pin) == GPIO_PIN_RESET); /* 低电平=按下 */

    if (raw == b->stable) {
        b->count = 0;
    } else {
        b->count++;
        if (b->count >= DEBOUNCE_COUNT) {
            b->stable = raw;
            b->count  = 0;
        }
    }

    ButtonEvent_t evt = BTN_EVENT_NONE;
    if (b->stable && !b->prev)       evt = BTN_EVENT_PRESSED;
    else if (!b->stable && b->prev)  evt = BTN_EVENT_RELEASED;
    b->prev = b->stable;

    return evt;
}

bool Button_IsPressed(ButtonId_t id)
{
    if (id >= BTN_COUNT) return false;
    return btns[id].stable;
}

/* ==================== ButtonTask ==================== */
void ButtonTask(void const *argument)
{
    (void)argument;
    Button_Init();

    for (;;) {
        ButtonEvent_t evt_start  = Button_Poll(BTN_START);
        ButtonEvent_t evt_return = Button_Poll(BTN_RETURN);
        ButtonEvent_t evt_mode   = Button_Poll(BTN_MODE);

        /* E-STOP 复位: START + MODE 同时按下 */
        if (g_estop_latched) {
            if (Button_IsPressed(BTN_START) && Button_IsPressed(BTN_MODE)) {
                RobotState_TryResetEstop();
            }
            osDelay(20);
            continue;
        }

        /* 正常按钮事件处理 */
        if (evt_start == BTN_EVENT_PRESSED) {
            AutoNav_StartToExit();
        }
        if (evt_return == BTN_EVENT_PRESSED) {
            AutoNav_ReturnToStart();
        }
        if (evt_mode == BTN_EVENT_PRESSED) {
            /* 模式切换: EXPLORING <-> NAVIGATING */
            RobotState_t cur = RobotState_Get();
            if (cur == ROBOT_EXPLORING)
                RobotState_Set(ROBOT_NAVIGATING);
            else if (cur == ROBOT_NAVIGATING)
                RobotState_Set(ROBOT_EXPLORING);
        }

        osDelay(20);
    }
}
