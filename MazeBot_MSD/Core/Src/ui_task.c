/**
 * @file ui_task.c
 * @brief OLED 显示刷新任务 (200ms) + Pot_Update()
 */
#include "ui_task.h"
#include "ssd1306.h"
#include "potentiometer.h"
#include "robot_state.h"
#include "encoder.h"
#include "im948.h"
#include "cmsis_os.h"
#include <stdio.h>

/* 状态名称字符串 */
static const char *state_names[] = {
    "IDLE", "EXPLORE", "NAVIG", "RETURN", "ESTOP!", "FAULT"
};

void UITask(void const *argument)
{
    (void)argument;

    bool oled_ok = SSD1306_Init();
    if (!oled_ok) {
        printf("[UI] SSD1306 init FAILED - I2C not responding\r\n");
    } else {
        printf("[UI] SSD1306 init OK\r\n");
    }
    char line[22]; /* 128/6 = 21 字符 + '\0' */

    for (;;) {
        /* 更新电位器 EMA 滤波 + 参数映射 */
        Pot_Update();

        SSD1306_Clear();

        /* 第0行: 状态 */
        RobotState_t st = RobotState_Get();
        if (st <= ROBOT_FAULT)
            snprintf(line, sizeof(line), "ST: %s", state_names[st]);
        else
            snprintf(line, sizeof(line), "ST: ???");
        SSD1306_WriteString(0, 0, line);

        /* 第1行: 位姿 x, y (cm) */
        snprintf(line, sizeof(line), "X:%5.1f Y:%5.1f",
                 odom_x * 100.0f, odom_y * 100.0f);
        SSD1306_WriteString(0, 2, line);

        /* 第2行: 航向 + 速度 */
        snprintf(line, sizeof(line), "H:%5.1f V:%4.0f",
                 AngleZ,
                 (Encoder_GetLeftSpeed(0.005f) + Encoder_GetRightSpeed(0.005f)) * 0.5f);
        SSD1306_WriteString(0, 4, line);

        /* 第3行: 电位器参数 */
        snprintf(line, sizeof(line), "Kp:%.1f B:%.0f T:%.0f",
                 g_pot_values.kp_heading,
                 g_pot_values.base_duty,
                 g_pot_values.turn_duty);
        SSD1306_WriteString(0, 6, line);

        SSD1306_Update();

        osDelay(200);
    }
}
