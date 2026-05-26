/**
 * @file ui_task.c
 * @brief OLED 显示刷新任务 (200ms) + Pot_Update()
 */
#include "ui_task.h"
#include "autonav.h"
#include "ssd1306.h"
#include "potentiometer.h"
#include "robot_state.h"
#include "encoder.h"
#include "im948.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/* 状态名称字符串 */
static const char *state_names[] = {
    "IDLE", "EXPLORE", "NAVIG", "RETURN", "ESTOP!", "FAULT"
};

/* 限幅到[min_v, max_v]，保证OLED格式化长度可控 */
static int clamp_i(int v, int min_v, int max_v)
{
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

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
        int x10 = (int)lroundf(odom_x * 1000.0f); /* cm with 1 decimal */
        int y10 = (int)lroundf(odom_y * 1000.0f); /* cm with 1 decimal */
        x10 = clamp_i(x10, -9999, 9999); /* -999.9 ~ 999.9 */
        y10 = clamp_i(y10, -9999, 9999); /* -999.9 ~ 999.9 */
        snprintf(line, sizeof(line), "X:%4d.%1d Y:%4d.%1d",
                 x10 / 10, abs(x10 % 10),
                 y10 / 10, abs(y10 % 10));
        SSD1306_WriteString(0, 2, line);

        /* 第2行: 航向 + 速度 */
        int h10 = (int)lroundf(AngleZ * 10.0f); /* deg with 1 decimal */
        int v0 = (int)lroundf(
            (Encoder_GetLeftSpeed(0.005f) + Encoder_GetRightSpeed(0.005f)) * 0.5f);
        h10 = clamp_i(h10, -1800, 1800); /* -180.0 ~ 180.0 */
        v0  = clamp_i(v0, 0, 9999);      /* 显示范围限制 */
        snprintf(line, sizeof(line), "H:%4d.%1d V:%4d",
                 h10 / 10, abs(h10 % 10), v0);
        SSD1306_WriteString(0, 4, line);

        /* 第3行: AutoNav状态优先, 空闲时显示电位器参数 */
        if (AutoNav_IsActive()) {
            AutoNavMetrics_t nav_m;
            AutoNav_GetMetrics(&nav_m);
            snprintf(line, sizeof(line), "NAV:%s C%d%d M%02d",
                     AutoNav_StateName(AutoNav_GetState()),
                     nav_m.cell_x, nav_m.cell_y,
                     clamp_i((int)lroundf(nav_m.match_score), 0, 99));
        } else {
            int kp10 = (int)lroundf(g_pot_values.kp_heading * 10.0f);
            int b0   = (int)lroundf(g_pot_values.base_duty);
            int t0   = (int)lroundf(g_pot_values.turn_duty);
            kp10 = clamp_i(kp10, 0, 99);   /* 0.0 ~ 9.9 */
            b0   = clamp_i(b0, 0, 99);
            t0   = clamp_i(t0, 0, 99);
            snprintf(line, sizeof(line), "Kp:%1d.%1d B:%02d T:%02d",
                     kp10 / 10, abs(kp10 % 10), b0, t0);
        }
        SSD1306_WriteString(0, 6, line);

        SSD1306_Update();

        osDelay(200);
    }
}
