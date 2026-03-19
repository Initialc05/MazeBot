/**
 * @file ui_task.h
 * @brief UITask 原型: OLED 显示刷新 + 电位器更新
 */
#ifndef UI_TASK_H
#define UI_TASK_H

/* UITask 入口 (FreeRTOS, 200ms 周期) */
void UITask(void const *argument);

#endif /* UI_TASK_H */
