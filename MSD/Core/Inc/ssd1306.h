/**
 * @file ssd1306.h
 * @brief SSD1306 128x64 OLED I2C 驱动 (最小实现)
 */
#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include <stdbool.h>

#define SSD1306_WIDTH   128
#define SSD1306_HEIGHT  64
#define SSD1306_ADDR    0x78  /* 7-bit 0x3C << 1 */

/* 初始化 OLED (需先初始化 I2C1) */
bool SSD1306_Init(void);

/* 清空帧缓冲 */
void SSD1306_Clear(void);

/* 将帧缓冲刷新到 OLED */
void SSD1306_Update(void);

/* 在 (x, y) 绘制单个像素 */
void SSD1306_DrawPixel(uint8_t x, uint8_t y, bool on);

/* 在 (x, page*8) 写一个 6x8 字符 */
void SSD1306_WriteChar(uint8_t x, uint8_t page, char c);

/* 在 (x, page*8) 写字符串 */
void SSD1306_WriteString(uint8_t x, uint8_t page, const char *str);

/* 设置对比度 (0~255) */
void SSD1306_SetContrast(uint8_t val);

#endif /* SSD1306_H */
