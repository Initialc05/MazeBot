#ifndef SSD1306_H
#define SSD1306_H

#include <Arduino.h>

bool SSD1306_Init();
void SSD1306_Clear();
void SSD1306_Update();
void SSD1306_DrawPixel(int x, int y, bool on);
void SSD1306_WriteChar(int x, int y, char c);
void SSD1306_WriteString(int x, int y, const char *s);
void SSD1306_SetContrast(uint8_t value);

#endif
