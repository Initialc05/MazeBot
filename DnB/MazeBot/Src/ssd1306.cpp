#include "../Inc/ssd1306.h"

#include <Wire.h>
#include <string.h>

#include "../Inc/font6x8.h"

namespace {
constexpr uint8_t SSD1306_ADDR = 0x3C;
constexpr uint8_t SSD1306_WIDTH = 128;
constexpr uint8_t SSD1306_HEIGHT = 64;
constexpr uint16_t BUFFER_SIZE = SSD1306_WIDTH * SSD1306_HEIGHT / 8;

uint8_t g_framebuffer[BUFFER_SIZE];
bool g_initialized = false;

void sendCommand(uint8_t command) {
  Wire.beginTransmission(SSD1306_ADDR);
  Wire.write(0x00);
  Wire.write(command);
  Wire.endTransmission();
}

void setAddressWindow() {
  sendCommand(0x21);
  sendCommand(0x00);
  sendCommand(SSD1306_WIDTH - 1);
  sendCommand(0x22);
  sendCommand(0x00);
  sendCommand((SSD1306_HEIGHT / 8) - 1);
}
}

bool SSD1306_Init() {
  Wire.setSCL(PB8);
  Wire.setSDA(PB9);
  Wire.begin();
  Wire.setClock(400000);

  Wire.beginTransmission(SSD1306_ADDR);
  if (Wire.endTransmission() != 0) {
    g_initialized = false;
    return false;
  }

  sendCommand(0xAE);
  sendCommand(0xD5); sendCommand(0x80);
  sendCommand(0xA8); sendCommand(0x3F);
  sendCommand(0xD3); sendCommand(0x00);
  sendCommand(0x40);
  sendCommand(0x8D); sendCommand(0x14);
  sendCommand(0x20); sendCommand(0x00);
  sendCommand(0xA1);
  sendCommand(0xC8);
  sendCommand(0xDA); sendCommand(0x12);
  sendCommand(0x81); sendCommand(0xCF);
  sendCommand(0xD9); sendCommand(0xF1);
  sendCommand(0xDB); sendCommand(0x40);
  sendCommand(0xA4);
  sendCommand(0xA6);
  sendCommand(0xAF);

  g_initialized = true;
  SSD1306_Clear();
  SSD1306_Update();
  return true;
}

void SSD1306_Clear() {
  memset(g_framebuffer, 0, sizeof(g_framebuffer));
}

void SSD1306_Update() {
  if (!g_initialized) {
    return;
  }

  setAddressWindow();
  for (uint8_t page = 0; page < 8; ++page) {
    Wire.beginTransmission(SSD1306_ADDR);
    Wire.write(0x40);
    for (uint8_t col = 0; col < 16; ++col) {
      Wire.write(&g_framebuffer[page * SSD1306_WIDTH + col * 8], 8);
    }
    Wire.endTransmission();
  }
}

void SSD1306_DrawPixel(int x, int y, bool on) {
  if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) {
    return;
  }

  uint16_t index = static_cast<uint16_t>(x + (y / 8) * SSD1306_WIDTH);
  uint8_t mask = static_cast<uint8_t>(1U << (y % 8));

  if (on) {
    g_framebuffer[index] |= mask;
  } else {
    g_framebuffer[index] &= static_cast<uint8_t>(~mask);
  }
}

void SSD1306_WriteChar(int x, int y, char c) {
  if (c < 32 || c > 126) {
    c = '?';
  }

  const uint8_t *glyph = FONT6X8[c - 32];
  for (int col = 0; col < 6; ++col) {
    uint8_t column = glyph[col];
    for (int row = 0; row < 8; ++row) {
      SSD1306_DrawPixel(x + col, y + row, (column >> row) & 0x01);
    }
  }
}

void SSD1306_WriteString(int x, int y, const char *s) {
  int cursorX = x;
  while (*s != '\0') {
    SSD1306_WriteChar(cursorX, y, *s++);
    cursorX += 6;
    if (cursorX > (SSD1306_WIDTH - 6)) {
      break;
    }
  }
}

void SSD1306_SetContrast(uint8_t value) {
  if (!g_initialized) {
    return;
  }
  sendCommand(0x81);
  sendCommand(value);
}
