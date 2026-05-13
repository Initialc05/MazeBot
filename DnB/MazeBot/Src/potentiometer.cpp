#include "../Inc/potentiometer.h"

namespace {
constexpr uint8_t POT_KP_PIN = PA4;
constexpr uint8_t POT_BASE_PIN = PA5;
constexpr uint8_t POT_TURN_PIN = PA6;
constexpr float EMA_ALPHA = 0.1f;

PotValues g_pot_values = {2.0f, 40, 30};
float g_kp_ema = 0.0f;
float g_base_ema = 0.0f;
float g_turn_ema = 0.0f;

float mapFloat(float value, float inMin, float inMax, float outMin, float outMax) {
  float ratio = (value - inMin) / (inMax - inMin);
  return outMin + ratio * (outMax - outMin);
}

int readAveragedAnalog(uint8_t pin) {
  long sum = 0;
  for (int i = 0; i < 8; ++i) {
    sum += analogRead(pin);
  }
  return static_cast<int>(sum / 8);
}

void updatePotValuesFromRaw() {
  PotValues values;
  values.kp_heading = mapFloat(g_kp_ema, 0.0f, 4095.0f, 0.5f, 5.0f);
  values.base_duty = constrain(static_cast<int>(lroundf(mapFloat(g_base_ema, 0.0f, 4095.0f, 20.0f, 80.0f))), 20, 80);
  values.turn_duty = constrain(static_cast<int>(lroundf(mapFloat(g_turn_ema, 0.0f, 4095.0f, 15.0f, 50.0f))), 15, 50);
  g_pot_values = values;
}
}

void Pot_Init() {
  analogReadResolution(12);

  g_kp_ema = static_cast<float>(readAveragedAnalog(POT_KP_PIN));
  g_base_ema = static_cast<float>(readAveragedAnalog(POT_BASE_PIN));
  g_turn_ema = static_cast<float>(readAveragedAnalog(POT_TURN_PIN));

  updatePotValuesFromRaw();
}

void Pot_Update() {
  g_kp_ema = EMA_ALPHA * static_cast<float>(analogRead(POT_KP_PIN)) + (1.0f - EMA_ALPHA) * g_kp_ema;
  g_base_ema = EMA_ALPHA * static_cast<float>(analogRead(POT_BASE_PIN)) + (1.0f - EMA_ALPHA) * g_base_ema;
  g_turn_ema = EMA_ALPHA * static_cast<float>(analogRead(POT_TURN_PIN)) + (1.0f - EMA_ALPHA) * g_turn_ema;

  updatePotValuesFromRaw();
}

PotValues Pot_GetValues() {
  noInterrupts();
  PotValues values = g_pot_values;
  interrupts();
  return values;
}
