#ifndef HOST_DISPLAY_SYNC_H
#define HOST_DISPLAY_SYNC_H

#include <Arduino.h>

struct HostDisplayData {
  float x;
  float y;
  float heading;
  bool valid;
  uint32_t updatedAtMs;
};

void HostDisplay_Init();
bool HostDisplay_ProcessByte(char byte);
HostDisplayData HostDisplay_Get();

#endif
