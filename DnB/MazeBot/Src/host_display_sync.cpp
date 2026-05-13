#include "../Inc/host_display_sync.h"

#include <stdlib.h>
#include <string.h>

namespace {
HostDisplayData g_hostDisplay = {0.0f, 0.0f, 0.0f, false, 0};
String g_displayBuffer = "";
bool g_receivingDisplayPacket = false;

bool parseDisplayPacket(const String &packet) {
  if (!packet.startsWith("@D,")) {
    return false;
  }

  int firstComma = packet.indexOf(',');
  int secondComma = packet.indexOf(',', firstComma + 1);
  int thirdComma = packet.indexOf(',', secondComma + 1);

  if (firstComma < 0 || secondComma < 0 || thirdComma < 0) {
    return false;
  }

  String xText = packet.substring(firstComma + 1, secondComma);
  String yText = packet.substring(secondComma + 1, thirdComma);
  String hText = packet.substring(thirdComma + 1);

  g_hostDisplay.x = xText.toFloat();
  g_hostDisplay.y = yText.toFloat();
  g_hostDisplay.heading = hText.toFloat();
  g_hostDisplay.valid = true;
  g_hostDisplay.updatedAtMs = millis();
  return true;
}
}

void HostDisplay_Init() {
  g_hostDisplay = {0.0f, 0.0f, 0.0f, false, 0};
  g_displayBuffer = "";
  g_receivingDisplayPacket = false;
}

bool HostDisplay_ProcessByte(char byte) {
  if (!g_receivingDisplayPacket) {
    if (byte != '@') {
      return false;
    }
    g_receivingDisplayPacket = true;
    g_displayBuffer = "@";
    return true;
  }

  if (byte == '\r') {
    return true;
  }

  if (byte == '\n') {
    String packet = g_displayBuffer;
    g_displayBuffer = "";
    g_receivingDisplayPacket = false;
    parseDisplayPacket(packet);
    return true;
  }

  if (g_displayBuffer.length() >= 63) {
    g_displayBuffer = "";
    g_receivingDisplayPacket = false;
    return true;
  }

  g_displayBuffer += byte;
  return true;
}

HostDisplayData HostDisplay_Get() {
  noInterrupts();
  HostDisplayData data = g_hostDisplay;
  interrupts();
  return data;
}
