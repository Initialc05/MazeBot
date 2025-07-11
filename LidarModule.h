#ifndef LIDAR_MODULE_H
#define LIDAR_MODULE_H

static uint8_t buf[6];  // 内部缓冲区: 0 sync+quality，1 quality+flags，2 angle LSB，3 angle MSB，4 dist LSB，5 dist MSB
static uint8_t idx = 0;

// 雷达初始化
void initLidar() {
  Serial.begin(115200);
  LidarSerial.begin(460800);

  Serial.println(F("发送雷达启动命令..."));
  const uint8_t startCmd[] = { 0xA5, 0x20 };  
  LidarSerial.write(startCmd, sizeof(startCmd));
  LidarSerial.flush();

  delay(3000);  // 等雷达自检
  Serial.println(F("雷达启动完毕"));
}

// 数据读取
inline void readAndSendLidar() {
  while (LidarSerial.available()) {
    uint8_t b = LidarSerial.read();

    // Step-1：同步字节（bit7 与 bit0 同为 1）
    if (idx == 0) {
      if ((b & 0x81) == 0x81) {
        buf[idx++] = b;
      }
      continue;
    }

    // Step-2：收满 6 字节
    buf[idx++] = b;
    if (idx < 6) continue;
    idx = 0;  // 为下一帧做准备

    // Q 字节 bit7 必须为 1
    if ((buf[1] & 0x80) == 0) continue;

    uint8_t quality = buf[1] >> 2;                          // 0-63
    uint16_t angle_raw = ((uint16_t)buf[3] << 8) | buf[2];  // little-endian
    float angle = (angle_raw >> 1) / 64.0f;                 // deg
    uint16_t dist_raw = ((uint16_t)buf[5] << 8) | buf[4];
    float distance = dist_raw / 4.0f;  // mm

    // 串口输出
    Serial.print(F("LIDAR:"));
    Serial.print(angle, 1);  // 一位小数
    Serial.print(',');
    Serial.print(distance, 2);  // 两位小数
    Serial.print(F(",Q:"));
    Serial.println(quality);

    // // 蓝牙输出 
    // BTSerial.print(F("LIDAR:"));
    // BTSerial.print(angle, 1);  // 一位小数
    // BTSerial.print(',');
    // BTSerial.print(distance, 2);  // 两位小数
    // BTSerial.print(F(",Q:"));
    // BTSerial.println(quality);
  }
}

#endif
