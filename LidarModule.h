#ifndef LIDAR_MODULE_H
#define LIDAR_MODULE_H

#include "Arduino.h"

//------------ 用户硬件映射 ------------//
// 你的雷达串口：TX->PD2 ，RX->PC12
HardwareSerial Serial5(PD2, PC12);

//------------ 内部缓冲区 ------------//
// RPLidar legacy 测量节点：6 byte =
// 0 sync+quality，1 quality+flags，2 angle LSB，3 angle MSB，4 dist LSB，5 dist MSB
static uint8_t buf[6];
static uint8_t idx = 0;

//------------ 初始化 ------------//
void initLidar() {
  Serial.begin(115200);
  Serial5.begin(460800);

  Serial.println(F("发送雷达启动命令..."));
  const uint8_t startCmd[] = { 0xA5, 0x20 };   // Express/Normal scan
  Serial5.write(startCmd, sizeof(startCmd));
  Serial5.flush();

  delay(3000);                                 // 等雷达自检
  Serial.println(F("雷达启动完毕"));
}

//------------ 数据读取 ------------//
inline void readAndSendLidar() {
  while (Serial5.available()) {
    uint8_t b = Serial5.read();

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
    idx = 0;                      // 为下一帧做准备

    // ---------- 解析 ----------
    // Q 字节 bit7 必须为 1
    if ((buf[1] & 0x80) == 0) continue;

    uint8_t  quality   = buf[1] >> 2;                        // 0-63
    uint16_t angle_raw = ((uint16_t)buf[3] << 8) | buf[2];   // little-endian
    float    angle     = (angle_raw >> 1) / 64.0f;           // deg
    uint16_t dist_raw  = ((uint16_t)buf[5] << 8) | buf[4];
    float    distance  = dist_raw / 4.0f;                    // mm

    // ---------- 串口输出 ----------
    Serial.print(F("LIDAR:"));
    Serial.print(angle, 1);          // 一位小数即可
    Serial.print(',');
    Serial.print(distance, 2);       // 两位小数
    Serial.print(F(",Q:"));
    Serial.println(quality);
    // ---------- 蓝牙输出 ----------
    BTSerial.print(F("LIDAR:"));
    BTSerial.print(angle, 1);          // 一位小数即可
    BTSerial.print(',');
    BTSerial.print(distance, 2);       // 两位小数
    BTSerial.print(F(",Q:"));
    BTSerial.println(quality);
  }
}

#endif  // LIDAR_MODULE_H


// #ifndef LIDAR_MODULE_H
// #define LIDAR_MODULE_H

// #include "Arduino.h"
// #include "IMUFilterModule.h"

// // 使用 PD2/PC12 连接雷达串口
// HardwareSerial Serial5(PD2, PC12);

// char lidarBuf[80];
// static uint8_t buf[5];  // 缓冲区，5字节
// static int idx = 0;     // 缓冲区索引

// void initLidar() {
//   Serial.begin(115200);  // 你原代码里也有
//   Serial5.begin(460800);

//   Serial.println("初始化 RPLidar...");
//   const uint8_t startCmd[] = { 0xA5, 0x20 };
//   Serial5.write(startCmd, sizeof(startCmd));
//   Serial5.flush();

//   delay(3000);
//   Serial.println("雷达启动命令已发送");
// }

// void readAndSendLidar() {
//   while (Serial5.available()) {
//     uint8_t b = Serial5.read();
//     if (idx == 0) {
//       // 查找同步字节（bit7=1, bit0=1）
//       if ((b & 0x81) == 0x81) {
//         buf[idx++] = b;
//       }
//     } else {
//       buf[idx++] = b;
//       if (idx == 5) { // 5字节包
//         // 校验Q字节 bit7=1
//         if ((buf[1] & 0x80) == 0x80) {
//           uint8_t quality = buf[1] >> 2;
//           uint16_t angle_raw = ((buf[3] << 8) | buf[2]) >> 1;
//           float angle = angle_raw / 64.0f;  // 度
//           uint16_t dist_raw = (buf[4] << 8) | buf[5]; // 距离拼接
//           float distance = dist_raw / 4.0f / 1000.0f;  // m

//           if ((int)angle % 30 == 0 && distance > 0.05f && quality > 0) {
//             snprintf(lidarBuf, sizeof(lidarBuf), "LIDAR:%.1f,%.2f,Q:%d\n",
//                      angle, distance, quality);
//             BTSerial.print(lidarBuf);  // 蓝牙输出
//             Serial.print(lidarBuf);    // 串口调试输出
//           }
//         }
//         idx = 0;  // 重置
//       }
//     }
//   }
// }

// #endif


// #ifndef LIDAR_MODULE_H
// #define LIDAR_MODULE_H

// #include "rplidar_driver_impl.h"

// RPLidar rplidar;
// char lidarBuf[80];

// void initLidar() {
//   Serial.println("初始化 RPLidar...");
//   rplidar.begin(); 
//   delay(100);

//   rplidar_response_device_info_t info;
//   if (rplidar.getDeviceInfo(info)) {
//     Serial.printf("RPLidar 型号:%d  固件:%d.%d\n", info.model, info.firmware_version >> 8, info.firmware_version & 0xFF);
//   } else {
//     Serial.println("无法获取雷达信息");
//   }

//   rplidar.startScanNormal(true);  // 启动标准扫描
// }

// void readAndSendLidar() {
//   if (!rplidar.isScanning()) {
//     rplidar.startScanNormal(true);
//     delay(10);
//   }

//   rplidar.loopScanData();
//   rplidar_response_measurement_node_hq_t nodes[512];
//   size_t count = 512;

//   if (IS_OK(rplidar.grabScanData(nodes, count))) {
//     int sent = 0;

//     for (size_t i = 0; i < count && sent < 12; ++i) {
//       float angle = nodes[i].angle_z_q14 * 90.f / (1 << 14);
//       float dist = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);
//       int qual = nodes[i].quality;

//       if ((int)angle % 30 == 0 && dist > 0.05f && qual > 0) {
//         // ✅ 格式清晰，和 IMU 区分
//         snprintf(lidarBuf, sizeof(lidarBuf), "LIDAR:%.1f,%.2f,Q:%d\n", angle, dist, qual);
//         BTSerial.print(lidarBuf);     // ✅ 蓝牙输出
//         Serial.print(lidarBuf);       // 调试输出
//         sent++;
//       }
//     }
//   } else {
//     Serial.println("Radar data capture failure");
//     BTSerial.println("Radar data capture failure");
//   }
// }

// #endif
