#ifndef LIDAR_MODULE_H
#define LIDAR_MODULE_H

#include <Arduino.h>

extern HardwareSerial BTSerial;
extern HardwareSerial LidarSerial;

// ==================== 调试模式配置 ====================
// 设置为 true: 使用蓝牙模式 (BTSerial)
// 设置为 false: 使用串口调试模式 (Serial USB)
#define USE_BLUETOOTH_MODE true

#if USE_BLUETOOTH_MODE
    #define DATA_SERIAL BTSerial
    #define DEBUG_PRINT_ENABLED true
#else
    #define DATA_SERIAL Serial
    #define DEBUG_PRINT_ENABLED false
#endif

// ==================== RPLIDAR C1 协议定义 ====================
#define RPLIDAR_CMD_SYNC_BYTE        0xA5
#define RPLIDAR_CMD_STOP             0x25
#define RPLIDAR_CMD_SCAN             0x20
#define RPLIDAR_CMD_FORCE_SCAN       0x21
#define RPLIDAR_CMD_RESET            0x40
#define RPLIDAR_CMD_GET_DEVICE_INFO  0x50

#define RPLIDAR_ANS_SYNC_BYTE1       0xA5
#define RPLIDAR_ANS_SYNC_BYTE2       0x5A
#define RPLIDAR_ANS_TYPE_MEASUREMENT 0x81

#define RPLIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define RPLIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1

struct __attribute__((packed)) rplidar_response_measurement_node_t {
    uint8_t  sync_quality;
    uint16_t angle_q6_checkbit;
    uint16_t distance_q2;
};

struct __attribute__((packed)) LidarDataPoint {
    uint16_t angle_deg_q8;
    uint16_t distance_mm;
    uint8_t  quality;
    uint8_t  sync_flag;
};

#define LIDAR_DEBUG_MODE false

struct __attribute__((packed)) BluetoothLidarPacket {
    uint16_t header;
    uint32_t angle_deg_q8;
    uint16_t distance_mm;
    uint8_t  quality;
    int16_t  odom_x_cm;
    int16_t  odom_y_cm;
    int32_t  odom_theta_deg_q8;
    uint8_t  checksum;
};

extern float OffsetX, OffsetY, OffsetZ;
extern float AngleX, AngleY, AngleZ;

void sendLidarCommand(uint8_t cmd);
void initLidar();
void parseLidarNode(rplidar_response_measurement_node_t* node);
void readAndSendLidar();
void stopLidar();
void restartLidar();

#endif // LIDAR_MODULE_H
