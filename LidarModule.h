#ifndef LIDAR_MODULE_H
#define LIDAR_MODULE_H

#include <Arduino.h>

// ==================== 调试模式配置 ====================
// 设置为 true: 使用蓝牙模式 (BTSerial)
// 设置为 false: 使用串口调试模式 (Serial USB)
#define USE_BLUETOOTH_MODE true

#if USE_BLUETOOTH_MODE
    #define DATA_SERIAL BTSerial   // 蓝牙模式
    #define DEBUG_PRINT_ENABLED true  // 允许调试输出到Serial
#else
    #define DATA_SERIAL Serial     // 串口调试模式
    #define DEBUG_PRINT_ENABLED false  // 禁用调试输出（避免干扰二进制数据）
#endif

// ==================== RPLIDAR C1 协议定义 ====================
// 参考: https://github.com/slamtec/rplidar_sdk

// 命令定义
#define RPLIDAR_CMD_SYNC_BYTE        0xA5
#define RPLIDAR_CMD_STOP             0x25
#define RPLIDAR_CMD_SCAN             0x20
#define RPLIDAR_CMD_FORCE_SCAN       0x21
#define RPLIDAR_CMD_RESET            0x40
#define RPLIDAR_CMD_GET_DEVICE_INFO  0x50

// 应答头定义
#define RPLIDAR_ANS_SYNC_BYTE1       0xA5
#define RPLIDAR_ANS_SYNC_BYTE2       0x5A
#define RPLIDAR_ANS_TYPE_MEASUREMENT 0x81

// 数据包同步位
#define RPLIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define RPLIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1

// 数据结构：标准扫描节点（5字节）
struct __attribute__((packed)) rplidar_response_measurement_node_t {
    uint8_t  sync_quality;      // syncbit:1; syncbit_inverse:1; quality:6
    uint16_t angle_q6_checkbit; // check_bit:1; angle_q6:15
    uint16_t distance_q2;       // 距离，单位: 0.25mm
};

// 高质量数据节点（用于输出）
struct __attribute__((packed)) LidarDataPoint {
    uint16_t angle_deg_q8;  // 角度 * 256 (0-360°)
    uint16_t distance_mm;   // 距离，单位: mm
    uint8_t  quality;       // 质量 (0-63)
    uint8_t  sync_flag;     // 同步标志位
};

// ==================== 全局变量 ====================
static uint8_t lidar_rx_buffer[5];  // 接收缓冲区
static uint8_t lidar_rx_index = 0;
static bool lidar_is_scanning = false;
static uint32_t lidar_point_count = 0;  // 扫描点计数

// 调试开关（设为true可打印前20个数据包的原始数据）
#define LIDAR_DEBUG_MODE false

// 蓝牙数据包格式（雷达 + Odom融合）
struct __attribute__((packed)) BluetoothLidarPacket {
    uint16_t header;        // 固定头：0xAA55
    uint32_t angle_deg_q8;  // 角度 * 256 (uint32_t避免360°*256=92160溢出uint16_t)
    uint16_t distance_mm;   // 距离 mm
    uint8_t  quality;       // 质量
    int16_t  odom_x_cm;     // 里程计X (cm)
    int16_t  odom_y_cm;     // 里程计Y (cm)
    int32_t  odom_theta_deg_q8; // 里程计Theta (度*256) - 使用int32_t避免溢出
    uint8_t  checksum;      // 校验和
};

// ==================== 外部变量声明（来自im948_CMD.ino） ====================
extern float OffsetX, OffsetY, OffsetZ;  // IMU惯导位移（单位：米）
extern float AngleX, AngleY, AngleZ;     // IMU欧拉角（单位：度）

// ==================== 函数声明 ====================

// 发送命令到雷达
void sendLidarCommand(uint8_t cmd) {
    uint8_t packet[2] = {RPLIDAR_CMD_SYNC_BYTE, cmd};
    LidarSerial.write(packet, 2);
    LidarSerial.flush();
}

// 雷达初始化 (C1波特率: 460800)
void initLidar() {
    // C1型号波特率: 460800
    LidarSerial.begin(460800);
    
    delay(100);
    
    // 停止之前的扫描
    sendLidarCommand(RPLIDAR_CMD_STOP);
    delay(100);
    
    // 清空串口缓冲区
    while(LidarSerial.available()) {
        LidarSerial.read();
    }
    
    #if DEBUG_PRINT_ENABLED
    Serial.println(F("[LIDAR] 发送扫描启动命令..."));
    #endif
    
    sendLidarCommand(RPLIDAR_CMD_SCAN);
    
    // 等待应答描述符到达并丢弃（7字节）
    delay(100);
    uint8_t discardCount = 0;
    while (discardCount < 7 && LidarSerial.available()) {
        LidarSerial.read();
        discardCount++;
    }
    
    #if DEBUG_PRINT_ENABLED
    Serial.print(F("[LIDAR] 已丢弃应答字节: "));
    Serial.println(discardCount);
    Serial.println(F("[LIDAR] C1雷达初始化完成 (460800 baud)"));
    #endif
    
    delay(400);  // 等待雷达开始发送测量数据
    
    lidar_is_scanning = true;
    lidar_point_count = 0;
    
    // 发送启动标记
    DATA_SERIAL.write(0xFF);
    DATA_SERIAL.write(0xFF);
    #if USE_BLUETOOTH_MODE
    DATA_SERIAL.println("LIDAR_START");
    #endif
}

// ==================== 性能优化说明 ====================
// 当前配置针对921600波特率蓝牙模块优化：
//   - 质量过滤阈值：15 (发送更多有效数据)
//   - 距离范围：0.1m - 3m
//   - 数据包大小：18字节 (header(2) + angle(4) + dist(2) + qual(1) + odom(8) + checksum(1))
//   - 理论吞吐：~5100包/秒 (带宽充足)
//   - 实际发送：~2000-4000包/秒 (过滤后)
//   - 带宽使用率：40-75% (还有余量)
// =====================================================

// 解析单个扫描节点并通过蓝牙发送
void parseLidarNode(rplidar_response_measurement_node_t* node) {
    // 提取同步位
    uint8_t sync_bit = (node->sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);
    
    // 提取质量值 (0-63)
    uint8_t quality = (node->sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
    
    // 提取角度 (Q6格式: angle_q6 = angle_deg * 64)
    uint16_t angle_q6 = (node->angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT);
    float angle_deg = angle_q6 / 64.0f;
    
    // 提取距离 (Q2格式: dist_q2 = dist_mm * 4)
    uint16_t distance_mm = node->distance_q2 / 4;
    
    // 调试模式：打印前20个数据包的原始数据
    #if LIDAR_DEBUG_MODE && DEBUG_PRINT_ENABLED
    static uint32_t debug_counter = 0;
    if (debug_counter < 20) {
        Serial.print("RAW[");
        Serial.print(debug_counter);
        Serial.print("]: ");
        for(int i=0; i<5; i++) {
            if(lidar_rx_buffer[i] < 0x10) Serial.print("0");
            Serial.print(lidar_rx_buffer[i], HEX);
            Serial.print(" ");
        }
        Serial.print("-> A=");
        Serial.print(angle_deg, 1);
        Serial.print("° D=");
        Serial.print(distance_mm);
        Serial.print("mm Q=");
        Serial.print(quality);
        Serial.print(" S=");
        Serial.println(sync_bit);
        debug_counter++;
    }
    #endif
    
    // 过滤无效数据（进一步降低质量阈值以获取更多点）
    if (quality < 5 || distance_mm > 1500 || distance_mm < 200) {
        return;  // 过滤低质量点、超出3m范围、以及过近的点（<20cm）
    }
    
    // 构建蓝牙数据包（雷达 + Odom融合）
    BluetoothLidarPacket bt_packet;
    bt_packet.header = 0xAA55;  // 固定包头
    bt_packet.angle_deg_q8 = (uint32_t)(angle_deg * 256);  // 角度*256 (uint32_t避免溢出)
    bt_packet.distance_mm = distance_mm;
    bt_packet.quality = quality;
    
    // ========== Odom数据源选择 ==========
    #if USE_ENCODER_ODOM
      // 新版：编码器位移 + IMU朝向（推荐）
      bt_packet.odom_x_cm = (int16_t)(encoder_odom_x * 100);  // 编码器X → 厘米
      bt_packet.odom_y_cm = (int16_t)(encoder_odom_y * 100);  // 编码器Y → 厘米
    #else
      // 旧版：纯IMU（向后兼容）
      bt_packet.odom_x_cm = (int16_t)(OffsetX * 100);         // IMU X → 厘米
      bt_packet.odom_y_cm = (int16_t)(OffsetY * 100);         // IMU Y → 厘米
    #endif
    
    // 朝向始终使用IMU（准确可靠）
    bt_packet.odom_theta_deg_q8 = (int32_t)(AngleZ * 256);  // 角度*256 (int32_t避免±128°溢出)
    
    // 计算校验和（简单XOR）
    uint8_t* packet_bytes = (uint8_t*)&bt_packet;
    bt_packet.checksum = 0;
    for (uint8_t i = 2; i < sizeof(BluetoothLidarPacket) - 1; i++) {
        bt_packet.checksum ^= packet_bytes[i];
    }
    
    // 发送数据包（18字节二进制包：header(2) + angle(4) + dist(2) + qual(1) + odom(8) + checksum(1)）
    DATA_SERIAL.write((uint8_t*)&bt_packet, sizeof(BluetoothLidarPacket));
    
    // 同步帧标记（新的一圈开始）
    if (sync_bit) {
        lidar_point_count = 0;
        // 发送同步标记
        DATA_SERIAL.write(0xEE);
        DATA_SERIAL.write(0xEE);
    }
    
    lidar_point_count++;
}

// 数据读取与解析（在FreeRTOS任务中循环调用）
void readAndSendLidar() {
    if (!lidar_is_scanning) return;
    
    while (LidarSerial.available()) {
        uint8_t byte = LidarSerial.read();
        
        // 状态机解析5字节数据包
        switch (lidar_rx_index) {
            case 0:  // 等待同步字节 (sync_quality)
                // 检查 bit[0] 和 bit[1] 是否互为反码
                if (((byte & 0x01) ^ ((byte & 0x02) >> 1)) == 0x01) {
                    lidar_rx_buffer[lidar_rx_index++] = byte;
                }
                break;
                
            case 1:  // angle_q6_checkbit LSB
                // 检查 check_bit (bit[0]) 必须为1
                if (byte & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) {
                    lidar_rx_buffer[lidar_rx_index++] = byte;
                } else {
                    lidar_rx_index = 0;  // 重新同步
                }
                break;
                
            case 2:  // angle_q6_checkbit MSB
            case 3:  // distance_q2 LSB
                lidar_rx_buffer[lidar_rx_index++] = byte;
                break;
                
            case 4:  // distance_q2 MSB (最后一字节)
                lidar_rx_buffer[lidar_rx_index++] = byte;
                
                // 解析完整数据包
                rplidar_response_measurement_node_t* node = 
                    (rplidar_response_measurement_node_t*)lidar_rx_buffer;
                parseLidarNode(node);
                
                lidar_rx_index = 0;  // 准备下一帧
                break;
        }
    }
}

// 停止扫描
void stopLidar() {
    sendLidarCommand(RPLIDAR_CMD_STOP);
    lidar_is_scanning = false;
    
    #if DEBUG_PRINT_ENABLED
    Serial.println(F("[LIDAR] 扫描已停止"));
    #endif
    
    // 发送停止标记
    DATA_SERIAL.write(0xDD);
    DATA_SERIAL.write(0xDD);
    #if USE_BLUETOOTH_MODE
    DATA_SERIAL.println("LIDAR_STOP");
    #endif
}

// 重启扫描
void restartLidar() {
    stopLidar();
    delay(100);
    sendLidarCommand(RPLIDAR_CMD_SCAN);
    delay(100);
    lidar_is_scanning = true;
    
    #if DEBUG_PRINT_ENABLED
    Serial.println(F("[LIDAR] 扫描已重启"));
    #endif
}

#endif // LIDAR_MODULE_H
