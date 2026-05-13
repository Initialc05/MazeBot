#include "../Inc/LidarModule.h"

#include "../Inc/EncoderModule.h"
#include "../Inc/im948_CMD.h"

static uint8_t lidar_rx_buffer[5];
static uint8_t lidar_rx_index = 0;
static bool lidar_is_scanning = false;
static uint32_t lidar_point_count = 0;

void sendLidarCommand(uint8_t cmd) {
  uint8_t packet[2] = {RPLIDAR_CMD_SYNC_BYTE, cmd};
  LidarSerial.write(packet, 2);
  LidarSerial.flush();
}

void initLidar() {
  LidarSerial.begin(460800);

  delay(100);

  sendLidarCommand(RPLIDAR_CMD_STOP);
  delay(100);

  while(LidarSerial.available()) {
    LidarSerial.read();
  }

  #if DEBUG_PRINT_ENABLED
  Serial.println(F("[LIDAR] 发送扫描启动命令..."));
  #endif

  sendLidarCommand(RPLIDAR_CMD_SCAN);

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
  delay(400);  
  lidar_is_scanning = true;
  lidar_point_count = 0;  
  DATA_SERIAL.write(0xFF);
  DATA_SERIAL.write(0xFF);
  #if USE_BLUETOOTH_MODE
  DATA_SERIAL.println("LIDAR_START");
  #endif
}

void parseLidarNode(rplidar_response_measurement_node_t* node) {
  uint8_t sync_bit = (node->sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);
  uint8_t quality = (node->sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
  uint16_t angle_q6 = (node->angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT);
  float angle_deg = angle_q6 / 64.0f;
  uint16_t distance_mm = node->distance_q2 / 4;  
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

  if (quality < 5 || distance_mm > 1500 || distance_mm < 200) {
    return;
  }

  BluetoothLidarPacket bt_packet;
  bt_packet.header = 0xAA55;
  bt_packet.angle_deg_q8 = (uint32_t)(angle_deg * 256);
  bt_packet.distance_mm = distance_mm;
  bt_packet.quality = quality;

  #if USE_ENCODER_ODOM
    bt_packet.odom_x_cm = (int16_t)(encoder_odom_x * 100);
    bt_packet.odom_y_cm = (int16_t)(encoder_odom_y * 100);
  #else
    bt_packet.odom_x_cm = (int16_t)(OffsetX * 100);
    bt_packet.odom_y_cm = (int16_t)(OffsetY * 100);
  #endif

  bt_packet.odom_theta_deg_q8 = (int32_t)(AngleZ * 256);

  uint8_t* packet_bytes = (uint8_t*)&bt_packet;
  bt_packet.checksum = 0;
  for (uint8_t i = 2; i < sizeof(BluetoothLidarPacket) - 1; i++) {
    bt_packet.checksum ^= packet_bytes[i];
  }

  DATA_SERIAL.write((uint8_t*)&bt_packet, sizeof(BluetoothLidarPacket));

  if (sync_bit) {
    lidar_point_count = 0;
    DATA_SERIAL.write(0xEE);
    DATA_SERIAL.write(0xEE);
  }

    lidar_point_count++;
}

void readAndSendLidar() {
  if (!lidar_is_scanning) return;

  while (LidarSerial.available()) {
    uint8_t byte = LidarSerial.read();

    switch (lidar_rx_index) {
      case 0:
        if (((byte & 0x01) ^ ((byte & 0x02) >> 1)) == 0x01) {
          lidar_rx_buffer[lidar_rx_index++] = byte;
        }
        break;

      case 1:
        if (byte & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) {
          lidar_rx_buffer[lidar_rx_index++] = byte;
        } else {
          lidar_rx_index = 0;
        }
        break;

      case 2:
      case 3:
        lidar_rx_buffer[lidar_rx_index++] = byte;
        break;

      case 4:
        lidar_rx_buffer[lidar_rx_index++] = byte;

        rplidar_response_measurement_node_t* node =
          (rplidar_response_measurement_node_t*)lidar_rx_buffer;
        parseLidarNode(node);

        lidar_rx_index = 0;
        break;
    }
  }
}

void stopLidar() {
  sendLidarCommand(RPLIDAR_CMD_STOP);
  lidar_is_scanning = false;

  #if DEBUG_PRINT_ENABLED
  Serial.println(F("[LIDAR] 扫描已停止"));
  #endif

  DATA_SERIAL.write(0xDD);
  DATA_SERIAL.write(0xDD);
  #if USE_BLUETOOTH_MODE
  DATA_SERIAL.println("LIDAR_STOP");
  #endif
}

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
