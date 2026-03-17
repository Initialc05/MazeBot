/**
 * @file lidar.c
 * @brief RPLIDAR C1 驱动实现
 *
 * 移植变更:
 *   - LidarSerial.write() → Lidar_SendBytes()
 *   - LidarSerial.available()/read() → DMA+IDLE (Lidar_ReadDMA)
 *   - DATA_SERIAL.write() → BT_SendBytes()
 *   - delay() → osDelay()
 */
#include "lidar.h"
#include "uart_device.h"
#include "im948.h"
#include "encoder.h"
#include "cmsis_os.h"

/* ==================== 状态 ==================== */
static bool     lidar_scanning = false;
static uint8_t  rx_buf[5];
static uint8_t  rx_idx = 0;
static uint32_t point_count = 0;

/* ==================== 内部: 发送RPLIDAR命令 ==================== */
static void Lidar_SendCmd(uint8_t cmd)
{
    uint8_t pkt[2] = { RPLIDAR_CMD_SYNC_BYTE, cmd };
    Lidar_SendBytes(pkt, 2);
}

/* ==================== 内部: 解析并发送融合包 ==================== */
static void parseLidarNode(void)
{
    uint8_t sync_quality      = rx_buf[0];
    uint16_t angle_q6_checkbit = (uint16_t)rx_buf[1] | ((uint16_t)rx_buf[2] << 8);
    uint16_t distance_q2       = (uint16_t)rx_buf[3] | ((uint16_t)rx_buf[4] << 8);

    uint8_t sync_bit = (sync_quality & 0x01);
    uint8_t quality  = (sync_quality >> 2);
    uint16_t angle_q6 = (angle_q6_checkbit >> 1);
    float angle_deg   = angle_q6 / 64.0f;
    uint16_t dist_mm  = distance_q2 / 4;

    /* 过滤 */
    if (quality < LIDAR_QUALITY_MIN || dist_mm > LIDAR_DIST_MAX_MM || dist_mm < LIDAR_DIST_MIN_MM)
        return;

    /* 构建18字节融合包 */
    LidarBtPacket pkt;
    pkt.header       = 0xAA55;
    pkt.angle_deg_q8 = (uint32_t)(angle_deg * 256);
    pkt.distance_mm  = dist_mm;
    pkt.quality      = quality;

#if USE_ENCODER_ODOM
    pkt.odom_x_cm = (int16_t)(odom_x * 100);
    pkt.odom_y_cm = (int16_t)(odom_y * 100);
#else
    pkt.odom_x_cm = (int16_t)(OffsetX * 100);
    pkt.odom_y_cm = (int16_t)(OffsetY * 100);
#endif
    pkt.odom_theta_deg_q8 = (int32_t)(AngleZ * 256);

    /* XOR校验 (跳过header, 不含checksum) */
    uint8_t *bytes = (uint8_t *)&pkt;
    pkt.checksum = 0;
    for (uint8_t i = 2; i < sizeof(LidarBtPacket) - 1; i++) {
        pkt.checksum ^= bytes[i];
    }

    BT_SendBytes((uint8_t *)&pkt, sizeof(LidarBtPacket));

    /* 同步帧标记 (新一圈) */
    if (sync_bit) {
        uint8_t sync_marker[2] = { 0xEE, 0xEE };
        BT_SendBytes(sync_marker, 2);
        point_count = 0;
    }
    point_count++;
}

/* ==================== 初始化 ==================== */
void Lidar_Init(void)
{
    /* 停止之前的扫描 */
    Lidar_SendCmd(RPLIDAR_CMD_STOP);
    osDelay(100);

    /* 清空DMA缓冲区中的残留数据 */
    {
        uint8_t discard[64];
        while (Lidar_ReadDMA(discard, sizeof(discard)) > 0) { }
    }

    /* 发送扫描命令 */
    Lidar_SendCmd(RPLIDAR_CMD_SCAN);
    osDelay(100);

    /* 丢弃7字节应答描述符 */
    {
        uint8_t discard[7];
        uint16_t got = 0;
        uint32_t t0 = HAL_GetTick();
        while (got < 7 && (HAL_GetTick() - t0 < 200)) {
            got += Lidar_ReadDMA(&discard[got], 7 - got);
            if (got < 7) osDelay(1);
        }
    }

    osDelay(400);

    lidar_scanning = true;
    point_count = 0;
    rx_idx = 0;

    /* 发送启动标记 */
    uint8_t start_marker[2] = { 0xFF, 0xFF };
    BT_SendBytes(start_marker, 2);
    BT_SendString("LIDAR_START\r\n");
}

/* ==================== 周期更新 ==================== */
void Lidar_Update(void)
{
    if (!lidar_scanning) return;

    uint8_t tmp[64];
    uint16_t n = Lidar_ReadDMA(tmp, sizeof(tmp));

    for (uint16_t i = 0; i < n; i++) {
        uint8_t byte = tmp[i];

        switch (rx_idx) {
        case 0: /* sync_quality: bit0 ^ bit1 == 1 */
            if (((byte & 0x01) ^ ((byte & 0x02) >> 1)) == 0x01) {
                rx_buf[rx_idx++] = byte;
            }
            break;
        case 1: /* angle LSB: check_bit must be 1 */
            if (byte & 0x01) {
                rx_buf[rx_idx++] = byte;
            } else {
                rx_idx = 0;
            }
            break;
        case 2: /* angle MSB */
        case 3: /* distance LSB */
            rx_buf[rx_idx++] = byte;
            break;
        case 4: /* distance MSB → 完整包 */
            rx_buf[rx_idx] = byte;
            parseLidarNode();
            rx_idx = 0;
            break;
        default:
            rx_idx = 0;
            break;
        }
    }
}

/* ==================== 停止 ==================== */
void Lidar_Stop(void)
{
    Lidar_SendCmd(RPLIDAR_CMD_STOP);
    lidar_scanning = false;

    uint8_t stop_marker[2] = { 0xDD, 0xDD };
    BT_SendBytes(stop_marker, 2);
    BT_SendString("LIDAR_STOP\r\n");
}

/* ==================== 重启 ==================== */
void Lidar_Restart(void)
{
    Lidar_Stop();
    osDelay(100);
    Lidar_SendCmd(RPLIDAR_CMD_SCAN);
    osDelay(100);
    lidar_scanning = true;
    rx_idx = 0;
}
