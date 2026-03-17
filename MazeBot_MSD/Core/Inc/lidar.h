/**
 * @file lidar.h
 * @brief RPLIDAR C1 驱动 (UART5 460800, DMA接收)
 *
 * 协议: RPLIDAR标准扫描模式, 5字节测量节点
 * 输出: 18字节融合包(0xAA55) → 蓝牙发送
 */
#ifndef LIDAR_H
#define LIDAR_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== RPLIDAR协议常量 ==================== */
#define RPLIDAR_CMD_SYNC_BYTE   0xA5
#define RPLIDAR_CMD_STOP        0x25
#define RPLIDAR_CMD_SCAN        0x20
#define RPLIDAR_CMD_RESET       0x40

/* ==================== 过滤参数 ==================== */
#define LIDAR_QUALITY_MIN       5
#define LIDAR_DIST_MIN_MM       200    /* 20cm */
#define LIDAR_DIST_MAX_MM       1500   /* 1.5m */

/* ==================== 融合数据包 (18字节) ==================== */
typedef struct __attribute__((packed)) {
    uint16_t header;              /* 0xAA55 */
    uint32_t angle_deg_q8;        /* 角度*256 */
    uint16_t distance_mm;         /* 距离mm */
    uint8_t  quality;             /* 质量0-63 */
    int16_t  odom_x_cm;           /* 里程计X cm */
    int16_t  odom_y_cm;           /* 里程计Y cm */
    int32_t  odom_theta_deg_q8;   /* 里程计Theta 度*256 */
    uint8_t  checksum;            /* XOR校验 */
} LidarBtPacket;

/* ==================== 接口函数 ==================== */

/** 初始化LIDAR: 发送停止+扫描命令, 丢弃应答头 */
void Lidar_Init(void);

/** 处理DMA缓冲区数据, 解析并发送融合包 (在LidarTask中周期调用) */
void Lidar_Update(void);

/** 停止扫描 */
void Lidar_Stop(void);

/** 重启扫描 */
void Lidar_Restart(void);

#endif /* LIDAR_H */
