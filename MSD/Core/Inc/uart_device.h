/**
 * @file uart_device.h
 * @brief UART DMA接收 + printf重定向 + 各串口收发接口
 *
 * USART1 (115200)  - IMU900
 * USART2 (115200)  - Debug (printf)
 * UART4  (921600)  - Bluetooth HC-04
 * UART5  (460800)  - RPLIDAR C1
 */
#ifndef UART_DEVICE_H
#define UART_DEVICE_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== DMA环形缓冲区大小 ==================== */
#define IMU_RX_BUF_SIZE    256   /* USART1 IMU */
#define DEBUG_RX_BUF_SIZE   64   /* USART2 Debug (基本不接收) */
#define BT_RX_BUF_SIZE    256   /* UART4  Bluetooth */
#define LIDAR_RX_BUF_SIZE 512   /* UART5  LIDAR (460800高速) */

/* ==================== DMA接收缓冲区 (extern) ==================== */
extern uint8_t imu_rx_buf[IMU_RX_BUF_SIZE];
extern uint8_t debug_rx_buf[DEBUG_RX_BUF_SIZE];
extern uint8_t bt_rx_buf[BT_RX_BUF_SIZE];
extern uint8_t lidar_rx_buf[LIDAR_RX_BUF_SIZE];

/* 各通道上次DMA读取位置 */
extern volatile uint16_t imu_rx_old_pos;
extern volatile uint16_t bt_rx_old_pos;
extern volatile uint16_t lidar_rx_old_pos;

/* ==================== 初始化 ==================== */

/**
 * @brief 初始化所有UART的DMA接收 + IDLE中断
 *        在MX_xxx_Init()之后、osKernelStart()之前调用
 */
void UART_Device_Init(void);

/* ==================== 发送接口 ==================== */

/** Debug串口发送 (USART2, 阻塞) */
void Debug_SendBytes(const uint8_t *data, uint16_t len);

/** IMU串口发送 (USART1, 阻塞) */
void IMU_SendBytes(const uint8_t *data, uint16_t len);

/** 蓝牙串口发送 (UART4, 阻塞) */
void BT_SendBytes(const uint8_t *data, uint16_t len);

/** 蓝牙发送字符串 */
void BT_SendString(const char *str);

/** LIDAR串口发送 (UART5, 阻塞) */
void Lidar_SendBytes(const uint8_t *data, uint16_t len);

/* ==================== DMA接收处理 ==================== */

/**
 * @brief UART IDLE中断回调 (在stm32f4xx_it.c的USARTx_IRQHandler中调用)
 *        检测IDLE标志并清除，触发数据处理
 */
void UART_IDLE_Callback(UART_HandleTypeDef *huart);

/**
 * @brief 处理IMU DMA缓冲区中的新数据
 *        逐字节喂给 Cmd_GetPkt()
 */
void IMU_ProcessDMA(void);

/**
 * @brief 处理蓝牙DMA缓冲区中的新数据
 *        返回读取到的字节数，数据写入out_buf
 * @param out_buf 输出缓冲区
 * @param max_len 最大读取长度
 * @return 实际读取的字节数
 */
uint16_t BT_ReadDMA(uint8_t *out_buf, uint16_t max_len);

/**
 * @brief 处理LIDAR DMA缓冲区中的新数据
 * @param out_buf 输出缓冲区
 * @param max_len 最大读取长度
 * @return 实际读取的字节数
 */
uint16_t Lidar_ReadDMA(uint8_t *out_buf, uint16_t max_len);

#endif /* UART_DEVICE_H */
