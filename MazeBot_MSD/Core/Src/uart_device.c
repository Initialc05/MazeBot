/**
 * @file uart_device.c
 * @brief UART DMA接收 + printf重定向 实现
 */
#include "uart_device.h"
#include <stdio.h>
#include <string.h>

/* ==================== 外部HAL句柄 ==================== */
extern UART_HandleTypeDef huart1;  /* IMU */
extern UART_HandleTypeDef huart2;  /* Debug */
extern UART_HandleTypeDef huart4;  /* Bluetooth */
extern UART_HandleTypeDef huart5;  /* LIDAR */

/* ==================== DMA接收缓冲区 ==================== */
uint8_t imu_rx_buf[IMU_RX_BUF_SIZE];
uint8_t debug_rx_buf[DEBUG_RX_BUF_SIZE];
uint8_t bt_rx_buf[BT_RX_BUF_SIZE];
uint8_t lidar_rx_buf[LIDAR_RX_BUF_SIZE];

/* 各通道上次读取位置 */
volatile uint16_t imu_rx_old_pos   = 0;
volatile uint16_t bt_rx_old_pos    = 0;
volatile uint16_t lidar_rx_old_pos = 0;

/* ==================== printf重定向 ==================== */
#ifdef __GNUC__
int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
#endif

/* ==================== 初始化 ==================== */
void UART_Device_Init(void)
{
    /* 启用UART IDLE中断 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);

    /* 启动DMA循环接收 */
    HAL_UART_Receive_DMA(&huart1, imu_rx_buf,   IMU_RX_BUF_SIZE);
    HAL_UART_Receive_DMA(&huart4, bt_rx_buf,    BT_RX_BUF_SIZE);
    HAL_UART_Receive_DMA(&huart5, lidar_rx_buf, LIDAR_RX_BUF_SIZE);
}

/* ==================== 发送接口 ==================== */
void Debug_SendBytes(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart2, data, len, HAL_MAX_DELAY);
}

void IMU_SendBytes(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart1, data, len, HAL_MAX_DELAY);
}

void BT_SendBytes(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart4, data, len, HAL_MAX_DELAY);
}

void BT_SendString(const char *str)
{
    HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

void Lidar_SendBytes(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart5, data, len, HAL_MAX_DELAY);
}

/* ==================== DMA环形缓冲区读取通用宏 ==================== */

/**
 * 从DMA环形缓冲区读取新数据的通用逻辑
 * buf_size: 缓冲区总大小
 * huart:    UART句柄指针
 * rx_buf:   DMA缓冲区
 * old_pos:  上次读取位置(volatile指针)
 * out_buf:  输出缓冲区
 * max_len:  最大输出长度
 * 返回:     实际读取字节数
 */
static uint16_t DMA_ReadRingBuffer(uint16_t buf_size,
                                   UART_HandleTypeDef *huart,
                                   const uint8_t *rx_buf,
                                   volatile uint16_t *old_pos,
                                   uint8_t *out_buf,
                                   uint16_t max_len)
{
    uint16_t write_pos = buf_size - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    uint16_t rd = *old_pos;
    uint16_t count = 0;

    while (rd != write_pos && count < max_len) {
        out_buf[count++] = rx_buf[rd];
        rd++;
        if (rd >= buf_size) rd = 0;
    }
    *old_pos = rd;
    return count;
}

/* ==================== IDLE中断回调 ==================== */
void UART_IDLE_Callback(UART_HandleTypeDef *huart)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        /* IDLE标志已清除，数据处理由各任务轮询DMA位置完成 */
    }
}

/* ==================== 各通道DMA数据处理 ==================== */
void IMU_ProcessDMA(void)
{
    uint16_t write_pos = IMU_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    uint16_t rd = imu_rx_old_pos;

    while (rd != write_pos) {
        /* 逐字节喂给IMU协议解析器，后续在im948模块中对接 */
        extern uint8_t Cmd_GetPkt(uint8_t byte);
        Cmd_GetPkt(imu_rx_buf[rd]);
        rd++;
        if (rd >= IMU_RX_BUF_SIZE) rd = 0;
    }
    imu_rx_old_pos = rd;
}

uint16_t BT_ReadDMA(uint8_t *out_buf, uint16_t max_len)
{
    return DMA_ReadRingBuffer(BT_RX_BUF_SIZE, &huart4,
                              bt_rx_buf, &bt_rx_old_pos,
                              out_buf, max_len);
}

uint16_t Lidar_ReadDMA(uint8_t *out_buf, uint16_t max_len)
{
    return DMA_ReadRingBuffer(LIDAR_RX_BUF_SIZE, &huart5,
                              lidar_rx_buf, &lidar_rx_old_pos,
                              out_buf, max_len);
}
