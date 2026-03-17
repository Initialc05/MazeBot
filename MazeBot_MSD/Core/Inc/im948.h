/**
 * @file im948.h
 * @brief IM900/IM948 IMU驱动 (从Arduino移植到STM32 HAL)
 *
 * 移植变更:
 *   - #include "Arduino.h" → #include "main.h"
 *   - millis() → HAL_GetTick()
 *   - delay() → HAL_Delay() / osDelay()
 *   - IMU900Serial.write() → IMU_SendBytes()
 *   - DATA_SERIAL.write() → BT_SendBytes()
 *   - Serial.available()/read() → DMA+IDLE (uart_device.c)
 */
#ifndef IM948_H
#define IM948_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* ==================== 类型定义 ==================== */
typedef signed char    S8;
typedef unsigned char  U8;
typedef signed short   S16;
typedef unsigned short U16;
typedef signed long    S32;
typedef unsigned long  U32;
typedef float          F32;
#define pow2(x) ((x) * (x))

/* ==================== 传输缩放比例 ==================== */
#define scaleAccel           0.00478515625f
#define scaleQuat            0.000030517578125f
#define scaleAngle           0.0054931640625f
#define scaleAngleSpeed      0.06103515625f
#define scaleMag             0.15106201171875f
#define scaleTemperature     0.01f
#define scaleAirPressure     0.0002384185791f
#define scaleHeight          0.0010728836f

/* ==================== 协议常量 ==================== */
#define CmdPacket_Begin          0x49
#define CmdPacket_End            0x4D
#define CmdPacketMaxDatSizeRx    73
#define CmdPacketMaxDatSizeTx    31

/* ==================== 编码器里程计配置 ==================== */
#define USE_ENCODER_ODOM         1
#define USE_BLUETOOTH_MODE       1

/* ==================== IMU时间戳缓冲区 ==================== */
#define IMU_BUFFER_SIZE          100

typedef struct {
    uint32_t timestamp_ms;
    float x;
    float y;
    float theta;
} IMUSnapshot;

extern IMUSnapshot imu_buffer[IMU_BUFFER_SIZE];
extern volatile uint8_t imu_buffer_head;
extern volatile uint8_t imu_buffer_tail;

/* ==================== IMU全局数据 ==================== */
extern F32 AngleX, AngleY, AngleZ;
extern F32 OffsetX, OffsetY, OffsetZ;
extern U8  isNewData;

/* ==================== 接口函数 ==================== */

/** 协议字节解析 (逐字节喂入) */
U8 Cmd_GetPkt(U8 byte);

/** IMU初始化 (唤醒+配置+开启上报) */
void IMU_Init(void);

/** IMU数据更新 (在IMU任务中调用, DMA数据由uart_device处理) */
void IMU_Update(void);

/** 发送独立Odom数据包到蓝牙 */
void IMU_SendOdomUpdate(void);

/** 根据时间戳查找最接近的IMU数据 */
bool IMU_GetSnapshotByTimestamp(uint32_t target_ms, IMUSnapshot *out);

/* ==================== IMU命令 ==================== */
extern U8 targetDeviceAddress;

void Cmd_02(void);   /* 睡眠 */
void Cmd_03(void);   /* 唤醒 */
void Cmd_05(void);   /* Z轴角归零 */
void Cmd_06(void);   /* xyz世界坐标系清零 */
void Cmd_12(U8 accStill, U8 stillToZero, U8 moveToZero, U8 isCompassOn,
            U8 barometerFilter, U8 reportHz, U8 gyroFilter, U8 accFilter,
            U8 compassFilter, U16 Cmd_ReportTag);
void Cmd_13(void);   /* 惯导位置清零 */
void Cmd_18(void);   /* 关闭主动上报 */
void Cmd_19(void);   /* 开启主动上报 */

#endif /* IM948_H */
