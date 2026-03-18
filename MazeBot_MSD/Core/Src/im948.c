/**
 * @file im948.c
 * @brief IM900/IM948 IMU驱动 (从Arduino移植到STM32 HAL)
 *
 * 移植变更:
 *   - IMU900Serial.write() → IMU_SendBytes()
 *   - DATA_SERIAL.write() → BT_SendBytes()
 *   - millis() → HAL_GetTick()
 *   - delay() → osDelay()
 *   - Serial.available()/read() → DMA+IDLE (uart_device.c IMU_ProcessDMA)
 *   - updateEncoderOdometry() → Encoder_UpdateOdometry() (encoder.c)
 */
#include "im948.h"
#include "uart_device.h"
#include "encoder.h"
#include "cmsis_os.h"
#include <stdlib.h>

/* ==================== 前向声明 ==================== */
static void Cmd_Write(U8 *pBuf, int Len);
static void Cmd_RxUnpack(U8 *buf, U8 DLen);
static int  Cmd_PackAndTx(U8 *pDat, U8 DLen);

/* ==================== 全局变量 ==================== */
U8 targetDeviceAddress = 255;

IMUSnapshot imu_buffer[IMU_BUFFER_SIZE];
volatile uint8_t imu_buffer_head = 0;
volatile uint8_t imu_buffer_tail = 0;

F32 AngleX, AngleY, AngleZ;
F32 OffsetX, OffsetY, OffsetZ;
U8  isNewData = 0;

/* ==================== 工具函数 ==================== */
static U8 CalcSum1(U8 *Buf, int Len)
{
    U8 Sum = 0;
    while (Len-- > 0) {
        Sum += Buf[Len];
    }
    return Sum;
}

static void *Memcpy_local(void *s1, const void *s2, unsigned int n)
{
    char *p1 = (char *)s1;
    const char *p2 = (const char *)s2;
    if (n) {
        n++;
        while (--n > 0) {
            *p1++ = *p2++;
        }
    }
    return s1;
}

/* ==================== IMU时间戳查找 ==================== */
bool IMU_GetSnapshotByTimestamp(uint32_t target_ms, IMUSnapshot *out)
{
    if (imu_buffer_head == imu_buffer_tail) {
        out->timestamp_ms = HAL_GetTick();
        out->x = OffsetX;
        out->y = OffsetY;
        out->theta = AngleZ;
        return false;
    }

    uint8_t best_idx = imu_buffer_head;
    uint32_t min_diff = 0xFFFFFFFF;

    uint8_t idx = imu_buffer_tail;
    while (idx != imu_buffer_head) {
        uint32_t diff = (imu_buffer[idx].timestamp_ms > target_ms)
                      ? (imu_buffer[idx].timestamp_ms - target_ms)
                      : (target_ms - imu_buffer[idx].timestamp_ms);
        if (diff < min_diff) {
            min_diff = diff;
            best_idx = idx;
        }
        idx = (idx + 1) % IMU_BUFFER_SIZE;
    }

    /* 检查head位置 */
    uint32_t diff = (imu_buffer[imu_buffer_head].timestamp_ms > target_ms)
                  ? (imu_buffer[imu_buffer_head].timestamp_ms - target_ms)
                  : (target_ms - imu_buffer[imu_buffer_head].timestamp_ms);
    if (diff < min_diff) {
        best_idx = imu_buffer_head;
    }

    *out = imu_buffer[best_idx];
    return true;
}

/* ==================== Odom数据包发送 ==================== */
#if USE_BLUETOOTH_MODE
typedef struct __attribute__((packed)) {
    uint16_t header;
    int16_t  odom_x_cm;
    int16_t  odom_y_cm;
    int32_t  odom_theta_deg_q8;
    uint8_t  checksum;
} OdomOnlyPacket;
#endif

void IMU_SendOdomUpdate(void)
{
#if USE_BLUETOOTH_MODE
    OdomOnlyPacket pkt;
    pkt.header = 0xBB55;

#if USE_ENCODER_ODOM
    pkt.odom_x_cm = (int16_t)(odom_x * 100);
    pkt.odom_y_cm = (int16_t)(odom_y * 100);
#else
    pkt.odom_x_cm = (int16_t)(OffsetX * 100);
    pkt.odom_y_cm = (int16_t)(OffsetY * 100);
#endif

    pkt.odom_theta_deg_q8 = (int32_t)(AngleZ * 256);

    uint8_t *bytes = (uint8_t *)&pkt;
    pkt.checksum = 0;
    for (uint8_t i = 2; i < sizeof(OdomOnlyPacket) - 1; i++) {
        pkt.checksum ^= bytes[i];
    }

    BT_SendBytes((uint8_t *)&pkt, sizeof(OdomOnlyPacket));
#endif
}

/* ==================== IMU初始化 ==================== */
void IMU_Init(void)
{
    osDelay(3000);

    imu_buffer_head = 0;
    imu_buffer_tail = 0;

    Cmd_03();                                     /* 唤醒 */
    osDelay(200);
    IMU_ProcessDMA();                             /* 消费ACK回复 */

    Cmd_12(3, 0, 0, 0, 3, 2, 250, 4, 9, 0xFFF);  /* 配置参数 */
    osDelay(200);
    IMU_ProcessDMA();

    Cmd_13();                                     /* 惯导位置清零 */
    osDelay(100);
    IMU_ProcessDMA();

    Cmd_05();                                     /* Z轴角归零 */
    osDelay(100);
    IMU_ProcessDMA();

    Cmd_19();                                     /* 开启主动上报 */
    osDelay(100);
    IMU_ProcessDMA();                             /* 消费最后的ACK */
}

/* ==================== IMU数据更新 ==================== */
void IMU_Update(void)
{
    /* DMA数据由uart_device.c的IMU_ProcessDMA()逐字节喂给Cmd_GetPkt() */
    IMU_ProcessDMA();

    if (isNewData) {
        isNewData = 0;

        /* 保存到环形缓冲区 */
        uint32_t now = HAL_GetTick();
        uint8_t next = (imu_buffer_head + 1) % IMU_BUFFER_SIZE;
        if (next == imu_buffer_tail) {
            imu_buffer_tail = (imu_buffer_tail + 1) % IMU_BUFFER_SIZE;
        }
        imu_buffer[next].timestamp_ms = now;
        imu_buffer[next].x = OffsetX;
        imu_buffer[next].y = OffsetY;
        imu_buffer[next].theta = AngleZ;
        imu_buffer_head = next;

        /* 更新编码器里程计 (使用最新AngleZ) */
#if USE_ENCODER_ODOM
        {
            float heading_rad = AngleZ * (3.14159265f / 180.0f);
            Encoder_UpdateOdometry(heading_rad);
        }
#endif

        /* 发送Odom数据包 */
        IMU_SendOdomUpdate();
    }
}

/* ==================== 协议层: 打包发送 ==================== */
static int Cmd_PackAndTx(U8 *pDat, U8 DLen)
{
    U8 buf[50 + 5 + CmdPacketMaxDatSizeTx] = {
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x00,0xff
    };

    if ((DLen == 0) || (DLen > CmdPacketMaxDatSizeTx) || (pDat == NULL))
        return -1;

    buf[50] = CmdPacket_Begin;
    buf[51] = targetDeviceAddress;
    buf[52] = DLen;
    Memcpy_local(&buf[53], pDat, DLen);
    buf[53 + DLen] = CalcSum1(&buf[51], DLen + 2);
    buf[54 + DLen] = CmdPacket_End;

    Cmd_Write(buf, DLen + 55);
    return 0;
}

/* ==================== 协议层: 字节解析状态机 ==================== */
U8 Cmd_GetPkt(U8 byte)
{
    static U8 CS = 0;
    static U8 i = 0;
    static U8 RxIndex = 0;
    static U8 buf[5 + CmdPacketMaxDatSizeRx];

#define cmdAddress buf[1]
#define cmdLen     buf[2]

    CS += byte;
    switch (RxIndex) {
    case 0:
        if (byte == CmdPacket_Begin) {
            i = 0;
            buf[i++] = CmdPacket_Begin;
            CS = 0;
            RxIndex = 1;
        }
        break;
    case 1:
        buf[i++] = byte;
        if (byte == 255) { RxIndex = 0; break; }
        RxIndex++;
        break;
    case 2:
        buf[i++] = byte;
        if ((byte > CmdPacketMaxDatSizeRx) || (byte == 0)) { RxIndex = 0; break; }
        RxIndex++;
        break;
    case 3:
        buf[i++] = byte;
        if (i >= cmdLen + 3) RxIndex++;
        break;
    case 4:
        CS -= byte;
        if (CS == byte) { buf[i++] = byte; RxIndex++; }
        else { RxIndex = 0; }
        break;
    case 5:
        RxIndex = 0;
        if (byte == CmdPacket_End) {
            buf[i++] = byte;
            if ((targetDeviceAddress == cmdAddress) || (targetDeviceAddress == 255)) {
                Cmd_RxUnpack(&buf[3], i - 5);
                return 1;
            }
        }
        break;
    default:
        RxIndex = 0;
        break;
    }
    return 0;

#undef cmdAddress
#undef cmdLen
}

/* ==================== 协议层: 串口发送 ==================== */
static void Cmd_Write(U8 *pBuf, int Len)
{
    IMU_SendBytes(pBuf, (uint16_t)Len);
}

/* ==================== 命令函数 ==================== */
void Cmd_02(void) { U8 b[1] = {0x02}; Cmd_PackAndTx(b, 1); }
void Cmd_03(void) { U8 b[1] = {0x03}; Cmd_PackAndTx(b, 1); }
void Cmd_05(void) { U8 b[1] = {0x05}; Cmd_PackAndTx(b, 1); }
void Cmd_06(void) { U8 b[1] = {0x06}; Cmd_PackAndTx(b, 1); }
void Cmd_13(void) { U8 b[1] = {0x13}; Cmd_PackAndTx(b, 1); }
void Cmd_18(void) { U8 b[1] = {0x18}; Cmd_PackAndTx(b, 1); }
void Cmd_19(void) { U8 b[1] = {0x19}; Cmd_PackAndTx(b, 1); }

void Cmd_12(U8 accStill, U8 stillToZero, U8 moveToZero, U8 isCompassOn,
            U8 barometerFilter, U8 reportHz, U8 gyroFilter, U8 accFilter,
            U8 compassFilter, U16 Cmd_ReportTag)
{
    U8 buf[11] = {0x12};
    buf[1] = accStill;
    buf[2] = stillToZero;
    buf[3] = moveToZero;
    buf[4] = ((barometerFilter & 3) << 1) | (isCompassOn & 1);
    buf[5] = reportHz;
    buf[6] = gyroFilter;
    buf[7] = accFilter;
    buf[8] = compassFilter;
    buf[9] = Cmd_ReportTag & 0xff;
    buf[10] = (Cmd_ReportTag >> 8) & 0xff;
    Cmd_PackAndTx(buf, 11);
}

/* ==================== 数据解包 ==================== */
static void Cmd_RxUnpack(U8 *buf, U8 DLen)
{
    U16 ctl;
    U8 L;
    F32 tmpX, tmpY, tmpZ;

    switch (buf[0]) {
    case 0x11: /* 订阅功能数据 回复/主动上报 */
        ctl = ((U16)buf[2] << 8) | buf[1];
        L = 7;

        if ((ctl & 0x0001) != 0) { /* 加速度xyz (去重力) */
            L += 6;
        }
        if ((ctl & 0x0002) != 0) { /* 加速度xyz (含重力) */
            L += 6;
        }
        if ((ctl & 0x0004) != 0) { /* 角速度xyz */
            L += 6;
        }
        if ((ctl & 0x0008) != 0) { /* 磁场xyz */
            L += 6;
        }
        if ((ctl & 0x0010) != 0) { /* 温度+气压+高度 */
            L += 8; /* 2+3+3 */
        }
        if ((ctl & 0x0020) != 0) { /* 四元数wxyz */
            L += 8;
        }
        if ((ctl & 0x0040) != 0) { /* 欧拉角xyz */
            tmpX = (S16)(((S16)buf[L + 1] << 8) | buf[L]) * scaleAngle;
            L += 2;
            tmpY = (S16)(((S16)buf[L + 1] << 8) | buf[L]) * scaleAngle;
            L += 2;
            tmpZ = (S16)(((S16)buf[L + 1] << 8) | buf[L]) * scaleAngle;
            L += 2;
            AngleX = tmpX;
            AngleY = tmpY;
            AngleZ = tmpZ;
        }
        if ((ctl & 0x0080) != 0) { /* 空间位移xyz (mm→m) */
            tmpX = (S16)(((S16)buf[L + 1] << 8) | buf[L]) / 1000.0f;
            L += 2;
            tmpY = (S16)(((S16)buf[L + 1] << 8) | buf[L]) / 1000.0f;
            L += 2;
            tmpZ = (S16)(((S16)buf[L + 1] << 8) | buf[L]) / 1000.0f;
            L += 2;
            OffsetX = tmpX;
            OffsetY = tmpY;
            OffsetZ = tmpZ;
        }
        if ((ctl & 0x0100) != 0) { /* 活动检测 */
            L += 5; /* 4+1 */
        }
        if ((ctl & 0x0200) != 0) { /* 加速度xyz (静止坐标系) */
            L += 6;
        }
        if ((ctl & 0x0400) != 0) { /* ADC */
            L += 2;
        }
        if ((ctl & 0x0800) != 0) { /* GPIO */
            L += 1;
        }
        isNewData = 1;
        break;

    /* 其余命令回复不需要处理数据，仅标记收到即可 */
    default:
        break;
    }
    (void)DLen;
    (void)L;
}
