/**
 ******************************************************************************
 * @file    telemetry.c
 * @brief   遥测模块实现 — 统一串口数据帧
 *
 * 两种帧格式 (TELEM,ASCII / TELEM,BIN 切换, 默认 ASCII 便于人眼调试):
 *
 * 1) ASCII 帧 ($T, ...) — 调试优先, 人眼可读
 *   $T,<tick_ms>,<enc1_delta>,...,<accel_x_mg>,... (23 个整数字段)
 *
 * 2) 二进制帧 (0xAA 0x55 + CRC8) — RK3588 解包优先, 含原始 int16
 *   offset size 字段
 *   0     2     header 0xAA 0x55
 *   2     1     type 0x01
 *   3     2     payload_len = 81 (type..wheel_speed 字节数)
 *   5     4     timestamp (ms)
 *   9     16    enc_raw[4] int32 (编码器原始 CNT)
 *   25    6     accel_raw[3] int16 (原始 LSB)
 *   31    6     gyro_raw[3]  int16 (原始 LSB)
 *   37    6     mag_raw[3]   int16 (原始 LSB, 未乘 ASA)
 *   43    12    pose[3] float: x(m), y(m), yaw(rad, 0=北, 逆时针正)
 *   55    12    twist[3] float: vx, vy (m/s), wz (rad/s)
 *   67    16    wheel_speed[4] float (RPM)
 *   83    1     crc8 (CRC-8 多项式 0x07, 初值 0, 覆盖 type..wheel_speed 81 字节)
 *   total 84
 *
 * 字节序: 小端 (STM32F4 与 RK3588/AArch64 均为小端)
 * 注: 相比要求.md 增加了 mag_raw[3], 其余字段与规格一致
 ******************************************************************************
 */
#include "telemetry.h"
#include "odometry.h"
#include "uart_io.h"
#include <stdio.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* 默认配置                                                                   */
/* -------------------------------------------------------------------------- */
#define DEFAULT_INTERVAL_MS   20     /**< 50Hz */
#define DEFAULT_ENABLED       false  /**< 默认关闭, 由命令开启 */
#define DEFAULT_MODE          TELEMETRY_MODE_ASCII  /**< 调试优先 */

/* -------------------------------------------------------------------------- */
/* 二进制帧常量 (与 serial-commands.md 保持一致)                              */
/* -------------------------------------------------------------------------- */
#define BIN_HEADER0          0xAA
#define BIN_HEADER1          0x55
#define BIN_TYPE             0x01
#define BIN_PAYLOAD_LEN      81      /**< type..wheel_speed 字节数 */
#define BIN_FRAME_SIZE       84      /**< 2 头 + 81 载荷 + 1 CRC */

/* -------------------------------------------------------------------------- */
/* 内部状态                                                                   */
/* -------------------------------------------------------------------------- */
static Telemetry_Config g_cfg;
static uint32_t g_last_send_tick = 0;

/* -------------------------------------------------------------------------- */
/* CRC-8 (多项式 0x07, 初值 0x00, 无反射 — SMBus 风格)                        */
/* -------------------------------------------------------------------------- */
static uint8_t Telemetry_Crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* -------------------------------------------------------------------------- */
/* 初始化                                                                     */
/* -------------------------------------------------------------------------- */
void Telemetry_Init(const Telemetry_Config *cfg)
{
    if (cfg != NULL) {
        g_cfg = *cfg;
    } else {
        g_cfg.interval_ms = DEFAULT_INTERVAL_MS;
        g_cfg.enabled     = DEFAULT_ENABLED;
        g_cfg.mode        = DEFAULT_MODE;
    }
    g_last_send_tick = 0;
}

/* -------------------------------------------------------------------------- */
/* ASCII 帧: $T,... (调试用, 人眼可读)                                        */
/* -------------------------------------------------------------------------- */
static void Telemetry_SendAscii(void)
{
    const Odom_Data *d = Odom_GetData();
    if (d == NULL) return;

    /* 将浮点数乘以倍数转为整数传输, 避免 printf %f 的开销和精度问题 */
    /* 上位机解析: value = raw / divisor */
    UART_IO_Printf(
        "$T,%lu,"
        "%ld,%ld,%ld,%ld,"
        "%d,%d,%d,"
        "%d,%d,%d,"
        "%d,%d,%d,"
        "%d,%d,%d,"
        "%d,%d,%d,"
        "%d,%d,%d,%d"
        "\r\n",

        /* 时间戳 */
        (unsigned long)HAL_GetTick(),

        /* 编码器增量 (原始) */
        (long)d->enc_delta[0], (long)d->enc_delta[1],
        (long)d->enc_delta[2], (long)d->enc_delta[3],

        /* 加速度计 (mg = milli-g, ×1000) */
        (int)(d->accel_x * 1000.0f),
        (int)(d->accel_y * 1000.0f),
        (int)(d->accel_z * 1000.0f),

        /* 陀螺仪 (0.01 °/s, ×100) */
        (int)(d->gyro_x * 100.0f),
        (int)(d->gyro_y * 100.0f),
        (int)(d->gyro_z * 100.0f),

        /* 磁力计 (0.01 µT, ×100) */
        (int)(d->mag_x * 100.0f),
        (int)(d->mag_y * 100.0f),
        (int)(d->mag_z * 100.0f),

        /* 世界坐标 (mm, ×1000) */
        (int)(d->pose.x * 1000.0f),
        (int)(d->pose.y * 1000.0f),

        /* 航向 (0.1°, ×10) */
        (int)(d->pose.heading * 10.0f),

        /* 机器人速度 (mm/s, ×1000) */
        (int)(d->pose.vx * 1000.0f),
        (int)(d->pose.vy * 1000.0f),

        /* 角速度 (mrad/s, ×1000) */
        (int)(d->pose.omega * 1000.0f),

        /* 轮速 (0.1 RPM, ×10) */
        (int)(d->wheel_rpm[0] * 10.0f),
        (int)(d->wheel_rpm[1] * 10.0f),
        (int)(d->wheel_rpm[2] * 10.0f),
        (int)(d->wheel_rpm[3] * 10.0f)
    );
}

/* -------------------------------------------------------------------------- */
/* 二进制帧: 0xAA 0x55 + ... + CRC8 (RK3588 解包用, 含原始 int16)            */
/*   字段布局见文件头注释; 全部小端 memcpy 打包, 避免对齐问题                 */
/* -------------------------------------------------------------------------- */
static void Telemetry_SendBinary(void)
{
    const Odom_Data *d = Odom_GetData();
    if (d == NULL) return;

    uint8_t frame[BIN_FRAME_SIZE];
    uint16_t p = 0;

    frame[p++] = BIN_HEADER0;
    frame[p++] = BIN_HEADER1;
    frame[p++] = BIN_TYPE;

    uint16_t plen = BIN_PAYLOAD_LEN;
    memcpy(&frame[p], &plen, sizeof(plen)); p += 2;

    uint32_t tick = HAL_GetTick();
    memcpy(&frame[p], &tick, sizeof(tick)); p += 4;

    for (int i = 0; i < 4; i++) {
        memcpy(&frame[p], &d->enc_raw[i], sizeof(int32_t)); p += 4;
    }
    for (int i = 0; i < 3; i++) {
        memcpy(&frame[p], &d->accel_raw[i], sizeof(int16_t)); p += 2;
    }
    for (int i = 0; i < 3; i++) {
        memcpy(&frame[p], &d->gyro_raw[i], sizeof(int16_t)); p += 2;
    }
    for (int i = 0; i < 3; i++) {
        memcpy(&frame[p], &d->mag_raw[i], sizeof(int16_t)); p += 2;
    }

    /* pose: x(m), y(m), yaw(rad, 0=北, 逆时针正) */
    float pose[3] = { d->pose.x, d->pose.y, d->pose.heading * 0.01745329252f };
    memcpy(&frame[p], pose, sizeof(pose)); p += 12;

    /* twist: vx, vy (m/s), wz (rad/s) — 机器人坐标系 */
    float twist[3] = { d->pose.vx, d->pose.vy, d->pose.omega };
    memcpy(&frame[p], twist, sizeof(twist)); p += 12;

    /* wheel_speed: 4 轮 RPM (float) */
    memcpy(&frame[p], d->wheel_rpm, sizeof(d->wheel_rpm)); p += 16;

    /* CRC8 覆盖 type..wheel_speed (frame[2..82], 81 字节) */
    frame[p] = Telemetry_Crc8(&frame[2], BIN_PAYLOAD_LEN);

    UART_IO_SendBytes(frame, BIN_FRAME_SIZE);
}

/* -------------------------------------------------------------------------- */
/* 发送一帧 (按当前模式)                                                      */
/* -------------------------------------------------------------------------- */
void Telemetry_SendNow(void)
{
    if (g_cfg.mode == TELEMETRY_MODE_BINARY) {
        Telemetry_SendBinary();
    } else {
        Telemetry_SendAscii();
    }
}

/* -------------------------------------------------------------------------- */
/* 定时更新                                                                   */
/* -------------------------------------------------------------------------- */
void Telemetry_Update(void)
{
    if (!g_cfg.enabled) return;

    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - g_last_send_tick;

    if (g_cfg.interval_ms == 0 || elapsed >= g_cfg.interval_ms) {
        Telemetry_SendNow();
        g_last_send_tick = now;
    }
}

/* -------------------------------------------------------------------------- */
/* 配置                                                                       */
/* -------------------------------------------------------------------------- */
void Telemetry_SetEnabled(bool enabled)
{
    g_cfg.enabled = enabled;
    if (enabled) {
        g_last_send_tick = HAL_GetTick();  /* 立即发送第一帧 */
    }
}

void Telemetry_SetInterval(uint16_t interval_ms)
{
    g_cfg.interval_ms = interval_ms;
}

void Telemetry_SetMode(Telemetry_Mode mode)
{
    g_cfg.mode = mode;
}