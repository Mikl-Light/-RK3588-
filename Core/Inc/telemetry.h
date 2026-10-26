/**
 ******************************************************************************
 * @file    telemetry.h
 * @brief   遥测模块 — 统一串口数据帧打包发送
 *
 * 两种帧格式 (TELEM,ASCII / TELEM,BIN 切换):
 *   1) ASCII 帧 ($T, 默认, 人眼调试用):
 *      $T,<tick>,<enc1>,<enc2>,<enc3>,<enc4>,
 *         <ax>,<ay>,<az>,<gx>,<gy>,<gz>,<mx>,<my>,<mz>,
 *         <x>,<y>,<hdg>,<vx>,<vy>,<wz>,<spd1>..<spd4>\r\n
 *   2) 二进制帧 (TELEM,BIN, RK3588 解包用):
 *      0xAA 0x55 + type + payload_len + timestamp +
 *      enc_raw[4]int32 + accel/gyro/mag_raw[3]int16 +
 *      pose[3] + twist[3] + wheel_speed[4] float +
 *      ODOM EKF 诊断 (flags + 6 floats) + crc8 (共 112 字节, type=0x02)
 *      详见 telemetry.c 注释与 serial-commands.md
 *
 * 更新频率: 可配置 (默认 50Hz = 每 20ms)
 ******************************************************************************
 */
#ifndef __TELEMETRY_H__
#define __TELEMETRY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/* 遥测配置                                                                   */
/* -------------------------------------------------------------------------- */
/** 帧模式 */
typedef enum {
    TELEMETRY_MODE_ASCII = 0,   /**< $T 文本帧 (默认, 便于调试) */
    TELEMETRY_MODE_BINARY = 1,  /**< 0xAA 0x55 二进制帧 (含原始 int16 + CRC8) */
} Telemetry_Mode;

typedef struct {
    uint16_t interval_ms;       /**< 发送间隔 (ms), 默认 20 (50Hz) */
    bool     enabled;           /**< 是否启用自动遥测 */
    Telemetry_Mode mode;        /**< 帧模式 */
} Telemetry_Config;

/* -------------------------------------------------------------------------- */
/* API 函数                                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief 初始化遥测模块
 * @param cfg 配置 (传 NULL 使用默认: 50Hz 自动发送)
 */
void Telemetry_Init(const Telemetry_Config *cfg);

/**
 * @brief 遥测定时更新 (需每控制周期调用)
 * @note  内部自动按 interval_ms 间隔发送遥测帧
 */
void Telemetry_Update(void);

/**
 * @brief 手动发送一帧遥测数据 (不受 interval 限制)
 */
void Telemetry_SendNow(void);

/**
 * @brief 启用/禁用自动遥测
 */
void Telemetry_SetEnabled(bool enabled);

/**
 * @brief 设置发送间隔
 * @param interval_ms 间隔 (ms), 0=最快 (每周期都发)
 */
void Telemetry_SetInterval(uint16_t interval_ms);

/**
 * @brief 设置帧模式
 * @param mode ASCII 文本帧 或 二进制帧
 */
void Telemetry_SetMode(Telemetry_Mode mode);

#ifdef __cplusplus
}
#endif

#endif /* __TELEMETRY_H__ */
