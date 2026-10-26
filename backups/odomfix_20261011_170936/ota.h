/**
 ******************************************************************************
 * @file    ota.h
 * @brief   RK3588 OTA 升级支持 (App 侧)
 ******************************************************************************
 * 配合 "V1 - 副本" Bootloader 使用:
 *   RK 发 CMD_GET_VERSION -> App 回 "VERSION:<ver> <date>\r\n"
 *   RK 发 CMD_OTA_START   -> App 在 OTA_FLAG_ADDR 写升级标记并软复位,
 *                            Bootloader 检测到标记进入升级模式,
 *                            随后 RK 用 UPDATE/ADDR/DATA/RUN 协议传固件。
 *
 * 升级标记约定 (与 bootloader.h 保持一致):
 *   地址 0x0800FC00 (扇区3末页, Bootloader 镜像只占扇区0, 互不影响)
 *   魔数 0x31544F41 ("AOT1" 小端)
 ******************************************************************************
 */
#ifndef __OTA_H
#define __OTA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 升级标记 (与 bootloader.h 的 BL_FLAG_ADDR / BL_FLAG_MAGIC 一致) */
#define OTA_FLAG_ADDR       0x0800FC00U
#define OTA_FLAG_MAGIC      0x31544F41U  /* "AOT1" 小端 */

/* 固件版本 (RK 侧对比用, 格式: VERSION:v3.0-ODOMTEST 26.08.29)
 * v3.0-ODOMTEST (2026-08-29): 可靠里程计 ODOM 测试版 —
 *   自适应 EKF + 数据质量监督 + 动态 Q/R + ZUPT; 串口遥测保留原始数据,
 *   并新增 EKF 诊断字段; odom_compare_web 增加 /odom_diag 实时显示 */
#define FIRMWARE_VERSION    "v3.0-ODOMTEST"
#define FIRMWARE_DATE       "26.08.29"

/**
 * @brief  处理 OTA 相关串口命令
 * @param  cmd 行命令 (已去除行尾符)
 * @retval true 已处理 (调用方直接 return), false 不是 OTA 命令
 *
 * 命令:
 *   CMD_GET_VERSION -> 回 "VERSION:v3.0-ODOMTEST 26.08.29\r\n"
 *   CMD_OTA_START   -> 写升级标记, 回 "OK OTA reboot", 软复位 (不返回)
 */
bool OTA_HandleCommand(const char *cmd);

#ifdef __cplusplus
}
#endif

#endif /* __OTA_H */
