/**
 ******************************************************************************
 * @file    bootloader.h
 * @brief   STM32F407 串口 Bootloader：Flash 写入 + 应用跳转 + 升级协议
 ******************************************************************************
 * Flash 分区 (F407ZGT6, 1MB, 非均匀扇区):
 *   扇区 0~3  (0x08000000 ~ 0x0800FFFF, 各16KB)  -> Bootloader (64KB)
 *   扇区 4    (0x08010000 ~ 0x0801FFFF, 64KB)    -> App 起始
 *   扇区 5~11 (0x08020000 ~ 0x080FFFFF, 各128KB) -> App 续
 *
 * 触发方式 (RK3588 OTA 方案):
 *   App 收到 CMD_OTA_START 后在 BL_FLAG_ADDR 写入标记并软复位,
 *   Bootloader 检测到标记即进入升级模式; RUN 成功后清除标记再跳转。
 *   传输中途断电标记仍在, 重启后继续等待升级 (不会误跳半成品固件)。
 *
 * 升级协议 (USART2 460800 8N1, 行命令, \r 或 \n 结尾):
 *   UPDATE            -> 握手, 回 "OK BL ready"
 *   INFO              -> 查询版本与 App 状态
 *   ERASE             -> 擦除 App 区 (扇区 4~11)
 *   ADDR,<8位十六进制> -> 设置写入地址 (如 ADDR,08010000)
 *   DATA,<十六进制串>  -> 写入数据 (每行最多 64 字节, 如 DATA,8000f000....)
 *                         写入时自动按扇区擦除
 *   RUN               -> 校验并跳转到 App (成功后清除升级标记)
 ******************************************************************************
 */
#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ---------------- 分区配置 ---------------- */
#define BL_APP_ADDR         0x08010000U   /* App 起始地址 (扇区4) */
#define BL_APP_MAX_SIZE     (960U * 1024U)
#define BL_FLASH_END        0x08100000U

/* ---------------- 升级标记 ---------------- */
/* 扇区3 末页 (0x0800FC00), bootloader 镜像只占扇区0 (~9KB), 互不影响 */
#define BL_FLAG_ADDR        0x0800FC00U
#define BL_FLAG_MAGIC       0x31544F41U   /* "AOT1" 小端 */

#define BL_VERSION          "V1.2.1"
#define BL_BUILD_DATE       __DATE__

/* ---------------- 对外接口 ---------------- */
uint8_t  BL_AppValid(void);               /* 检查 App 向量是否有效 */
void     BL_JumpToApp(void);              /* 关外设 -> 设MSP -> 跳转 */
uint8_t  BL_UpdateFlagSet(void);          /* 升级标记是否已置位 */
void     BL_UpdateFlagClear(void);        /* 擦除升级标记 (RUN 成功后调用) */
void     BL_UpdateLoop(void);             /* 升级主循环 (RUN 后跳转) */
uint8_t  BL_ButtonPressed(void);          /* PA15 按键是否按住 (上电强制进升级模式) */

/* Flash 操作 */
int      BL_FlashEraseApp(void);          /* 擦除整个 App 区 */
int      BL_FlashWrite(uint32_t addr, const uint8_t *data, uint32_t len);
int      BL_FlushBuffer(void);            /* 冲刷未满 4 字节的写缓冲 */

/* 串口辅助 (阻塞发送) */
void     BL_SendStr(const char *s);
void     BL_SendHex32(uint32_t v);

#ifdef __cplusplus
}
#endif

#endif /* __BOOTLOADER_H */
