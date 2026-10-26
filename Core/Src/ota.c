/**
 ******************************************************************************
 * @file    ota.c
 * @brief   RK3588 OTA 升级支持 (App 侧) 实现
 ******************************************************************************
 * 注意: 写标记需要先擦除扇区3 (0x0800C000~0x0800FFFF, 16KB)。
 *       App 代码在扇区 4~11, 不受影响; CPU 从 Flash 取指时擦除会
 *       短暂停顿 (最长 ~1s), 期间串口中断无法响应, 因此先回复再擦除。
 ******************************************************************************
 */
#include "ota.h"
#include "uart_io.h"
#include "main.h"
#include <string.h>

bool OTA_HandleCommand(const char *cmd)
{
    /* RK 定时轮询版本 (30s 一次或升级触发前查询) */
    if (strcmp(cmd, "CMD_GET_VERSION") == 0) {
        UART_IO_Printf("VERSION:" FIRMWARE_VERSION " " FIRMWARE_DATE "\r\n");
        return true;
    }

    /* RK 判定版本过旧, 触发 OTA: 写标记 -> 软复位进 Bootloader */
    if (strcmp(cmd, "CMD_OTA_START") == 0) {
        /* 先回复并等发完, 再擦 Flash (擦除期间 CPU 停顿无法发数据) */
        UART_IO_Print("OK OTA reboot\r\n");
        UART_IO_FlushTx();

        HAL_FLASH_Unlock();

        FLASH_EraseInitTypeDef ei = {0};
        uint32_t page_err = 0;
        ei.TypeErase    = FLASH_TYPEERASE_SECTORS;
        ei.Banks        = FLASH_BANK_1;
        ei.Sector       = FLASH_SECTOR_3;
        ei.NbSectors    = 1;
        ei.VoltageRange = FLASH_VOLTAGE_RANGE_3;

        if (HAL_FLASHEx_Erase(&ei, &page_err) != HAL_OK ||
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                              OTA_FLAG_ADDR, OTA_FLAG_MAGIC) != HAL_OK) {
            HAL_FLASH_Lock();
            UART_IO_Print("ERR OTA flag write\r\n");
            return true;              /* 标记未写入, 不复位 */
        }

        HAL_FLASH_Lock();

        /* 标记已落盘, 等串口发完 + Bootloader 侧稳定后复位 */
        UART_IO_FlushTx();
        HAL_Delay(50);
        NVIC_SystemReset();           /* 不返回 */
    }

    return false;
}
