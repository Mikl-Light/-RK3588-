/**
  ******************************************************************************
  * @file    debug.c
  * @brief   统一调试日志系统实现
  ******************************************************************************
  */
#include "debug.h"
#include "uart_io.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* -------------------------------------------------------------------------- */
/* 全局调试等级                                                               */
/* -------------------------------------------------------------------------- */
static Debug_Level g_debug_level = DEBUG_LEVEL_INFO;

/* -------------------------------------------------------------------------- */
/* 初始化                                                                     */
/* -------------------------------------------------------------------------- */
void Debug_Init(Debug_Level level)
{
    g_debug_level = level;
    UART_IO_Printf("\r\n========================================\r\n");
    UART_IO_Printf("  DEBUG SYSTEM INITIALIZED\r\n");
    UART_IO_Printf("  Level: %s (%d)\r\n", Debug_LevelName(level), (int)level);
    UART_IO_Printf("  Levels: NONE(0) ERR(1) WARN(2) INFO(3) DEBUG(4) VERBOSE(5)\r\n");
    UART_IO_Printf("  CMD: DEBUG,<0-5>  to change level\r\n");
    UART_IO_Printf("========================================\r\n\r\n");
}

/* -------------------------------------------------------------------------- */
/* 等级管理                                                                   */
/* -------------------------------------------------------------------------- */
void Debug_SetLevel(Debug_Level level)
{
    g_debug_level = level;
    /* 无论当前等级如何，等级变更消息强制输出 */
    UART_IO_Printf("\r\n[DEBUG] Level changed to: %s (%d)\r\n",
                   Debug_LevelName(level), (int)level);
}

Debug_Level Debug_GetLevel(void)
{
    return g_debug_level;
}

const char *Debug_LevelName(Debug_Level level)
{
    switch (level) {
        case DEBUG_LEVEL_NONE:    return "NONE";
        case DEBUG_LEVEL_ERROR:   return "ERROR";
        case DEBUG_LEVEL_WARN:    return "WARN";
        case DEBUG_LEVEL_INFO:    return "INFO";
        case DEBUG_LEVEL_DEBUG:   return "DEBUG";
        case DEBUG_LEVEL_VERBOSE: return "VERBOSE";
        default:                  return "???";
    }
}

/* -------------------------------------------------------------------------- */
/* 核心日志输出                                                               */
/* -------------------------------------------------------------------------- */
void Debug_Log(Debug_Level level, const char *tag, const char *type_str,
               const char *fmt, ...)
{
    if (level > g_debug_level) return;
    if (g_debug_level == DEBUG_LEVEL_NONE) return;

    /* 格式: [12345678] TAG TYP message\r\n */
    uint32_t tick = HAL_GetTick();

    /* 先输出前缀（不用 printf 避免两次格式化） */
    UART_IO_Printf("[%8lu] %-8s %-3s ", (unsigned long)tick, tag, type_str);

    /* 输出消息体 */
    va_list args;
    va_start(args, fmt);
    static char msg_buf[256];
    vsnprintf(msg_buf, sizeof(msg_buf), fmt, args);
    va_end(args);
    UART_IO_Print(msg_buf);
    UART_IO_Print("\r\n");
}

/* -------------------------------------------------------------------------- */
/* 系统信息摘要                                                               */
/* -------------------------------------------------------------------------- */
void Debug_PrintSysInfo(void)
{
    UART_IO_Printf("\r\n========== SYSTEM INFO ==========\r\n");
    UART_IO_Printf("  Tick:      %lu ms\r\n", (unsigned long)HAL_GetTick());
    UART_IO_Printf("  SysClock:  %lu Hz\r\n", (unsigned long)HAL_RCC_GetSysClockFreq());
    UART_IO_Printf("  HCLK:      %lu Hz\r\n", (unsigned long)HAL_RCC_GetHCLKFreq());
    UART_IO_Printf("  PCLK1:     %lu Hz\r\n", (unsigned long)HAL_RCC_GetPCLK1Freq());
    UART_IO_Printf("  PCLK2:     %lu Hz\r\n", (unsigned long)HAL_RCC_GetPCLK2Freq());
    UART_IO_Printf("  Debug Lvl: %s (%d)\r\n", Debug_LevelName(g_debug_level), (int)g_debug_level);
    UART_IO_Printf("==================================\r\n");
}

/* -------------------------------------------------------------------------- */
/* 解析调试指令                                                               */
/* -------------------------------------------------------------------------- */
bool Debug_ParseCommand(const char *cmd)
{
    /* 格式: DEBUG,<0-5> 或 DEBUG */
    if (strncmp(cmd, "DEBUG", 5) != 0) {
        return false;
    }

    /* 仅 "DEBUG" → 打印当前等级和系统信息 */
    if (cmd[5] == '\0' || cmd[5] == '\r' || cmd[5] == ' ') {
        Debug_PrintSysInfo();
        return true;
    }

    /* DEBUG,<level> */
    if (cmd[5] == ',') {
        int level = -1;
        if (sscanf(cmd + 6, "%d", &level) == 1) {
            if (level >= DEBUG_LEVEL_NONE && level <= DEBUG_LEVEL_VERBOSE) {
                Debug_SetLevel((Debug_Level)level);
            } else {
                UART_IO_Printf("ERR DEBUG level must be 0-5\r\n");
            }
        } else {
            /* 尝试匹配名称 */
            const char *name = cmd + 6;
            while (*name == ' ') name++;
            if (strncmp(name, "NONE", 4) == 0)    Debug_SetLevel(DEBUG_LEVEL_NONE);
            else if (strncmp(name, "ERROR", 5) == 0) Debug_SetLevel(DEBUG_LEVEL_ERROR);
            else if (strncmp(name, "WARN", 4) == 0)  Debug_SetLevel(DEBUG_LEVEL_WARN);
            else if (strncmp(name, "INFO", 4) == 0)  Debug_SetLevel(DEBUG_LEVEL_INFO);
            else if (strncmp(name, "DEBUG", 5) == 0) Debug_SetLevel(DEBUG_LEVEL_DEBUG);
            else if (strncmp(name, "VERBOSE", 7) == 0) Debug_SetLevel(DEBUG_LEVEL_VERBOSE);
            else UART_IO_Printf("ERR Unknown level. Use: 0-5 or NONE/ERROR/WARN/INFO/DEBUG/VERBOSE\r\n");
        }
        return true;
    }

    return false;
}

/* -------------------------------------------------------------------------- */
/* LED 故障指示（无串口时的最后手段）                                          */
/* -------------------------------------------------------------------------- */
void Debug_BlinkError(uint8_t count, uint16_t period_ms)
{
    /* 强制使用板载 LED (假设 PC13 或其他 GPIO) */
    /* 注意: 此处需要根据实际硬件板级 LED 引脚调整 */
    /* 如果未配置 LED GPIO，此函数无效 */

    /* 通用实现：使用 systick 忙等（调试用，不推荐生产使用） */
    uint16_t half = period_ms / 2;
    uint8_t i = 0;

    while (count == 0 || i < count) {
        /* 这里无法操作 GPIO 因为需要知道具体 LED 引脚 */
        /* 使用 HAL_GPIO_TogglePin 需要引脚和端口定义 */
        HAL_Delay(half);
        HAL_Delay(half);
        if (count > 0) i++;
    }
}
