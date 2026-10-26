/**
  ******************************************************************************
  * @file    debug.h
  * @brief   统一调试日志系统
  *          - 5 级日志等级：NONE / ERROR / WARN / INFO / DEBUG / VERBOSE
  *          - 带时间戳 (ms) + 模块名 的格式化输出
  *          - 运行时可通过 "DEBUG,<level>" 指令动态切换等级
  *          - 支持断言宏，失败时自动打印文件/行号
  ******************************************************************************
  */
#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/* 日志等级                                                                   */
/* -------------------------------------------------------------------------- */
typedef enum {
    DEBUG_LEVEL_NONE    = 0,    /**< 关闭所有日志 */
    DEBUG_LEVEL_ERROR   = 1,    /**< 仅致命错误 */
    DEBUG_LEVEL_WARN    = 2,    /**< 错误 + 警告 */
    DEBUG_LEVEL_INFO    = 3,    /**< 错误 + 警告 + 关键信息（默认） */
    DEBUG_LEVEL_DEBUG   = 4,    /**< 详细调试信息 */
    DEBUG_LEVEL_VERBOSE = 5,    /**< 极详细（含每周期数据） */
} Debug_Level;

/* -------------------------------------------------------------------------- */
/* 日志宏（使用模块名 TAG 便于过滤）                                          */
/* -------------------------------------------------------------------------- */

/** 错误：致命故障，系统可能无法继续运行 */
#define DBG_ERROR(tag, fmt, ...) \
    Debug_Log(DEBUG_LEVEL_ERROR, tag, "ERR", fmt, ##__VA_ARGS__)

/** 警告：非预期但可恢复的情况 */
#define DBG_WARN(tag, fmt, ...) \
    Debug_Log(DEBUG_LEVEL_WARN, tag, "WRN", fmt, ##__VA_ARGS__)

/** 信息：关键状态变化、初始化完成等 */
#define DBG_INFO(tag, fmt, ...) \
    Debug_Log(DEBUG_LEVEL_INFO, tag, "INF", fmt, ##__VA_ARGS__)

/** 调试：函数调用、参数值等 */
#define DBG_DEBUG(tag, fmt, ...) \
    Debug_Log(DEBUG_LEVEL_DEBUG, tag, "DBG", fmt, ##__VA_ARGS__)

/** 详细：每周期的高频数据 */
#define DBG_VERBOSE(tag, fmt, ...) \
    Debug_Log(DEBUG_LEVEL_VERBOSE, tag, "VRB", fmt, ##__VA_ARGS__)

/* -------------------------------------------------------------------------- */
/* 条件日志宏（减少不必要的字符串格式化）                                    */
/* -------------------------------------------------------------------------- */

#define DBG_ERROR_IF(cond, tag, fmt, ...)   do { if (cond) DBG_ERROR(tag, fmt, ##__VA_ARGS__); } while(0)
#define DBG_WARN_IF(cond, tag, fmt, ...)    do { if (cond) DBG_WARN(tag, fmt, ##__VA_ARGS__); } while(0)
#define DBG_INFO_IF(cond, tag, fmt, ...)    do { if (cond) DBG_INFO(tag, fmt, ##__VA_ARGS__); } while(0)

/* -------------------------------------------------------------------------- */
/* 断言宏（失败时自动打印并进入 Error_Handler）                               */
/* -------------------------------------------------------------------------- */
#define DBG_ASSERT(cond, tag, msg) \
    do { \
        if (!(cond)) { \
            DBG_ERROR(tag, "ASSERT FAILED: %s | %s:%d", msg, __FILE__, __LINE__); \
            Error_Handler(); \
        } \
    } while(0)

/** 带返回值的断言（用于初始化函数中，失败返回 err_ret） */
#define DBG_ASSERT_RET(cond, tag, msg, err_ret) \
    do { \
        if (!(cond)) { \
            DBG_ERROR(tag, "ASSERT FAILED: %s | %s:%d", msg, __FILE__, __LINE__); \
            return (err_ret); \
        } \
    } while(0)

/* -------------------------------------------------------------------------- */
/* 模块 TAG 定义（统一管理，避免散落字符串）                                  */
/* -------------------------------------------------------------------------- */
#define TAG_MAIN        "MAIN"
#define TAG_SYSTEM      "SYSCLK"
#define TAG_GPIO        "GPIO"
#define TAG_USART       "USART"
#define TAG_I2C         "I2C"
#define TAG_TIM         "TIM"
#define TAG_MOTOR       "MOTOR"
#define TAG_MPU         "MPU9250"
#define TAG_AHRS        "AHRS"
#define TAG_CHASSIS     "CHASSIS"
#define TAG_PID         "PID"
#define TAG_CMD         "CMD"

/* -------------------------------------------------------------------------- */
/* API 函数                                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief 初始化调试系统
 * @param level 初始日志等级（默认 DEBUG_LEVEL_INFO）
 */
void Debug_Init(Debug_Level level);

/**
 * @brief 设置全局日志等级
 * @param level 新的日志等级
 */
void Debug_SetLevel(Debug_Level level);

/**
 * @brief 获取当前日志等级
 * @return 当前日志等级
 */
Debug_Level Debug_GetLevel(void);

/**
 * @brief 获取日志等级名称字符串
 * @param level 等级
 * @return 名称 (如 "INFO", "DEBUG")
 */
const char *Debug_LevelName(Debug_Level level);

/**
 * @brief 格式化日志输出（核心函数，由宏调用）
 * @param level     本条日志等级
 * @param tag       模块标签（4~8 字符）
 * @param type_str  类型字符串（"ERR"/"WRN"/"INF"/"DBG"/"VRB"）
 * @param fmt       格式化字符串
 * @param ...       可变参数
 */
void Debug_Log(Debug_Level level, const char *tag, const char *type_str,
               const char *fmt, ...);

/**
 * @brief 打印系统信息摘要（各模块状态）
 */
void Debug_PrintSysInfo(void);

/**
 * @brief 解析调试指令 (如 "DEBUG,4" → 设置等级为 DEBUG)
 * @param cmd 完整指令字符串（不含换行）
 * @return true=成功解析, false=不是调试指令
 */
bool Debug_ParseCommand(const char *cmd);

/**
 * @brief 阻塞闪烁 LED 进行故障指示（调试用）
 * @param count 闪烁次数（0=无限循环）
 * @param period_ms 闪烁周期 (ms)
 * @note  在没有串口可用时的最后手段
 */
void Debug_BlinkError(uint8_t count, uint16_t period_ms);

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_H__ */
