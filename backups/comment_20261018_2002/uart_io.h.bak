/**
  ******************************************************************************
  * @file    uart_io.h
  * @brief   串口输入输出驱动头文件
  *          - printf 重定向到 USART2
  *          - TX: 环形缓冲 + DMA 异步发送 (DMA1 Stream6 Ch4)
  *          - 中断接收环形缓冲区
  *
  * 硬件: USART2, PD5(TX) / PD6(RX), 460800 8N1, 无流控
  ******************************************************************************
  */
#ifndef __UART_IO_H__
#define __UART_IO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"
#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/* 配置                                                                       */
/* -------------------------------------------------------------------------- */
#define UART_RX_BUF_SIZE    256     /**< 接收环形缓冲区大小 */
#define UART_TX_BUF_SIZE    512     /**< printf 格式化暂存缓冲区大小 */
#define UART_TX_RING_SIZE   2048    /**< TX 环形缓冲区大小 (DMA 异步发送) */

/* -------------------------------------------------------------------------- */
/* API 函数                                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief 初始化串口 IO（使能 printf 重定向、启动 DMA/中断接收）
 */
void UART_IO_Init(void);

/**
 * @brief 发送单个字符（入 TX 环形缓冲, DMA 异步发出）
 * @note  缓冲满时有限等待后丢弃, 保证主循环不挂死; DMA 不可用时自动回退轮询
 * @param c 字符
 */
void UART_IO_PutChar(char c);

/**
 * @brief 发送字符串（非阻塞, DMA 异步发出）
 * @param str 字符串
 */
void UART_IO_Print(const char *str);

/**
 * @brief 发送原始字节（二进制安全, 含 \0, 非阻塞）
 * @param data 数据指针
 * @param len  字节数
 */
void UART_IO_SendBytes(const uint8_t *data, uint16_t len);

/**
 * @brief 格式化打印（类似 printf，阻塞）
 * @param fmt 格式字符串
 * @param ... 可变参数
 */
void UART_IO_Printf(const char *fmt, ...);

/**
 * @brief 从接收缓冲区读取一个字节（线程安全）
 * @param ch 输出字符指针
 * @return   true=读到数据, false=缓冲区空
 */
bool UART_IO_GetChar(uint8_t *ch);

/**
 * @brief 获取接收缓冲区中可用字节数
 * @return 可用字节数
 */
uint16_t UART_IO_Available(void);

/**
 * @brief 清空接收缓冲区
 */
void UART_IO_Flush(void);

/**
 * @brief 等待所有数据发送完成（环形缓冲排空 + DMA 完成, 带超时保护）
 * @note  仅需在进入低功耗模式前或确认数据已发出时调用
 */
void UART_IO_FlushTx(void);

/**
 * @brief 接收中断回调（在 USART2_IRQHandler 中调用）
 * @param byte 接收到的字节
 */
void UART_IO_RxCallback(uint8_t byte);

/**
 * @brief USART2 中断处理（在 stm32f4xx_it.c 中调用）
 */
void USART2_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __UART_IO_H__ */
