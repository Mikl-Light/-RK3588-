/**
  ******************************************************************************
  * @file    uart_io.c
  * @brief   串口输入输出驱动实现
  *          - printf 重定向 via _write()
  *          - TX: 环形缓冲 + DMA1 Stream6 Ch4 异步发送 (460800 8N1)
  *          - 中断接收环形缓冲区（无锁 SPSC，关中断保护临界区）
  *          - 格式化打印
  ******************************************************************************
  */
#include "uart_io.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/* 环形接收缓冲区                                                             */
/* -------------------------------------------------------------------------- */
static uint8_t  rx_buf[UART_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;   /**< ISR 写入 */
static volatile uint16_t rx_tail = 0;   /**< 主循环写入 */
static volatile bool rx_full = false;   /**< 满标志，解决"空/满"歧义 */

/* -------------------------------------------------------------------------- */
/* 发送环形缓冲区 + DMA 异步发送                                               */
/*   USART2_TX = DMA1 Stream6, Channel4 (STM32F407 RM0090)                    */
/*   生产者: 主循环 (PutChar/Print/Printf) 只写 tx_head                        */
/*   消费者: DMA 完成中断 (DMA1_Stream6_IRQHandler) 只推进 tx_tail             */
/*   无锁 SPSC; 缓冲满时有限等待后丢弃并计数, 保证控制循环不挂死               */
/* -------------------------------------------------------------------------- */
#define TX_BLOCK_WAIT_MS     20U    /**< 缓冲满时每字节最多等待 (ms) */
#define TX_FLUSH_TIMEOUT_MS 200U    /**< FlushTx 等待 DMA 排空超时 (ms) */

static uint8_t  tx_ring[UART_TX_RING_SIZE];
static volatile uint16_t tx_head = 0;        /**< 生产者写入位置 */
static volatile uint16_t tx_tail = 0;        /**< 已由 DMA 发出的位置 */
static volatile bool     tx_dma_busy = false;/**< DMA 正在传输一段 */
static volatile uint16_t tx_dma_len = 0;     /**< 当前 DMA 段长度 */
static volatile uint32_t tx_drop_count = 0;  /**< 满时丢弃字节计数 (诊断) */
static bool              tx_dma_ok = false;  /**< DMA 初始化成功标志 */
static DMA_HandleTypeDef hdma_uart2_tx;

static void UART_IO_TxPump(void);
static void UART_TxDmaCplt(DMA_HandleTypeDef *hdma);
static void UART_TxDmaError(DMA_HandleTypeDef *hdma);

/* -------------------------------------------------------------------------- */
/* 初始化串口 IO                                                              */
/* -------------------------------------------------------------------------- */
void UART_IO_Init(void)
{
    rx_head = 0;
    rx_tail = 0;
    rx_full = false;
    tx_head = 0;
    tx_tail = 0;
    tx_dma_busy = false;
    tx_drop_count = 0;

    /* 确保 UART 发送器和接收器已使能（HAL 有时不会显式置位 TE/RE） */
    huart2.Instance->CR1 |= (USART_CR1_TE | USART_CR1_RE);

    /* ================= TX DMA: DMA1 Stream6 Ch4 (USART2_TX) ================= */
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_uart2_tx.Instance                 = DMA1_Stream6;
    hdma_uart2_tx.Init.Channel             = DMA_CHANNEL_4;
    hdma_uart2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_uart2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_uart2_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_uart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_uart2_tx.Init.Mode                = DMA_NORMAL;
    hdma_uart2_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_uart2_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart2_tx) == HAL_OK) {
        hdma_uart2_tx.XferCpltCallback  = UART_TxDmaCplt;
        hdma_uart2_tx.XferErrorCallback = UART_TxDmaError;
        HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

        /* 使能 USART2 DMA 发送请求 (DMAT): 此后 TX 数据由 DMA 搬运 */
        huart2.Instance->CR3 |= USART_CR3_DMAT;

        tx_dma_ok = true;
    }
    /* DMA 初始化失败 → tx_dma_ok=false, PutChar 自动回退为轮询发送 */

    /* 配置 USART2 NVIC 中断 */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    /* 使能 USART2 接收中断 */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

/* -------------------------------------------------------------------------- */
/* 发送单个字符 (入 TX 环形缓冲, DMA 异步发出)                                */
/* -------------------------------------------------------------------------- */
void UART_IO_PutChar(char c)
{
    /* DMA 不可用 → 回退: 逐字节轮询发送 (与旧行为一致, 保证功能可用) */
    if (!tx_dma_ok) {
        while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == RESET);
        huart2.Instance->DR = (uint8_t)c;
        return;
    }

    /* 环形缓冲满: 有限等待 DMA 排空 (防 DMA 异常时挂死主循环, 20ms 上限) */
    uint32_t wait_start = HAL_GetTick();
    while (((tx_head + 1) % UART_TX_RING_SIZE) == tx_tail) {
        if (HAL_GetTick() - wait_start >= TX_BLOCK_WAIT_MS) {
            tx_drop_count++;      /* 丢弃该字节, 保持控制循环稳定 */
            return;
        }
    }

    tx_ring[tx_head] = (uint8_t)c;
    tx_head = (uint16_t)((tx_head + 1) % UART_TX_RING_SIZE);

    UART_IO_TxPump();   /* DMA 空闲则立即启动 (已在传则直接返回) */
}

/* -------------------------------------------------------------------------- */
/* 等待所有数据发送完成（用于需要确认数据已发出的场景）                        */
/* -------------------------------------------------------------------------- */
void UART_IO_FlushTx(void)
{
    /* 轮询回退模式: 等待最后一个字节移出 */
    if (!tx_dma_ok) {
        while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET);
        return;
    }

    /* DMA 模式: 等待环形缓冲排空且当前段传输完成 */
    uint32_t wait_start = HAL_GetTick();
    while (tx_head != tx_tail || tx_dma_busy) {
        if (HAL_GetTick() - wait_start >= TX_FLUSH_TIMEOUT_MS) break;
    }
}

/* -------------------------------------------------------------------------- */
/* 发送字符串 (入 TX 环形缓冲, DMA 异步发出, 非阻塞)                           */
/* -------------------------------------------------------------------------- */
void UART_IO_Print(const char *str)
{
    while (*str) {
        UART_IO_PutChar(*str++);
    }
}

/* -------------------------------------------------------------------------- */
/* 发送原始字节 (二进制安全: 逐字节入环, \0 不会被截断)                       */
/* -------------------------------------------------------------------------- */
void UART_IO_SendBytes(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        UART_IO_PutChar((char)data[i]);
    }
}

/* -------------------------------------------------------------------------- */
/* 格式化打印                                                                 */
/* -------------------------------------------------------------------------- */
void UART_IO_Printf(const char *fmt, ...)
{
    static char tx_buf[UART_TX_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    vsnprintf(tx_buf, sizeof(tx_buf), fmt, args);
    va_end(args);
    UART_IO_Print(tx_buf);
}

/* -------------------------------------------------------------------------- */
/* TX DMA 泵: 从环形缓冲取一段连续数据交给 DMA, 处理回绕与多段拼接            */
/*   调用点: PutChar (主循环) / DMA 完成中断 (回绕续传)                        */
/* -------------------------------------------------------------------------- */
static void UART_IO_TxPump(void)
{
    if (!tx_dma_ok || tx_dma_busy) return;

    uint16_t head = tx_head;
    uint16_t tail = tx_tail;
    if (head == tail) return;

    /* 连续段长度: 不能越过缓冲区末尾 (回绕部分等 DMA 完成后由中断续传) */
    uint16_t avail = (uint16_t)((head - tail + UART_TX_RING_SIZE) % UART_TX_RING_SIZE);
    uint16_t chunk = (uint16_t)(UART_TX_RING_SIZE - tail);
    if (chunk > avail) chunk = avail;

    tx_dma_len  = chunk;
    tx_dma_busy = true;

    if (HAL_DMA_Start_IT(&hdma_uart2_tx,
                         (uint32_t)&tx_ring[tail],
                         (uint32_t)&huart2.Instance->DR,
                         chunk) != HAL_OK) {
        /* 启动失败 (理论不应发生): 丢弃该段, 跳过继续 */
        tx_dma_busy = false;
        tx_tail = (uint16_t)((tail + chunk) % UART_TX_RING_SIZE);
        UART_IO_TxPump();
    }
}

/* DMA 传输完成回调 (ISR 上下文) — 推进 tail 并续传 (含回绕段) */
static void UART_TxDmaCplt(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    tx_tail = (uint16_t)((tx_tail + tx_dma_len) % UART_TX_RING_SIZE);
    tx_dma_busy = false;
    UART_IO_TxPump();
}

/* DMA 传输错误回调 (ISR 上下文) — 放弃当前段, 复位后继续 */
static void UART_TxDmaError(DMA_HandleTypeDef *hdma)
{
    __HAL_DMA_DISABLE(hdma);
    tx_tail = (uint16_t)((tx_tail + tx_dma_len) % UART_TX_RING_SIZE);
    tx_dma_busy = false;
    UART_IO_TxPump();
}

/* -------------------------------------------------------------------------- */
/* DMA1 Stream6 中断服务函数 (USART2_TX)                                      */
/* -------------------------------------------------------------------------- */
void DMA1_Stream6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_uart2_tx);
}

/* -------------------------------------------------------------------------- */
/* 从环形缓冲区读取一个字节（关中断保护临界区）                                */
/* -------------------------------------------------------------------------- */
bool UART_IO_GetChar(uint8_t *ch)
{
    bool got = false;

    __disable_irq();  /* 临界区：保护读-改-写 rx_tail 不被 ISR 打断 */
    if (rx_head != rx_tail || rx_full) {
        *ch = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % UART_RX_BUF_SIZE;
        rx_full = false;  /* 读取后一定不满了 */
        got = true;
    }
    __enable_irq();

    return got;
}

/* -------------------------------------------------------------------------- */
/* 获取可用字节数                                                             */
/* -------------------------------------------------------------------------- */
uint16_t UART_IO_Available(void)
{
    uint16_t count;
    __disable_irq();
    if (rx_full) {
        count = UART_RX_BUF_SIZE;
    } else {
        count = (rx_head - rx_tail + UART_RX_BUF_SIZE) % UART_RX_BUF_SIZE;
    }
    __enable_irq();
    return count;
}

/* -------------------------------------------------------------------------- */
/* 清空接收缓冲区                                                             */
/* -------------------------------------------------------------------------- */
void UART_IO_Flush(void)
{
    __disable_irq();
    rx_head = 0;
    rx_tail = 0;
    rx_full = false;
    __enable_irq();
}

/* -------------------------------------------------------------------------- */
/* 接收中断回调（仅在 ISR 上下文调用）                                        */
/* -------------------------------------------------------------------------- */
void UART_IO_RxCallback(uint8_t byte)
{
    if (rx_full) {
        /* 缓冲区满，丢弃最早的数据以容纳新数据 */
        rx_tail = (rx_tail + 1) % UART_RX_BUF_SIZE;
    }

    rx_buf[rx_head] = byte;
    rx_head = (rx_head + 1) % UART_RX_BUF_SIZE;

    /* 检查是否变满 */
    if (rx_head == rx_tail) {
        rx_full = true;
    }
}

/* -------------------------------------------------------------------------- */
/* USART2 中断服务函数                                                        */
/* -------------------------------------------------------------------------- */
void USART2_IRQHandler(void)
{
    /* 一次性读取 SR，避免多次读取时标志位变化导致漏处理 */
    uint32_t sr = huart2.Instance->SR;

    /* 接收数据就绪 */
    if (sr & USART_SR_RXNE) {
        uint8_t byte = (uint8_t)(huart2.Instance->DR & 0xFF);
        UART_IO_RxCallback(byte);
    }

    /* 溢出错误 / 帧错误 / 噪声错误 / 校验错误 → 读 DR 清除 */
    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        (void)huart2.Instance->DR;   /* 读 DR 即可清除 FE/NE/PE/ORE */
    }

    /* 如果还有 RXNE（错误帧也会置 RXNE），再读一次 */
    if (huart2.Instance->SR & USART_SR_RXNE) {
        (void)huart2.Instance->DR;
    }
}

/* -------------------------------------------------------------------------- */
/* printf 重定向 (_write)                                                     */
/* -------------------------------------------------------------------------- */
#ifdef __GNUC__
int _write(int file, char *ptr, int len)
{
    (void)file;
    for (int i = 0; i < len; i++) {
        if (ptr[i] == '\n') UART_IO_PutChar('\r');
        UART_IO_PutChar(ptr[i]);
    }
    return len;
}
#endif

/* -------------------------------------------------------------------------- */
/* __io_putchar 重定向（兼容某些 newlib-nano 配置）                           */
/* -------------------------------------------------------------------------- */
int __io_putchar(int ch)
{
    if (ch == '\n') UART_IO_PutChar('\r');
    UART_IO_PutChar((char)ch);
    return ch;
}
