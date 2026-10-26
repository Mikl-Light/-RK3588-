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

/* ============ 本文件在做什么（一图看懂） ============
 * 串口 = 车和外界的"嘴"：接收上位机发来的命令，把数据发回上位机。
 *
 * 【接收方向】上位机发字节来 → 芯片每收到一个字节就"举手"打断CPU(中断)
 *             → 中断把字节塞进"接收仓库"(rx_buf) → 主循环慢慢取出来解析
 * 【发送方向】主循环把要发的字节塞进"发送仓库"(tx_ring)
 *             → 请一个叫 DMA 的"搬运工"把它搬出去(CPU不用管)
 *
 * 三个角色（你之前已经建立的模型）：
 *   仓库     = rx_buf / tx_ring（被动，放着就放着）
 *   放货的人 = 主循环 / 中断
 *   搬运工   = DMA（发送时才有，主动搬，但不叫它它不动）
 *
 * 关键别名（收发两侧通用）：
 *   head（进货口）= 往里放的那一端
 *   tail（出货口）= 往外拿的那一端
 */
#include "uart_io.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/* 环形接收缓冲区                                                             */
/* -------------------------------------------------------------------------- */
/* 【什么是环形缓冲区】一块内存围成一圈，两头分别有"进货口"和"出货口"。
 * 装满了就丢最老的（对控制命令来说，新命令比旧命令值钱） */
static uint8_t  rx_buf[UART_RX_BUF_SIZE];     /* 接收仓库本体：256 个格子 */
static volatile uint16_t rx_head = 0;   /**< ISR 写入 —— 进货口（中断往里放） */
static volatile uint16_t rx_tail = 0;   /**< 主循环写入 —— 出货口（主循环往外取） */
static volatile bool rx_full = false;   /**< 满标志，解决"空/满"歧义（head==tail 时分不清） */

/* 【volatile 是什么】告诉编译器"这变量可能被别的地方(中断)改，别自作聪明优化掉，
 * 每次都要真的去内存里读一遍"。因为中断和主循环是"两个人"，编译器只看主循环会误判"它不变"。 */

/* -------------------------------------------------------------------------- */
/* 发送环形缓冲区 + DMA 异步发送                                               */
/*   USART2_TX = DMA1 Stream6, Channel4 (STM32F407 RM0090)                    */
/*   生产者: 主循环 (PutChar/Print/Printf) 只写 tx_head                        */
/*   消费者: DMA 完成中断 (DMA1_Stream6_IRQHandler) 只推进 tx_tail             */
/*   无锁 SPSC; 缓冲满时有限等待后丢弃并计数, 保证控制循环不挂死               */
/* -------------------------------------------------------------------------- */
#define TX_BLOCK_WAIT_MS     20U    /**< 缓冲满时每字节最多等待 (ms)，等不到就丢 */
#define TX_FLUSH_TIMEOUT_MS 200U    /**< FlushTx 等待 DMA 排空超时 (ms) */

static uint8_t  tx_ring[UART_TX_RING_SIZE];   /* 发送仓库本体：2048 个格子 */
static volatile uint16_t tx_head = 0;        /**< 生产者写入位置（主循环写）——进货口 */
static volatile uint16_t tx_tail = 0;        /**< 已由 DMA 发出的位置（中断写）——出货口 */
static volatile bool     tx_dma_busy = false;/**< DMA 正在传输一段（搬运工忙不忙的牌子） */
static volatile uint16_t tx_dma_len = 0;     /**< 当前 DMA 段长度（出发前记的"这趟搬几个"） */
static volatile uint32_t tx_drop_count = 0;  /**< 满时丢弃字节计数 (诊断，目前没被读取) */
static bool              tx_dma_ok = false;  /**< DMA 初始化成功标志（失败则退回老式发送） */
static DMA_HandleTypeDef hdma_uart2_tx;      /* 搬运工 DMA 的"档案"（存它的配置信息） */

static void UART_IO_TxPump(void);
static void UART_TxDmaCplt(DMA_HandleTypeDef *hdma);
static void UART_TxDmaError(DMA_HandleTypeDef *hdma);

/* -------------------------------------------------------------------------- */
/* 初始化串口 IO                                                              */
/* -------------------------------------------------------------------------- */
/* 【中文名】初始化串口：清空缓冲区指针、配置发送用的"DMA搬运工"、打开接收中断 */
void UART_IO_Init(void)
{
    /* ① 把所有指针和标志清零。必须在开中断之前做，否则中断可能往还没清好的状态里写 */
    rx_head = 0;
    rx_tail = 0;
    rx_full = false;
    tx_head = 0;
    tx_tail = 0;
    tx_dma_busy = false;
    tx_drop_count = 0;

    /* ② 确保串口的"发送器 TE"和"接收器 RE"都打开（有的情况 HAL 不会自动开） */
    huart2.Instance->CR1 |= (USART_CR1_TE | USART_CR1_RE);

    /* ================= TX DMA: DMA1 Stream6 Ch4 (USART2_TX) ================= */
    /* ③ 给 DMA 开时钟（芯片里的外设要先通电才能用） */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* ④ 填搬运工 DMA 的档案：告诉它"从内存搬到串口、逐个字节搬、搬完通知我" */
    hdma_uart2_tx.Instance                 = DMA1_Stream6;      /* 用 DMA1 的第6路 */
    hdma_uart2_tx.Init.Channel             = DMA_CHANNEL_4;     /* 对应串口2的通道4 */
    hdma_uart2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH; /* 方向：内存→外设(串口) */
    hdma_uart2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;  /* 外设地址不变（串口就一个口） */
    hdma_uart2_tx.Init.MemInc              = DMA_MINC_ENABLE;   /* 内存地址逐个后移 */
    hdma_uart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; /* 每次搬1字节 */
    hdma_uart2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_uart2_tx.Init.Mode                = DMA_NORMAL;        /* 普通模式（搬完就停） */
    hdma_uart2_tx.Init.Priority            = DMA_PRIORITY_HIGH; /* 高优先级 */
    hdma_uart2_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE; /* 不用FIFO */
    if (HAL_DMA_Init(&hdma_uart2_tx) == HAL_OK) {
        /* ⑤ 初始化成功 → 注册"搬完时叫我"和"出错时叫我"两个回调函数 */
        hdma_uart2_tx.XferCpltCallback  = UART_TxDmaCplt;   /* 搬完一块 → 调这个 */
        hdma_uart2_tx.XferErrorCallback = UART_TxDmaError;  /* 出错 → 调这个 */
        HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 1, 0);      /* 设中断优先级为1 */
        HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);              /* 打开DMA中断 */

        /* ⑥ 使能串口的"DMA发送请求"(DMAT)：从此发送数据不用CPU管，DMA自动搬 */
        huart2.Instance->CR3 |= USART_CR3_DMAT;

        tx_dma_ok = true;   /* 记个"搬运工已就位"的标记 */
    }
    /* ⑦ 如果 DMA 初始化失败 → tx_dma_ok 保持 false，
     *    后面 PutChar 会自动退回"老式的 CPU 逐个字节发"，功能照常能用（降级保命） */

    /* ⑧ 配置接收中断：串口每收到一个字节就打断 CPU 一次 */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 1);   /* 优先级0（最高） */
    HAL_NVIC_EnableIRQ(USART2_IRQn);           /* 打开串口2中断 */

    /* ⑨ 使能"收到字节"这个具体的中断类型（RXNE = 接收数据就绪） */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

/* -------------------------------------------------------------------------- */
/* 发送单个字符 (入 TX 环形缓冲, DMA 异步发出)                                */
/* -------------------------------------------------------------------------- */
/* 【中文名】发送一个字符：往发送仓库里放一个字节，然后喊搬运工来搬（放不进去就丢） */
void UART_IO_PutChar(char c)
{
    /* ① DMA 搬运工没就位（初始化失败）→ 退回老式发法：
     *    死等"发送寄存器空了"，然后把字节写进串口数据寄存器（写进去=送出去） */
    if (!tx_dma_ok) {
        while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == RESET);  /* 干等，等发送口空出来 */
        huart2.Instance->DR = (uint8_t)c;   /* 写数据寄存器 = 把这个字节送出去 */
        return;
    }

    /* ② 发送仓库满了（进货口再往前一格就撞上出货口）→ 有限等待最多20ms，
     *    等不到就把这个字节丢掉。为什么敢丢？因为主循环每10ms要跑一遍PID，
     *    不能为了发个日志把车卡住。这是"宁可串口丢一行字，也不能丢控制周期" */
    uint32_t wait_start = HAL_GetTick();   /* 记下开始等的时间 */
    while (((tx_head + 1) % UART_TX_RING_SIZE) == tx_tail) {   /* 满：head+1 == tail */
        if (HAL_GetTick() - wait_start >= TX_BLOCK_WAIT_MS) {  /* 等超20ms */
            tx_drop_count++;      /* 丢弃计数器+1（诊断用） */
            return;               /* 直接放弃这个字节 */
        }
    }

    /* ③ 没满 → 把字节放进进货口，进货口往后挪一格 */
    tx_ring[tx_head] = (uint8_t)c;
    tx_head = (uint16_t)((tx_head + 1) % UART_TX_RING_SIZE);

    /* ④ 喊一声搬运工：有货了，闲的话就来搬 */
    UART_IO_TxPump();
}

/* -------------------------------------------------------------------------- */
/* 等待所有数据发送完成（用于需要确认数据已发出的场景）                        */
/* -------------------------------------------------------------------------- */
/* 【中文名】等发送完：卡在这里，直到仓库空、DMA也搬完。只在"必须确认话已说出去"时用 */
void UART_IO_FlushTx(void)
{
    /* ① 老式发法：死等"最后一个字节移出完成"（TC=发送完成标志） */
    if (!tx_dma_ok) {
        while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET);
        return;
    }

    /* ② DMA 模式：等到"仓库空 + 搬运工不忙"（都成立=全发完了），最多等200ms */
    uint32_t wait_start = HAL_GetTick();
    while (tx_head != tx_tail || tx_dma_busy) {
        if (HAL_GetTick() - wait_start >= TX_FLUSH_TIMEOUT_MS) break;  /* 超时就放弃等待 */
    }
}

/* -------------------------------------------------------------------------- */
/* 发送字符串 (入 TX 环形缓冲, DMA 异步发出, 非阻塞)                           */
/* -------------------------------------------------------------------------- */
/* 【中文名】发送一串文字：逐个字符调用 PutChar，直到遇到字符串结尾的 \0 */
void UART_IO_Print(const char *str)
{
    /* while(*str)：C 字符串以 \0 结尾，\0 的值是0，就是"假"，所以循环到 \0 就停 */
    while (*str) {
        UART_IO_PutChar(*str++);  /* 发当前字符，然后指针指到下一个 */
    }
}

/* -------------------------------------------------------------------------- */
/* 发送原始字节 (二进制安全: 逐字节入环, \0 不会被截断)                       */
/* -------------------------------------------------------------------------- */
/* 【中文名】发送一串原始字节：和 Print 的区别是它不靠 \0 结尾，靠长度 len 指定发几个 */
void UART_IO_SendBytes(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {   /* 发 len 个，不含 \0 也不被 \0 截断 */
        UART_IO_PutChar((char)data[i]);
    }
}

/* -------------------------------------------------------------------------- */
/* 格式化打印                                                                 */
/* -------------------------------------------------------------------------- */
/* 【中文名】格式化打印：像 printf 那样，把数字拼进文字里发出去（比如 "速度=%.2f"） */
void UART_IO_Printf(const char *fmt, ...)
{
    /* static 缓冲：这块内存只用一次分配，函数反复调用不反复申请（省内存/时间） */
    static char tx_buf[UART_TX_BUF_SIZE];
    /* va_list / vsnprintf：处理"可变参数"（... 里那堆不定数量的参数）。
     * vsnprintf 的作用 = 把 fmt 和后面参数拼成完整字符串，最多写 tx_buf 里 */
    va_list args;
    va_start(args, fmt);                            /* 开始解析可变参数 */
    vsnprintf(tx_buf, sizeof(tx_buf), fmt, args);   /* 拼成字符串放进 tx_buf */
    va_end(args);                                   /* 结束解析 */
    UART_IO_Print(tx_buf);                          /* 把拼好的字符串发出去 */
}

/* -------------------------------------------------------------------------- */
/* TX DMA 泵: 从环形缓冲取一段连续数据交给 DMA, 处理回绕与多段拼接            */
/*   调用点: PutChar (主循环) / DMA 完成中断 (回绕续传)                        */
/* -------------------------------------------------------------------------- */
/* 【中文名】派活函数（发送侧核心）：看一眼仓库有没有货，有货就算出"这一趟搬几格"，交给搬运工DMA */
static void UART_IO_TxPump(void)
{
    /* ① 关卡一：搬运工没就位 / 正忙着 → 什么都不干，直接返回 */
    if (!tx_dma_ok || tx_dma_busy) return;

    /* ② 抄一份进货口、出货口的快照（避免算到一半被中断改掉） */
    uint16_t head = tx_head;
    uint16_t tail = tx_tail;
    /* ③ 关卡二：仓库空（进货口==出货口）→ 没货可搬，返回 */
    if (head == tail) return;

    /* ④ 算"这一趟搬几格"：
     *    avail = 仓库里一共几个货（用 +N %N 防止负数差值）
     *    chunk = 从出货口到仓库末尾还有几格（DMA 只会一路往后搬，不会拐弯）
     *    取两者最小值 = 这一趟实际能搬的数量 */
    uint16_t avail = (uint16_t)((head - tail + UART_TX_RING_SIZE) % UART_TX_RING_SIZE);
    uint16_t chunk = (uint16_t)(UART_TX_RING_SIZE - tail);
    if (chunk > avail) chunk = avail;   /* 能走的距离 > 实际有货 → 缩到货的个数 */

    /* ⑤ 出发前写两样"凭证"：
     *    tx_dma_len = 这趟搬几个（回来结账时用"tail 往前挪几格"）
     *    tx_dma_busy = 挂牌子"我在忙"（挡住别的调用再来启动） */
    tx_dma_len  = chunk;
    tx_dma_busy = true;

    /* ⑥ 给搬运工填提货单，启动 DMA：
     *    三个参数 = 从哪搬(仓库出货口) → 搬到哪(串口数据寄存器DR) → 搬几个(chunk) */
    if (HAL_DMA_Start_IT(&hdma_uart2_tx,
                         (uint32_t)&tx_ring[tail],
                         (uint32_t)&huart2.Instance->DR,
                         chunk) != HAL_OK) {
        /* ⑦ 启动失败（理论上几乎不发生）→ 摘牌子、假装搬过了、再试下一段 */
        tx_dma_busy = false;
        tx_tail = (uint16_t)((tail + chunk) % UART_TX_RING_SIZE);
        UART_IO_TxPump();
    }
}

/* DMA 传输完成回调 (ISR 上下文) — 推进 tail 并续传 (含回绕段) */
/* 【中文名】搬运工搬完一段后回调：照"纸条"结账，摘牌子，再喊自己搬下一段 */
static void UART_TxDmaCplt(DMA_HandleTypeDef *hdma)
{
    (void)hdma;   /* 这个参数没用，加 (void) 告诉编译器"我知道它是空的" */
    /* 结账：出货口往前挪"这一趟搬的数量"格 */
    tx_tail = (uint16_t)((tx_tail + tx_dma_len) % UART_TX_RING_SIZE);
    tx_dma_busy = false;   /* 摘掉"忙"的牌子 */
    UART_IO_TxPump();      /* 再派一次活：仓库还有货就继续搬，搬完为止（自动接力） */
}

/* DMA 传输错误回调 (ISR 上下文) — 放弃当前段, 复位后继续 */
/* 【中文名】搬运出错时回调：放弃当前这段，摘牌子，继续搬下一段 */
static void UART_TxDmaError(DMA_HandleTypeDef *hdma)
{
    __HAL_DMA_DISABLE(hdma);   /* 关掉出错的 DMA */
    tx_tail = (uint16_t)((tx_tail + tx_dma_len) % UART_TX_RING_SIZE);  /* 结账（跳过这段） */
    tx_dma_busy = false;       /* 摘牌子 */
    UART_IO_TxPump();          /* 继续搬 */
}

/* -------------------------------------------------------------------------- */
/* DMA1 Stream6 中断服务函数 (USART2_TX)                                      */
/* -------------------------------------------------------------------------- */
/* 【中文名】DMA中断入口：搬运工搬完（或出错）时，芯片自动跳进这里，转交给 HAL 处理 */
void DMA1_Stream6_IRQHandler(void)
{
    /* HAL 会根据"搬完成/出错"自动去调我们注册过的那两个回调函数 */
    HAL_DMA_IRQHandler(&hdma_uart2_tx);
}

/* -------------------------------------------------------------------------- */
/* 从环形缓冲区读取一个字节（关中断保护临界区）                                */
/* -------------------------------------------------------------------------- */
/* 【中文名】取一个字节：从接收仓库出货口拿一个字节出来。必须关中断保护 */
bool UART_IO_GetChar(uint8_t *ch)
{
    bool got = false;

    /* 关中断 = 锁门：因为 rx_tail 这个"出货口"，中断里(满时丢最老)也会改，
     * 主循环和中断两边都要动它。这行"读-改-写"不能被打断，所以要锁住 */
    __disable_irq();
    if (rx_head != rx_tail || rx_full) {   /* 仓库里有货吗？（进货口≠出货口 或 满标志） */
        *ch = rx_buf[rx_tail];            /* 从出货口取一个字节，交给调用者（*ch 是"写到那个地址"） */
        rx_tail = (rx_tail + 1) % UART_RX_BUF_SIZE;  /* 出货口往后挪一格 */
        rx_full = false;                  /* 取走一个，一定不满了 */
        got = true;
    }
    __enable_irq();   /* 开锁 */

    return got;   /* 返回"有没有取到" */
}

/* -------------------------------------------------------------------------- */
/* 获取可用字节数                                                             */
/* -------------------------------------------------------------------------- */
/* 【中文名】数一数仓库里还有几个字节（也要关中断，因为要读多个变量保持一致） */
uint16_t UART_IO_Available(void)
{
    uint16_t count;
    __disable_irq();   /* 锁住，防止读 rx_head/rx_tail 到一半被中断改掉 */
    if (rx_full) {
        count = UART_RX_BUF_SIZE;   /* 满了 → 可用数就是整个仓库大小 */
    } else {
        /* 没满 → 用 (进货口−出货口 + N) % N 算出仓库里有几个（+N%N 防负数） */
        count = (rx_head - rx_tail + UART_RX_BUF_SIZE) % UART_RX_BUF_SIZE;
    }
    __enable_irq();   /* 开锁 */
    return count;
}

/* -------------------------------------------------------------------------- */
/* 清空接收缓冲区                                                             */
/* -------------------------------------------------------------------------- */
/* 【中文名】清空接收仓库：把进货口、出货口都归零，等于"把所有货倒掉" */
void UART_IO_Flush(void)
{
    __disable_irq();   /* 锁住 */
    rx_head = 0;
    rx_tail = 0;
    rx_full = false;
    __enable_irq();    /* 开锁 */
}

/* -------------------------------------------------------------------------- */
/* 接收中断回调（仅在 ISR 上下文调用）                                        */
/* -------------------------------------------------------------------------- */
/* 【中文名】接收存放（在中断里被调用）：把刚收到的字节放进接收仓库的进货口 */
void UART_IO_RxCallback(uint8_t byte)
{
    if (rx_full) {
        /* 仓库满了 → 把最老的挤掉（出货口往后挪一格，腾出一个空位） */
        rx_tail = (rx_tail + 1) % UART_RX_BUF_SIZE;
    }

    rx_buf[rx_head] = byte;                 /* 放进进货口当前指的格子 */
    rx_head = (rx_head + 1) % UART_RX_BUF_SIZE;  /* 进货口往后挪一格 */

    /* 检查是否变满：进货口追上了出货口 = 满了 */
    if (rx_head == rx_tail) {
        rx_full = true;
    }
}

/* -------------------------------------------------------------------------- */
/* USART2 中断服务函数                                                        */
/* -------------------------------------------------------------------------- */
/* 【中文名】串口接收中断入口：芯片每收到一个字节就自动跳进这里，把字节读走并转交存放 */
void USART2_IRQHandler(void)
{
    /* 一次性读 SR（状态寄存器），避免读多次时标志位变化导致漏处理。
     * SR 是一排"指示灯"，告诉你"是谁把我叫醒的"（收到字节？出错？） */
    uint32_t sr = huart2.Instance->SR;

    /* ① 指示灯 RXNE 亮 = 收到一个字节 */
    if (sr & USART_SR_RXNE) {
        /* 读数据寄存器 DR = 把字节端走。
         * & 0xFF 是位与：只取低8位（真正的字节），忽略高位多余信息 */
        uint8_t byte = (uint8_t)(huart2.Instance->DR & 0xFF);
        UART_IO_RxCallback(byte);   /* 转交给"存放"函数 */
    }

    /* ② 各种错误标志：溢出/帧错/噪声/校验错 → 读一下 DR 就能清除标志，
     *    否则错误标志一直挂着会卡死中断（这是"擦屁股"代码） */
    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        (void)huart2.Instance->DR;   /* 读 DR 即可清除 FE/NE/PE/ORE */
    }

    /* ③ 如果还挂着 RXNE（错误帧也可能触发 RXNE），再读一次清掉 */
    if (huart2.Instance->SR & USART_SR_RXNE) {
        (void)huart2.Instance->DR;
    }
}

/* -------------------------------------------------------------------------- */
/* printf 重定向 (_write)                                                     */
/* -------------------------------------------------------------------------- */
/* 【中文名】接管 printf：让程序里所有 printf 的输出，都改走我们的串口 */
#ifdef __GNUC__
int _write(int file, char *ptr, int len)
{
    (void)file;   /* file 参数用不到，忽略 */
    /* 逐个字符发出去。注意：遇到换行 \n 要额外补一个回车 \r。
     * 因为 Windows 终端需要 \r\n 两个字符才显示换行；只发 \n 会"换行不复位" */
    for (int i = 0; i < len; i++) {
        if (ptr[i] == '\n') UART_IO_PutChar('\r');   /* 先补回车 */
        UART_IO_PutChar(ptr[i]);                     /* 再发字符本身 */
    }
    return len;   /* 返回"发出了几个字节" */
}
#endif

/* -------------------------------------------------------------------------- */
/* __io_putchar 重定向（兼容某些 newlib-nano 配置）                           */
/* -------------------------------------------------------------------------- */
/* 【中文名】另一个 printf 底层函数：和 _write 类似，补 \r\n，兼容不同的编译环境 */
int __io_putchar(int ch)
{
    if (ch == '\n') UART_IO_PutChar('\r');   /* 换行前补回车 */
    UART_IO_PutChar((char)ch);               /* 发字符 */
    return ch;
}
