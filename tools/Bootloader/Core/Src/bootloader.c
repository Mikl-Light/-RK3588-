/**
 ******************************************************************************
 * @file    bootloader.c
 * @brief   STM32F407 串口 Bootloader 实现
 ******************************************************************************
 * 协议见 bootloader.h 头部注释。
 * 写入采用 4 字节缓冲凑整编程；每个扇区在本会话首次写入前自动擦除。
 ******************************************************************************
 */
#include "bootloader.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>

/* ============================ 工具: 串口发送 ============================ */

void BL_SendStr(const char *s)
{
    HAL_UART_Transmit(&huart2, (const uint8_t *)s, (uint16_t)strlen(s), 1000);
}

void BL_SendHex32(uint32_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    char buf[9];
    for (int i = 0; i < 8; i++)
        buf[i] = hex[(v >> (28 - 4 * i)) & 0xF];
    buf[8] = '\0';
    BL_SendStr(buf);
}

/* ============================ 工具: 行接收 ============================ */

/**
 * @brief 阻塞接收一行 (\r 或 \n 结尾), 超时返回 -1
 * @note  返回长度 >=0 表示收到一行; 超时前收到的部分字符保留在 buf, 由
 *        调用者下次继续 (部分行不丢弃)。timeout 为"无新字符"的间隔超时。
 *
 * 2026-08-16 修复 (V1.2.1): 逐字节轮询改用寄存器直接读取 RXNE,
 *   HAL_UART_Receive 每次调用有函数开销, USB-TTL 突发发送长行 (如
 *   DATA 133 字符) 时 1 字节硬件缓冲会 ORE 溢出丢字节 -> 行不完整无响应。
 *   紧密轮询 RXNE 在 168MHz 下每字节 20µs 间隔内必然读到, 不丢字节。
 */
static int BL_GetLine(char *buf, uint16_t max_len, uint32_t timeout_ms, uint16_t *cur_len)
{
    uint32_t last_tick = HAL_GetTick();
    USART_TypeDef *u = huart2.Instance;
    uint8_t ch;

    while (1) {
        if (u->SR & USART_SR_RXNE) {
            ch = (uint8_t)(u->DR & 0xFFU);      /* 读 DR 自动清 RXNE */
            last_tick = HAL_GetTick();
            if (ch == '\r' || ch == '\n') {
                int len = (int)*cur_len;
                buf[*cur_len] = '\0';
                *cur_len = 0;
                if (len == 0) continue;          /* 空行跳过 */
                return len;
            }
            if (*cur_len < max_len - 1)
                buf[(*cur_len)++] = (char)ch;
        } else if (u->SR & USART_SR_ORE) {
            /* 溢出恢复: 读 SR(已读) 后读 DR 清除 ORE, 继续接收 */
            (void)u->DR;
            last_tick = HAL_GetTick();
        } else if ((HAL_GetTick() - last_tick) >= timeout_ms) {
            return -1;                            /* 超时 */
        }
    }
}

/* ============================ 工具: 十六进制解析 ============================ */

static int HexVal(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static long ParseHex32(const char *s)
{
    uint32_t v = 0;
    for (int i = 0; i < 8; i++) {
        int h = HexVal(s[i]);
        if (h < 0) return -1;
        v = (v << 4) | (uint32_t)h;
    }
    return (long)v;
}

/* ============================ Flash 操作 ============================ */

/* F407 扇区号: 0-3 各16K, 4=64K, 5-11 各128K */
static uint32_t SectorOf(uint32_t addr)
{
    if (addr < 0x08010000U) return (addr - 0x08000000U) >> 14;          /* 0~3 */
    if (addr < 0x08020000U) return 4U;
    return 5U + ((addr - 0x08020000U) >> 17);                            /* 5~11 */
}

/* 本会话已擦除扇区位图 (仅跟踪 App 区扇区 4~11) */
static uint16_t erased_mask = 0;

static int EraseSectorIfNeeded(uint32_t addr)
{
    uint32_t sec = SectorOf(addr);
    HAL_StatusTypeDef st;
    FLASH_EraseInitTypeDef ei;
    uint32_t page_err = 0;

    if (sec < 4U || sec > 11U) return -1;          /* 不允许写 bootloader 区 */
    if (erased_mask & (1U << sec)) return 0;       /* 本会话已擦除 */

    ei.TypeErase    = FLASH_TYPEERASE_SECTORS;
    ei.Banks        = FLASH_BANK_1;
    ei.Sector       = sec;
    ei.NbSectors    = 1;
    ei.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    st = HAL_FLASHEx_Erase(&ei, &page_err);
    if (st != HAL_OK || page_err != 0xFFFFFFFFU) return -1;

    erased_mask |= (uint16_t)(1U << sec);
    return 0;
}

int BL_FlashEraseApp(void)
{
    HAL_StatusTypeDef st;
    FLASH_EraseInitTypeDef ei;
    uint32_t page_err = 0;

    BL_FlushBuffer();
    erased_mask = 0;                                /* 强制全擦 */

    ei.TypeErase    = FLASH_TYPEERASE_SECTORS;
    ei.Banks        = FLASH_BANK_1;
    ei.Sector       = 4;
    ei.NbSectors    = 8;                            /* 扇区 4~11 */
    ei.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    st = HAL_FLASHEx_Erase(&ei, &page_err);
    if (st != HAL_OK || page_err != 0xFFFFFFFFU) return -1;
    return 0;
}

/* ---- 4 字节凑整写缓冲 ---- */
static uint8_t  wr_buf[4];
static uint32_t wr_cnt = 0;
static uint32_t wr_addr = 0;      /* 缓冲对应的目标地址 */

int BL_FlushBuffer(void)
{
    if (wr_cnt == 0) return 0;

    /* 不足 4 字节用 0xFF 补齐 (擦除后本就是 0xFF, 不影响数据) */
    while (wr_cnt < 4) wr_buf[wr_cnt++] = 0xFF;

    uint32_t word = ((uint32_t)wr_buf[0])
                  | ((uint32_t)wr_buf[1] << 8)
                  | ((uint32_t)wr_buf[2] << 16)
                  | ((uint32_t)wr_buf[3] << 24);

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, wr_addr, word) != HAL_OK)
        return -1;
    /* 回读校验 */
    if (*(volatile uint32_t *)wr_addr != word) return -2;

    wr_addr += 4;
    wr_cnt = 0;
    return 0;
}

int BL_FlashWrite(uint32_t addr, const uint8_t *data, uint32_t len)
{
    if (addr < BL_APP_ADDR || (addr + len) > BL_FLASH_END) return -1;

    /* 地址跳变 -> 先冲刷旧缓冲 */
    if (wr_cnt != 0 && addr != (wr_addr + wr_cnt)) {
        if (BL_FlushBuffer() != 0) return -2;
    }
    if (wr_cnt == 0) wr_addr = addr;

    for (uint32_t i = 0; i < len; i++) {
        /* 进入新扇区前确保已擦除 */
        if ((wr_addr & 0x3U) == 0 && wr_cnt == 0) {
            if (EraseSectorIfNeeded(wr_addr) != 0) return -3;
        }
        wr_buf[wr_cnt++] = data[i];
        if (wr_cnt == 4) {
            if (BL_FlushBuffer() != 0) return -4;
        }
    }
    return 0;
}

/* ============================ App 校验与跳转 ============================ */

uint8_t BL_AppValid(void)
{
    uint32_t sp = *(volatile uint32_t *)BL_APP_ADDR;
    uint32_t pc = *(volatile uint32_t *)(BL_APP_ADDR + 4U);

    /* 栈顶必须落在主 SRAM 0x20000000~0x2001FFFF (128K) */
    if (sp < 0x20000000U || sp > 0x20020000U) return 0;
    /* 复位向量必须落在 App Flash 区且为 Thumb 地址 */
    if (pc < BL_APP_ADDR || pc >= BL_FLASH_END) return 0;
    if ((pc & 1U) == 0) return 0;
    return 1;
}

void BL_JumpToApp(void)
{
    uint32_t sp = *(volatile uint32_t *)BL_APP_ADDR;
    uint32_t pc = *(volatile uint32_t *)(BL_APP_ADDR + 4U);
    void (*app_reset)(void) = (void (*)(void))pc;

    /* 1. 反初始化外设, 时钟回到复位默认 (App 会重新配置) */
    HAL_UART_DeInit(&huart2);
    HAL_RCC_DeInit();
    HAL_DeInit();

    /* 2. 关 SysTick */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    /* 3. 关中断并清所有挂起 */
    __disable_irq();
    for (int i = 0; i < 8; i++)
        NVIC->ICPR[i] = 0xFFFFFFFFU;

    /* 4. 重定位向量表到 App, 设置主栈指针后跳转 */
    SCB->VTOR = BL_APP_ADDR;
    __DSB();
    __ISB();
    __set_MSP(sp);
    __set_CONTROL(0);          /* 退回特权级、MSP (若用过 PSP) */
    /* 5. 恢复全局中断: 跳转不是真实复位, PRIMASK 保持关闭会让 App 的
     *    SysTick 中断被屏蔽 -> HAL_Delay 死等 (灯常亮/灭灯, 2026-08-15 修复) */
    __enable_irq();
    app_reset();               /* 不返回 */

    while (1) { }              /* 不会到达 */
}

/* ============================ 升级标记 ============================ */

uint8_t BL_UpdateFlagSet(void)
{
    return *(volatile uint32_t *)BL_FLAG_ADDR == BL_FLAG_MAGIC;
}

void BL_UpdateFlagClear(void)
{
    /* 标记位于扇区3, 整扇区擦除 (该扇区无 bootloader 代码, 安全) */
    FLASH_EraseInitTypeDef ei;
    uint32_t page_err = 0;

    ei.TypeErase    = FLASH_TYPEERASE_SECTORS;
    ei.Banks        = FLASH_BANK_1;
    ei.Sector       = 3U;
    ei.NbSectors    = 1;
    ei.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASHEx_Erase(&ei, &page_err) != HAL_OK) {
        /* 擦除失败: 无 0xFF 无法写 0, 尝试直接编程 0 清除标记 */
        (void)HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, BL_FLAG_ADDR, 0U);
    }
}

/* ============================ 按键检测 ============================ */

/**
 * @brief  PA15 按键是否按住 (连续 ~200ms 保持低电平才判定, 防抖)
 * @note   上电时按住按键 -> 无论标记/App 状态如何都强制进升级模式。
 *         按键接法: 一端 PA15, 一端 GND; 按下为低电平 (内部上拉)。
 *         若按键接 VCC, 需把 gpio.c 中 Pull 改为 GPIO_PULLDOWN 并反转判断。
 */
uint8_t BL_ButtonPressed(void)
{
    for (int i = 0; i < 4; i++) {
        if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) != GPIO_PIN_RESET)
            return 0;                    /* 任意一次读到高电平 -> 未按住 */
        HAL_Delay(50);
    }
    return 1;
}

/* ============================ 升级主循环 ============================ */

void BL_UpdateLoop(void)
{
    char line[300];
    uint16_t cur = 0;
    static uint8_t data_buf[64];
    uint32_t led_last = HAL_GetTick();

    /* 横幅: RK/上位机以 "BOOTLOADER" 行识别已进入升级模式 */
    BL_SendStr("\r\nBOOTLOADER " BL_VERSION " " BL_BUILD_DATE " ready\r\n");
    BL_SendStr("flag=");
    BL_SendStr(BL_UpdateFlagSet() ? "1" : "0");
    BL_SendStr(" app_valid=");
    BL_SendStr(BL_AppValid() ? "1" : "0");
    BL_SendStr("\r\n");
    BL_SendStr("Update mode. Commands: UPDATE INFO ERASE ADDR,<hex> DATA,<hex> RUN\r\n");

    while (1) {
        /* LED 心跳: 固定 200ms 翻转 (PC13 低电平点亮)。
         * 板子在跑就一直闪; Flash 擦写期间 CPU 停顿灯停住, 恢复后继续闪。 */
        if (HAL_GetTick() - led_last >= 200U) {
            led_last = HAL_GetTick();
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }

        int n = BL_GetLine(line, sizeof(line), 100, &cur);
        if (n < 0) continue;

        /* ---------------- 握手 ---------------- */
        if (strcmp(line, "UPDATE") == 0) {
            BL_SendStr("OK BL ready\r\n");
        }
        /* ---------------- 查询 ---------------- */
        else if (strcmp(line, "INFO") == 0) {
            BL_SendStr("BL " BL_VERSION " " BL_BUILD_DATE
                       " app@0x08010000 size<=960K valid=");
            BL_SendStr(BL_AppValid() ? "YES" : "NO");
            BL_SendStr("\r\n");
        }
        /* ---------------- 整体擦除 ---------------- */
        else if (strcmp(line, "ERASE") == 0) {
            HAL_FLASH_Unlock();
            int r = BL_FlashEraseApp();
            HAL_FLASH_Lock();
            BL_SendStr(r == 0 ? "OK erased sectors 4-11\r\n" : "ERR erase\r\n");
        }
        /* ---------------- 设置地址 ---------------- */
        else if (strncmp(line, "ADDR,", 5) == 0) {
            long a = ParseHex32(line + 5);
            if (a < 0) { BL_SendStr("ERR ADDR,<8hex>\r\n"); continue; }
            if (BL_FlushBuffer() != 0) { BL_SendStr("ERR flush\r\n"); continue; }
            if ((uint32_t)a < BL_APP_ADDR || (uint32_t)a >= BL_FLASH_END) {
                BL_SendStr("ERR range 08010000-080FFFFF\r\n");
                continue;
            }
            wr_addr = (uint32_t)a;
            BL_SendStr("OK @0x");
            BL_SendHex32(wr_addr);
            BL_SendStr("\r\n");
        }
        /* ---------------- 写数据 ---------------- */
        else if (strncmp(line, "DATA,", 5) == 0) {
            const char *hex = line + 5;
            int nhex = n - 5;

            if (nhex <= 0 || (nhex & 1) != 0 || nhex > (int)(sizeof(data_buf) * 2)) {
                BL_SendStr("ERR DATA,<hex pairs, <=64B>\r\n");
                continue;
            }
            uint32_t nbytes = (uint32_t)nhex / 2U;
            for (uint32_t i = 0; i < nbytes; i++) {
                int hi = HexVal(hex[2 * i]);
                int lo = HexVal(hex[2 * i + 1]);
                if (hi < 0 || lo < 0) { nbytes = 0; break; }
                data_buf[i] = (uint8_t)((hi << 4) | lo);
            }
            if (nbytes == 0) { BL_SendStr("ERR bad hex\r\n"); continue; }

            uint32_t start_addr = wr_addr + wr_cnt;
            HAL_FLASH_Unlock();
            int r = BL_FlashWrite(start_addr, data_buf, nbytes);
            HAL_FLASH_Lock();

            if (r != 0) {
                BL_SendStr("ERR write code=");
                BL_SendHex32((uint32_t)r);
                BL_SendStr("\r\n");
            } else {
                BL_SendStr("OK ");
                BL_SendHex32(nbytes);
                BL_SendStr("B @0x");
                BL_SendHex32(start_addr);
                BL_SendStr("\r\n");
            }
        }
        /* ---------------- 跳转 ---------------- */
        else if (strcmp(line, "RUN") == 0) {
            HAL_FLASH_Unlock();
            BL_FlushBuffer();
            HAL_FLASH_Lock();

            if (!BL_AppValid()) {
                BL_SendStr("ERR no valid app at 0x08010000\r\n");
                continue;
            }
            BL_SendStr("OK jumping\r\n");

            /* 升级成功, 清除标记再跳转 (断电也不会误入升级模式) */
            HAL_FLASH_Unlock();
            BL_UpdateFlagClear();
            HAL_FLASH_Lock();

            HAL_Delay(5);                     /* 让 TX 发完 */
            BL_JumpToApp();                   /* 不返回 */
        }
        /* ---------------- 未知命令 ---------------- */
        else {
            BL_SendStr("ERR unknown: ");
            BL_SendStr(line);
            BL_SendStr("\r\n");
        }
    }
}
