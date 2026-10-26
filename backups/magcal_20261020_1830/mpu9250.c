/**
  ******************************************************************************
  * @file    mpu9250.c
  * @brief   MPU9250 九轴传感器驱动实现
  *          MPU9250 = MPU6500 (6-axis) + AK8963 (3-axis magnetometer)
  ******************************************************************************
  */
/* ============ 本文件在做什么 ============
 * MPU9250 是一个"九轴传感器"，其实是一颗芯片里装了三种传感器：
 *   陀螺仪（测转多快）+ 加速度计（测倾斜）+ 磁力计（测朝向）
 * 本文件负责和它"对话"：通过 I2C 总线读写它的寄存器，把原始数据读出来、换算成物理量。
 *
 * 三个关键词：
 *   I2C  = 芯片之间的"对话总线"（两根线：SCL时钟 + SDA数据）
 *   寄存器 = 传感器身上的"开关格子"，读写它有固定地址
 *   原始LSB = 传感器吐出来的原始数字，要乘个系数才变成物理量（°/s、g、µT）
 *
 * 一个重要现实：电机一开，会产生电磁干扰(EMI)，I2C 总线的 BUSY 标志会卡死。
 * 所以本文件有一大段"I2C 故障自恢复"代码——这是它在真实场景下能稳定工作的关键。
 */
#include "mpu9250.h"
#include "debug.h"
#include "gpio.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* I2C 超时配置 (电机运行时 I2C 易受 EMI 干扰, 必须快速失败)                  */
/* -------------------------------------------------------------------------- */
#define I2C_TIMEOUT_MS       3      /**< 正常运行超时 (ms)，超时就失败，不傻等 */
#define I2C_INIT_TIMEOUT_MS 50      /**< 初始化阶段超时 (ms)，上电时允许等久一点 */
#define I2C_COOLDOWN_MS    500      /**< 恢复失败后冷却期 (ms)，期间跳过 I2C，避免刷屏 */
#define I2C_LOG_ENABLE       0      /**< 调试电机时关掉: 0=静音, 1=输出日志 */

/* -------------------------------------------------------------------------- */
/* 内部变量                                                                   */
/* -------------------------------------------------------------------------- */
/* 三个"换算系数"：LSB→物理量的比例。例如陀螺读数是 131，不代表 131 度/秒，而是 131÷131=1 度/秒 */
static float gyro_lsb_per_dps  = 131.0f;    /* 陀螺：131 LSB = 1 度/秒 */
static float accel_lsb_per_g   = 16384.0f;  /* 加速度计：16384 LSB = 1g */
static float mag_lsb_per_uT    = 0.15f;     /**< 磁力计 AK8963: ~0.15 µT/LSB @ 16-bit */
static MPU9250_Calib calib_data = {0};     /* 校准数据（零偏、磁硬铁软铁等，全存这里） */

/* 磁力计上次成功读数 (读失败时保持旧值, 避免遥测里 mag 突然变 0) */
static float last_mag[3] = {0.0f, 0.0f, 0.0f};

/* I2C 故障统计 (用于自动恢复) */
static uint32_t i2c_fail_count = 0;     /**< 连续 I2C 失败计数 */
static uint32_t i2c_ok_count   = 0;     /**< 连续 I2C 成功计数 */

/* I2C 冷却期机制: 恢复失败后一段时间内跳过所有 I2C 操作, 避免日志刷屏 */
static uint32_t i2c_cooldown_until_ms = 0;  /**< 冷却期结束时间戳 (ms)，到点之前不碰I2C */

/* 磁力计灵敏度校准 (ASA) — 从 FUSE ROM 读取（每颗芯片出厂标定的系数） */
static float mag_asa_x = 1.0f;
static float mag_asa_y = 1.0f;
static float mag_asa_z = 1.0f;

/* -------------------------------------------------------------------------- */
/* I2C 总线恢复 (解决 STM32F4 I2C BUSY 标志卡死问题)                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief GPIO 位拆裂恢复: 手动翻转 SCL 释放被拉死的 SDA, 再发 STOP
 * @note  当 SWRST 无法清除 BUSY 时使用, 比纯软件复位更可靠
 */
/* 【中文名】GPIO手动恢复I2C（终极手段）：软件复位没用时，手动翻转两根线，把卡死的总线硬拽回来 */
static void I2C_GPIO_Recover(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* 1. 关闭 I2C 外设 */
    hi2c1.Instance->CR1 &= ~I2C_CR1_PE;

    /* 2. 将 PB8(SCL), PB9(SDA) 切换为 GPIO 开漏输出, 内部上拉 */
    gpio.Pin   = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* 3. 初始拉高两条线 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    for (volatile int d = 0; d < 50; d++) { __NOP(); }

    /* 4. 如果 SDA 被从机拉低 → 翻转 SCL 最多 9 次让从机释放 */
    for (int i = 0; i < 9; i++) {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET) break;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        for (volatile int d = 0; d < 50; d++) { __NOP(); }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        for (volatile int d = 0; d < 50; d++) { __NOP(); }
    }

    /* 5. 发 STOP 条件: SDA 低→高 在 SCL 高电平时 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    for (volatile int d = 0; d < 50; d++) { __NOP(); }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    for (volatile int d = 0; d < 50; d++) { __NOP(); }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    for (volatile int d = 0; d < 50; d++) { __NOP(); }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    for (volatile int d = 0; d < 50; d++) { __NOP(); }

    /* 6. 恢复 I2C 复用功能 */
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* 7. 重新使能 I2C */
    hi2c1.Instance->CR1 |= I2C_CR1_PE;
    for (volatile int d = 0; d < 1000; d++) { __NOP(); }
}

/**
 * @brief 检查并恢复 I2C 总线 (带冷却期防刷屏)
 * @return true=总线空闲可用, false=总线卡死且恢复失败
 */
/* 【中文名】检查+恢复I2C总线：如果总线卡死了，尝试两套办法救回来，救不回来就"冷却"一会儿别碰它 */
static bool I2C_Recover(void)
{
    uint32_t now = HAL_GetTick();

    /* ---- BUSY 已清除 → 总线正常，直接用 ---- */
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) == RESET) {
        if (i2c_cooldown_until_ms != 0) {
            /* 之前卡过，现在自己好了 → 清掉冷却期标记 */
            i2c_cooldown_until_ms = 0;
#if I2C_LOG_ENABLE
            DBG_INFO(TAG_MPU, "I2C1 recovered OK");
#endif
        }
        return true;
    }

    /* ---- BUSY 卡死了 ---- */

    /* 还在冷却期内 → 直接放弃，不重试、不刷日志（等冷却期过再说） */
    if (now < i2c_cooldown_until_ms) {
        return false;
    }

    /* 开始新一轮恢复 */
#if I2C_LOG_ENABLE
    DBG_WARN(TAG_MPU, "I2C1 BUSY stuck (EMI from motors), recovering...");
#endif

    /* 第一步：SWRST 软件复位（就是"重新开机"I2C这个外设） */
    hi2c1.Instance->CR1 &= ~I2C_CR1_PE;      /* 关掉I2C */
    for (volatile int i = 0; i < 1000; i++) { __NOP(); }  /* 空转等一会儿 */
    hi2c1.Instance->CR1 |= I2C_CR1_SWRST;    /* 触发软件复位 */
    for (volatile int i = 0; i < 1000; i++) { __NOP(); }
    hi2c1.Instance->CR1 &= ~I2C_CR1_SWRST;   /* 撤销复位 */
    hi2c1.Instance->CR1 |= I2C_CR1_PE;       /* 重新开I2C */
    for (volatile int i = 0; i < 1000; i++) { __NOP(); }

    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) == RESET) {
        /* 软件复位成功。但注意：SWRST 会清空 I2C 的时序配置，
         * 必须重新初始化一遍，否则后续通信全失败（这是踩过的坑） */
        HAL_I2C_Init(&hi2c1);
        i2c_cooldown_until_ms = 0;
#if I2C_LOG_ENABLE
        DBG_INFO(TAG_MPU, "I2C1 recovered OK (SWRST, re-init)");
#endif
        return true;
    }

    /* 第二步：GPIO 手动恢复（更强力，直接翻两根线硬拽回来） */
    I2C_GPIO_Recover();

    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) == RESET) {
        /* GPIO 恢复成功，同样要重新初始化 I2C */
        HAL_I2C_Init(&hi2c1);
        i2c_cooldown_until_ms = 0;
#if I2C_LOG_ENABLE
        DBG_INFO(TAG_MPU, "I2C1 recovered OK (GPIO, re-init)");
#endif
        return true;
    }

    /* 两招都失败 → 进入冷却期，期间跳过所有 I2C 操作（500ms 内别再碰它） */
    i2c_cooldown_until_ms = now + I2C_COOLDOWN_MS;
#if I2C_LOG_ENABLE
    DBG_ERROR(TAG_MPU, "I2C1 recovery FAILED, cooldown %dms", I2C_COOLDOWN_MS);
#endif
    return false;
}

/* -------------------------------------------------------------------------- */
/* I2C 读写的安全封装: 冷却期内跳过, 失败时自动恢复总线                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief 带自动恢复的 I2C 读操作
 * @return HAL_OK=成功, 其他=失败(已尝试恢复但未成功或处于冷却期)
 */
/* 【中文名】安全读：从某设备的某寄存器读一段数据。失败会尝试恢复总线，掉进冷却期也直接放弃 */
static HAL_StatusTypeDef I2C_SafeRead(uint8_t dev_addr, uint8_t reg,
                                       uint8_t *data, uint8_t len, uint32_t timeout_ms)
{
    HAL_StatusTypeDef ret;

    /* ① 若还在冷却期内，直接放弃（不发起真正通讯，也不刷屏） */
    if (i2c_cooldown_until_ms != 0) {
        uint32_t now = HAL_GetTick();
        if (now < i2c_cooldown_until_ms) {
            return HAL_BUSY;
        }
        /* ② 冷却期过了 → 先试着恢复一次，恢复失败就重新进入冷却期 */
        i2c_cooldown_until_ms = 0;
        if (!I2C_Recover()) {
            return HAL_BUSY;
        }
    }

    /* ③ 真正读：通过 I2C 读设备(dev_addr)的寄存器(reg)开始，读 len 字节。
     * 注意 dev_addr<<1：I2C 地址是7位，协议里要左移1位（最低位是读/写标志） */
    ret = HAL_I2C_Mem_Read(&hi2c1, dev_addr << 1, reg,
                           I2C_MEMADD_SIZE_8BIT, data, len, timeout_ms);

    if (ret == HAL_OK) {
        i2c_fail_count = 0;   /* 成功 → 失败计数清零 */
        i2c_ok_count++;
        return HAL_OK;
    }

    /* ④ 失败 → 计数，连续失败3次就尝试复位总线 */
    i2c_fail_count++;
    i2c_ok_count = 0;
    if (i2c_fail_count >= 3) {
        I2C_Recover();
        i2c_fail_count = 0;
    }

    return ret;
}

/**
 * @brief 带自动恢复的 I2C 写操作
 */
/* 【中文名】安全写：往某设备的某寄存器写一段数据。逻辑和 SafeRead 完全对称 */
static HAL_StatusTypeDef I2C_SafeWrite(uint8_t dev_addr, uint8_t reg,
                                        uint8_t *data, uint8_t len, uint32_t timeout_ms)
{
    HAL_StatusTypeDef ret;

    /* 冷却期内直接返回（同上） */
    if (i2c_cooldown_until_ms != 0) {
        uint32_t now = HAL_GetTick();
        if (now < i2c_cooldown_until_ms) {
            return HAL_BUSY;
        }
        i2c_cooldown_until_ms = 0;
        if (!I2C_Recover()) {
            return HAL_BUSY;
        }
    }

    /* 真正写：往设备(dev_addr)的寄存器(reg)写 len 字节 */
    ret = HAL_I2C_Mem_Write(&hi2c1, dev_addr << 1, reg,
                            I2C_MEMADD_SIZE_8BIT, data, len, timeout_ms);

    if (ret == HAL_OK) {
        i2c_fail_count = 0;
        i2c_ok_count++;
        return HAL_OK;
    }

    i2c_fail_count++;
    i2c_ok_count = 0;
    if (i2c_fail_count >= 3) {
        I2C_Recover();
        i2c_fail_count = 0;
    }

    return ret;
}

/* -------------------------------------------------------------------------- */
/* I2C 基础操作（下面这些是"最底层"的读写封装，其他函数都调它们）                */
/* -------------------------------------------------------------------------- */

/* 【中文名】写 MPU9250 的一个寄存器（写1字节） */
static HAL_StatusTypeDef MPU_WriteReg(uint8_t reg, uint8_t value)
{
    return I2C_SafeWrite(MPU9250_ADDR, reg, &value, 1, I2C_TIMEOUT_MS);
}

/* 【中文名】读 MPU9250 的寄存器（一次读多个字节） */
static HAL_StatusTypeDef MPU_ReadReg(uint8_t reg, uint8_t *data, uint8_t len)
{
    return I2C_SafeRead(MPU9250_ADDR, reg, data, len, I2C_TIMEOUT_MS);
}

/* 【中文名】读 MPU9250 的一个寄存器（单字节） */
static HAL_StatusTypeDef MPU_ReadReg8(uint8_t reg, uint8_t *value)
{
    return MPU_ReadReg(reg, value, 1);
}

/* 【中文名】写 AK8963(磁力计) 的一个寄存器。AK8963 挂在 MPU9250 后面，通过"旁路"访问 */
static HAL_StatusTypeDef AK_WriteReg(uint8_t reg, uint8_t value)
{
    return I2C_SafeWrite(AK8963_ADDR, reg, &value, 1, I2C_TIMEOUT_MS);
}

/* 【中文名】读 AK8963 的寄存器（多字节） */
static HAL_StatusTypeDef AK_ReadReg(uint8_t reg, uint8_t *data, uint8_t len)
{
    return I2C_SafeRead(AK8963_ADDR, reg, data, len, I2C_TIMEOUT_MS);
}

/* 【中文名】读 AK8963 的单字节寄存器 */
static HAL_StatusTypeDef AK_ReadReg8(uint8_t reg, uint8_t *value)
{
    return AK_ReadReg(reg, value, 1);
}

/* -------------------------------------------------------------------------- */
/* 使能/禁用 I2C 旁路模式 (直通 AK8963)                                       */
/* -------------------------------------------------------------------------- */
/* 什么是"旁路"？AK8963 磁力计不是直接挂 I2C 总线上的，它是挂在 MPU9250 后面。
 * MPU9250 有个"旁路开关"，打开后，外面的 I2C 信号就能直接穿透到磁力计。
 * 所以读磁力计前必须先"开旁路"。 */
static void MPU_I2CBypass_Enable(void)
{
    /* INT_PIN_CFG 寄存器的第1位写1 = 开旁路（主机 I2C 直连磁力计） */
    MPU_WriteReg(MPU9250_REG_INT_PIN_CFG, 0x02);
    HAL_Delay(1);
}

/* 【中文名】关旁路 */
static void MPU_I2CBypass_Disable(void)
{
    MPU_WriteReg(MPU9250_REG_INT_PIN_CFG, 0x00);
    HAL_Delay(1);
}

/* -------------------------------------------------------------------------- */
/* 初始化磁力计 AK8963                                                        */
/* -------------------------------------------------------------------------- */
/* 【中文名】初始化磁力计 AK8963：开旁路→复位→验ID→读ASA→设成连续测量模式 */
static int AK8963_Init(AK8963_Mode mode)
{
    uint8_t id = 0;
    uint8_t raw[3];

    /* 1. 开旁路（否则够不到磁力计） */
    MPU_I2CBypass_Enable();

    /* 2. 复位磁力计：CNTL2 寄存器写 0x01（软复位），等10ms */
    AK_WriteReg(AK8963_REG_CNTL2, 0x01);
    HAL_Delay(10);

    /* 3. 读设备ID(WIA)，应该等于 0x48。等于才是真的 AK8963 */
    if (AK_ReadReg8(AK8963_REG_WIA, &id) != HAL_OK) {
        DBG_ERROR(TAG_MPU, "AK8963 I2C read WIA failed! Check if magnetometer is present");
        MPU_I2CBypass_Disable();
        return -3;  /* 磁力计通信失败 */
    }
    DBG_DEBUG(TAG_MPU, "AK8963 WIA = 0x%02X (expected 0x48)", id);
    if (id != 0x48) {
        DBG_ERROR(TAG_MPU, "AK8963 WIA mismatch: got 0x%02X, expected 0x48", id);
        MPU_I2CBypass_Disable();
        return -4;  /* 磁力计 ID 不匹配 */
    }

    /* 4. 读出厂灵敏度校准值 ASA（每颗芯片出厂时标定好的系数，存 FUSE ROM 里） */
    AK_WriteReg(AK8963_REG_CNTL1, 0x0F);  /* 切到 FUSE ROM 访问模式 */
    HAL_Delay(10);

    if (AK_ReadReg(AK8963_REG_ASAX, raw, 3) == HAL_OK) {
        /* 公式: actual = (raw - 128) / 256 + 1（手册规定） */
        mag_asa_x = ((float)raw[0] - 128.0f) / 256.0f + 1.0f;
        mag_asa_y = ((float)raw[1] - 128.0f) / 256.0f + 1.0f;
        mag_asa_z = ((float)raw[2] - 128.0f) / 256.0f + 1.0f;
        DBG_DEBUG(TAG_MPU, "AK8963 ASA: X=%.3f Y=%.3f Z=%.3f", mag_asa_x, mag_asa_y, mag_asa_z);
    } else {
        DBG_WARN(TAG_MPU, "AK8963 ASA read failed, using default=1.0");
    }

    /* 5. 先掉电（写0），等10ms，再进入正常工作模式 */
    AK_WriteReg(AK8963_REG_CNTL1, 0x00);  /* Power-down */
    HAL_Delay(10);

    /* 6. 设成"连续测量模式"(16-bit)：mode|0x10 = 在模式基础上加"16位输出"位。
     * 写进去后再读回来核对，写失败就重试3次 */
    uint8_t cntl1_mode = (uint8_t)(mode | 0x10);
    int mode_ok = 0;
    for (int retry = 0; retry < 3; retry++) {
        if (AK_WriteReg(AK8963_REG_CNTL1, cntl1_mode) == HAL_OK) {
            uint8_t chk = 0;
            if (AK_ReadReg8(AK8963_REG_CNTL1, &chk) == HAL_OK && chk == cntl1_mode) {
                mode_ok = 1;   /* 写进去并核对成功 */
                break;
            }
        }
        HAL_Delay(10);
    }
    if (!mode_ok) {
        DBG_ERROR(TAG_MPU, "AK8963 CNTL1 mode write/verify failed (want 0x%02X)", cntl1_mode);
        return -5;  /* 磁力计模式设置失败 */
    }
    HAL_Delay(10);

    /* 7. 旁路保持开启（这样后面才能继续读磁力计数据） */
    /* MPU_I2CBypass_Disable();  — 故意不关，保持开启以便读取 */

    return 0;
}

/* -------------------------------------------------------------------------- */
/* 初始化 MPU9250                                                             */
/* -------------------------------------------------------------------------- */
/* 【中文名】初始化整个九轴传感器：复位→唤醒→验ID→设量程/采样率→初始化磁力计 */
int MPU9250_Init(MPU9250_GyroRange gyro_range, MPU9250_AccelRange accel_range,
                 MPU9250_DLPF dlpf, AK8963_Mode mag_mode)
{
    uint8_t whoami = 0;

    /* --- MPU9250 部分（先配置主芯片） --- */

    /* 复位设备：PWR_MGMT_1 写0x80（复位位），等100ms */
    MPU_WriteReg(MPU9250_REG_PWR_MGMT_1, 0x80);
    HAL_Delay(100);

    /* 唤醒并选择时钟源：写0x01（=退出睡眠，用内部振荡器起步） */
    MPU_WriteReg(MPU9250_REG_PWR_MGMT_1, 0x01);
    HAL_Delay(10);

    /* 验证设备ID：WHO_AM_I 应该是 0x70（注意：实际是MPU6500内核，仍是0x70） */
    if (MPU_ReadReg8(MPU9250_REG_WHO_AM_I, &whoami) != HAL_OK) {
        DBG_ERROR(TAG_MPU, "I2C read WHO_AM_I failed! Check SDA/SCL wiring");
        return -1;
    }
    DBG_DEBUG(TAG_MPU, "MPU9250 WHO_AM_I = 0x%02X (expected 0x70)", whoami);
    if (whoami != 0x70) {
        DBG_ERROR(TAG_MPU, "WHO_AM_I mismatch: got 0x%02X, expected 0x70", whoami);
        return -2;
    }

    /* 采样率分频器给 0 = 采样率最高（1kHz） */
    MPU_WriteReg(MPU9250_REG_SMPLRT_DIV, 0x00);

    /* 低通滤波 DLPF（平滑数据，减少噪声） */
    MPU_WriteReg(MPU9250_REG_CONFIG, (uint8_t)dlpf);

    /* 陀螺仪量程（决定满刻度多少，也决定换算系数） */
    MPU_WriteReg(MPU9250_REG_GYRO_CONFIG, (uint8_t)gyro_range);

    /* 加速度计量程 */
    MPU_WriteReg(MPU9250_REG_ACCEL_CONFIG, (uint8_t)accel_range);

    /* 加速度计 DLPF（MPU9250 独有寄存器），0x03=41Hz 带宽 */
    MPU_WriteReg(MPU9250_REG_ACCEL_CONFIG2, 0x03);

    /* 根据量程，设置好对应的"换算系数"（LSB→物理量） */
    switch (gyro_range) {
        case GYRO_FS_250:  gyro_lsb_per_dps = 131.0f;   break;  /* ±250度/秒，131 LSB=1度 */
        case GYRO_FS_500:  gyro_lsb_per_dps = 65.5f;    break;
        case GYRO_FS_1000: gyro_lsb_per_dps = 32.8f;    break;
        case GYRO_FS_2000: gyro_lsb_per_dps = 16.4f;    break;
    }
    switch (accel_range) {
        case ACCEL_FS_2G:  accel_lsb_per_g = 16384.0f;  break;  /* ±2g，16384 LSB=1g */
        case ACCEL_FS_4G:  accel_lsb_per_g = 8192.0f;   break;
        case ACCEL_FS_8G:  accel_lsb_per_g = 4096.0f;   break;
        case ACCEL_FS_16G: accel_lsb_per_g = 2048.0f;   break;
    }

    /* --- AK8963 磁力计部分 --- */
    int mag_ret = AK8963_Init(mag_mode);

    return mag_ret;  /* 0=全部成功, <0=磁力计初始化失败 */
}

/* -------------------------------------------------------------------------- */
/* 设备 ID                                                                     */
/* -------------------------------------------------------------------------- */
/* 【中文名】读主芯片 MPU9250 的"我是谁"ID（应该=0x70），用来确认接线正确 */
uint8_t MPU9250_ReadWhoAmI(void)
{
    uint8_t id = 0;
    MPU_ReadReg8(MPU9250_REG_WHO_AM_I, &id);
    return id;
}

/* 【中文名】读磁力计 AK8963 的"我是谁"ID（应该=0x48），注意要先开旁路 */
uint8_t MPU9250_ReadMagWhoAmI(void)
{
    uint8_t id = 0;
    MPU_I2CBypass_Enable();   /* 先开旁路才能访问磁力计 */
    AK_ReadReg8(AK8963_REG_WIA, &id);
    return id;
}

/* -------------------------------------------------------------------------- */
/* 读取全部 9 轴数据 + 温度                                                    */
/* -------------------------------------------------------------------------- */
/* 【中文名】全量读取（本文件核心读函数）：一次读出加速度计+温度+陀螺仪+磁力计，换算成物理量 */
int MPU9250_ReadAll(MPU9250_Data *data)
{
    uint8_t raw[14];   /* 暂存原始字节 */
    int16_t raw_val;   /* 两个字节拼成一个16位有符号数 */

    /* 1. 读加速度计 + 温度 + 陀螺仪，共14字节（从 ACCEL_XOUT_H 开始连续读） */
    if (MPU_ReadReg(MPU9250_REG_ACCEL_XOUT_H, raw, 14) != HAL_OK) {
        return -1;
    }

    /* 加速度计：每轴2字节，高字节在前。
     * (raw[0]<<8)|raw[1] = 把高字节左移8位再和低字节拼成16位数。
     * 然后 ÷换算系数 − 校准偏移 = 物理量(g) */
    raw_val = (int16_t)((raw[0] << 8) | raw[1]);
    data->accel_raw[0] = raw_val;   /* 原始值也存一份 */
    data->accel_x = (float)raw_val / accel_lsb_per_g - calib_data.accel_offset_x;
    raw_val = (int16_t)((raw[2] << 8) | raw[3]);
    data->accel_raw[1] = raw_val;
    data->accel_y = (float)raw_val / accel_lsb_per_g - calib_data.accel_offset_y;
    raw_val = (int16_t)((raw[4] << 8) | raw[5]);
    data->accel_raw[2] = raw_val;
    data->accel_z = (float)raw_val / accel_lsb_per_g - calib_data.accel_offset_z;

    /* 温度：公式 = 读数/333.87 + 21（摄氏） */
    raw_val = (int16_t)((raw[6] << 8) | raw[7]);
    data->temperature = (float)raw_val / 333.87f + 21.0f;

    /* 陀螺仪：同样每轴2字节，÷换算系数 − 零偏 = 物理量(°/s) */
    raw_val = (int16_t)((raw[8] << 8) | raw[9]);
    data->gyro_raw[0] = raw_val;
    data->gyro_x = (float)raw_val / gyro_lsb_per_dps - calib_data.gyro_offset_x;
    raw_val = (int16_t)((raw[10] << 8) | raw[11]);
    data->gyro_raw[1] = raw_val;
    data->gyro_y = (float)raw_val / gyro_lsb_per_dps - calib_data.gyro_offset_y;
    raw_val = (int16_t)((raw[12] << 8) | raw[13]);
    data->gyro_raw[2] = raw_val;
    data->gyro_z = (float)raw_val / gyro_lsb_per_dps - calib_data.gyro_offset_z;

    /* 2. 读磁力计（要先确保旁路已开） */
    /* 先读 ST1 状态寄存器，看 DRDY 位(最低位)是不是"有数据就绪" */
    uint8_t st1 = 0;
    if (AK_ReadReg8(AK8963_REG_ST1, &st1) == HAL_OK && (st1 & 0x01)) {
        /* ⚠ 关键：一次读 7 字节，把紧跟数据之后的 ST2(0x09) 也读走。
         * 这是 2026-10-18 修复的"磁力计冻结"bug：
         * AK8963 要求每次读完数据必须读 ST2 解锁，否则 DRDY 不再置位、数据永远不更新 */
        uint8_t m[7];  /* HXL, HXH, HYL, HYH, HZL, HZH, ST2 */
        if (AK_ReadReg(AK8963_REG_HXL, m, 7) == HAL_OK) {
            /* 磁力计：×量程系数 × 出厂ASA灵敏度 − 硬铁偏移，再 ×软铁尺度 */
            raw_val = (int16_t)((m[1] << 8) | m[0]);
            data->mag_raw[0] = raw_val;
            data->mag_x = ((float)raw_val * mag_lsb_per_uT * mag_asa_x
                           - calib_data.mag_offset_x) * calib_data.mag_scale_x;
            raw_val = (int16_t)((m[3] << 8) | m[2]);
            data->mag_raw[1] = raw_val;
            data->mag_y = ((float)raw_val * mag_lsb_per_uT * mag_asa_y
                           - calib_data.mag_offset_y) * calib_data.mag_scale_y;
            raw_val = (int16_t)((m[5] << 8) | m[4]);
            data->mag_raw[2] = raw_val;
            data->mag_z = ((float)raw_val * mag_lsb_per_uT * mag_asa_z
                           - calib_data.mag_offset_z) * calib_data.mag_scale_z;

            /* 缓存本次成功读数（供下次读失败时兜底） */
            last_mag[0] = data->mag_x;
            last_mag[1] = data->mag_y;
            last_mag[2] = data->mag_z;
        }
    } else {
        /* 读失败 → 保持上次成功值，而不是置0（否则 AHRS 和遥测会突然拿到 0 的假数据） */
        data->mag_x = last_mag[0];
        data->mag_y = last_mag[1];
        data->mag_z = last_mag[2];
    }

    return 0;
}

/* -------------------------------------------------------------------------- */
/* 陀螺仪快速读取 (仅 6 字节, 不读加速度计和磁力计, 避免阻塞主循环)            */
/* -------------------------------------------------------------------------- */

/**
 * @brief 仅读取陀螺仪 3 轴 (6 bytes), 无磁力计/加速度计
 * @note  用于 AHRS 高频更新, 避免每周期触发超时的磁力计读取
 * @return 0=成功, -1=I2C 失败
 */
/* 【中文名】只读陀螺仪（快速版）：只读6字节，跳过磁力计和加速度计。因为AHRS每10ms都要读陀螺，
 * 全量读太慢、还会触发磁力计的超时等待，所以给陀螺单开一个"快速通道" */
static int MPU9250_ReadGyroFast(float *gx, float *gy, float *gz)
{
    uint8_t raw[6];
    if (MPU_ReadReg(MPU9250_REG_GYRO_XOUT_H, raw, 6) != HAL_OK) {
        return -1;
    }
    /* 三轴各2字节，拼成16位，÷换算系数 − 零偏 = 物理量(°/s) */
    int16_t raw_gx = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t raw_gy = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t raw_gz = (int16_t)((raw[4] << 8) | raw[5]);
    *gx = (float)raw_gx / gyro_lsb_per_dps - calib_data.gyro_offset_x;
    *gy = (float)raw_gy / gyro_lsb_per_dps - calib_data.gyro_offset_y;
    *gz = (float)raw_gz / gyro_lsb_per_dps - calib_data.gyro_offset_z;
    return 0;
}

/* -------------------------------------------------------------------------- */
/* 各轴单独读取 (Gyro 使用快速通道; 其他仍需全量读取)                         */
/* -------------------------------------------------------------------------- */
/* 【中文名】读加速度计三轴（全量读然后取加速度部分） */
int MPU9250_ReadAccel(float *ax, float *ay, float *az)
{
    MPU9250_Data data;
    if (MPU9250_ReadAll(&data) != 0) return -1;
    *ax = data.accel_x; *ay = data.accel_y; *az = data.accel_z;
    return 0;
}

/* 【中文名】读陀螺三轴：优先走快速通道(只6字节)，失败了再退回全量读 */
int MPU9250_ReadGyro(float *gx, float *gy, float *gz)
{
    if (MPU9250_ReadGyroFast(gx, gy, gz) == 0) return 0;   /* 快速通道成功就返回 */

    /* 快速通道失败 → 试全量读（还能顺带触发 I2C 恢复） */
    MPU9250_Data data;
    if (MPU9250_ReadAll(&data) != 0) return -1;
    *gx = data.gyro_x; *gy = data.gyro_y; *gz = data.gyro_z;
    return 0;
}

/* 【中文名】读磁力计三轴（全量读然后取磁力计部分） */
int MPU9250_ReadMag(float *mx, float *my, float *mz)
{
    MPU9250_Data data;
    if (MPU9250_ReadAll(&data) != 0) return -1;
    *mx = data.mag_x; *my = data.mag_y; *mz = data.mag_z;
    return 0;
}

/* 【中文名】读温度 */
int MPU9250_ReadTemp(float *temp)
{
    MPU9250_Data data;
    if (MPU9250_ReadAll(&data) != 0) return -1;
    *temp = data.temperature;
    return 0;
}

/* -------------------------------------------------------------------------- */
/* 陀螺仪校准                                                                 */
/* -------------------------------------------------------------------------- */
/* 【中文名】校准陀螺零偏：车静止时采N个样求平均，平均值就是零偏。
 * 原理：静止时真实转速=0，所以读到的数全是零偏 */
void MPU9250_CalibrateGyro(uint16_t samples, MPU9250_Calib *calib)
{
    if (samples == 0 || calib == NULL) return;

    float sum_gx = 0, sum_gy = 0, sum_gz = 0;   /* 三个累加器 */

    memset(&calib_data, 0, sizeof(MPU9250_Calib));   /* 先清空校准数据 */

    for (uint16_t i = 0; i < samples; i++) {
        /* 只读陀螺仪（更快），丢弃磁力计和加速度计 */
        uint8_t raw[6];
        if (MPU_ReadReg(MPU9250_REG_GYRO_XOUT_H, raw, 6) == HAL_OK) {
            int16_t gx = (int16_t)((raw[0] << 8) | raw[1]);
            int16_t gy = (int16_t)((raw[2] << 8) | raw[3]);
            int16_t gz = (int16_t)((raw[4] << 8) | raw[5]);
            sum_gx += (float)gx / gyro_lsb_per_dps;   /* 累加（换算成°/s后） */
            sum_gy += (float)gy / gyro_lsb_per_dps;
            sum_gz += (float)gz / gyro_lsb_per_dps;
        }
        HAL_Delay(1);   /* 隔1ms取一次，让传感器出新数据 */
    }

    /* 平均值 = 零偏 */
    calib->gyro_offset_x = sum_gx / (float)samples;
    calib->gyro_offset_y = sum_gy / (float)samples;
    calib->gyro_offset_z = sum_gz / (float)samples;
    /* 其它校准值归默认（加速度和磁力计这次不校准） */
    calib->accel_offset_x = 0.0f;
    calib->accel_offset_y = 0.0f;
    calib->accel_offset_z = 0.0f;
    calib->mag_offset_x = 0.0f;
    calib->mag_offset_y = 0.0f;
    calib->mag_offset_z = 0.0f;
    calib->mag_scale_x = 1.0f;
    calib->mag_scale_y = 1.0f;
    calib->mag_scale_z = 1.0f;

    memcpy(&calib_data, calib, sizeof(MPU9250_Calib));   /* 同步到全局校准数据 */
}

/* 【中文名】设置校准数据（从外面传一份进来覆盖） */
void MPU9250_SetCalibration(const MPU9250_Calib *calib)
{
    if (calib) {
        memcpy(&calib_data, calib, sizeof(MPU9250_Calib));
    }
}

/* 【中文名】拿校准数据（只读，返回指针） */
const MPU9250_Calib *MPU9250_GetCalibration(void)
{
    return &calib_data;
}

/* -------------------------------------------------------------------------- */
/* 磁力计 2D 平面校准（硬铁 + 软铁）                                          */
/*                                                                             */
/*  原理: 理想无干扰时，水平旋转一周的 mag_x, mag_y 应画出一个圆心在原点       */
/*        的圆。实际环境中：                                                   */
/*        - 硬铁 (hard iron): 圆偏移了 → 用 (max+min)/2 作为偏移量减去        */
/*        - 软铁 (soft iron): 圆被拉成椭圆 → 用轴比作为尺度因子补偿           */
/*                                                                             */
/*  步骤:                                                                      */
/*    1. 用户水平旋转机器人 ≥360°, 采集 mag_x, mag_y 样本                     */
/*    2. 找 min/max → 计算硬铁偏移: offset = (max+min)/2                      */
/*    3. 计算半轴: semi = (max-min)/2                                          */
/*    4. 平均半径: avg_r = (semi_x + semi_y) / 2                              */
/*    5. 软铁尺度: scale = avg_r / semi                                        */
/*                                                                             */
/*  校正后值: mag_corrected = (mag_raw - offset) * scale                       */
/* -------------------------------------------------------------------------- */
/* 【中文名】磁力计2D校准（对应 MAGCAL 命令）：水平转一圈，算硬铁偏移和软铁尺度，让磁航向更准 */
void MPU9250_CalibrateMag(uint32_t duration_ms, MPU9250_Calib *calib)
{
    if (calib == NULL || duration_ms == 0) return;

    /* 限制范围：最短3秒，最长60秒 */
    if (duration_ms < 3000)  duration_ms = 3000;
    if (duration_ms > 60000) duration_ms = 60000;

    /* 四个变量记录 mag_x/mag_y 的"最大最小值"，初始给极端值方便比较 */
    float mag_x_min =  1e9f, mag_x_max = -1e9f;
    float mag_y_min =  1e9f, mag_y_max = -1e9f;
    uint32_t sample_count = 0;
    uint32_t start_tick = HAL_GetTick();
    uint32_t last_report = start_tick;

    /* 采集阶段：持续 duration_ms，每2ms采一个点，记录最大最小值 */
    while (HAL_GetTick() - start_tick < duration_ms) {
        float mx, my, mz;
        if (MPU9250_ReadMag(&mx, &my, &mz) == 0) {
            if (mx < mag_x_min) mag_x_min = mx;   /* 更新 min/max */
            if (mx > mag_x_max) mag_x_max = mx;
            if (my < mag_y_min) mag_y_min = my;
            if (my > mag_y_max) mag_y_max = my;
            sample_count++;
        }
        HAL_Delay(2);

        /* 每秒翻转一次LED，告诉用户"我在干活"，防止以为卡死 */
        if (HAL_GetTick() - last_report >= 1000) {
            last_report = HAL_GetTick();
            uint32_t elapsed = last_report - start_tick;
            uint32_t remaining = duration_ms - elapsed;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            (void)remaining;
        }
    }

    /* 样本太少(<50)就不校准，保持原值 */
    if (sample_count < 50) {
        return;
    }

    /* ---- 硬铁偏移 = (最大值 + 最小值) / 2：把圆心从偏移处拉回原点 ---- */
    float hard_x = (mag_x_max + mag_x_min) / 2.0f;
    float hard_y = (mag_y_max + mag_y_min) / 2.0f;

    /* ---- 软铁尺度：椭圆轴比（圆被拉成椭圆后，用轴长比扯回正圆） ---- */
    float semi_x = (mag_x_max - mag_x_min) / 2.0f;   /* x 半轴长 */
    float semi_y = (mag_y_max - mag_y_min) / 2.0f;   /* y 半轴长 */
    if (semi_x <= 0.0f) semi_x = 1.0f;
    if (semi_y <= 0.0f) semi_y = 1.0f;
    float avg_radius = (semi_x + semi_y) / 2.0f;     /* 平均半径 */
    float scale_x = avg_radius / semi_x;             /* 尺度因子 */
    float scale_y = avg_radius / semi_y;

    /* 保存到 calib 结构体 */
    calib->mag_offset_x = hard_x;
    calib->mag_offset_y = hard_y;
    calib->mag_offset_z = 0.0f;
    calib->mag_scale_x  = scale_x;
    calib->mag_scale_y  = scale_y;
    calib->mag_scale_z  = 1.0f;

    /* 同步到全局校准数据 */
    calib_data.mag_offset_x = hard_x;
    calib_data.mag_offset_y = hard_y;
    calib_data.mag_offset_z = 0.0f;
    calib_data.mag_scale_x  = scale_x;
    calib_data.mag_scale_y  = scale_y;
    calib_data.mag_scale_z  = 1.0f;
}
