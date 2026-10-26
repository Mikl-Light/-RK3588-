/**
  ******************************************************************************
  * @file    mpu9250.c
  * @brief   MPU9250 九轴传感器驱动实现
  *          MPU9250 = MPU6500 (6-axis) + AK8963 (3-axis magnetometer)
  ******************************************************************************
  */
#include "mpu9250.h"
#include "debug.h"
#include "gpio.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* I2C 超时配置 (电机运行时 I2C 易受 EMI 干扰, 必须快速失败)                  */
/* -------------------------------------------------------------------------- */
#define I2C_TIMEOUT_MS       3      /**< 正常运行超时 (ms), 100kHz I2C 6字节≈0.5ms */
#define I2C_INIT_TIMEOUT_MS 50      /**< 初始化阶段超时 (ms), 允许稍长等待    */
#define I2C_COOLDOWN_MS    500      /**< 恢复失败后冷却期 (ms), 期间跳过 I2C   */
#define I2C_LOG_ENABLE       0      /**< 调试电机时关掉: 0=静音, 1=输出日志     */

/* -------------------------------------------------------------------------- */
/* 内部变量                                                                   */
/* -------------------------------------------------------------------------- */
static float gyro_lsb_per_dps  = 131.0f;
static float accel_lsb_per_g   = 16384.0f;
static float mag_lsb_per_uT    = 0.15f;    /**< AK8963: ~0.15 µT/LSB @ 16-bit */
static MPU9250_Calib calib_data = {0};

/* 磁力计上次成功读数 (读失败时保持旧值, 避免遥测里 mag 突然变 0) */
static float last_mag[3] = {0.0f, 0.0f, 0.0f};

/* I2C 故障统计 (用于自动恢复) */
static uint32_t i2c_fail_count = 0;     /**< 连续 I2C 失败计数  */
static uint32_t i2c_ok_count   = 0;     /**< 连续 I2C 成功计数  */

/* I2C 冷却期机制: 恢复失败后一段时间内跳过所有 I2C 操作, 避免日志刷屏 */
static uint32_t i2c_cooldown_until_ms = 0;  /**< 冷却期结束时间戳 (ms) */

/* 磁力计灵敏度校准 (ASA) — 从 FUSE ROM 读取 */
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
static bool I2C_Recover(void)
{
    uint32_t now = HAL_GetTick();

    /* ---- BUSY 已清除 → 总线正常 ---- */
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) == RESET) {
        if (i2c_cooldown_until_ms != 0) {
            /* 冷却期内 BUSY 自行清除，报告恢复 */
            i2c_cooldown_until_ms = 0;
#if I2C_LOG_ENABLE
            DBG_INFO(TAG_MPU, "I2C1 recovered OK");
#endif
        }
        return true;
    }

    /* ---- BUSY 卡死 ---- */

    /* 冷却期内: 直接返回, 不重试, 不刷日志 */
    if (now < i2c_cooldown_until_ms) {
        return false;
    }

    /* 开始新一轮恢复 (每个冷却期只输出一次 WRN+ERR) */
#if I2C_LOG_ENABLE
    DBG_WARN(TAG_MPU, "I2C1 BUSY stuck (EMI from motors), recovering...");
#endif

    /* 第一步: SWRST 软件复位 */
    hi2c1.Instance->CR1 &= ~I2C_CR1_PE;
    for (volatile int i = 0; i < 1000; i++) { __NOP(); }
    hi2c1.Instance->CR1 |= I2C_CR1_SWRST;
    for (volatile int i = 0; i < 1000; i++) { __NOP(); }
    hi2c1.Instance->CR1 &= ~I2C_CR1_SWRST;
    hi2c1.Instance->CR1 |= I2C_CR1_PE;
    for (volatile int i = 0; i < 1000; i++) { __NOP(); }

    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) == RESET) {
        /* SWRST 成功 */
        /* 注意: SWRST 会清空 I2C 外设全部寄存器 (CR1/CCR/TRISE 时序配置),
         * 必须重新 HAL_I2C_Init 重配, 否则后续通信全失败 (2026-08-16 修复) */
        HAL_I2C_Init(&hi2c1);
        i2c_cooldown_until_ms = 0;
#if I2C_LOG_ENABLE
        DBG_INFO(TAG_MPU, "I2C1 recovered OK (SWRST, re-init)");
#endif
        return true;
    }

    /* 第二步: GPIO 位拆裂恢复 (更强力) */
    I2C_GPIO_Recover();

    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) == RESET) {
        /* GPIO 恢复成功 */
        /* 同上: GPIO 切换会清掉 I2C 外设配置 (CR1 PE 被关过), 重新初始化 */
        HAL_I2C_Init(&hi2c1);
        i2c_cooldown_until_ms = 0;
#if I2C_LOG_ENABLE
        DBG_INFO(TAG_MPU, "I2C1 recovered OK (GPIO, re-init)");
#endif
        return true;
    }

    /* 恢复失败 → 进入冷却期, 期间跳过所有 I2C */
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
static HAL_StatusTypeDef I2C_SafeRead(uint8_t dev_addr, uint8_t reg,
                                       uint8_t *data, uint8_t len, uint32_t timeout_ms)
{
    HAL_StatusTypeDef ret;

    /* 冷却期内直接返回, 不发起 I2C 通讯, 不刷日志 */
    if (i2c_cooldown_until_ms != 0) {
        uint32_t now = HAL_GetTick();
        if (now < i2c_cooldown_until_ms) {
            return HAL_BUSY;
        }
        /* 冷却期已过 → 尝试恢复, 若失败则重新进入冷却期 */
        i2c_cooldown_until_ms = 0;
        if (!I2C_Recover()) {
            return HAL_BUSY;  /* 恢复失败, 冷却期已重新设定 */
        }
    }

    ret = HAL_I2C_Mem_Read(&hi2c1, dev_addr << 1, reg,
                           I2C_MEMADD_SIZE_8BIT, data, len, timeout_ms);

    if (ret == HAL_OK) {
        i2c_fail_count = 0;
        i2c_ok_count++;
        return HAL_OK;
    }

    /* I2C 失败 → 计数并尝试恢复 */
    i2c_fail_count++;
    i2c_ok_count = 0;

    /* 连续失败 ≥3 次 → 尝试复位 I2C 总线 */
    if (i2c_fail_count >= 3) {
        I2C_Recover();
        i2c_fail_count = 0;
    }

    return ret;
}

/**
 * @brief 带自动恢复的 I2C 写操作
 */
static HAL_StatusTypeDef I2C_SafeWrite(uint8_t dev_addr, uint8_t reg,
                                        uint8_t *data, uint8_t len, uint32_t timeout_ms)
{
    HAL_StatusTypeDef ret;

    /* 冷却期内直接返回 */
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
/* I2C 基础操作                                                               */
/* -------------------------------------------------------------------------- */

/** 写 MPU9250 寄存器 */
static HAL_StatusTypeDef MPU_WriteReg(uint8_t reg, uint8_t value)
{
    return I2C_SafeWrite(MPU9250_ADDR, reg, &value, 1, I2C_TIMEOUT_MS);
}

/** 读 MPU9250 寄存器（多字节） */
static HAL_StatusTypeDef MPU_ReadReg(uint8_t reg, uint8_t *data, uint8_t len)
{
    return I2C_SafeRead(MPU9250_ADDR, reg, data, len, I2C_TIMEOUT_MS);
}

/** 读 MPU9250 单字节寄存器 */
static HAL_StatusTypeDef MPU_ReadReg8(uint8_t reg, uint8_t *value)
{
    return MPU_ReadReg(reg, value, 1);
}

/** 写 AK8963 寄存器 (通过 I2C 旁路) */
static HAL_StatusTypeDef AK_WriteReg(uint8_t reg, uint8_t value)
{
    return I2C_SafeWrite(AK8963_ADDR, reg, &value, 1, I2C_TIMEOUT_MS);
}

/** 读 AK8963 寄存器 */
static HAL_StatusTypeDef AK_ReadReg(uint8_t reg, uint8_t *data, uint8_t len)
{
    return I2C_SafeRead(AK8963_ADDR, reg, data, len, I2C_TIMEOUT_MS);
}

/** 读 AK8963 单字节 */
static HAL_StatusTypeDef AK_ReadReg8(uint8_t reg, uint8_t *value)
{
    return AK_ReadReg(reg, value, 1);
}

/* -------------------------------------------------------------------------- */
/* 使能/禁用 I2C 旁路模式 (直通 AK8963)                                       */
/* -------------------------------------------------------------------------- */
static void MPU_I2CBypass_Enable(void)
{
    /* INT_PIN_CFG[1] = 1: 旁路使能，主机 I2C 直连到磁力计 */
    MPU_WriteReg(MPU9250_REG_INT_PIN_CFG, 0x02);
    HAL_Delay(1);
}

static void MPU_I2CBypass_Disable(void)
{
    MPU_WriteReg(MPU9250_REG_INT_PIN_CFG, 0x00);
    HAL_Delay(1);
}

/* -------------------------------------------------------------------------- */
/* 初始化磁力计 AK8963                                                        */
/* -------------------------------------------------------------------------- */
static int AK8963_Init(AK8963_Mode mode)
{
    uint8_t id = 0;
    uint8_t raw[3];

    /* 1. 使能 I2C 旁路 */
    MPU_I2CBypass_Enable();

    /* 2. 复位磁力计 */
    AK_WriteReg(AK8963_REG_CNTL2, 0x01);
    HAL_Delay(10);

    /* 3. 验证设备 ID */
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

    /* 4. 读取 FUSE ROM 灵敏度校准值 (ASA) */
    /* 先进入 FUSE ROM 访问模式 */
    AK_WriteReg(AK8963_REG_CNTL1, 0x0F);  /* FUSE ROM 模式 */
    HAL_Delay(10);

    if (AK_ReadReg(AK8963_REG_ASAX, raw, 3) == HAL_OK) {
        /* 公式: actual = (raw - 128) / 256 + 1 */
        mag_asa_x = ((float)raw[0] - 128.0f) / 256.0f + 1.0f;
        mag_asa_y = ((float)raw[1] - 128.0f) / 256.0f + 1.0f;
        mag_asa_z = ((float)raw[2] - 128.0f) / 256.0f + 1.0f;
        DBG_DEBUG(TAG_MPU, "AK8963 ASA: X=%.3f Y=%.3f Z=%.3f", mag_asa_x, mag_asa_y, mag_asa_z);
    } else {
        DBG_WARN(TAG_MPU, "AK8963 ASA read failed, using default=1.0");
    }

    /* 5. 掉电后再设置模式 */
    AK_WriteReg(AK8963_REG_CNTL1, 0x00);  /* Power-down */
    HAL_Delay(10);

    /* 6. 设置连续测量模式 (16-bit 输出) */
    AK_WriteReg(AK8963_REG_CNTL1, (uint8_t)(mode | 0x10));  /* 16-bit */
    HAL_Delay(10);

    /* 7. 关闭旁路（可选：保持开启方便后续读取） */
    /* MPU_I2CBypass_Disable();  — 保持开启以便读取磁力计数据 */

    return 0;
}

/* -------------------------------------------------------------------------- */
/* 初始化 MPU9250                                                             */
/* -------------------------------------------------------------------------- */
int MPU9250_Init(MPU9250_GyroRange gyro_range, MPU9250_AccelRange accel_range,
                 MPU9250_DLPF dlpf, AK8963_Mode mag_mode)
{
    uint8_t whoami = 0;

    /* --- MPU9250 部分 --- */

    /* 复位设备 */
    MPU_WriteReg(MPU9250_REG_PWR_MGMT_1, 0x80);
    HAL_Delay(100);

    /* 唤醒，时钟源 = 自动选择最佳 */
    MPU_WriteReg(MPU9250_REG_PWR_MGMT_1, 0x01);
    HAL_Delay(10);

    /* 验证设备 ID */
    if (MPU_ReadReg8(MPU9250_REG_WHO_AM_I, &whoami) != HAL_OK) {
        DBG_ERROR(TAG_MPU, "I2C read WHO_AM_I failed! Check SDA/SCL wiring");
        return -1;
    }
    DBG_DEBUG(TAG_MPU, "MPU9250 WHO_AM_I = 0x%02X (expected 0x70)", whoami);
    if (whoami != 0x70) {
        DBG_ERROR(TAG_MPU, "WHO_AM_I mismatch: got 0x%02X, expected 0x70", whoami);
        return -2;
    }

    /* 采样率: 1kHz / (1+0) */
    MPU_WriteReg(MPU9250_REG_SMPLRT_DIV, 0x00);

    /* DLPF */
    MPU_WriteReg(MPU9250_REG_CONFIG, (uint8_t)dlpf);

    /* 陀螺仪量程 */
    MPU_WriteReg(MPU9250_REG_GYRO_CONFIG, (uint8_t)gyro_range);

    /* 加速度计量程 */
    MPU_WriteReg(MPU9250_REG_ACCEL_CONFIG, (uint8_t)accel_range);

    /* 加速度计 DLPF (MPU9250 独有寄存器) */
    MPU_WriteReg(MPU9250_REG_ACCEL_CONFIG2, 0x03);  /* 41Hz 带宽 */

    /* 设置 LSB 换算系数 */
    switch (gyro_range) {
        case GYRO_FS_250:  gyro_lsb_per_dps = 131.0f;   break;
        case GYRO_FS_500:  gyro_lsb_per_dps = 65.5f;    break;
        case GYRO_FS_1000: gyro_lsb_per_dps = 32.8f;    break;
        case GYRO_FS_2000: gyro_lsb_per_dps = 16.4f;    break;
    }
    switch (accel_range) {
        case ACCEL_FS_2G:  accel_lsb_per_g = 16384.0f;  break;
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
uint8_t MPU9250_ReadWhoAmI(void)
{
    uint8_t id = 0;
    MPU_ReadReg8(MPU9250_REG_WHO_AM_I, &id);
    return id;
}

uint8_t MPU9250_ReadMagWhoAmI(void)
{
    uint8_t id = 0;
    MPU_I2CBypass_Enable();
    AK_ReadReg8(AK8963_REG_WIA, &id);
    return id;
}

/* -------------------------------------------------------------------------- */
/* 读取全部 9 轴数据 + 温度                                                    */
/* -------------------------------------------------------------------------- */
int MPU9250_ReadAll(MPU9250_Data *data)
{
    uint8_t raw[14];
    int16_t raw_val;

    /* 1. 读取加速度计 + 温度 + 陀螺仪 (14 bytes) */
    if (MPU_ReadReg(MPU9250_REG_ACCEL_XOUT_H, raw, 14) != HAL_OK) {
        return -1;
    }

    /* 加速度计 */
    raw_val = (int16_t)((raw[0] << 8) | raw[1]);
    data->accel_raw[0] = raw_val;
    data->accel_x = (float)raw_val / accel_lsb_per_g - calib_data.accel_offset_x;
    raw_val = (int16_t)((raw[2] << 8) | raw[3]);
    data->accel_raw[1] = raw_val;
    data->accel_y = (float)raw_val / accel_lsb_per_g - calib_data.accel_offset_y;
    raw_val = (int16_t)((raw[4] << 8) | raw[5]);
    data->accel_raw[2] = raw_val;
    data->accel_z = (float)raw_val / accel_lsb_per_g - calib_data.accel_offset_z;

    /* 温度 */
    raw_val = (int16_t)((raw[6] << 8) | raw[7]);
    data->temperature = (float)raw_val / 333.87f + 21.0f;  /* MPU9250 公式 */

    /* 陀螺仪 */
    raw_val = (int16_t)((raw[8] << 8) | raw[9]);
    data->gyro_raw[0] = raw_val;
    data->gyro_x = (float)raw_val / gyro_lsb_per_dps - calib_data.gyro_offset_x;
    raw_val = (int16_t)((raw[10] << 8) | raw[11]);
    data->gyro_raw[1] = raw_val;
    data->gyro_y = (float)raw_val / gyro_lsb_per_dps - calib_data.gyro_offset_y;
    raw_val = (int16_t)((raw[12] << 8) | raw[13]);
    data->gyro_raw[2] = raw_val;
    data->gyro_z = (float)raw_val / gyro_lsb_per_dps - calib_data.gyro_offset_z;

    /* 2. 读取磁力计 (需要确保 I2C 旁路已开启) */
    /* 检查 ST1 寄存器的 DRDY 位 */
    uint8_t st1 = 0;
    if (AK_ReadReg8(AK8963_REG_ST1, &st1) == HAL_OK && (st1 & 0x01)) {
        uint8_t mag_raw[7];  /* ST1 + HXL~HZH + ST2 */
        if (AK_ReadReg(AK8963_REG_ST1, mag_raw, 7) == HAL_OK) {
            /* 磁力计 X (注意: AK8963 顺序为 HXL, HXH, HYL, HYH, HZL, HZH) */
            raw_val = (int16_t)((mag_raw[2] << 8) | mag_raw[1]);
            data->mag_raw[0] = raw_val;
            data->mag_x = ((float)raw_val * mag_lsb_per_uT * mag_asa_x
                           - calib_data.mag_offset_x) * calib_data.mag_scale_x;
            raw_val = (int16_t)((mag_raw[4] << 8) | mag_raw[3]);
            data->mag_raw[1] = raw_val;
            data->mag_y = ((float)raw_val * mag_lsb_per_uT * mag_asa_y
                           - calib_data.mag_offset_y) * calib_data.mag_scale_y;
            raw_val = (int16_t)((mag_raw[6] << 8) | mag_raw[5]);
            data->mag_raw[2] = raw_val;
            data->mag_z = ((float)raw_val * mag_lsb_per_uT * mag_asa_z
                           - calib_data.mag_offset_z) * calib_data.mag_scale_z;

            /* 缓存本次成功读数 */
            last_mag[0] = data->mag_x;
            last_mag[1] = data->mag_y;
            last_mag[2] = data->mag_z;

            /* 检查溢出 */
            if (mag_raw[7] & 0x08) {
                /* 磁力计溢出，数据可能不正确 */
            }
        }
    } else {
        /* 读取失败 → 保持上次成功值, 不置 0 (否则 AHRS/遥测会拿到假数据) */
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
static int MPU9250_ReadGyroFast(float *gx, float *gy, float *gz)
{
    uint8_t raw[6];
    if (MPU_ReadReg(MPU9250_REG_GYRO_XOUT_H, raw, 6) != HAL_OK) {
        return -1;
    }
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
int MPU9250_ReadAccel(float *ax, float *ay, float *az)
{
    MPU9250_Data data;
    if (MPU9250_ReadAll(&data) != 0) return -1;
    *ax = data.accel_x; *ay = data.accel_y; *az = data.accel_z;
    return 0;
}

int MPU9250_ReadGyro(float *gx, float *gy, float *gz)
{
    /* 优先使用快速通道 (6字节, ~0.3ms)，失败时回退到 ReadAll */
    if (MPU9250_ReadGyroFast(gx, gy, gz) == 0) return 0;

    /* 快速通道失败 → 尝试完整读取 (会连带恢复 I2C 总线) */
    MPU9250_Data data;
    if (MPU9250_ReadAll(&data) != 0) return -1;
    *gx = data.gyro_x; *gy = data.gyro_y; *gz = data.gyro_z;
    return 0;
}

int MPU9250_ReadMag(float *mx, float *my, float *mz)
{
    MPU9250_Data data;
    if (MPU9250_ReadAll(&data) != 0) return -1;
    *mx = data.mag_x; *my = data.mag_y; *mz = data.mag_z;
    return 0;
}

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
void MPU9250_CalibrateGyro(uint16_t samples, MPU9250_Calib *calib)
{
    if (samples == 0 || calib == NULL) return;

    float sum_gx = 0, sum_gy = 0, sum_gz = 0;

    memset(&calib_data, 0, sizeof(MPU9250_Calib));

    for (uint16_t i = 0; i < samples; i++) {
        /* 只读陀螺仪（更快），丢弃磁力计和加速度计 */
        uint8_t raw[6];
        if (MPU_ReadReg(MPU9250_REG_GYRO_XOUT_H, raw, 6) == HAL_OK) {
            int16_t gx = (int16_t)((raw[0] << 8) | raw[1]);
            int16_t gy = (int16_t)((raw[2] << 8) | raw[3]);
            int16_t gz = (int16_t)((raw[4] << 8) | raw[5]);
            sum_gx += (float)gx / gyro_lsb_per_dps;
            sum_gy += (float)gy / gyro_lsb_per_dps;
            sum_gz += (float)gz / gyro_lsb_per_dps;
        }
        HAL_Delay(1);
    }

    calib->gyro_offset_x = sum_gx / (float)samples;
    calib->gyro_offset_y = sum_gy / (float)samples;
    calib->gyro_offset_z = sum_gz / (float)samples;
    calib->accel_offset_x = 0.0f;
    calib->accel_offset_y = 0.0f;
    calib->accel_offset_z = 0.0f;
    calib->mag_offset_x = 0.0f;
    calib->mag_offset_y = 0.0f;
    calib->mag_offset_z = 0.0f;
    calib->mag_scale_x = 1.0f;
    calib->mag_scale_y = 1.0f;
    calib->mag_scale_z = 1.0f;

    memcpy(&calib_data, calib, sizeof(MPU9250_Calib));
}

void MPU9250_SetCalibration(const MPU9250_Calib *calib)
{
    if (calib) {
        memcpy(&calib_data, calib, sizeof(MPU9250_Calib));
    }
}

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
void MPU9250_CalibrateMag(uint32_t duration_ms, MPU9250_Calib *calib)
{
    if (calib == NULL || duration_ms == 0) return;

    /* 限制范围 */
    if (duration_ms < 3000)  duration_ms = 3000;
    if (duration_ms > 60000) duration_ms = 60000;

    float mag_x_min =  1e9f, mag_x_max = -1e9f;
    float mag_y_min =  1e9f, mag_y_max = -1e9f;
    uint32_t sample_count = 0;
    uint32_t start_tick = HAL_GetTick();
    uint32_t last_report = start_tick;

    /* 采集阶段: 持续 duration_ms, 每 2ms 采一个点 */
    while (HAL_GetTick() - start_tick < duration_ms) {
        float mx, my, mz;
        if (MPU9250_ReadMag(&mx, &my, &mz) == 0) {
            if (mx < mag_x_min) mag_x_min = mx;
            if (mx > mag_x_max) mag_x_max = mx;
            if (my < mag_y_min) mag_y_min = my;
            if (my > mag_y_max) mag_y_max = my;
            sample_count++;
        }
        HAL_Delay(2);

        /* 每秒上报一次进度 (通过 UART, 防止用户以为卡死) */
        if (HAL_GetTick() - last_report >= 1000) {
            last_report = HAL_GetTick();
            uint32_t elapsed = last_report - start_tick;
            uint32_t remaining = duration_ms - elapsed;
            /* 用直接寄存器写避免调 UART_IO_Printf 依赖问题 */
            /* 这里用轻量提示: 翻转 LED 表示工作中 */
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            (void)remaining; /* 避免未使用警告 */
        }
    }

    if (sample_count < 50) {
        /* 样本太少, 无法可靠校准, 保持原有校准值 */
        return;
    }

    /* ---- 硬铁偏移: (max + min) / 2 ---- */
    float hard_x = (mag_x_max + mag_x_min) / 2.0f;
    float hard_y = (mag_y_max + mag_y_min) / 2.0f;

    /* ---- 软铁尺度: 椭圆轴比 ---- */
    float semi_x = (mag_x_max - mag_x_min) / 2.0f;
    float semi_y = (mag_y_max - mag_y_min) / 2.0f;
    if (semi_x <= 0.0f) semi_x = 1.0f;
    if (semi_y <= 0.0f) semi_y = 1.0f;
    float avg_radius = (semi_x + semi_y) / 2.0f;
    float scale_x = avg_radius / semi_x;
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
