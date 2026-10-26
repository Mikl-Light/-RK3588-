/**
  ******************************************************************************
  * @file    mpu9250.h
  * @brief   MPU9250 九轴传感器驱动头文件 (I2C)
  *          - 3 轴加速度计
  *          - 3 轴陀螺仪
  *          - 3 轴磁力计 (AK8963)
  *          - 温度读取
  *
  * 硬件: I2C1, PB8(SCL) / PB9(SDA), 100kHz
  ******************************************************************************
  */
#ifndef __MPU9250_H__
#define __MPU9250_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "i2c.h"
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* MPU9250 I2C 地址                                                           */
/* -------------------------------------------------------------------------- */
#define MPU9250_ADDR            0x68    /**< AD0=GND; AD0=VCC → 0x69 */
#define AK8963_ADDR             0x0C    /**< 磁力计 I2C 地址 (固定) */

/* -------------------------------------------------------------------------- */
/* MPU9250 寄存器 (加速度计/陀螺仪，与 MPU6050 兼容)                           */
/* -------------------------------------------------------------------------- */
#define MPU9250_REG_WHO_AM_I      0x75  /**< 设备 ID (应返回 0x71) */
#define MPU9250_REG_PWR_MGMT_1    0x6B  /**< 电源管理 1 */
#define MPU9250_REG_SMPLRT_DIV    0x19  /**< 采样率分频 */
#define MPU9250_REG_CONFIG        0x1A  /**< DLPF 配置 */
#define MPU9250_REG_GYRO_CONFIG   0x1B  /**< 陀螺仪量程 */
#define MPU9250_REG_ACCEL_CONFIG  0x1C  /**< 加速度计量程 */
#define MPU9250_REG_ACCEL_CONFIG2 0x1D  /**< 加速度计 DLPF (MPU9250 独有) */
#define MPU9250_REG_INT_PIN_CFG   0x37  /**< 中断/旁路配置 */
#define MPU9250_REG_USER_CTRL     0x6A  /**< 用户控制 */

#define MPU9250_REG_ACCEL_XOUT_H  0x3B  /**< 加速度计 X 高字节 */
#define MPU9250_REG_TEMP_OUT_H    0x41  /**< 温度高字节 */
#define MPU9250_REG_GYRO_XOUT_H   0x43  /**< 陀螺仪 X 高字节 */

/* -------------------------------------------------------------------------- */
/* AK8963 磁力计寄存器                                                        */
/* -------------------------------------------------------------------------- */
#define AK8963_REG_WIA            0x00  /**< 设备 ID (应返回 0x48) */
#define AK8963_REG_ST1            0x02  /**< 状态 1 (DRDY) */
#define AK8963_REG_HXL            0x03  /**< 磁力计 X 低字节 */
#define AK8963_REG_ST2            0x09  /**< 状态 2 (数据溢出/读锁存解除) */
#define AK8963_REG_CNTL1          0x0A  /**< 控制 1 (模式选择) */
#define AK8963_REG_CNTL2          0x0B  /**< 控制 2 (复位) */
#define AK8963_REG_ASAX           0x10  /**< X 轴灵敏度校准 */

/* -------------------------------------------------------------------------- */
/* 陀螺仪量程 (与 MPU6050 兼容)                                               */
/* -------------------------------------------------------------------------- */
typedef enum {
    GYRO_FS_250  = 0x00,    /**< ±250 °/s  (131 LSB/°/s) */
    GYRO_FS_500  = 0x08,    /**< ±500 °/s  (65.5 LSB/°/s) */
    GYRO_FS_1000 = 0x10,    /**< ±1000 °/s (32.8 LSB/°/s) */
    GYRO_FS_2000 = 0x18,    /**< ±2000 °/s (16.4 LSB/°/s) */
} MPU9250_GyroRange;

/* -------------------------------------------------------------------------- */
/* 加速度计量程 (与 MPU6050 兼容)                                             */
/* -------------------------------------------------------------------------- */
typedef enum {
    ACCEL_FS_2G  = 0x00,    /**< ±2g  (16384 LSB/g) */
    ACCEL_FS_4G  = 0x08,    /**< ±4g  (8192 LSB/g) */
    ACCEL_FS_8G  = 0x10,    /**< ±8g  (4096 LSB/g) */
    ACCEL_FS_16G = 0x18,    /**< ±16g (2048 LSB/g) */
} MPU9250_AccelRange;

/* -------------------------------------------------------------------------- */
/* DLPF 带宽                                                                  */
/* -------------------------------------------------------------------------- */
typedef enum {
    DLPF_256HZ = 0x00,      /**< 加速度 260Hz / 陀螺仪 256Hz */
    DLPF_188HZ = 0x01,      /**< 加速度 184Hz / 陀螺仪 188Hz */
    DLPF_98HZ  = 0x02,      /**< 加速度 94Hz  / 陀螺仪 98Hz */
    DLPF_42HZ  = 0x03,      /**< 加速度 44Hz  / 陀螺仪 42Hz */
    DLPF_20HZ  = 0x04,      /**< 加速度 21Hz  / 陀螺仪 20Hz */
    DLPF_10HZ  = 0x05,      /**< 加速度 10Hz  / 陀螺仪 10Hz */
    DLPF_5HZ   = 0x06,      /**< 加速度 5Hz   / 陀螺仪 5Hz */
} MPU9250_DLPF;

/* -------------------------------------------------------------------------- */
/* AK8963 磁力计模式                                                          */
/* -------------------------------------------------------------------------- */
typedef enum {
    MAG_MODE_POWERDOWN  = 0x00, /**< 掉电 */
    MAG_MODE_SINGLE     = 0x01, /**< 单次测量 */
    MAG_MODE_CONT_8HZ   = 0x02, /**< 连续测量 8Hz */
    MAG_MODE_CONT_100HZ = 0x06, /**< 连续测量 100Hz */
} AK8963_Mode;

/* -------------------------------------------------------------------------- */
/* 传感器数据结构 (9 轴 + 温度)                                                */
/* -------------------------------------------------------------------------- */
typedef struct {
    /* 加速度计 (g) */
    float accel_x;
    float accel_y;
    float accel_z;

    /* 陀螺仪 (°/s) */
    float gyro_x;
    float gyro_y;
    float gyro_z;

    /* 磁力计 (µT) */
    float mag_x;
    float mag_y;
    float mag_z;

    /* 温度 (°C) */
    float temperature;

    /* 原始寄存器值 (int16, 未校准未缩放 — 供遥测二进制帧与 RK3588 对比) */
    int16_t accel_raw[3];   /**< 加速度计原始 LSB */
    int16_t gyro_raw[3];    /**< 陀螺仪原始 LSB */
    int16_t mag_raw[3];     /**< 磁力计原始 LSB (AK8963, 未乘 ASA) */
} MPU9250_Data;

/* -------------------------------------------------------------------------- */
/* 校准偏移                                                                   */
/* -------------------------------------------------------------------------- */
typedef struct {
    float gyro_offset_x;
    float gyro_offset_y;
    float gyro_offset_z;
    float accel_offset_x;
    float accel_offset_y;
    float accel_offset_z;
    float mag_offset_x;
    float mag_offset_y;
    float mag_offset_z;
    float mag_scale_x;
    float mag_scale_y;
    float mag_scale_z;
} MPU9250_Calib;

/* -------------------------------------------------------------------------- */
/* API 函数                                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief 初始化 MPU9250（含磁力计 AK8963）
 * @param gyro_range   陀螺仪量程
 * @param accel_range  加速度计量程
 * @param dlpf         DLPF 带宽
 * @param mag_mode     磁力计模式
 * @return             0=成功, 非0=失败
 */
int MPU9250_Init(MPU9250_GyroRange gyro_range, MPU9250_AccelRange accel_range,
                 MPU9250_DLPF dlpf, AK8963_Mode mag_mode);

/**
 * @brief 读取设备 ID (MPU9250)
 * @return WHO_AM_I 值 (应为 0x71)
 */
uint8_t MPU9250_ReadWhoAmI(void);

/**
 * @brief 读取磁力计设备 ID (AK8963)
 * @return WIA 值 (应为 0x48)
 */
uint8_t MPU9250_ReadMagWhoAmI(void);

/**
 * @brief 读取全部 9 轴传感器数据
 * @param data 数据输出结构体
 * @return     0=成功
 */
int MPU9250_ReadAll(MPU9250_Data *data);

/**
 * @brief 读取加速度计
 */
int MPU9250_ReadAccel(float *ax, float *ay, float *az);

/**
 * @brief 读取陀螺仪
 */
int MPU9250_ReadGyro(float *gx, float *gy, float *gz);

/**
 * @brief 读取磁力计
 */
int MPU9250_ReadMag(float *mx, float *my, float *mz);

/**
 * @brief 读取温度
 */
int MPU9250_ReadTemp(float *temp);

/**
 * @brief 陀螺仪校准（保持传感器静止）
 * @param samples 采样次数 (建议 ≥500)
 * @param calib   输出校准值
 */
void MPU9250_CalibrateGyro(uint16_t samples, MPU9250_Calib *calib);

/**
 * @brief 磁力计 2D 平面校准（水平旋转机器人 ≥360°）
 * @param duration_ms 采集时长 (ms), 建议 ≥10000 (10秒)
 * @param calib       输出校准值 (mag_offset_x/y, mag_scale_x/y)
 * @note  调用期间会阻塞主循环，采集完毕后自动计算硬铁/软铁补偿
 *        原理: 水平面内收集 mag_x, mag_y 样本, 拟合椭圆求中心和轴比
 */
void MPU9250_CalibrateMag(uint32_t duration_ms, MPU9250_Calib *calib);

/**
 * @brief 设置/获取校准偏移
 */
void MPU9250_SetCalibration(const MPU9250_Calib *calib);
const MPU9250_Calib *MPU9250_GetCalibration(void);

#ifdef __cplusplus
}
#endif

#endif /* __MPU9250_H__ */
