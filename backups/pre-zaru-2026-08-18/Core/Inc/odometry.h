/**
 ******************************************************************************
 * @file    odometry.h
 * @brief   里程计模块 — 编码器 + IMU 数据融合
 *          - 麦轮正向运动学: 四轮转速 → Vx, Vy, ωz (机器人坐标系)
 *          - 航位推算:      Vx, Vy + AHRS 航向 → 世界坐标 x, y
 *          - 融合位姿输出:  x(m), y(m), heading(°), vx(m/s), vy(m/s), ωz(rad/s)
 *
 * 坐标系定义 (ROS 约定, 2026-08-12):
 *   机器人坐标系: X=前, Y=左, ωz=逆时针为正 (左转)
 *   世界坐标系:   X=北(起始前方), Y=西 (右手系)
 *   heading=罗盘角 (0°=北, 顺时针增加); ROS yaw(逆时针正) = -heading
 ******************************************************************************
 */
#ifndef __ODOMETRY_H__
#define __ODOMETRY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/* 麦轮几何参数                                                               */
/* -------------------------------------------------------------------------- */
typedef struct {
    float wheel_radius;         /**< 轮子半径 (m), 默认 0.0325 (65mm 直径) */
    float wheel_base_x;         /**< 左右轮半间距 Lx (m), 默认 0.075 (150mm/2) */
    float wheel_base_y;         /**< 前后轮半间距 Ly (m), 默认 0.075 (150mm/2) */
    float encoder_cnt_per_rev;  /**< 编码器每转计数值 (含 x4 倍频), 默认 52 */
    float gear_ratio;           /**< 减速比 (电机:输出), 默认 30.0 */
} Odom_Config;

/* -------------------------------------------------------------------------- */
/* 融合位姿                                                                   */
/* -------------------------------------------------------------------------- */
typedef struct {
    float x;            /**< 世界坐标 X (东, m) */
    float y;            /**< 世界坐标 Y (北, m) */
    float heading;      /**< 航向角 (度, 0=北, 90=东) */
    float vx;           /**< 机器人坐标系 X 方向速度 (m/s, 右为正) */
    float vy;           /**< 机器人坐标系 Y 方向速度 (m/s, 前为正) */
    float omega;        /**< 机器人坐标系旋转角速度 (rad/s, 逆时针为正) */
} Odom_Pose;

/* -------------------------------------------------------------------------- */
/* 原始传感器数据 (用于遥测打包)                                               */
/* -------------------------------------------------------------------------- */
typedef struct {
    /* 编码器原始值 */
    int32_t enc_raw[4];         /**< 4 路编码器原始计数值 (uint16 转 int32) */
    int32_t enc_delta[4];       /**< 4 路编码器增量 (本周期) */
    float   wheel_rpm[4];       /**< 4 轮转速 (RPM) */
    float   wheel_mps[4];       /**< 4 轮线速度 (m/s) */

    /* IMU 原始值 */
    float accel_x, accel_y, accel_z;  /**< 加速度计 (g) */
    float gyro_x,  gyro_y,  gyro_z;  /**< 陀螺仪 (°/s) */
    float mag_x,   mag_y,   mag_z;   /**< 磁力计 (µT) */
    float imu_temp;                   /**< IMU 温度 (°C) */

    /* IMU 原始寄存器值 (int16 LSB, 遥测二进制帧用) */
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t mag_raw[3];

    /* 融合结果 */
    Odom_Pose pose;             /**< 当前融合位姿 */
} Odom_Data;

/* -------------------------------------------------------------------------- */
/* API 函数                                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief 初始化里程计
 * @param cfg 麦轮几何参数 (传 NULL 使用默认值)
 */
void Odom_Init(const Odom_Config *cfg);

/**
 * @brief 获取默认配置
 */
void Odom_GetDefaultConfig(Odom_Config *cfg);

/**
 * @brief 里程计更新 (需每控制周期调用)
 * @param dt 时间间隔 (秒)
 */
void Odom_Update(float dt);

/**
 * @brief 获取融合位姿
 * @return 当前位姿指针
 */
const Odom_Pose *Odom_GetPose(void);

/**
 * @brief 获取完整传感器+融合数据 (用于遥测)
 * @return 数据指针
 */
const Odom_Data *Odom_GetData(void);

/**
 * @brief 重置里程计 (位置归零, 航向保持)
 */
void Odom_Reset(void);

/**
 * @brief 设置当前位置
 */
void Odom_SetPose(float x, float y, float heading);

#ifdef __cplusplus
}
#endif

#endif /* __ODOMETRY_H__ */
