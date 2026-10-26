/**
  ******************************************************************************
  * @file    ahrs.h
  * @brief   航向姿态参考系统 (AHRS)
  *          - 互补滤波融合陀螺仪 + 磁力计，计算绝对航向角
  *          - 0° = 磁北, 90° = 东, 180° = 南, 270° = 西
  ******************************************************************************
  */
#ifndef __AHRS_H__
#define __AHRS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/* 航向数据结构                                                               */
/* -------------------------------------------------------------------------- */
typedef struct {
    float yaw;              /**< 当前航向角 (度, 0~360, 0=北) */
    float gyro_z;           /**< 陀螺仪 Z 轴 (°/s) */
    float mag_heading;      /**< 磁力计原始航向 (度) */
} AHRS_Data;

/* -------------------------------------------------------------------------- */
/* API 函数                                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief 初始化 AHRS（加载校准数据）
 */
void AHRS_Init(void);

/**
 * @brief 更新航向估计（需定时调用，如每 10ms）
 * @param dt 距上次调用的时间间隔（秒）
 */
void AHRS_Update(float dt);

/**
 * @brief 获取当前航向角
 * @return 航向角 (度), 0=磁北, 90=东, 180=南, 270=西
 */
float AHRS_GetHeading(void);

/**
 * @brief 获取 AHRS 完整数据
 * @param data 输出结构体
 */
void AHRS_GetData(AHRS_Data *data);

/**
 * @brief 计算两个角度之间的最短差值
 * @param target 目标角度 (度)
 * @param current 当前角度 (度)
 * @return 差值, 范围 [-180, 180]，正值表示需要顺时针转
 */
float AHRS_AngleDiff(float target, float current);

/**
 * @brief 磁力计融合开关 (默认 OFF: 纯陀螺积分)
 * @note  电机磁场干扰/磁轴未标定时用 OFF 更稳 (2026-08-12 新增)
 */
void AHRS_SetMagFusion(bool enable);

/**
 * @brief 查询磁力计融合状态
 */
bool AHRS_IsMagFusionEnabled(void);

/**
 * @brief 手动重新校准陀螺仪 Z 轴零偏
 * @param samples 采样点数，建议 500~1000（每点约 1ms）
 * @note  调用前必须保证小车完全静止
 */
void AHRS_RecalibrateGyro(uint16_t samples);


/**
 * @brief 由磁力计水平分量计算机器人磁航向 (度, 0~360)
 * @note  与 AHRS 内部 MagHeadingDeg 同源；用于可靠里程计 EKF 的动态 R/监督
 */
float AHRS_ComputeMagHeadingDeg(float mx, float my);

/**
 * @brief 由外部里程计覆盖 AHRS 航向 (用于 EKF 磁力计模式时保持各模块一致)
 * @param heading_deg 罗盘角 (度, 0=北, 90=东)
 */
void AHRS_OverrideHeadingDeg(float heading_deg);

/**
 * @brief 设置静止状态 (由主循环按编码器判定, 2026-08-18)
 * @note  静止时 AHRS 会慢速重估陀螺零偏, 消除 HDG 匀速漂移
 */
void AHRS_SetStationary(bool is_stationary);

#ifdef __cplusplus
}
#endif

#endif /* __AHRS_H__ */
