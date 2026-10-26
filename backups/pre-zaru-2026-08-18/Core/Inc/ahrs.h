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

#ifdef __cplusplus
}
#endif

#endif /* __AHRS_H__ */
