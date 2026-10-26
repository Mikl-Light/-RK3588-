/**
  ******************************************************************************
  * @file    chassis.h
  * @brief   底盘控制模块 — 麦轮 (Mecanum) 运动学
  *          - 麦轮逆运动学: Vx, Vy, ωz → 四轮转速
  *          - 麦轮正运动学: 四轮转速 → Vx, Vy, ωz (在 odometry 模块)
  *          - 状态机: IDLE → TURNING → MOVING → DONE
  *          - 支持 V:x,y,ωz 速度矢量控制
  *          - 支持单独轮速控制
  *
  * 麦轮布局 (俯视):
  *        前
  *    M1(FL)  M2(FR)
  *        ↑     ↑
  *    M3(RL)  M4(RR)
  *
  * 逆运动学 (机器人速度 → 轮速, ROS 约定 X=前, Y=左, ωz 逆时针正):
  *   ω_FL = (1/R) * ( Vx - Vy - ωz * (Lx + Ly) )
  *   ω_FR = (1/R) * ( Vx + Vy + ωz * (Lx + Ly) )
  *   ω_RL = (1/R) * ( Vx + Vy - ωz * (Lx + Ly) )
  *   ω_RR = (1/R) * ( Vx - Vy + ωz * (Lx + Ly) )
  *
  * 坐标系 (ROS 约定): X=前, Y=左, ωz=逆时针为正
  * 物理模式: 前进=四轮同向, 横移=对角反向, 旋转=斜对角反向
  ******************************************************************************
  */
#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/* 底盘状态                                                                   */
/* -------------------------------------------------------------------------- */
typedef enum {
    CHASSIS_STATE_IDLE = 0,     /**< 空闲, 等待命令 */
    CHASSIS_STATE_TURNING,      /**< 正在转向对准目标方向 */
    CHASSIS_STATE_MOVING,       /**< 正在前进 (带距离限制) */
    CHASSIS_STATE_MANUAL,       /**< 手动连续控制 (速度矢量/方向控制, 无自动停止) */
    CHASSIS_STATE_DONE,         /**< 任务完成, 已停止 */
} Chassis_State;

/* -------------------------------------------------------------------------- */
/* 底盘配置 (麦轮)                                                             */
/* -------------------------------------------------------------------------- */
typedef struct {
    float wheel_diameter;       /**< 轮子直径 (m), 默认 0.065 */
    float wheel_base_x;         /**< 左右轮半间距 Lx (m), 默认 0.075 */
    float wheel_base_y;         /**< 前后轮半间距 Ly (m), 默认 0.075 */
    float max_speed;            /**< 最大线速度 (m/s), 默认 1.0 */
    float max_angular;          /**< 最大角速度 (rad/s), 默认 3.0 */
    float heading_tolerance;    /**< 航向容差 (度), 默认 3.0 */
    float distance_tolerance;   /**< 距离容差 (m), 默认 0.02 */
} Chassis_Config;

/* -------------------------------------------------------------------------- */
/* API 函数                                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief 初始化底盘
 * @param cfg 底盘配置参数 (传 NULL 使用默认值)
 */
void Chassis_Init(const Chassis_Config *cfg);

/**
 * @brief 设置默认配置
 */
void Chassis_GetDefaultConfig(Chassis_Config *cfg);

/**
 * @brief 底盘主更新循环 (需定时调用, 如每 10ms)
 * @param dt 距上次调用的时间间隔 (秒)
 */
void Chassis_Update(float dt);

/* ================================================================ */
/*  麦轮速度矢量控制 (核心命令)                                      */
/* ================================================================ */

/**
 * @brief 麦轮速度矢量控制 (机器人坐标系)
 * @param vx    横向速度 (m/s), X 轴, 右为正
 * @param vy    纵向速度 (m/s), Y 轴, 前为正
 * @param omega 旋转角速度 (rad/s), 逆时针为正
 * @note  连续运动, 需 STOP 停止
 */
void Chassis_MoveMecanum(float vx, float vy, float omega);

/* ================================================================ */
/*  方向控制 (保持兼容, 底层使用麦轮运动学)                           */
/* ================================================================ */

/**
 * @brief 执行方向指令 (导航模式, 自动停止)
 * @param heading_deg 目标航向 (度, 0=北, 90=东, 180=南, 270=西)
 * @param speed_mps   前进速度 (m/s)
 * @param distance_m  前进距离 (m)
 * @return true=指令接受, false=底盘忙
 */
bool Chassis_GoDirection(float heading_deg, float speed_mps, float distance_m);

/**
 * @brief 手动连续运动 (兼容旧接口, 底层使用麦轮运动学)
 * @param linear_mps    线速度 (m/s), 正=前进, 负=后退
 * @param angular_degps 角速度 (度/秒), 正=逆时针, 负=顺时针
 */
void Chassis_MoveManual(float linear_mps, float angular_degps);

/* ================================================================ */
/*  单轮控制                                                         */
/* ================================================================ */

/**
 * @brief 控制单个麦轮 (速度闭环)
 * @param wheel_id 轮子编号 (1~4, 对应 M1~M4)
 * @param rpm      目标转速 (RPM)
 */
void Chassis_SetWheelSpeed(uint8_t wheel_id, float rpm);

/**
 * @brief 控制单个麦轮 (开环)
 * @param wheel_id 轮子编号 (1~4)
 * @param duty     占空比 [-1, 1]
 */
void Chassis_SetWheelDuty(uint8_t wheel_id, float duty);

/**
 * @brief 停止单个麦轮
 * @param wheel_id 轮子编号 (1~4)
 */
void Chassis_StopWheel(uint8_t wheel_id);

/* ================================================================ */
/*  系统控制                                                         */
/* ================================================================ */

/**
 * @brief 紧急停止 (全部电机)
 */
void Chassis_Stop(void);

/**
 * @brief 获取当前状态
 */
Chassis_State Chassis_GetState(void);

/**
 * @brief 获取当前航向 (度)
 */
float Chassis_GetCurrentHeading(void);

/**
 * @brief 获取已行驶距离 (m)
 */
float Chassis_GetTraveledDistance(void);

/**
 * @brief 获取目标剩余距离 (m)
 */
float Chassis_GetRemainingDistance(void);

/**
 * @brief 获取状态名称字符串
 */
const char *Chassis_GetStateName(void);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_H__ */
