/**
  ******************************************************************************
  * @file    pid.h
  * @brief   增量式 PID 控制器 (Incremental / Velocity PID)
  *
  *  公式:
  *    Δu = Kp·(eₖ − eₖ₋₁)  +  Ki·eₖ·Δt  +  Kd·(eₖ − 2eₖ₋₁ + eₖ₋₂)/Δt
  *    uₖ = clamp(uₖ₋₁ + Δu, out_min, out_max)
  *
  *  优点 vs 位置式:
  *    - 天然抗积分饱和 (无累加积分项)
  *    - 目标突变时平滑过渡 (bumpless)
  *    - 输出限幅只需一行 clamp
  *    - D 项对误差微分, 比微分测量值更抗噪声
  ******************************************************************************
  */
#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 增量式 PID 控制器结构体
 *
 * 【中文名】PID 控制器（一个"抽屉柜"/"盒子"）
 * 作用：PID 运行时要"记住"上一轮误差、上上轮误差、上一次输出这些记忆，
 *       散着放容易乱，所以把它们全装进这个结构体里，起名叫 PID_Controller。
 *       以后程序里所有 pid->xxx，都是"打开这个盒子，取/放某个格子"。
 */
typedef struct {
    float Kp;           /**< 比例系数：P 项的劲有多大（看"误差变化"） */
    float Ki;           /**< 积分系数：I 项的劲有多大（看"误差本身"） */
    float Kd;           /**< 微分系数：D 项的劲有多大（看"变化快慢"） */

    float setpoint;     /**< 目标值：我想让什么达到多少，比如"轮子 60 转/分" */
    float prev_error;   /**< eₖ₋₁: 上一轮误差（P、D 项用它算"变化"） */
    float prev2_error;  /**< eₖ₋₂: 上上轮误差（D 项用它算"变化的变化"） */

    float prev_output;  /**< uₖ₋₁: 上一次输出（增量式的"起点"，积分藏在这里面） */
    float out_min;      /**< 输出下限：不允许输出低于这个值 */
    float out_max;      /**< 输出上限：不允许输出高于这个值 */

    float output;       /**< 当前输出值 uₖ：这一轮刚算出来的结果 */
} PID_Controller;

/**
 * @brief 初始化增量式 PID 控制器
 */
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd,
              float out_min, float out_max);

/**
 * @brief 设置 PID 目标值
 */
void PID_SetSetpoint(PID_Controller *pid, float setpoint);

/**
 * @brief 在线修改 PID 参数
 */
void PID_SetTunings(PID_Controller *pid, float Kp, float Ki, float Kd);

/**
 * @brief 重置 PID 状态 (保留上次输出做 bumpless 切换)
 * @param pid         控制器指针
 * @param measurement 当前测量值 (RPM / rad/s 等)，用于正确初始化误差历史
 * @note  传入当前测量值可保证 P 项和 D 项在重置后第一周期输出为零，
 *        仅 I 项从零开始平滑累积，实现真正的 bumpless 切换
 */
void PID_Reset(PID_Controller *pid, float measurement);

/**
 * @brief 增量式 PID 计算 (需按固定周期调用)
 * @param pid       控制器指针
 * @param measure   当前测量值 (RPM / rad/s 等)
 * @param dt        采样周期 (秒)
 * @return          限幅后的输出 uₖ
 */
float PID_Compute(PID_Controller *pid, float measure, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
