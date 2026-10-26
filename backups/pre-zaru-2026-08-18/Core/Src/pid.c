/**
  ******************************************************************************
  * @file    pid.c
  * @brief   增量式 PID 控制器实现
  *
  *  增量式公式 (Velocity PID):
  *    eₖ  = setpoint − measure
  *    Δu  = Kp·(eₖ − eₖ₋₁)  +  Ki·eₖ·Δt  +  Kd·(eₖ − 2eₖ₋₁ + eₖ₋₂)/Δt
  *    uₖ  = clamp(uₖ₋₁ + Δu, out_min, out_max)
  *
  *  特性:
  *    - 无积分累加器 → 天然抗饱和
  *    - 输出限幅只需一行, 无需回退积分
  *    - setpoint 突变时自动 bumpless (P/D 项仅对 Δe 响应)
  *    - D 项对误差微分, 抗编码器量化噪声优于微分测量值
  ******************************************************************************
  */
#include "pid.h"
#include <string.h>

/**
 * @brief 初始化增量式 PID 控制器
 */
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd,
              float out_min, float out_max)
{
    memset(pid, 0, sizeof(PID_Controller));
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

/**
 * @brief 设置 PID 目标值
 * @note  增量式 PID 中, setpoint 变化只通过 Ki·eₖ·Δt 项渐进响应,
 *        不会像位置式那样因 Kp·eₖ 产生跳变, 天然 bumpless
 */
void PID_SetSetpoint(PID_Controller *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

/**
 * @brief 在线修改 PID 参数
 */
void PID_SetTunings(PID_Controller *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

/**
 * @brief 重置 PID 状态
 * @note  保留 prev_output = output 实现 bumpless 无缝切换。
 *        用当前测量值初始化误差历史 (eₖ₋₁ = eₖ₋₂ = setpoint − measurement)，
 *        使得 P 项 = Kp×(e−e) = 0, D 项 = Kd×(e−2e+e)/Δt = 0，
 *        仅 I 项 = Ki×e×Δt 从零开始平滑累积，无冲击。
 */
void PID_Reset(PID_Controller *pid, float measurement)
{
    pid->prev_output = pid->output;
    float error = pid->setpoint - measurement;
    pid->prev_error  = error;
    pid->prev2_error = error;
}

/**
 * @brief 增量式 PID 计算
 */
float PID_Compute(PID_Controller *pid, float measure, float dt)
{
    /* 防止除零 */
    if (dt < 0.000001f) {
        return pid->output;
    }

    /* 当前误差 */
    float error = pid->setpoint - measure;

    /* === P 项: Kp · (eₖ − eₖ₋₁) === */
    float p_term = pid->Kp * (error - pid->prev_error);

    /* === I 项: Ki · eₖ · Δt ===
     *  增量式积分隐含在 uₖ₋₁ 中, 此项提供持续的误差修正
     *  无累加器 → 天然抗饱和, 不需要 integral_limit */
    float i_term = pid->Ki * error * dt;

    /* === D 项: Kd · (eₖ − 2eₖ₋₁ + eₖ₋₂) / Δt ===
     *  对误差的二阶差分, 等效于误差加速度
     *  比位置式 D-on-measurement 更抗量化噪声 */
    float d_term = pid->Kd * (error - 2.0f * pid->prev_error + pid->prev2_error) / dt;

    /* 累加增量 */
    float output = pid->prev_output + p_term + i_term + d_term;

    /* 输出限幅 (唯一需要的地方, 无需回退积分) */
    if (output > pid->out_max) {
        output = pid->out_max;
    } else if (output < pid->out_min) {
        output = pid->out_min;
    }

    /* 滚动误差历史 */
    pid->prev2_error = pid->prev_error;
    pid->prev_error  = error;
    pid->prev_output = output;
    pid->output      = output;

    return output;
}
