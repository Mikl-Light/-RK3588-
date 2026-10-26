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

/* ============ 本文件用到的两个工具文件 ============
 * pid.h : 定义了 PID_Controller 这个"盒子"长什么样（有哪些格子）
 * string.h : 提供 memset 这个"批量清零"的工具函数
 * #include 的意思就是"把这文件的内容搬进来用"
 */
#include "pid.h"
#include <string.h>

/**
 * @brief 初始化增量式 PID 控制器
 */
/* 【中文名】初始化：开机时把 PID 盒子整个清零，再把三个系数和输出上下限设好 */
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd,
              float out_min, float out_max)
{
    /* memset = 把一块内存批量填成同一个值。
     * 这里三个参数：pid(=要填的内存起点)、0(=填成0)、sizeof(PID_Controller)(=要填多长)
     * sizeof 是"量尺寸"：量出 PID_Controller 这个盒子一共占多少个字节
     * 合起来：把整个 PID 盒子，一个字节不落地全写 0 = 清空 */
    memset(pid, 0, sizeof(PID_Controller));

    /* pid 是一个"指向盒子的指针"。pid->Kp 的意思是"打开盒子，找到 Kp 这个格子"
     * 等号右边是外面传进来的参数，左边是盒子里的格子
     * 所以 pid->Kp = Kp 就是："把传进来的 Kp 值，存进盒子里的 Kp 格子" */
    pid->Kp      = Kp;       /* 存比例系数 Kp（P 有多大劲） */
    pid->Ki      = Ki;       /* 存积分系数 Ki（I 有多大劲） */
    pid->Kd      = Kd;       /* 存微分系数 Kd（D 有多大劲） */
    pid->out_min = out_min;  /* 存输出下限（最小允许输出到多少） */
    pid->out_max = out_max;  /* 存输出上限（最大允许输出到多少） */
}

/**
 * @brief 设置 PID 目标值
 * @note  增量式 PID 中, setpoint 变化只通过 Ki·eₖ·Δt 项渐进响应,
 *        不会像位置式那样因 Kp·eₖ 产生跳变, 天然 bumpless
 */
/* 【中文名】设置目标值：告诉 PID"我要让什么达到多少"，比如"要让轮子转到60转/分" */
void PID_SetSetpoint(PID_Controller *pid, float setpoint)
{
    /* 把"目标值"存进盒子的 setpoint 格子。就这一件事，没别的 */
    pid->setpoint = setpoint;
}

/**
 * @brief 在线修改 PID 参数
 */
/* 【中文名】改参数：程序跑起来之后，想临时调大调小 Kp/Ki/Kd，就调这个，不用重启 */
void PID_SetTunings(PID_Controller *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;   /* 更新比例系数 */
    pid->Ki = Ki;   /* 更新积分系数 */
    pid->Kd = Kd;   /* 更新微分系数 */
}

/**
 * @brief 重置 PID 状态
 * @note  保留 prev_output = output 实现 bumpless 无缝切换。
 *        用当前测量值初始化误差历史 (eₖ₋₁ = eₖ₋₂ = setpoint − measurement)，
 *        使得 P 项 = Kp×(e−e) = 0, D 项 = Kd×(e−2e+e)/Δt = 0，
 *        仅 I 项 = Ki×e×Δt 从零开始平滑累积，无冲击。
 */
/* 【中文名】重置记忆：换目标/停车时调用。把"误差历史"抹平，假装"误差一直是这个数"，
 * 这样 P 项和 D 项（它们只看"变化"）就不会当成"误差突然跳变"而猛冲一下 */
void PID_Reset(PID_Controller *pid, float measurement)
{
    /* ① 把"上一次输出"保留成"当前输出"，这样力度不会从零重新开始（不会顿一下） */
    pid->prev_output = pid->output;

    /* ② 用"现在这一刻"算一次误差 = 目标 − 实测 */
    float error = pid->setpoint - measurement;

    /* ③④ 把"上一轮误差"和"上上轮误差"都设成这个数
     *     后果：P项 = Kp×(误差−上一轮误差) = Kp×0 = 0，D项同理=0
     *     所以这一时刻 P、D 都不出力，只有 I 项慢慢从零累积 → 起步平稳 */
    pid->prev_error  = error;
    pid->prev2_error = error;
}

/**
 * @brief 增量式 PID 计算
 */
/* 【中文名】计算输出（核心函数，每10ms调一次）：根据"现在误差多大"，算出"输出该加减多少"，返回夹在上下限内的结果 */
float PID_Compute(PID_Controller *pid, float measure, float dt)
{
    /* 防止除零：如果时间间隔 dt 小到几乎为零（比如第一次调用），直接返回旧输出，不做任何计算 */
    if (dt < 0.000001f) {
        return pid->output;
    }

    /* ① 算误差：目标 − 实测。比如目标60转，实测55转 → 误差 = 5 */
    float error = pid->setpoint - measure;

    /* === ② P项 = Kp × (误差 − 上一轮误差) ===
     * 它看的是"误差变了多少"。误差没变时，这一项 = 0 */
    float p_term = pid->Kp * (error - pid->prev_error);

    /* === ③ I项 = Ki × 误差 × 时间 ===
     * 它看的是"误差本身有多大"。误差一直在，它就每轮都出一小点力，
     * 只要误差没消，它就持续出力，最终把误差彻底抹平 */
    float i_term = pid->Ki * error * dt;

    /* === ④ D项 = Kd × (误差 − 2×上一轮误差 + 上上轮误差) ÷ 时间 ===
     * 它看的是"误差变化的速度"（变化得越猛，它越要压一压） */
    float d_term = pid->Kd * (error - 2.0f * pid->prev_error + pid->prev2_error) / dt;

    /* ⑤ 新输出 = 上一次输出 + P增量 + I增量 + D增量
     * 这就是"增量式"的核心：不是重算输出，而是"在上一轮基础上加减" */
    float output = pid->prev_output + p_term + i_term + d_term;

    /* ⑥ 夹限幅：算出来超过上限就压到上限，低于下限就抬到下限，
     * 防止输出无限涨，把电机驱动到超出安全范围 */
    if (output > pid->out_max) {
        output = pid->out_max;
    } else if (output < pid->out_min) {
        output = pid->out_min;
    }

    /* ⑦ 滚动记忆：为下一轮做准备
     * 把"本轮"的值，推到"上一轮"的格子里（上上轮 ← 上一轮 ← 本轮） */
    pid->prev2_error = pid->prev_error;  /* 上上轮误差 ← 上一轮误差 */
    pid->prev_error  = error;            /* 上一轮误差 ← 本轮误差 */
    pid->prev_output = output;           /* 上一次输出 ← 本轮输出 */
    pid->output      = output;           /* 记录当前输出 */

    /* ⑧ 把算好的输出交出去 */
    return output;
}