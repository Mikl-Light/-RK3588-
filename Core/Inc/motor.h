/**
  ******************************************************************************
  * @file    motor.h
  * @brief   编码器电机驱动头文件
  *          - 编码器位置/速度读取
  *          - PWM 占空比控制
  *          - 速度 PID 闭环控制
  *
  * 硬件映射:
  *   电机1: TIM1 编码器 (PA8/PA9) + TIM2 CH1 PWM (PA0)
  *   电机2: TIM3 编码器 (PA6/PA7) + TIM2 CH2 PWM (PA1)
  *   电机3: TIM4 编码器 (PB6/PB7) + TIM2 CH3 PWM (PA2)
  *   电机4: TIM8 编码器 (PC6/PC7) + TIM2 CH4 PWM (PA3)
  ******************************************************************************
  */
#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "tim.h"
#include "pid.h"
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/* 电机编号                                                                   */
/* -------------------------------------------------------------------------- */
#define MOTOR_ID_1    0
#define MOTOR_ID_2    1
#define MOTOR_ID_3    2
#define MOTOR_ID_4    3
#define MOTOR_COUNT   4

/* -------------------------------------------------------------------------- */
/* 底盘电机分组 (差速驱动)                                                    */
/*   左轮: 电机1 + 电机3                                                      */
/*   右轮: 电机2 + 电机4                                                      */
/*   如果你的小车左右分配不同, 修改下面的宏即可                                */
/* -------------------------------------------------------------------------- */
#define MOTOR_LEFT_FRONT   MOTOR_ID_1
#define MOTOR_LEFT_REAR    MOTOR_ID_3
#define MOTOR_RIGHT_FRONT  MOTOR_ID_2
#define MOTOR_RIGHT_REAR   MOTOR_ID_4

/* -------------------------------------------------------------------------- */
/* 电机方向定义                                                               */
/* -------------------------------------------------------------------------- */
#define MOTOR_DIR_FORWARD   1
#define MOTOR_DIR_REVERSE  -1
#define MOTOR_DIR_STOP      0

/* -------------------------------------------------------------------------- */
/* 电机控制模式                                                               */
/* -------------------------------------------------------------------------- */
typedef enum {
    MOTOR_MODE_OPEN_LOOP = 0,   /**< 开环控制（直接 PWM 占空比） */
    MOTOR_MODE_SPEED,           /**< 速度闭环 PID 控制 */
} Motor_Mode;

/* -------------------------------------------------------------------------- */
/* 电机句柄结构体                                                             */
/* -------------------------------------------------------------------------- */
/* 【中文名】电机"档案袋"：一个电机要工作，需要记住一堆东西，全装进这个结构体。
 * 电机1~4 各有一份（见 motor.c 里的 g_motor[4]），每个 m->xxx 就是打开某号电机的档案取东西 */
typedef struct {
    /* --- 硬件关联（这个电机接在芯片的哪个定时器上） --- */
    TIM_HandleTypeDef *htim_encoder;    /**< 编码器定时器（用哪个定时器来数轮子转了多少格） */
    TIM_HandleTypeDef *htim_pwm;        /**< PWM 定时器（用哪个定时器来输出转速） */
    uint32_t           pwm_channel;     /**< PWM 通道（该定时器的第几路输出） */

    /* --- 编码器参数（轮子转一圈能数到多少） --- */
    uint16_t encoder_ppr;               /**< 编码器每转脉冲数（倍频前，本车=13） */
    int32_t  encoder_cnt_per_rev;       /**< 每转计数值 = PPR×4 = 52（倍频后） */
    int8_t   direction_sign;            /**< 方向符号 ±1（纠正这个电机编码器接线反不反） */

    /* --- 状态量（运行时不断变化的读数） --- */
    int32_t  raw_count;                 /**< 当前编码器原始计数值 */
    int32_t  prev_raw_count;            /**< 上一次原始计数值（用于算增量） */
    int32_t  count_delta;               /**< 本周期计数差值（转了了几格，已处理翻转） */
    int64_t  total_count;               /**< 累计总计数（从开机到现在一共转了多少格） */
    float    speed_rpm;                 /**< 当前速度（转/分钟） */
    float    position;                  /**< 累计位置（多少圈） */

    /* --- 控制（怎么驱动它） --- */
    Motor_Mode mode;                    /**< 控制模式：开环 or 速度闭环 */
    PID_Controller pid_speed;           /**< 速度 PID 控制器（里面那个 PID 盒子） */
    float    duty;                      /**< 当前 PWM 占空比 [-1.0, 1.0]，正=正转 负=反转 */
    float    target_speed_rpm;          /**< 目标速度（转/分钟，PID 要追到这个值） */

    /* --- PWM 配置 --- */
    uint32_t pwm_period;                /**< PWM 周期（一个 PWM 周期的计数值，本车=99） */

    /* --- 减速比 --- */
    float    gear_ratio;                /**< 减速比 电机轴:输出轴，30 表示 30:1（电机转30圈轮子转1圈） */
    bool     invert_dir;                /**< 是否反转电机接线（true=IN1/IN2 对调） */

    /* --- TB6612 方向引脚（往哪两根脚写高/低电平来控制正反转） --- */
    GPIO_TypeDef *in1_port;             /**< AIN1/BIN1 所在端口（GPIOB 等） */
    uint16_t      in1_pin;              /**< AIN1/BIN1 引脚号 */
    GPIO_TypeDef *in2_port;             /**< AIN2/BIN2 所在端口 */
    uint16_t      in2_pin;              /**< AIN2/BIN2 引脚号 */
} Motor_Handle;

/* -------------------------------------------------------------------------- */
/* 全局电机句柄（供外部使用）                                                 */
/* -------------------------------------------------------------------------- */
extern Motor_Handle g_motor[MOTOR_COUNT];

/* -------------------------------------------------------------------------- */
/* API 函数                                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief 初始化所有电机
 * @note  需在 MX_TIMx_Init() 之后调用
 */
void Motor_InitAll(void);

/**
 * @brief 初始化单个电机
 * @param id         电机编号 (0~3)
 * @param ppr        编码器每转脉冲数 (电机轴端)
 * @param gear_ratio 减速比 (电机轴:输出轴), 如 30 表示 30:1
 * @param dir_sign   方向符号 (1 或 -1)
 * @param invert     反转电机转向 (true=IN1/IN2互换)
 */
void Motor_Init(uint8_t id, uint16_t ppr, float gear_ratio, int8_t dir_sign, bool invert);

/**
 * @brief 电机状态更新（需按固定周期调用，如 1ms 或 10ms）
 * @param id 电机编号
 * @param dt 距上次调用的时间间隔（秒）
 */
void Motor_Update(uint8_t id, float dt);

/**
 * @brief 更新所有电机状态
 * @param dt 距上次调用的时间间隔（秒）
 */
void Motor_UpdateAll(float dt);

/**
 * @brief 设置电机开环占空比
 * @param id   电机编号
 * @param duty 占空比 [-1.0, 1.0]，正转/反转
 */
void Motor_SetDuty(uint8_t id, float duty);

/**
 * @brief 设置电机目标速度（速度闭环模式）
 * @param id  电机编号
 * @param rpm 目标速度 (RPM)
 */
void Motor_SetSpeed(uint8_t id, float rpm);

/**
 * @brief 设置电机控制模式
 * @param id   电机编号
 * @param mode 控制模式
 */
void Motor_SetMode(uint8_t id, Motor_Mode mode);

/**
 * @brief 停止电机
 * @param id 电机编号
 */
void Motor_Stop(uint8_t id);

/**
 * @brief 获取编码器原始计数值
 * @param id 电机编号
 * @return   当前 CNT 值
 */
int32_t Motor_GetEncoderCount(uint8_t id);

/**
 * @brief 获取本周期编码器增量
 * @param id 电机编号
 * @return   本周期计数增量 (已处理翻转)
 */
int32_t Motor_GetEncoderDelta(uint8_t id);

/**
 * @brief 获取电机当前速度
 * @param id 电机编号
 * @return   速度 (RPM)
 */
float Motor_GetSpeed(uint8_t id);

/**
 * @brief 获取电机累计位置
 * @param id 电机编号
 * @return   位置（圈）
 */
float Motor_GetPosition(uint8_t id);

/**
 * @brief 设置速度 PID 参数
 * @param id 电机编号
 * @param Kp 比例系数
 * @param Ki 积分系数
 * @param Kd 微分系数
 */
void Motor_SetSpeedPID(uint8_t id, float Kp, float Ki, float Kd);

/**
 * @brief 停止所有电机
 */
void Motor_StopAll(void);

/**
 * @brief 设置左右电机目标速度 (差速驱动)
 * @param left_rpm  左侧所有电机目标转速 (RPM)
 * @param right_rpm 右侧所有电机目标转速 (RPM)
 */
void Motor_SetLeftRightSpeed(float left_rpm, float right_rpm);

/**
 * @brief 设置四个电机独立目标速度 (麦轮控制)
 * @param rpm1~rpm4  电机1~4 目标转速 (RPM)
 */
void Motor_SetAllSpeeds(float rpm1, float rpm2, float rpm3, float rpm4);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
