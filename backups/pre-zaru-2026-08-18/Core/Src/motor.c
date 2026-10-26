/**
  ******************************************************************************
  * @file    motor.c
  * @brief   编码器电机驱动实现
  ******************************************************************************
  */
#include "motor.h"
#include "debug.h"
#include <string.h>
#include <math.h>

/* -------------------------------------------------------------------------- */
/* 全局电机句柄                                                               */
/* -------------------------------------------------------------------------- */
Motor_Handle g_motor[MOTOR_COUNT];

/* -------------------------------------------------------------------------- */
/* 电机硬件映射表                                                             */
/* -------------------------------------------------------------------------- */
typedef struct {
    TIM_HandleTypeDef *htim_encoder;
    TIM_HandleTypeDef *htim_pwm;
    uint32_t           pwm_channel;
    uint32_t           pwm_period;
} Motor_HW_Map;

static const Motor_HW_Map motor_hw[MOTOR_COUNT] = {
    /* 电机1: TIM1 编码器 + TIM2 CH1 PWM */
    { &htim1, &htim2, TIM_CHANNEL_1, 99 },
    /* 电机2: TIM3 编码器 + TIM2 CH2 PWM */
    { &htim3, &htim2, TIM_CHANNEL_2, 99 },
    /* 电机3: TIM4 编码器 + TIM2 CH3 PWM */
    { &htim4, &htim2, TIM_CHANNEL_3, 99 },
    /* 电机4: TIM8 编码器 + TIM2 CH4 PWM */
    { &htim8, &htim2, TIM_CHANNEL_4, 99 },
};

/* TB6612 方向引脚映射 (AIN1/AIN2 或 BIN1/BIN2) */
static const struct {
    GPIO_TypeDef *in1_port;  uint16_t in1_pin;
    GPIO_TypeDef *in2_port;  uint16_t in2_pin;
} motor_dir_pins[MOTOR_COUNT] = {
    /* 电机1 (左前): PWMA=PA0, AIN1=PB0, AIN2=PB1 */
    { GPIOB, GPIO_PIN_0,  GPIOB, GPIO_PIN_1  },
    /* 电机2 (右前): PWMB=PA1, BIN1=PB2, BIN2=PB10 */
    { GPIOB, GPIO_PIN_2,  GPIOB, GPIO_PIN_10 },
    /* 电机3 (左后): PWMA=PA2, AIN1=PB11, AIN2=PB12 */
    { GPIOB, GPIO_PIN_11, GPIOB, GPIO_PIN_12 },
    /* 电机4 (右后): PWMB=PA3, BIN1=PB13, BIN2=PB14 */
    { GPIOB, GPIO_PIN_13, GPIOB, GPIO_PIN_14 },
};

/* -------------------------------------------------------------------------- */
/* 初始化所有电机                                                             */
/* -------------------------------------------------------------------------- */
void Motor_InitAll(void)
{
    /* 配置 TB6612 方向引脚为推挽输出 */
    GPIO_InitTypeDef gpio_dir = {0};
    gpio_dir.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_dir.Pull  = GPIO_NOPULL;
    gpio_dir.Speed = GPIO_SPEED_FREQ_LOW;

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        gpio_dir.Pin = motor_dir_pins[i].in1_pin;
        HAL_GPIO_Init(motor_dir_pins[i].in1_port, &gpio_dir);
        gpio_dir.Pin = motor_dir_pins[i].in2_pin;
        HAL_GPIO_Init(motor_dir_pins[i].in2_port, &gpio_dir);
        /* 初始化为刹车态 (IN1=L, IN2=L) */
        HAL_GPIO_WritePin(motor_dir_pins[i].in1_port, motor_dir_pins[i].in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_dir_pins[i].in2_port, motor_dir_pins[i].in2_pin, GPIO_PIN_RESET);
    }
    DBG_DEBUG(TAG_MOTOR, "TB6612 direction pins configured: PB0/1/2/10/11/12/13/14");

    /* MC520P30_12V: 编码器 13PPR (电机轴), 减速比 30:1 */
    /* 两个补偿维度相互独立, 需分别实测 (2026-08-10 三轮标定后定稿):
     *   direction_sign = -1 → 编码器 A/B 相接反 (计数方向与物理转向相反)
     *   invert_dir     = true → 电机正负引线接反 (IN1=H/IN2=L 时转向与代码假设相反)
     * 最终结论 (实测数据推导):
     *   M1: 仅编码器反 (REVERSE, invert=false)
     *   M2: 硬件完全正常 (FORWARD, invert=false)
     *   M3: 编码器反 + 引线反 (REVERSE, invert=true)
     *   M4: 仅引线反 (FORWARD, invert=true)
     * 注: M2/M3 在原始固件下均表现为"读数正+物理反", 但原因不同
     *     (M2=配置双反, M3=硬件双反), B 版实测确认 M3=REVERSE+true 正确 */
    Motor_Init(MOTOR_ID_1, 13, 30.0f, MOTOR_DIR_REVERSE, false);  /* 左前 — 仅编码器A/B反 */
    Motor_Init(MOTOR_ID_2, 13, 30.0f, MOTOR_DIR_FORWARD, false);  /* 右前 — 硬件完全正常 */
    Motor_Init(MOTOR_ID_3, 13, 30.0f, MOTOR_DIR_REVERSE, true);   /* 左后 — 编码器反 + 引线反 */
    Motor_Init(MOTOR_ID_4, 13, 30.0f, MOTOR_DIR_FORWARD, true);   /* 右后 — 仅引线反 */

    /* 启动编码器计数 */
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
    DBG_DEBUG(TAG_MOTOR, "Encoder timers started: TIM1,TIM3,TIM4,TIM8");

    /* 启动 PWM 输出 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    DBG_DEBUG(TAG_MOTOR, "PWM outputs started: TIM2 CH1~CH4");

    DBG_INFO(TAG_MOTOR, "All 4 motors ready (13PPR, x4=52cnt/rev, IncPID Kp=0.001 Ki=0.015 Kd=0.00001)");
}

/* -------------------------------------------------------------------------- */
/* 初始化单个电机                                                             */
/* -------------------------------------------------------------------------- */
void Motor_Init(uint8_t id, uint16_t ppr, float gear_ratio, int8_t dir_sign, bool invert)
{
    if (id >= MOTOR_COUNT) {
        DBG_ERROR(TAG_MOTOR, "Motor_Init: invalid id=%d", id);
        return;
    }

    Motor_Handle *m = &g_motor[id];
    memset(m, 0, sizeof(Motor_Handle));

    /* 硬件绑定 */
    m->htim_encoder  = motor_hw[id].htim_encoder;
    m->htim_pwm      = motor_hw[id].htim_pwm;
    m->pwm_channel   = motor_hw[id].pwm_channel;
    m->pwm_period    = motor_hw[id].pwm_period;

    /* TB6612 方向引脚 */
    m->in1_port = motor_dir_pins[id].in1_port;
    m->in1_pin  = motor_dir_pins[id].in1_pin;
    m->in2_port = motor_dir_pins[id].in2_port;
    m->in2_pin  = motor_dir_pins[id].in2_pin;

    /* 编码器参数 */
    m->encoder_ppr  = ppr;
    m->encoder_cnt_per_rev = (int32_t)ppr * 4;  /* TI1 编码器模式 = x4 倍频, 电机轴 */
    m->gear_ratio   = (gear_ratio > 0.0f) ? gear_ratio : 1.0f;
    m->direction_sign = (dir_sign >= 0) ? 1 : -1;
    m->invert_dir   = invert;

    /* 初始读取编码器位置 */
    m->raw_count      = (int32_t)__HAL_TIM_GET_COUNTER(m->htim_encoder);
    m->prev_raw_count = m->raw_count;

    /* PID 初始化（增量式速度环，输出为占空比 -1.0~1.0）
     * 增量式语义: Ki 承担"比例"角色(对误差直接响应), Kp 承担"阻尼"角色(对 Δe 响应)
     * Ki=0.015: 300RPM误差 → Δu=0.045/周期, 约 22 周期(220ms)到满量程
     * Kp=0.001: 10RPM速度波动 → Δu=0.01, 轻微阻尼
     * Kd=0.00001: 误差加速度阻尼, 有效 D=Kd/dt=0.001 */
    PID_Init(&m->pid_speed, 0.001f, 0.015f, 0.00001f, -1.0f, 1.0f);

    /* 默认开环模式 */
    m->mode = MOTOR_MODE_OPEN_LOOP;
    m->duty = 0.0f;
    m->target_speed_rpm = 0.0f;

    DBG_DEBUG(TAG_MOTOR, "M%d init: PPR=%d, gear=%.0f:1, cnt/rev=%ld, dir=%d, invert=%d, PWM_period=%lu",
              id + 1, ppr, (double)m->gear_ratio, (long)m->encoder_cnt_per_rev,
              dir_sign, invert, (unsigned long)m->pwm_period);
}

/* -------------------------------------------------------------------------- */
/* 读取编码器计数并处理 16 位翻转                                             */
/* -------------------------------------------------------------------------- */
static int32_t Motor_ReadEncoder(Motor_Handle *m)
{
    int32_t raw = (int32_t)(uint16_t)__HAL_TIM_GET_COUNTER(m->htim_encoder);
    int32_t delta = raw - m->prev_raw_count;

    /* 处理 16-bit 计数器溢出/翻转 */
    /* 编码器模式使用整个 16 位 CNT，范围 0~65535 */
    if (delta > 32767) {
        delta -= 65536;   /* 向上溢出 → 实际是负方向 */
    } else if (delta < -32767) {
        delta += 65536;   /* 向下溢出 → 实际是正方向 */
    }

    m->prev_raw_count = raw;
    m->raw_count = raw;
    m->count_delta = delta;
    m->total_count += delta;

    return delta;
}

/* -------------------------------------------------------------------------- */
/* 更新单个电机                                                               */
/* -------------------------------------------------------------------------- */
void Motor_Update(uint8_t id, float dt)
{
    if (id >= MOTOR_COUNT) return;
    if (dt <= 0.0f) return;

    Motor_Handle *m = &g_motor[id];

    /* 读取编码器 */
    int32_t delta = Motor_ReadEncoder(m);

    /* 计算速度 (RPM) — 电机轴转速 */
    /* motor_shaft_rpm = (delta_counts / dt_seconds) * 60 / cnt_per_rev */
    float speed_counts_per_sec = (float)delta / dt;
    float motor_shaft_rpm = (speed_counts_per_sec * 60.0f) / (float)m->encoder_cnt_per_rev
                            * m->direction_sign;
    /* 转换为输出轴 (轮子) 转速 */
    m->speed_rpm = motor_shaft_rpm / m->gear_ratio;

    /* 累计位置（圈） */
    m->position = (float)m->total_count / (float)m->encoder_cnt_per_rev
                  * m->direction_sign;

    /* 根据模式更新输出 */
    if (m->mode == MOTOR_MODE_SPEED) {
        /* 速度闭环 PID */
        PID_SetSetpoint(&m->pid_speed, m->target_speed_rpm);
        m->duty = PID_Compute(&m->pid_speed, m->speed_rpm, dt);
    }
    /* 开环模式下 duty 由外部 Motor_SetDuty 设置 */

    /* 更新 PWM 输出 */
    float abs_duty = fabsf(m->duty);
    if (abs_duty > 1.0f) abs_duty = 1.0f;

    uint32_t pulse = (uint32_t)(abs_duty * (float)m->pwm_period);

    /* TB6612 方向控制 */
    if (m->duty > 0.001f) {
        /* 正转: IN1=H, IN2=L (若 invert 则互换) */
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, m->invert_dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, m->invert_dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else if (m->duty < -0.001f) {
        /* 反转: IN1=L, IN2=H (若 invert 则互换) */
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, m->invert_dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, m->invert_dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
    } else {
        /* 停止/刹车: IN1=L, IN2=L */
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
        pulse = 0;
    }

    __HAL_TIM_SET_COMPARE(m->htim_pwm, m->pwm_channel, pulse);
}

/* -------------------------------------------------------------------------- */
/* 更新所有电机                                                               */
/* -------------------------------------------------------------------------- */
void Motor_UpdateAll(float dt)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor_Update(i, dt);
    }
}

/* -------------------------------------------------------------------------- */
/* 设置开环占空比                                                             */
/* -------------------------------------------------------------------------- */
void Motor_SetDuty(uint8_t id, float duty)
{
    if (id >= MOTOR_COUNT) return;
    Motor_Handle *m = &g_motor[id];

    if (duty > 1.0f)  duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;

    m->duty = duty;
    m->mode = MOTOR_MODE_OPEN_LOOP;
}

/* -------------------------------------------------------------------------- */
/* 设置目标速度（速度闭环模式）                                               */
/* -------------------------------------------------------------------------- */
void Motor_SetSpeed(uint8_t id, float rpm)
{
    if (id >= MOTOR_COUNT) return;
    Motor_Handle *m = &g_motor[id];

    /* 仅当首次进入速度模式或从停止态切换时重置PID */
    bool need_reset = (m->mode != MOTOR_MODE_SPEED) || (m->target_speed_rpm == 0.0f);

    m->target_speed_rpm = rpm;
    m->mode = MOTOR_MODE_SPEED;
    /* 必须先同步 setpoint 再重置: PID_Reset 用 setpoint 初始化误差历史,
     * 否则复位后首次计算会以"误差从 0 突变"为历史, 产生起步冲击 (2026-08-10 修复) */
    PID_SetSetpoint(&m->pid_speed, rpm);

    if (need_reset) {
        PID_Reset(&m->pid_speed, m->speed_rpm);
    }
}

/* -------------------------------------------------------------------------- */
/* 设置控制模式                                                               */
/* -------------------------------------------------------------------------- */
void Motor_SetMode(uint8_t id, Motor_Mode mode)
{
    if (id >= MOTOR_COUNT) return;
    g_motor[id].mode = mode;
}

/* -------------------------------------------------------------------------- */
/* 停止电机                                                                   */
/* -------------------------------------------------------------------------- */
void Motor_Stop(uint8_t id)
{
    if (id >= MOTOR_COUNT) return;
    Motor_Handle *m = &g_motor[id];

    m->duty = 0.0f;
    m->target_speed_rpm = 0.0f;
    m->mode = MOTOR_MODE_OPEN_LOOP;
    __HAL_TIM_SET_COMPARE(m->htim_pwm, m->pwm_channel, 0);
    PID_Reset(&m->pid_speed, m->speed_rpm);

    /* TB6612 刹车: IN1=L, IN2=L */
    HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
}

/* -------------------------------------------------------------------------- */
/* Getter 函数                                                                */
/* -------------------------------------------------------------------------- */
int32_t Motor_GetEncoderCount(uint8_t id)
{
    if (id >= MOTOR_COUNT) return 0;
    return g_motor[id].raw_count;
}

int32_t Motor_GetEncoderDelta(uint8_t id)
{
    if (id >= MOTOR_COUNT) return 0;
    return g_motor[id].count_delta;
}

float Motor_GetSpeed(uint8_t id)
{
    if (id >= MOTOR_COUNT) return 0.0f;
    return g_motor[id].speed_rpm;
}

float Motor_GetPosition(uint8_t id)
{
    if (id >= MOTOR_COUNT) return 0.0f;
    return g_motor[id].position;
}

void Motor_SetSpeedPID(uint8_t id, float Kp, float Ki, float Kd)
{
    if (id >= MOTOR_COUNT) return;
    PID_SetTunings(&g_motor[id].pid_speed, Kp, Ki, Kd);
}

/* -------------------------------------------------------------------------- */
/* 停止所有电机                                                               */
/* -------------------------------------------------------------------------- */
void Motor_StopAll(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor_Stop(i);
    }
}

/* -------------------------------------------------------------------------- */
/* 设置四个电机独立目标速度 (麦轮控制) — 仅设目标，不重置PID                   */
/* -------------------------------------------------------------------------- */
void Motor_SetAllSpeeds(float rpm1, float rpm2, float rpm3, float rpm4)
{
    float targets[4] = { rpm1, rpm2, rpm3, rpm4 };
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor_Handle *m = &g_motor[i];
        m->target_speed_rpm = targets[i];
        m->mode = MOTOR_MODE_SPEED;
        PID_SetSetpoint(&m->pid_speed, targets[i]);
    }
}

/* -------------------------------------------------------------------------- */
/* 设置左右电机目标速度 (差速驱动) — 仅设目标，不重置PID                      */
/* -------------------------------------------------------------------------- */
void Motor_SetLeftRightSpeed(float left_rpm, float right_rpm)
{
    /* 直接设目标，不重置PID（由Chassis_Update高频调用时保留积分历史） */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor_Handle *m = &g_motor[i];
        float target = (i == MOTOR_LEFT_FRONT || i == MOTOR_LEFT_REAR) ? left_rpm : right_rpm;
        m->target_speed_rpm = target;
        m->mode = MOTOR_MODE_SPEED;
        PID_SetSetpoint(&m->pid_speed, target);
    }
}
