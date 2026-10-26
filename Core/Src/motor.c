/**
  ******************************************************************************
  * @file    motor.c
  * @brief   编码器电机驱动实现
  ******************************************************************************
  */
/* ============ 本文件在做什么 ============
 * 电机这一层只负责一件事：你说"轮子转60转/分"，它保证轮子真的转到60转/分。
 * 怎么做到的？
 *   1. 编码器一直数"轮子转了多少格" → 换算出当前实际转速
 *   2. 实际转速 和目标转速 比较 → PID 算出"PWM(劲儿)该给多大"
 *   3. PWM + 方向脚 → 控制电机正反转和转速
 * 它不关心车往哪走、谁在指挥，只管"把某个轮子的转速稳定在目标值"。
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
/* 【中文名】初始化全部4个电机：配好方向脚、绑定编码器/PWM、启动计数和PWM输出 */
void Motor_InitAll(void)
{
    /* 配置 TB6612 方向引脚为推挽输出。
     * 什么叫"方向脚"？电机有两条控制线 IN1/IN2，它们决定正转还是反转。
     * 这两条线接在芯片的 GPIO（通用输入输出脚）上，要先告诉芯片"这两脚是输出口" */
    GPIO_InitTypeDef gpio_dir = {0};
    gpio_dir.Mode  = GPIO_MODE_OUTPUT_PP;   /* 推挽输出：能主动输出高/低电平 */
    gpio_dir.Pull  = GPIO_NOPULL;           /* 不加上下拉电阻 */
    gpio_dir.Speed = GPIO_SPEED_FREQ_LOW;   /* 低速（控制方向不需要快） */

    /* for 循环：i 从 0 到 3，把 4 个电机的 8 个方向脚都初始化一遍 */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        gpio_dir.Pin = motor_dir_pins[i].in1_pin;
        HAL_GPIO_Init(motor_dir_pins[i].in1_port, &gpio_dir);   /* 初始化 IN1 脚 */
        gpio_dir.Pin = motor_dir_pins[i].in2_pin;
        HAL_GPIO_Init(motor_dir_pins[i].in2_port, &gpio_dir);   /* 初始化 IN2 脚 */
        /* 初始化为刹车态 (IN1=H, IN2=H)。为什么一开机就刹车？
         * 防止上电瞬间电机乱转。IN1/IN2 都拉高 = 短刹车 = 电机锁定不动 */
        HAL_GPIO_WritePin(motor_dir_pins[i].in1_port, motor_dir_pins[i].in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_dir_pins[i].in2_port, motor_dir_pins[i].in2_pin, GPIO_PIN_SET);
    }
    DBG_DEBUG(TAG_MOTOR, "TB6612 direction pins configured: PB0/1/2/10/11/12/13/14");

    /* 下面是 4 个电机的"档案初始化"。
     * 13 = 编码器每转13个脉冲；30 = 减速比30:1；第三个参数 = 方向纠正符号；
     * false/true = 接线要不要对调。这 4 组参数是实测标定出来的（见下方注释） */
    Motor_Init(MOTOR_ID_1, 13, 30.0f, MOTOR_DIR_REVERSE, false);  /* 左前 — 仅编码器A/B反 */
    Motor_Init(MOTOR_ID_2, 13, 30.0f, MOTOR_DIR_FORWARD, false);  /* 右前 — 硬件完全正常 */
    Motor_Init(MOTOR_ID_3, 13, 30.0f, MOTOR_DIR_REVERSE, true);   /* 左后 — 编码器反 + 引线反 */
    Motor_Init(MOTOR_ID_4, 13, 30.0f, MOTOR_DIR_FORWARD, true);   /* 右后 — 仅引线反 */

    /* 启动编码器计数：让 TIM1/3/4/8 这4个定时器开始"数轮子转的格数" */
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
    DBG_DEBUG(TAG_MOTOR, "Encoder timers started: TIM1,TIM3,TIM4,TIM8");

    /* 启动 PWM 输出：让 TIM2 的 4 个通道开始输出控制转速的波 */
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
/* 【中文名】初始化单个电机：给某号电机填好档案（绑定定时器、设编码器参数、初始化PID） */
void Motor_Init(uint8_t id, uint16_t ppr, float gear_ratio, int8_t dir_sign, bool invert)
{
    if (id >= MOTOR_COUNT) {
        DBG_ERROR(TAG_MOTOR, "Motor_Init: invalid id=%d", id);
        return;
    }

    Motor_Handle *m = &g_motor[id];          /* 取出这个电机的档案（& 是"取地址"） */
    memset(m, 0, sizeof(Motor_Handle));      /* 把这份档案整个清零（和 pid.c 里 memset 一个道理） */

    /* 硬件绑定：把"这个电机接在哪个定时器哪路"抄进档案 */
    m->htim_encoder  = motor_hw[id].htim_encoder;   /* 编码器定时器（数格子的） */
    m->htim_pwm      = motor_hw[id].htim_pwm;       /* PWM定时器（控制转速的） */
    m->pwm_channel   = motor_hw[id].pwm_channel;    /* PWM通道（第几路） */
    m->pwm_period    = motor_hw[id].pwm_period;     /* PWM周期（=99） */

    /* TB6612 方向引脚：把"方向脚接在哪个脚"抄进档案 */
    m->in1_port = motor_dir_pins[id].in1_port;
    m->in1_pin  = motor_dir_pins[id].in1_pin;
    m->in2_port = motor_dir_pins[id].in2_port;
    m->in2_pin  = motor_dir_pins[id].in2_pin;

    /* 编码器参数 */
    m->encoder_ppr  = ppr;                              /* 每转脉冲数 = 13 */
    m->encoder_cnt_per_rev = (int32_t)ppr * 4;          /* 每转计数 = 13×4 = 52（倍频） */
    m->gear_ratio   = (gear_ratio > 0.0f) ? gear_ratio : 1.0f;  /* 减速比=30，若非法就用1 */
    m->direction_sign = (dir_sign >= 0) ? 1 : -1;       /* 方向符号：正输入→1，负输入→-1 */
    m->invert_dir   = invert;                           /* 接线要不要对调 */

    /* 初始读取编码器位置（记录起点，为后面算增量做准备） */
    m->raw_count      = (int32_t)__HAL_TIM_GET_COUNTER(m->htim_encoder);
    m->prev_raw_count = m->raw_count;

    /* PID 初始化：给这个电机的速度环创建一个 PID 盒子，
     * 三个系数 Kp=0.001, Ki=0.015, Kd=0.00001，输出范围 -1.0 ~ +1.0（即占空比 -100%~+100%）。
     * 为什么 Ki 比 Kp 大 15 倍？因为增量式 PID 里消稳态误差主要靠 I 项（P 只看变化） */
    PID_Init(&m->pid_speed, 0.001f, 0.015f, 0.00001f, -1.0f, 1.0f);

    /* 默认开环模式：不上电跑，占空比和目标转速都是0 */
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
/* 【中文名】读编码器增量：看看"这一小段时间里，轮子又往前(后)转了几格" */
static int32_t Motor_ReadEncoder(Motor_Handle *m)
{
    /* 读当前计数值。计数器是16位的，最大存到65535，满了会回绕到0 */
    int32_t raw = (int32_t)(uint16_t)__HAL_TIM_GET_COUNTER(m->htim_encoder);
    /* 增量 = 现在 − 上一次。比如上一轮数到100，现在数到120，说明转了20格 */
    int32_t delta = raw - m->prev_raw_count;

    /* 处理"回绕翻转"：计数器从 65535 突然跳回 0 时，直接相减会得出一个巨大的错数。
     * 比如上一轮 65530，这一轮回绕到 5，直接减 = -65525，其实是正转了 11 格。
     * 所以规定：差值超过 32767（半个周期）就说明发生了回绕，要补/减一个整周期 65536 */
    if (delta > 32767) {
        delta -= 65536;   /* 向上溢出（65535→0）→ 实际是正方向走了 delta-65536 格 */
    } else if (delta < -32767) {
        delta += 65536;   /* 向下溢出（0→65535）→ 实际是负方向 */
    }

    /* 把这次值记下来，为下一轮算增量做准备 */
    m->prev_raw_count = raw;
    m->raw_count = raw;
    m->count_delta = delta;          /* 记录本次增量 */
    m->total_count += delta;         /* 累计总格数 += 本次增量 */

    return delta;
}

/* -------------------------------------------------------------------------- */
/* 更新单个电机                                                               */
/* -------------------------------------------------------------------------- */
/* 【中文名】更新单个电机（每10ms调一次，电机控制的核心链路就在这里）：
 * 读编码器→换算成转速→需要就PWM调速→输出方向和PWM */
void Motor_Update(uint8_t id, float dt)
{
    if (id >= MOTOR_COUNT) return;   /* 编号不合法就退出 */
    if (dt <= 0.0f) return;          /* 时间间隔非法就退出 */

    Motor_Handle *m = &g_motor[id];  /* 取出这个电机的档案 */

    /* ① 读编码器：这一小段时间里轮子转了几格 */
    int32_t delta = Motor_ReadEncoder(m);

    /* ② 把"格数"换算成"转速"（转/分钟），分几步：
     *    格/秒 = 格数 ÷ 时间
     *    电机轴转/分钟 = 格/秒 × 60 ÷ 每转格数（×方向纠正符号）
     *    轮子转/分钟 = 电机轴转/分钟 ÷ 减速比 */
    float speed_counts_per_sec = (float)delta / dt;                    /* 第一步：格/秒 */
    float motor_shaft_rpm = (speed_counts_per_sec * 60.0f) / (float)m->encoder_cnt_per_rev
                            * m->direction_sign;                       /* 第二步：电机轴转速 */
    m->speed_rpm = motor_shaft_rpm / m->gear_ratio;                    /* 第三步：轮子转速 */

    /* ③ 累计位置（一共转了多少圈） */
    m->position = (float)m->total_count / (float)m->encoder_cnt_per_rev
                  * m->direction_sign;

    /* ④ 如果是速度闭环模式，就调用 PID：目标转速 vs 实际转速 → 算出该给多大劲儿 */
    if (m->mode == MOTOR_MODE_SPEED) {
        PID_SetSetpoint(&m->pid_speed, m->target_speed_rpm);  /* 告诉PID目标是多少 */
        m->duty = PID_Compute(&m->pid_speed, m->speed_rpm, dt); /* PID算出占空比 */
    }
    /* 开环模式：duty 由外部直接设置，这里不管 */

    /* ⑤ 把占空比(duty)转成PWM的"高电平时间"。duty取绝对值（方向另算），再×周期 */
    float abs_duty = fabsf(m->duty);
    if (abs_duty > 1.0f) abs_duty = 1.0f;   /* 限制在0~1 */

    uint32_t pulse = (uint32_t)(abs_duty * (float)m->pwm_period);  /* pulse=高电平计数值 */

    /* ⑥ TB6612 方向控制：靠 IN1/IN2 两根脚的高低电平决定正转/反转/刹车 */
    if (m->duty > 0.001f) {
        /* 正转：IN1=高, IN2=低（若这电机接线反了，就互换） */
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, m->invert_dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, m->invert_dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else if (m->duty < -0.001f) {
        /* 反转：IN1=低, IN2=高 */
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, m->invert_dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, m->invert_dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
    } else {
        /* 停止/刹车：IN1=IN2=高，是"短刹车"（电机两端短接，转不起来）。
         * 对比：IN1=IN2=低 是"滑行"（电机悬空，全靠摩擦停）。
         * 这里选刹车，是为了车停下来时能"刹住" */
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
        pulse = 0;
    }

    /* ⑦ 把算好的 pulse 写进 PWM，实际输出控制转速的波形 */
    __HAL_TIM_SET_COMPARE(m->htim_pwm, m->pwm_channel, pulse);
}

/* -------------------------------------------------------------------------- */
/* 更新所有电机                                                               */
/* -------------------------------------------------------------------------- */
/* 【中文名】更新全部4个电机：循环调用 Motor_Update，一次把4个轮子都更新一遍 */
void Motor_UpdateAll(float dt)
{
    /* for 循环：i 从 0 到 3，每轮调用一次 Motor_Update 更新第 i 号电机 */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor_Update(i, dt);
    }
}

/* -------------------------------------------------------------------------- */
/* 设置开环占空比                                                             */
/* -------------------------------------------------------------------------- */
/* 【中文名】直接设占空比（开环，不经过PID）：只给"劲儿"，不管实际转到多少 */
void Motor_SetDuty(uint8_t id, float duty)
{
    if (id >= MOTOR_COUNT) return;       /* 编号不合法（不是0~3）就退出 */
    Motor_Handle *m = &g_motor[id];      /* 取出这个电机的档案 */

    if (duty > 1.0f)  duty = 1.0f;       /* 占空比不许超过 +1.0（+100%） */
    if (duty < -1.0f) duty = -1.0f;      /* 占空比不许低于 -1.0（-100%） */

    m->duty = duty;                       /* 把占空比存进档案 */
    m->mode = MOTOR_MODE_OPEN_LOOP;       /* 设成开环模式（之后 Motor_Update 不再走 PID） */
}

/* -------------------------------------------------------------------------- */
/* 设置目标速度（速度闭环模式）                                               */
/* -------------------------------------------------------------------------- */
/* 【中文名】设目标转速（速度闭环，过PID）：告诉PID"我要这个轮子转到多少转/分" */
void Motor_SetSpeed(uint8_t id, float rpm)
{
    if (id >= MOTOR_COUNT) return;       /* 编号不合法就退出 */
    Motor_Handle *m = &g_motor[id];      /* 取出这个电机的档案 */

    /* 判断要不要重置 PID 记忆：只有"首次进入速度模式"或"从停止态重新启动"时才需要。
     * 否则连续调速时保留记忆，输出更平滑 */
    bool need_reset = (m->mode != MOTOR_MODE_SPEED) || (m->target_speed_rpm == 0.0f);

    m->target_speed_rpm = rpm;           /* 存新的目标转速 */
    m->mode = MOTOR_MODE_SPEED;          /* 设成速度闭环模式 */

    /* 关键顺序：必须先告诉 PID 新目标、再重置记忆。因为 PID_Reset 会用"目标−实测"来初始化误差历史，
     * 如果顺序反了，PID 会以为"误差是从 0 突然跳到很大"，导致第一下猛冲（这是曾经踩过的 bug） */
    PID_SetSetpoint(&m->pid_speed, rpm);  /* 先设目标 */

    if (need_reset) {
        PID_Reset(&m->pid_speed, m->speed_rpm);  /* 再重置记忆（用当前实测速度） */
    }
}

/* -------------------------------------------------------------------------- */
/* 设置控制模式                                                               */
/* -------------------------------------------------------------------------- */
/* 【中文名】切换控制模式：开环 or 速度闭环 */
void Motor_SetMode(uint8_t id, Motor_Mode mode)
{
    if (id >= MOTOR_COUNT) return;       /* 编号不合法就退出 */
    g_motor[id].mode = mode;             /* 直接把这个电机的 mode 格子改成新值 */
}

/* -------------------------------------------------------------------------- */
/* 停止电机                                                                   */
/* -------------------------------------------------------------------------- */
/* 【中文名】停止单个电机：占空比清零 + 短刹车，让轮子立刻刹住 */
void Motor_Stop(uint8_t id)
{
    if (id >= MOTOR_COUNT) return;       /* 编号不合法就退出 */
    Motor_Handle *m = &g_motor[id];      /* 取出这个电机的档案 */

    m->duty = 0.0f;                      /* 劲儿清零 */
    m->target_speed_rpm = 0.0f;          /* 目标转速清零 */
    m->mode = MOTOR_MODE_OPEN_LOOP;      /* 切到开环（脱离 PID 闭环） */
    __HAL_TIM_SET_COMPARE(m->htim_pwm, m->pwm_channel, 0);  /* PWM 输出清零 */
    PID_Reset(&m->pid_speed, m->speed_rpm);  /* 重置 PID 记忆，防止残留 */

    /* TB6612 短刹车: IN1=H, IN2=H。两根方向脚都拉高 = 电机两端短接，
     * 此时电机转起来的阻力很大，能"刹住"，而不是"滑行慢慢停" */
    HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
}

/* -------------------------------------------------------------------------- */
/* Getter 函数（读取电机状态用的，函数名都很直白）                             */
/* -------------------------------------------------------------------------- */
/* 【中文名】读编码器当前原始计数值 */
int32_t Motor_GetEncoderCount(uint8_t id)
{
    if (id >= MOTOR_COUNT) return 0;      /* 编号不合法就返回 0 */
    return g_motor[id].raw_count;         /* 返回这个电机的"当前原始计数" */
}

/* 【中文名】读本周期编码器增量（转了几格，已处理翻转） */
int32_t Motor_GetEncoderDelta(uint8_t id)
{
    if (id >= MOTOR_COUNT) return 0;      /* 编号不合法就返回 0 */
    return g_motor[id].count_delta;       /* 返回本周期转的格数 */
}

/* 【中文名】读当前速度（转/分钟） */
float Motor_GetSpeed(uint8_t id)
{
    if (id >= MOTOR_COUNT) return 0.0f;   /* 编号不合法就返回 0 */
    return g_motor[id].speed_rpm;         /* 返回当前转速 */
}

/* 【中文名】读累计位置（多少圈） */
float Motor_GetPosition(uint8_t id)
{
    if (id >= MOTOR_COUNT) return 0.0f;   /* 编号不合法就返回 0 */
    return g_motor[id].position;          /* 返回累计圈数 */
}

/* 【中文名】设置某个电机的 PID 参数（Kp/Ki/Kd），就是转发给 PID 盒子 */
void Motor_SetSpeedPID(uint8_t id, float Kp, float Ki, float Kd)
{
    if (id >= MOTOR_COUNT) return;        /* 编号不合法就退出 */
    /* 调用之前 pid.c 里学的 PID_SetTunings，把三个系数写进这个电机的 PID 盒子 */
    PID_SetTunings(&g_motor[id].pid_speed, Kp, Ki, Kd);
}

/* -------------------------------------------------------------------------- */
/* 停止所有电机                                                               */
/* 为了尽量“同时”停止，分两阶段：
 *   1) 先统一清空 4 个电机的控制状态和 PWM
 *   2) 再统一拉高 4 路 TB6612 IN 引脚，进入短刹车
 * 避免逐颗电机“状态复位+写刹车引脚”造成更大的时间差。
 * -------------------------------------------------------------------------- */
/* 【中文名】停止全部4个电机：分两步走，先统一清状态，再统一刹车，尽量让4轮同时停 */
void Motor_StopAll(void)
{
    /* 第一遍：先把 4 个电机的劲儿、目标、模式、PWM 全清零，PID 记忆也重置 */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor_Handle *m = &g_motor[i];
        m->duty = 0.0f;                       /* 劲儿清零 */
        m->target_speed_rpm = 0.0f;           /* 目标清零 */
        m->mode = MOTOR_MODE_OPEN_LOOP;       /* 脱离闭环 */
        __HAL_TIM_SET_COMPARE(m->htim_pwm, m->pwm_channel, 0);  /* PWM 清零 */
        PID_Reset(&m->pid_speed, m->speed_rpm);  /* 重置 PID 记忆 */
    }

    /* 第二遍：四路方向脚同时拉高，进入短刹车。
     * 为什么分两遍？如果一遍里"清状态+刹车"逐个做，4个轮子刹车时间点会差好几毫秒，
     * 车可能被带歪。两遍法让"状态清零"和"刹车"各自统一完成，更接近同步 */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor_Handle *m = &g_motor[i];
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);  /* IN1 拉高 */
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);  /* IN2 拉高 = 短刹车 */
    }
}

/* -------------------------------------------------------------------------- */
/* 设置四个电机独立目标速度 (麦轮控制) — 仅设目标，不重置PID                   */
/* -------------------------------------------------------------------------- */
/* 【中文名】分别设置4个电机的目标转速（麦轮控制用，不重置PID） */
void Motor_SetAllSpeeds(float rpm1, float rpm2, float rpm3, float rpm4)
{
    /* 把 4 个目标转速先装进一个数组，方便下面循环统一处理 */
    float targets[4] = { rpm1, rpm2, rpm3, rpm4 };
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor_Handle *m = &g_motor[i];        /* 取出第 i 号电机档案 */
        m->target_speed_rpm = targets[i];      /* 写目标转速 */
        m->mode = MOTOR_MODE_SPEED;           /* 切到速度闭环 */
        PID_SetSetpoint(&m->pid_speed, targets[i]);  /* 告诉 PID 新目标（不重置记忆） */
    }
}

/* -------------------------------------------------------------------------- */
/* 设置左右电机目标速度 (差速驱动) — 仅设目标，不重置PID                      */
/* -------------------------------------------------------------------------- */
/* 【中文名】设置左右两侧电机目标转速（差速驱动用，左轮统一一个转速、右轮统一一个转速） */
void Motor_SetLeftRightSpeed(float left_rpm, float right_rpm)
{
    /* 直接设目标，不重置PID（由Chassis_Update高频调用时保留积分历史） */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor_Handle *m = &g_motor[i];
        /* 判断这个电机是左轮还是右轮：左轮（前左/后左）给 left_rpm，否则给 right_rpm。
         * 三元运算符：条件 ? 成立时的值 : 不成立时的值 */
        float target = (i == MOTOR_LEFT_FRONT || i == MOTOR_LEFT_REAR) ? left_rpm : right_rpm;
        m->target_speed_rpm = target;          /* 写目标转速 */
        m->mode = MOTOR_MODE_SPEED;           /* 切到速度闭环 */
        PID_SetSetpoint(&m->pid_speed, target);  /* 告诉 PID 新目标 */
    }
}
