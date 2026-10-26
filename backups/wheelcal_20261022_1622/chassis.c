/**
 ******************************************************************************
 * @file    chassis.c
 * @brief   底盘控制实现 — 麦轮 (Mecanum) 运动学
 *
 * 麦轮逆运动学 (机器人坐标系 → 四轮转速, ROS 约定 X=前 Y=左 ωz逆时针正):
 *   ω_FL = (1/R) * ( Vx - Vy - ωz * L )
 *   ω_FR = (1/R) * ( Vx + Vy + ωz * L )
 *   ω_RL = (1/R) * ( Vx + Vy - ωz * L )
 *   ω_RR = (1/R) * ( Vx - Vy + ωz * L )
 *   其中 L = Lx + Ly, R = 轮半径
 *   物理模式: 前进=四轮同向转, 横移=对角轮反向(FL/RR 一组), 旋转=斜对角反向
 *   注: 2026-08-12 实测确认本公式即为标准麦轮 x=前/y=左 正确公式
 *       (此前误做 90° 旋转变换导致 y 轴变前后, 已恢复)
 *
 * 控制模式:
 *   A. 速度矢量 (Chassis_MoveMecanum): Vx, Vy, ωz 连续控制
 *   B. 导航模式 (Chassis_GoDirection): IDLE→TURNING→MOVING→DONE
 *   C. 手动模式 (Chassis_MoveManual): 兼容旧接口
 *   D. 单轮控制: 独立控制每个麦轮
 *
 * 轮序: M1=FL(左前), M2=FR(右前), M3=RL(左后), M4=RR(右后)
 ******************************************************************************
 */
/* ============ 本文件在做什么（一句话） ============
 * 它是一个"翻译器"：把"车想怎么动"（前进/横移/转），翻译成"四个轮子各转多少转"。
 * 它夹在"上层（ROS/导航给的 Vx,Vy,ωz）"和"下层（motor.c 管单个轮子）"之间。
 *
 * 三个速度记号（ROS 约定，已经统一过）：
 *   Vx = 前进速度（车头方向）
 *   Vy = 横移速度（朝车左侧）
 *   ωz = 旋转角速度（逆时针）
 *
 * 麦轮为什么能横着走？轮子表面有一圈 45° 的小辊子，能把"转动"拐成斜向的力。
 * 四个轮子分两组（FL+RR 一组、FR+RL 一组），两组辊子方向相反，
 * 于是"同转→前进、反着转→横移"。 */
#include "chassis.h"
#include "debug.h"
#include "motor.h"
#include "ahrs.h"
#include "pid.h"
#include <math.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* 默认配置 (65mm 麦轮, 150mm 半轴距)                                          */
/* -------------------------------------------------------------------------- */
/* 这些是车的"固定身材尺寸"，一开机就用默认值填好 */
#define DEFAULT_WHEEL_DIAMETER   0.065f    /**< 65mm 轮径 */
#define DEFAULT_WHEEL_BASE_X     0.075f    /**< 75mm 左右半间距 */
#define DEFAULT_WHEEL_BASE_Y     0.075f    /**< 75mm 前后半间距 */
#define DEFAULT_MAX_SPEED        1.0f      /**< 1.0 m/s 最大线速度 */
#define DEFAULT_MAX_ANGULAR      3.0f      /**< 3.0 rad/s 最大角速度 */
#define DEFAULT_HEADING_TOL      3.0f      /**< 3° 航向容差（转到这个精度内就算对准了） */
#define DEFAULT_DISTANCE_TOL     0.02f     /**< 2cm 距离容差（走到这个精度内就算到了） */

#ifndef M_PI
#define M_PI 3.14159265f
#endif

/* -------------------------------------------------------------------------- */
/* 内部数据结构                                                               */
/* -------------------------------------------------------------------------- */
/* 【中文名】底盘"档案袋"：存车当前的状态、目标、身材、PID 和距离统计 */
static struct {
    Chassis_State state;          /* 当前状态（IDLE/TURNING/MOVING/MANUAL/DONE） */
    Chassis_Config cfg;           /* 身材配置（轮径、轴距、限速等） */

    /* === 导航模式参数（发"走到哪"命令时记下目标和进度） === */
    float target_heading;         /* 目标航向（要转到朝哪） */
    float target_speed;           /* 目标速度（走多快） */
    float target_distance;        /* 目标距离（走多远） */
    float traveled_distance;      /* 已走距离（实时累加） */

    /* === 速度矢量参数（"V,"命令直接给的三个速度） === */
    float cmd_vx;                   /**< 命令: 前进速度 (m/s)，车头方向 */
    float cmd_vy;                   /**< 命令: 横移速度 (m/s)，朝车左侧 */
    float cmd_omega;                /**< 命令: 角速度 (rad/s)，逆时针 */

    /* PID 控制器（航向 PID，用来"转到目标角度"） */
    PID_Controller pid_heading;

    /* 上次编码器位置 (4 轮距离累计)，用于算"这一小段又走了多远" */
    float prev_pos[4];

} chassis;

/* -------------------------------------------------------------------------- */
/* 内部辅助函数                                                               */
/* -------------------------------------------------------------------------- */

/** 限幅到 [min, max] */
/* 【中文名】夹范围：把 val 限制在 [min, max] 之间。超上限就压到上限，低下限就抬到下限 */
static float Clamp(float val, float min, float max)
{
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

/** 线速度 (m/s) → 车轮转速 (RPM) */
/* 【中文名】换算：把"每秒走几米"换成"轮子每分钟转几圈"。
 * 原理：轮子转一圈前进 = 圆周 = π×直径；所以 转速(圈/分) = 速度(米/秒)×60 ÷ (π×直径) */
static float MpsToRPM(float mps)
{
    if (chassis.cfg.wheel_diameter <= 0.0f) return 0.0f;   /* 轮径非法就返回0 */
    return mps * 60.0f / (M_PI * chassis.cfg.wheel_diameter);
}

/** 编码器位置(圈) → 行驶距离(m) */
/* 【中文名】换算：把"轮子转了几圈"换成"走了几米"。走了=圈数×圆周(π×直径) */
static float RevToMeters(float rev)
{
    return rev * M_PI * chassis.cfg.wheel_diameter;
}

/** 更新距离累计 (四轮平均值) */
/* 【中文名】更新已走距离：看四个轮子各自"又转了多少"，换算成米，取平均，累加进总里程 */
static void UpdateDistance(void)
{
    float sum_delta = 0.0f;    /* 四个轮子本周期走的距离之和 */
    for (int i = 0; i < 4; i++) {
        float pos = Motor_GetPosition((uint8_t)i);  /* 读第 i 个轮子的累计圈数 */
        float delta = RevToMeters(pos - chassis.prev_pos[i]);  /* 这轮"又走了几米" */
        sum_delta += fabsf(delta);   /* 取绝对值累加（不管正反转，都算走过了） */
        chassis.prev_pos[i] = pos;   /* 记下当前位置，为下一周期算增量 */
    }
    /* 四轮平均 = 车实际前进的距离（打滑时也能互相补偿一点） */
    chassis.traveled_distance += sum_delta / 4.0f;
}

/** 重置距离累计 */
/* 【中文名】重置已走距离：把里程清零，并记下四个轮子的当前位置作为新起点 */
static void ResetDistance(void)
{
    chassis.traveled_distance = 0.0f;
    for (int i = 0; i < 4; i++) {
        chassis.prev_pos[i] = Motor_GetPosition((uint8_t)i);
    }
}

/**
 * @brief 麦轮逆运动学: Vx,Vy,ωz → 四轮 RPM
 * @param vx    纵向速度 (m/s), 前为正 (ROS 口径)
 * @param vy    横向速度 (m/s), 左为正 (ROS 口径)
 * @param omega 旋转角速度 (rad/s), 逆时针为正
 */
/* 【中文名】麦轮逆运动学（核心）：把三个速度(Vx前进,Vy横移,ωz旋转)算出四个轮子的转速 */
static void SetMecanumDrive(float vx, float vy, float omega)
{
    float R = chassis.cfg.wheel_diameter / 2.0f;   /* 轮半径 = 直径÷2 = 0.0325m */
    float L = chassis.cfg.wheel_base_x + chassis.cfg.wheel_base_y;  /* L = 半轴距和 = 0.15m */

    if (R <= 0.0f || L <= 0.0f) return;   /* 参数非法就退出 */

    /* 限幅：三个速度都不许超过最大速度/最大角速度 */
    vx    = Clamp(vx,    -chassis.cfg.max_speed,   chassis.cfg.max_speed);
    vy    = Clamp(vy,    -chassis.cfg.max_speed,   chassis.cfg.max_speed);
    omega = Clamp(omega, -chassis.cfg.max_angular, chassis.cfg.max_angular);

    /* 逆运动学：机器人三个速度 → 四个轮子的角速度（弧度/秒）
     * 关键看符号规律（不要背，看规律）：
     *   Vx（前进）→ 四个轮子全 +（同向转）
     *   Vy（横移）→ FL -、FR +、RL +、RR -（对角两组）
     *   ωz（旋转）→ FL -、FR +、RL -、RR +（左右两组）
     * 三份贡献加起来 = 任意运动 */
    float w[4];
    w[0] = ( vx - vy - omega * L) / R;  /* FL: M1 */
    w[1] = ( vx + vy + omega * L) / R;  /* FR: M2 */
    w[2] = ( vx + vy - omega * L) / R;  /* RL: M3 */
    w[3] = ( vx - vy + omega * L) / R;  /* RR: M4 */

    /* 角速度(弧度/秒) → 转速(RPM)：×60 换成分钟，÷(2π) 把弧度换成圈 */
    float rpm[4];
    float max_rpm = MpsToRPM(chassis.cfg.max_speed);   /* 算出最大允许转速 */
    for (int i = 0; i < 4; i++) {
        rpm[i] = w[i] * 60.0f / (2.0f * M_PI);
        rpm[i] = Clamp(rpm[i], -max_rpm, max_rpm);    /* 再夹到最大转速内 */
    }

    /* 把四个目标转速下发给电机层的"分别设目标转速"函数 */
    Motor_SetAllSpeeds(rpm[0], rpm[1], rpm[2], rpm[3]);
}

/** 停止所有电机 */
/* 【中文名】停所有电机：就是转发给电机层的 StopAll */
static void StopAllMotors(void)
{
    Motor_StopAll();
}

/* -------------------------------------------------------------------------- */
/* 公开函数                                                                   */
/* -------------------------------------------------------------------------- */

/* 【中文名】拿默认身材配置：把轮径、轴距、限速等填进 cfg */
void Chassis_GetDefaultConfig(Chassis_Config *cfg)
{
    if (cfg == NULL) return;   /* 指针为空就退出 */
    cfg->wheel_diameter    = DEFAULT_WHEEL_DIAMETER;
    cfg->wheel_base_x      = DEFAULT_WHEEL_BASE_X;
    cfg->wheel_base_y      = DEFAULT_WHEEL_BASE_Y;
    cfg->max_speed         = DEFAULT_MAX_SPEED;
    cfg->max_angular       = DEFAULT_MAX_ANGULAR;
    cfg->heading_tolerance = DEFAULT_HEADING_TOL;
    cfg->distance_tolerance = DEFAULT_DISTANCE_TOL;
}

/* 【中文名】初始化底盘：清空状态、加载身材、创建航向PID、置为IDLE(空闲) */
void Chassis_Init(const Chassis_Config *cfg)
{
    memset(&chassis, 0, sizeof(chassis));   /* 整个底盘档案清零 */

    if (cfg != NULL) {
        chassis.cfg = *cfg;   /* 用传进来的配置 */
    } else {
        Chassis_GetDefaultConfig(&chassis.cfg);  /* 没传就用默认 */
    }

    /* 航向 PID：Kp=2.0, Ki=0.1, Kd=0.05，输出范围 -5~+5 rad/s（即最大转向速度）。
     * 注意这是"位置式语义的航向环"，用来让车转到目标角度 */
    PID_Init(&chassis.pid_heading, 2.0f, 0.1f, 0.05f, -5.0f, 5.0f);

    chassis.state = CHASSIS_STATE_IDLE;   /* 初始空闲 */
    DBG_INFO(TAG_CHASSIS,
             "Chassis init (Mecanum): wheel=%.0fmm, Lx=%.0fmm, Ly=%.0fmm, "
             "max_spd=%.1fm/s, max_ang=%.1frad/s, hdg_tol=%.1f°, dist_tol=%.2fm",
             chassis.cfg.wheel_diameter * 1000.0f,
             chassis.cfg.wheel_base_x * 1000.0f,
             chassis.cfg.wheel_base_y * 1000.0f,
             chassis.cfg.max_speed, chassis.cfg.max_angular,
             chassis.cfg.heading_tolerance, chassis.cfg.distance_tolerance);
}

/* ================================================================== */
/*  麦轮速度矢量控制 (核心)                                            */
/* ================================================================== */

/* 【中文名】麦轮速度控制（ROS "V,"命令走这里）：直接设三个速度，进入 MANUAL(手动) 状态 */
void Chassis_MoveMecanum(float vx, float vy, float omega)
{
    /* 如果之前正在导航（转向/直行），先停掉，避免状态冲突 */
    if (chassis.state == CHASSIS_STATE_TURNING ||
        chassis.state == CHASSIS_STATE_MOVING) {
        Motor_StopAll();
    }

    chassis.cmd_vx    = vx;     /* 记下三个命令速度 */
    chassis.cmd_vy    = vy;
    chassis.cmd_omega = omega;
    chassis.state = CHASSIS_STATE_MANUAL;   /* 切换到手动状态（之后 Update 会持续套用） */
    DBG_DEBUG(TAG_CHASSIS, "Mecanum: Vx=%.3f Vy=%.3f ωz=%.3f", (double)vx, (double)vy, (double)omega);
}

/* ================================================================== */
/*  导航模式: 指定航向 + 速度 + 距离                                  */
/* ================================================================== */

/* 【中文名】导航模式入口（N/E/S/W/DIR 命令走这里）：给定目标航向+速度+距离，进入 TURNING 开始自动导航 */
bool Chassis_GoDirection(float heading_deg, float speed_mps, float distance_m)
{
    /* 只有在"空闲"或"已到达"状态才允许开始新导航（正在跑就拒绝） */
    if (chassis.state != CHASSIS_STATE_IDLE &&
        chassis.state != CHASSIS_STATE_DONE) {
        return false;
    }

    if (distance_m <= 0.0f) return false;   /* 距离必须大于0 */
    speed_mps = Clamp(fabsf(speed_mps), 0.01f, chassis.cfg.max_speed);  /* 速度取正并限幅 */

    /* 归一化目标航向到 0~360 */
    while (heading_deg < 0.0f)    heading_deg += 360.0f;
    while (heading_deg >= 360.0f) heading_deg -= 360.0f;

    chassis.target_heading  = heading_deg;   /* 记下三个目标：航向/速度/距离 */
    chassis.target_speed    = speed_mps;
    chassis.target_distance = distance_m;

    ResetDistance();   /* 里程清零，从头开始算 */
    {
        /* 先用"角度差"算出现在和目标航向差多少，用来预热航向PID（避免起步猛冲） */
        float hdg_err = AHRS_AngleDiff(heading_deg, AHRS_GetHeading());
        PID_Reset(&chassis.pid_heading, -hdg_err);
    }

    chassis.state = CHASSIS_STATE_TURNING;   /* 进入"转向对准"状态 */
    DBG_INFO(TAG_CHASSIS, "GoDirection: heading=%.0f°, speed=%.2fm/s, dist=%.2fm",
             heading_deg, speed_mps, distance_m);
    return true;
}

/* ================================================================== */
/*  手动模式: 连续运动, 无自动停止                                    */
/* ================================================================== */

/* 【中文名】手动模式：只有"前进速度+转向角速度"，没有横移，转成麦轮控制 */
void Chassis_MoveManual(float linear_mps, float angular_degps)
{
    /* 线性速度走 Vx(前进)，角速度从"度/秒"转成"弧度/秒"（×π/180） */
    float omega_radps = angular_degps * M_PI / 180.0f;
    Chassis_MoveMecanum(linear_mps, 0.0f, omega_radps);   /* Vx=前进, Vy=0, ωz=转向 */
}

/* ================================================================== */
/*  停止                                                              */
/* ================================================================== */

/* 【中文名】停：状态置空闲，三个命令速度清零，停所有电机 */
void Chassis_Stop(void)
{
    DBG_INFO(TAG_CHASSIS, "STOP requested");
    chassis.state = CHASSIS_STATE_IDLE;
    chassis.cmd_vx = 0.0f;
    chassis.cmd_vy = 0.0f;
    chassis.cmd_omega = 0.0f;
    StopAllMotors();
}

/* ================================================================== */
/*  单轮控制                                                          */
/* ================================================================== */

/* 【中文名】单独控制某个轮子的速度（wheel_id 是 1~4，注意内部是减去1用的 0~3） */
void Chassis_SetWheelSpeed(uint8_t wheel_id, float rpm)
{
    if (wheel_id < 1 || wheel_id > 4) return;   /* 只接受1~4 */
    Motor_SetSpeed(wheel_id - 1, rpm);          /* 减1转成内部编号0~3 */
}

/* 【中文名】单独设某个轮子的占空比 */
void Chassis_SetWheelDuty(uint8_t wheel_id, float duty)
{
    if (wheel_id < 1 || wheel_id > 4) return;
    Motor_SetDuty(wheel_id - 1, duty);
}

/* 【中文名】单独停某个轮子 */
void Chassis_StopWheel(uint8_t wheel_id)
{
    if (wheel_id < 1 || wheel_id > 4) return;
    Motor_Stop(wheel_id - 1);
}

/* ================================================================== */
/*  主更新循环 (每个控制周期调用一次)                                  */
/* ================================================================== */

/* 【中文名】底盘主更新（每10ms调一次）：看"现在在哪个状态"，就执行那个状态该做的事。
 * 这是状态机的核心 —— IDLE闲着 → TURNING转向 → MOVING直行 → DONE到达 */
void Chassis_Update(float dt)
{
    if (dt <= 0.0f) return;   /* 时间非法就退出 */

    /* switch 状态机：根据当前状态，走不同分支 */
    switch (chassis.state) {

    /* ----- 空闲 / 完成：不动作（啥也不干） ----- */
    case CHASSIS_STATE_IDLE:
    case CHASSIS_STATE_DONE:
        break;

    /* ----- 速度矢量模式（麦轮）：持续套用命令的 Vx/Vy/ωz ----- */
    case CHASSIS_STATE_MANUAL:
        SetMecanumDrive(chassis.cmd_vx, chassis.cmd_vy, chassis.cmd_omega);
        break;

    /* ----- 导航：转向对准航向 ----- */
    case CHASSIS_STATE_TURNING: {
        float current_heading = AHRS_GetHeading();   /* 读当前朝向 */
        float heading_error = AHRS_AngleDiff(chassis.target_heading, current_heading); /* 还差多少度 */

        /* 岔路A：误差已经小于容差(3°) → 对准了，转去直行 */
        if (fabsf(heading_error) < chassis.cfg.heading_tolerance) {
            DBG_INFO(TAG_CHASSIS, "Heading aligned (err=%.1f°), start moving", heading_error);
            chassis.state = CHASSIS_STATE_MOVING;   /* 换状态：直行 */
            ResetDistance();                        /* 清零里程，开始记这次走多远 */
            SetMecanumDrive(chassis.target_speed, 0.0f, 0.0f);  /* 开始直行(前进=目标速度) */
            break;
        }

        /* 岔路B：还没对准 → 原地转。用航向PID算出该转多快 */
        PID_SetSetpoint(&chassis.pid_heading, 0.0f);   /* PID目标是"误差=0" */
        float omega_radps = PID_Compute(&chassis.pid_heading, -heading_error, dt);
        SetMecanumDrive(0.0f, 0.0f, omega_radps);   /* Vx=Vy=0，只转不前进 */
        break;
    }

    /* ----- 导航：直行 + 航向微调 ----- */
    case CHASSIS_STATE_MOVING: {
        float current_heading = AHRS_GetHeading();
        float heading_error = AHRS_AngleDiff(chassis.target_heading, current_heading);

        UpdateDistance();   /* 更新"已经走了多远" */

        /* 岔路A：走够了（已走距离 ≥ 目标距离 − 容差）→ 到达，停 */
        if (chassis.traveled_distance >=
            chassis.target_distance - chassis.cfg.distance_tolerance) {
            DBG_INFO(TAG_CHASSIS, "Distance reached (%.2fm/%.2fm), stopping",
                     chassis.traveled_distance, chassis.target_distance);
            chassis.state = CHASSIS_STATE_DONE;   /* 换状态：到达 */
            StopAllMotors();                      /* 停轮 */
            break;
        }

        /* 岔路B：没走够 → 继续直行，同时用航向PID微调，别跑偏 */
        PID_SetSetpoint(&chassis.pid_heading, 0.0f);
        float omega_radps = PID_Compute(&chassis.pid_heading, -heading_error, dt);
        SetMecanumDrive(chassis.target_speed, 0.0f, omega_radps);  /* 前进+转向修正 */
        break;
    }

    default:
        break;
    }
}

/* -------------------------------------------------------------------------- */
/* Getter 函数                                                                */
/* -------------------------------------------------------------------------- */

/* 【中文名】读当前状态（IDLE/TURNING/...） */
Chassis_State Chassis_GetState(void)
{
    return chassis.state;
}

/* 【中文名】读当前航向（就是 AHRS 给的） */
float Chassis_GetCurrentHeading(void)
{
    return AHRS_GetHeading();
}

/* 【中文名】读已走距离 */
float Chassis_GetTraveledDistance(void)
{
    return chassis.traveled_distance;
}

/* 【中文名】读剩余距离（目标距离 − 已走；空闲时返回0） */
float Chassis_GetRemainingDistance(void)
{
    if (chassis.state == CHASSIS_STATE_IDLE) return 0.0f;
    return chassis.target_distance - chassis.traveled_distance;
}

/* 【中文名】把状态翻译成能打印的字符串（"IDLE"、"TURNING" 等） */
const char *Chassis_GetStateName(void)
{
    switch (chassis.state) {
        case CHASSIS_STATE_IDLE:    return "IDLE";
        case CHASSIS_STATE_TURNING: return "TURNING";
        case CHASSIS_STATE_MOVING:  return "MOVING";
        case CHASSIS_STATE_MANUAL:  return "MANUAL";
        case CHASSIS_STATE_DONE:    return "DONE";
        default:                    return "UNKNOWN";
    }
}
