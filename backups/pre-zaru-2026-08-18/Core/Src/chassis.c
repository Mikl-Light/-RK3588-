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
#define DEFAULT_WHEEL_DIAMETER   0.065f    /**< 65mm 轮径 */
#define DEFAULT_WHEEL_BASE_X     0.075f    /**< 75mm 左右半间距 */
#define DEFAULT_WHEEL_BASE_Y     0.075f    /**< 75mm 前后半间距 */
#define DEFAULT_MAX_SPEED        1.0f      /**< 1.0 m/s 最大线速度 */
#define DEFAULT_MAX_ANGULAR      3.0f      /**< 3.0 rad/s 最大角速度 */
#define DEFAULT_HEADING_TOL      3.0f      /**< 3° 航向容差 */
#define DEFAULT_DISTANCE_TOL     0.02f     /**< 2cm 距离容差 */

#ifndef M_PI
#define M_PI 3.14159265f
#endif

/* -------------------------------------------------------------------------- */
/* 内部数据结构                                                               */
/* -------------------------------------------------------------------------- */
static struct {
    Chassis_State state;
    Chassis_Config cfg;

    /* === 导航模式参数 === */
    float target_heading;
    float target_speed;
    float target_distance;
    float traveled_distance;

    /* === 速度矢量参数 === */
    float cmd_vx;                   /**< 命令: 横向速度 (m/s) */
    float cmd_vy;                   /**< 命令: 纵向速度 (m/s) */
    float cmd_omega;                /**< 命令: 角速度 (rad/s) */

    /* PID 控制器 */
    PID_Controller pid_heading;

    /* 上次编码器位置 (4 轮距离累计) */
    float prev_pos[4];

} chassis;

/* -------------------------------------------------------------------------- */
/* 内部辅助函数                                                               */
/* -------------------------------------------------------------------------- */

/** 限幅到 [min, max] */
static float Clamp(float val, float min, float max)
{
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

/** 线速度 (m/s) → 车轮转速 (RPM) */
static float MpsToRPM(float mps)
{
    if (chassis.cfg.wheel_diameter <= 0.0f) return 0.0f;
    return mps * 60.0f / (M_PI * chassis.cfg.wheel_diameter);
}

/** 编码器位置(圈) → 行驶距离(m) */
static float RevToMeters(float rev)
{
    return rev * M_PI * chassis.cfg.wheel_diameter;
}

/** 更新距离累计 (四轮平均值) */
static void UpdateDistance(void)
{
    float sum_delta = 0.0f;
    for (int i = 0; i < 4; i++) {
        float pos = Motor_GetPosition((uint8_t)i);
        float delta = RevToMeters(pos - chassis.prev_pos[i]);
        sum_delta += fabsf(delta);
        chassis.prev_pos[i] = pos;
    }
    chassis.traveled_distance += sum_delta / 4.0f;
}

/** 重置距离累计 */
static void ResetDistance(void)
{
    chassis.traveled_distance = 0.0f;
    for (int i = 0; i < 4; i++) {
        chassis.prev_pos[i] = Motor_GetPosition((uint8_t)i);
    }
}

/**
 * @brief 麦轮逆运动学: Vx,Vy,ωz → 四轮 RPM
 * @param vx    横向速度 (m/s), 右为正
 * @param vy    纵向速度 (m/s), 前为正
 * @param omega 旋转角速度 (rad/s), 逆时针为正
 */
static void SetMecanumDrive(float vx, float vy, float omega)
{
    float R = chassis.cfg.wheel_diameter / 2.0f;
    float L = chassis.cfg.wheel_base_x + chassis.cfg.wheel_base_y;

    if (R <= 0.0f || L <= 0.0f) return;

    /* 限幅 */
    vx    = Clamp(vx,    -chassis.cfg.max_speed,   chassis.cfg.max_speed);
    vy    = Clamp(vy,    -chassis.cfg.max_speed,   chassis.cfg.max_speed);
    omega = Clamp(omega, -chassis.cfg.max_angular, chassis.cfg.max_angular);

    /* 逆运动学: 机器人速度 → 轮子角速度 (rad/s) */
    float w[4];
    w[0] = ( vx - vy - omega * L) / R;  /* FL: M1 */
    w[1] = ( vx + vy + omega * L) / R;  /* FR: M2 */
    w[2] = ( vx + vy - omega * L) / R;  /* RL: M3 */
    w[3] = ( vx - vy + omega * L) / R;  /* RR: M4 */

    /* 角速度 → RPM */
    float rpm[4];
    float max_rpm = MpsToRPM(chassis.cfg.max_speed);
    for (int i = 0; i < 4; i++) {
        rpm[i] = w[i] * 60.0f / (2.0f * M_PI);
        rpm[i] = Clamp(rpm[i], -max_rpm, max_rpm);
    }

    Motor_SetAllSpeeds(rpm[0], rpm[1], rpm[2], rpm[3]);
}

/** 停止所有电机 */
static void StopAllMotors(void)
{
    Motor_Stop(MOTOR_ID_1);
    Motor_Stop(MOTOR_ID_2);
    Motor_Stop(MOTOR_ID_3);
    Motor_Stop(MOTOR_ID_4);
}

/* -------------------------------------------------------------------------- */
/* 公开函数                                                                   */
/* -------------------------------------------------------------------------- */

void Chassis_GetDefaultConfig(Chassis_Config *cfg)
{
    if (cfg == NULL) return;
    cfg->wheel_diameter    = DEFAULT_WHEEL_DIAMETER;
    cfg->wheel_base_x      = DEFAULT_WHEEL_BASE_X;
    cfg->wheel_base_y      = DEFAULT_WHEEL_BASE_Y;
    cfg->max_speed         = DEFAULT_MAX_SPEED;
    cfg->max_angular       = DEFAULT_MAX_ANGULAR;
    cfg->heading_tolerance = DEFAULT_HEADING_TOL;
    cfg->distance_tolerance = DEFAULT_DISTANCE_TOL;
}

void Chassis_Init(const Chassis_Config *cfg)
{
    memset(&chassis, 0, sizeof(chassis));

    if (cfg != NULL) {
        chassis.cfg = *cfg;
    } else {
        Chassis_GetDefaultConfig(&chassis.cfg);
    }

    /* 航向 PID: Kp=2.0, Ki=0.1, Kd=0.05, 输出限制 [-5, 5] rad/s */
    PID_Init(&chassis.pid_heading, 2.0f, 0.1f, 0.05f, -5.0f, 5.0f);

    chassis.state = CHASSIS_STATE_IDLE;
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

void Chassis_MoveMecanum(float vx, float vy, float omega)
{
    /* 允许在 IDLE/DONE/MANUAL 状态下直接进入 */
    if (chassis.state == CHASSIS_STATE_TURNING ||
        chassis.state == CHASSIS_STATE_MOVING) {
        Motor_StopAll();
    }

    chassis.cmd_vx    = vx;
    chassis.cmd_vy    = vy;
    chassis.cmd_omega = omega;
    chassis.state = CHASSIS_STATE_MANUAL;
    DBG_DEBUG(TAG_CHASSIS, "Mecanum: Vx=%.3f Vy=%.3f ωz=%.3f", (double)vx, (double)vy, (double)omega);
}

/* ================================================================== */
/*  导航模式: 指定航向 + 速度 + 距离                                  */
/* ================================================================== */

bool Chassis_GoDirection(float heading_deg, float speed_mps, float distance_m)
{
    if (chassis.state != CHASSIS_STATE_IDLE &&
        chassis.state != CHASSIS_STATE_DONE) {
        return false;
    }

    if (distance_m <= 0.0f) return false;
    speed_mps = Clamp(fabsf(speed_mps), 0.01f, chassis.cfg.max_speed);

    /* 归一化航向 */
    while (heading_deg < 0.0f)    heading_deg += 360.0f;
    while (heading_deg >= 360.0f) heading_deg -= 360.0f;

    chassis.target_heading  = heading_deg;
    chassis.target_speed    = speed_mps;
    chassis.target_distance = distance_m;

    ResetDistance();
    {
        float hdg_err = AHRS_AngleDiff(heading_deg, AHRS_GetHeading());
        PID_Reset(&chassis.pid_heading, -hdg_err);
    }

    chassis.state = CHASSIS_STATE_TURNING;
    DBG_INFO(TAG_CHASSIS, "GoDirection: heading=%.0f°, speed=%.2fm/s, dist=%.2fm",
             heading_deg, speed_mps, distance_m);
    return true;
}

/* ================================================================== */
/*  手动模式: 连续运动, 无自动停止                                    */
/* ================================================================== */

void Chassis_MoveManual(float linear_mps, float angular_degps)
{
    /* 兼容旧接口: 转为麦轮 Vx=0, Vy=linear, ωz=angular(rad/s) */
    float omega_radps = angular_degps * M_PI / 180.0f;
    Chassis_MoveMecanum(0.0f, linear_mps, omega_radps);
}

/* ================================================================== */
/*  停止                                                              */
/* ================================================================== */

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

void Chassis_SetWheelSpeed(uint8_t wheel_id, float rpm)
{
    if (wheel_id < 1 || wheel_id > 4) return;
    Motor_SetSpeed(wheel_id - 1, rpm);
}

void Chassis_SetWheelDuty(uint8_t wheel_id, float duty)
{
    if (wheel_id < 1 || wheel_id > 4) return;
    Motor_SetDuty(wheel_id - 1, duty);
}

void Chassis_StopWheel(uint8_t wheel_id)
{
    if (wheel_id < 1 || wheel_id > 4) return;
    Motor_Stop(wheel_id - 1);
}

/* ================================================================== */
/*  主更新循环 (每个控制周期调用一次)                                  */
/* ================================================================== */

void Chassis_Update(float dt)
{
    if (dt <= 0.0f) return;

    switch (chassis.state) {

    /* ----- 空闲 / 完成: 不动作 ----- */
    case CHASSIS_STATE_IDLE:
    case CHASSIS_STATE_DONE:
        break;

    /* ----- 速度矢量模式 (麦轮) ----- */
    case CHASSIS_STATE_MANUAL:
        SetMecanumDrive(chassis.cmd_vx, chassis.cmd_vy, chassis.cmd_omega);
        break;

    /* ----- 导航: 转向对准航向 ----- */
    case CHASSIS_STATE_TURNING: {
        float current_heading = AHRS_GetHeading();
        float heading_error = AHRS_AngleDiff(chassis.target_heading, current_heading);

        if (fabsf(heading_error) < chassis.cfg.heading_tolerance) {
            DBG_INFO(TAG_CHASSIS, "Heading aligned (err=%.1f°), start moving", heading_error);
            chassis.state = CHASSIS_STATE_MOVING;
            ResetDistance();

            /* 直行: Vy=目标速度, Vx=0, ωz=0 */
            SetMecanumDrive(0.0f, chassis.target_speed, 0.0f);
            break;
        }

        /* 原地转向: PID 输出角速度, 麦轮 Vx=Vy=0 */
        PID_SetSetpoint(&chassis.pid_heading, 0.0f);
        float omega_radps = PID_Compute(&chassis.pid_heading, -heading_error, dt);
        SetMecanumDrive(0.0f, 0.0f, omega_radps);
        break;
    }

    /* ----- 导航: 直行 + 航向微调 ----- */
    case CHASSIS_STATE_MOVING: {
        float current_heading = AHRS_GetHeading();
        float heading_error = AHRS_AngleDiff(chassis.target_heading, current_heading);

        UpdateDistance();

        if (chassis.traveled_distance >=
            chassis.target_distance - chassis.cfg.distance_tolerance) {
            DBG_INFO(TAG_CHASSIS, "Distance reached (%.2fm/%.2fm), stopping",
                     chassis.traveled_distance, chassis.target_distance);
            chassis.state = CHASSIS_STATE_DONE;
            StopAllMotors();
            break;
        }

        /* 航向微调 PID → 角速度修正 */
        PID_SetSetpoint(&chassis.pid_heading, 0.0f);
        float omega_radps = PID_Compute(&chassis.pid_heading, -heading_error, dt);

        /* 麦轮: Vy=前进, ωz=航向修正 */
        SetMecanumDrive(0.0f, chassis.target_speed, omega_radps);
        break;
    }

    default:
        break;
    }
}

/* -------------------------------------------------------------------------- */
/* Getter 函数                                                                */
/* -------------------------------------------------------------------------- */

Chassis_State Chassis_GetState(void)
{
    return chassis.state;
}

float Chassis_GetCurrentHeading(void)
{
    return AHRS_GetHeading();
}

float Chassis_GetTraveledDistance(void)
{
    return chassis.traveled_distance;
}

float Chassis_GetRemainingDistance(void)
{
    if (chassis.state == CHASSIS_STATE_IDLE) return 0.0f;
    return chassis.target_distance - chassis.traveled_distance;
}

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
