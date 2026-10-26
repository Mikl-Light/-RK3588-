/**
 ******************************************************************************
 * @file    odometry.c
 * @brief   里程计实现
 *
 * 麦轮正向运动学 (轮速 → 机器人速度, ROS 约定 X=前 Y=左):
 *   Vx  = (R/4) * ( ω₁ + ω₂ + ω₃ + ω₄ )    // 前 (四轮同向=前进)
 *   Vy  = (R/4) * ( -ω₁ + ω₂ + ω₃ - ω₄ )    // 左
 *   ωz  = (R / 4(Lx+Ly)) * ( -ω₁ + ω₂ - ω₃ + ω₄ )
 *
 * 航位推算 (机器人坐标系 → 世界坐标系, X=北 Y=西 右手系):
 *   x += (Vx*cosθ - Vy*sinθ) * dt
 *   y += (Vx*sinθ + Vy*cosθ) * dt
 *   θ = ROS yaw = -heading (heading 为罗盘角 0=北 顺时针)
 *   θ 来自 AHRS 互补滤波 (陀螺仪 + 磁力计)
 *
 * 轮序: M1=FL(左前), M2=FR(右前), M3=RL(左后), M4=RR(右后)
 * 注: 2026-08-12 实测确认本公式即为标准麦轮 x=前/y=左 正确公式
 ******************************************************************************
 */
#include "odometry.h"
#include "motor.h"
#include "ahrs.h"
#include "mpu9250.h"
#include "chassis.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265f
#endif

/* -------------------------------------------------------------------------- */
/* 默认几何参数 (65mm 麦轮, 150mm 半轴距)                                      */
/* -------------------------------------------------------------------------- */
#define DEFAULT_WHEEL_RADIUS     0.0325f   /**< 32.5mm 半径 (65mm 直径) */
#define DEFAULT_WHEEL_BASE_X     0.075f    /**< 75mm 左右半间距 (150mm/2) */
#define DEFAULT_WHEEL_BASE_Y     0.075f    /**< 75mm 前后半间距 */
#define DEFAULT_ENC_CNT_PER_REV  52.0f     /**< 13PPR × 4 倍频 = 52 */
#define DEFAULT_GEAR_RATIO       30.0f     /**< 30:1 减速比 */

/* -------------------------------------------------------------------------- */
/* 内部状态                                                                   */
/* -------------------------------------------------------------------------- */
static Odom_Config  g_cfg;
static Odom_Data    g_data;
static bool         g_initialized = false;

/* -------------------------------------------------------------------------- */
/* 三角函数: 直接用 libm 的 sinf/cosf (CMake 已链接 -lm, F407 硬浮点)          */
/* 注: 旧版自写泰勒近似仅在 |x|<40° 内有效, 航向达 180°/270° 时误差数百倍,   */
/*     导致里程计完全失效 — 已废弃 (2026-08-10 修复)                             */
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* 初始化                                                                     */
/* -------------------------------------------------------------------------- */
void Odom_GetDefaultConfig(Odom_Config *cfg)
{
    if (cfg == NULL) return;
    cfg->wheel_radius        = DEFAULT_WHEEL_RADIUS;
    cfg->wheel_base_x        = DEFAULT_WHEEL_BASE_X;
    cfg->wheel_base_y        = DEFAULT_WHEEL_BASE_Y;
    cfg->encoder_cnt_per_rev = DEFAULT_ENC_CNT_PER_REV;
    cfg->gear_ratio          = DEFAULT_GEAR_RATIO;
}

void Odom_Init(const Odom_Config *cfg)
{
    memset(&g_data, 0, sizeof(g_data));

    if (cfg != NULL) {
        g_cfg = *cfg;
    } else {
        Odom_GetDefaultConfig(&g_cfg);
    }

    g_data.pose.heading = AHRS_GetHeading();
    g_initialized = true;
}

/* -------------------------------------------------------------------------- */
/* 主更新                                                                     */
/* -------------------------------------------------------------------------- */
void Odom_Update(float dt)
{
    if (!g_initialized || dt <= 0.0f) return;

    /* ==== 1. 读取编码器原始数据 ==== */
    for (int i = 0; i < 4; i++) {
        g_data.enc_raw[i]   = Motor_GetEncoderCount((uint8_t)i);
        g_data.enc_delta[i] = Motor_GetEncoderDelta((uint8_t)i);
        g_data.wheel_rpm[i] = Motor_GetSpeed((uint8_t)i);

        /* 线速度用已含方向符号的 RPM 换算 (2026-08-10 修复:
         * 旧代码用原始 enc_delta, direction_sign=-1 的轮子(M1/M3)正转时
         * 里程计把其速度当负值, 导致 Vx/Vy/Wz 与 STATUS 显示不一致) */
        g_data.wheel_mps[i] = g_data.wheel_rpm[i] * (2.0f * M_PI / 60.0f)
                              * g_cfg.wheel_radius;
    }

    /* ==== 2. 读取 IMU 原始数据 ==== */
    MPU9250_Data imu;
    if (MPU9250_ReadAll(&imu) == 0) {
        g_data.accel_x = imu.accel_x;
        g_data.accel_y = imu.accel_y;
        g_data.accel_z = imu.accel_z;
        g_data.gyro_x  = imu.gyro_x;
        g_data.gyro_y  = imu.gyro_y;
        g_data.gyro_z  = imu.gyro_z;
        g_data.mag_x   = imu.mag_x;
        g_data.mag_y   = imu.mag_y;
        g_data.mag_z   = imu.mag_z;
        g_data.imu_temp = imu.temperature;
        /* 原始寄存器值 (int16 LSB, 供二进制遥测帧) */
        g_data.accel_raw[0] = imu.accel_raw[0];
        g_data.accel_raw[1] = imu.accel_raw[1];
        g_data.accel_raw[2] = imu.accel_raw[2];
        g_data.gyro_raw[0]  = imu.gyro_raw[0];
        g_data.gyro_raw[1]  = imu.gyro_raw[1];
        g_data.gyro_raw[2]  = imu.gyro_raw[2];
        g_data.mag_raw[0]   = imu.mag_raw[0];
        g_data.mag_raw[1]   = imu.mag_raw[1];
        g_data.mag_raw[2]   = imu.mag_raw[2];
    }
    /* IMU 读取失败 → 保持上次值, 不阻塞 */

    /* ==== 3. 麦轮正向运动学: 轮速 → Vx, Vy, ωz (机器人坐标系) ==== */
    /* ω_i = 各轮角速度 (rad/s) */
    float w[4];
    for (int i = 0; i < 4; i++) {
        w[i] = g_data.wheel_mps[i] / g_cfg.wheel_radius;
    }

    float R  = g_cfg.wheel_radius;
    float L  = g_cfg.wheel_base_x + g_cfg.wheel_base_y;  /* L = Lx + Ly */

    g_data.pose.vx   = (R / 4.0f) * ( w[0] + w[1] + w[2] + w[3]);  /* 前 */
    g_data.pose.vy   = (R / 4.0f) * (-w[0] + w[1] + w[2] - w[3]);  /* 左 */
    g_data.pose.omega = (R / (4.0f * L)) * (-w[0] + w[1] - w[2] + w[3]);

    /* ==== 4. 航位推算: 机器人坐标系 → 世界坐标系 ==== */
    /* 每周期同步 AHRS 航向 (2026-08-16 修复: 之前只在 Init/SetPose 赋值,
     * 运行中 pose.heading 冻结在初始值, 导致 $STAT HDG 不动、投影方向错) */
    g_data.pose.heading = AHRS_GetHeading();

    /* θ = ROS yaw = -heading (罗盘角 0=北 顺时针 → 逆时针正) */
    float heading_rad = -g_data.pose.heading * M_PI / 180.0f;
    float cos_h = cosf(heading_rad);
    float sin_h = sinf(heading_rad);

    /* 世界坐标系增量: dx_world = Vx*cosθ - Vy*sinθ, dy_world = Vx*sinθ + Vy*cosθ */
    float dx = (g_data.pose.vx * cos_h - g_data.pose.vy * sin_h) * dt;
    float dy = (g_data.pose.vx * sin_h + g_data.pose.vy * cos_h) * dt;

    g_data.pose.x += dx;
    g_data.pose.y += dy;
}

/* -------------------------------------------------------------------------- */
/* Getter                                                                     */
/* -------------------------------------------------------------------------- */
const Odom_Pose *Odom_GetPose(void)
{
    return &g_data.pose;
}

const Odom_Data *Odom_GetData(void)
{
    return &g_data;
}

void Odom_Reset(void)
{
    g_data.pose.x = 0.0f;
    g_data.pose.y = 0.0f;
    /* 航向保持 (不清零, 因为来自 AHRS) */
}

void Odom_SetPose(float x, float y, float heading)
{
    g_data.pose.x = x;
    g_data.pose.y = y;
    g_data.pose.heading = heading;
}
