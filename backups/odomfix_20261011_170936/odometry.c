/**
 ******************************************************************************
 * @file    odometry.c
 * @brief   可靠麦轮里程计实现
 *
 * 参考 Core/docs/AI给出的建议.md，把原“编码器航位推算 + AHRS 航向”升级为：
 *
 *   [原始传感器]
 *       ↓
 *   第一层 数据质量监督  —— 静止/打滑/磁干扰/异常加计
 *       ↓
 *   第二层 动态协方差    —— 根据运动状态实时调整 Q_xy/Q_yaw/R_heading
 *       ↓
 *   第三层 EKF 核心      —— 编码器预测 + 航向测量更新
 *       ↓
 *   第四层 ZUPT          —— 静止时锁死协方差，防停车零漂
 *
 * 状态向量 X = [x, y, yaw_ros]
 *   - x,y       世界坐标 (m)，与旧版一致：X=北(起始前方), Y=西
 *   - yaw_ros   机器人 ROS yaw (rad)，逆时针为正；对外 heading=罗盘角(°)
 * 预测输入 = 麦轮正运动学解算的 vx, vy, ωz (机器人坐标系)
 * 测量输入 = AHRS_GetHeading()（由 AHRS 内部按 AHRSMAG 开关选择
 *           纯陀螺或陀螺+磁力计互补），EKF 航向紧密跟随 AHRS，
 *           避免原始磁力计轴映射/标定偏差导致轮式 yaw 偏大。
 *
 * 轮序: M1=FL(左前), M2=FR(右前), M3=RL(左后), M4=RR(右后)
 ******************************************************************************
 */
#include "odometry.h"
#include "motor.h"
#include "ahrs.h"
#include "mpu9250.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265f
#endif

#define DEG2RAD  0.017453292519943295f
#define RAD2DEG  57.29577951308232f

/* -------------------------------------------------------------------------- */
/* 默认几何参数 (65mm 麦轮, 150mm 半轴距)                                      */
/* -------------------------------------------------------------------------- */
#define DEFAULT_WHEEL_RADIUS     0.0325f   /**< 32.5mm 半径 (65mm 直径) */
#define DEFAULT_WHEEL_BASE_X     0.075f    /**< 75mm 左右半间距 (150mm/2) */
#define DEFAULT_WHEEL_BASE_Y     0.075f    /**< 75mm 前后半间距 */
#define DEFAULT_ENC_CNT_PER_REV  52.0f     /**< 13PPR × 4 倍频 = 52 */
#define DEFAULT_GEAR_RATIO       30.0f     /**< 30:1 减速比 */

/* -------------------------------------------------------------------------- */
/* 可靠里程计监督层阈值 (现场可微调)                                           */
/* -------------------------------------------------------------------------- */
#define ODOM_STATIONARY_SPEED_MPS  0.01f   /**< 线速度低于此值视为静止 (m/s) */
#define ODOM_STATIONARY_OMEGA_RPS  0.01f   /**< 角速度低于此值视为静止 (rad/s) */
#define ODOM_ACCEL_SLIP_MS2        2.5f    /**< 水平加速度超过此值加强位置 Q (m/s^2) */
#define ODOM_GYRO_SLIP_RADPS       0.4f    /**< |omega_enc-gyro| 超过此值判定转向打滑 */
#define ODOM_MAG_JUMP_RAD          0.6f    /**< 磁航向与当前 yaw 偏差上限 (rad) */
#define ODOM_MAG_HIGH_GYRO_RADPS   1.0f    /**< 剧烈旋转时忽略磁力计 */
#define ODOM_MAG_DIFF_WARN_RAD     0.3f    /**< 偏差超过此值增大 R */
#define ODOM_VY_DEADBAND_MPS       0.05f   /**< 直线行驶时抑制微小侧滑漂移 */
#define ODOM_P_MIN                 1e-6f   /**< 协方差下限 */
#define ODOM_P_MAX                 10.0f   /**< 协方差上限，防发散 */

/* -------------------------------------------------------------------------- */
/* 内部状态                                                                   */
/* -------------------------------------------------------------------------- */
static Odom_Config  g_cfg;
static Odom_Data    g_data;
static bool         g_initialized = false;

/* EKF 状态与协方差 */
static float X[3];          /* x, y, yaw_ros */
static float P[3][3];       /* 协方差 (row-major) */

/* -------------------------------------------------------------------------- */
/* 小工具函数                                                                 */
/* -------------------------------------------------------------------------- */

static float Clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float NormalizeRad(float a)
{
    while (a > (float)M_PI)  a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

static float AngleDiffRad(float a, float b)
{
    return NormalizeRad(a - b);
}

static void Mat3Transpose(float a[3][3], float out[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out[j][i] = a[i][j];
        }
    }
}

static void Mat3Mul(float a[3][3], float b[3][3], float out[3][3])
{
    float tmp[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) {
                s += a[i][k] * b[k][j];
            }
            tmp[i][j] = s;
        }
    }
    memcpy(out, tmp, sizeof(tmp));
}

static void ClampCovariance(void)
{
    for (int i = 0; i < 3; i++) {
        if (P[i][i] < ODOM_P_MIN) P[i][i] = ODOM_P_MIN;
        if (P[i][i] > ODOM_P_MAX) P[i][i] = ODOM_P_MAX;
    }
    /* 非对角项简单限幅，保持数值稳定 */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (i == j) continue;
            float bound = sqrtf(fmaxf(P[i][i] * P[j][j], 0.0f));
            P[i][j] = Clampf(P[i][j], -bound, bound);
        }
    }
}

/* -------------------------------------------------------------------------- */
/* EKF 预测: 编码器速度作为控制输入                                            */
/* -------------------------------------------------------------------------- */
static void Odom_EKF_Predict(float vx, float vy, float omega, float dt,
                             float q_xy, float q_yaw)
{
    float psi = X[2];
    float cp = cosf(psi);
    float sp = sinf(psi);

    /* 状态预测 */
    X[0] += dt * (vx * cp - vy * sp);
    X[1] += dt * (vx * sp + vy * cp);
    X[2]  = NormalizeRad(X[2] + dt * omega);

    /* 雅可比 F = d(state_next)/d(state) */
    float F[3][3] = {{1.0f, 0.0f, 0.0f},
                     {0.0f, 1.0f, 0.0f},
                     {0.0f, 0.0f, 1.0f}};
    F[0][2] = -dt * (vx * sp + vy * cp);
    F[1][2] =  dt * (vx * cp - vy * sp);

    /* 控制输入雅可比 G = d(state_next)/d(input velocity) */
    float G[3][3] = {{0.0f, 0.0f, 0.0f},
                     {0.0f, 0.0f, 0.0f},
                     {0.0f, 0.0f, 0.0f}};
    G[0][0] =  dt * cp;
    G[0][1] = -dt * sp;
    G[1][0] =  dt * sp;
    G[1][1] =  dt * cp;
    G[2][2] =  dt;

    /* 动态过程噪声（输入速度空间，经 G 变换到状态空间） */
    float Qv[3][3] = {{0.0f, 0.0f, 0.0f},
                      {0.0f, 0.0f, 0.0f},
                      {0.0f, 0.0f, 0.0f}};
    Qv[0][0] = q_xy;
    Qv[1][1] = q_xy;
    Qv[2][2] = q_yaw;

    float Ft[3][3], FP[3][3], FPFt[3][3];
    Mat3Transpose(F, Ft);
    Mat3Mul(F, P, FP);
    Mat3Mul(FP, Ft, FPFt);

    float Gt[3][3], GQ[3][3], GQGt[3][3];
    Mat3Transpose(G, Gt);
    Mat3Mul(G, Qv, GQ);
    Mat3Mul(GQ, Gt, GQGt);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P[i][j] = FPFt[i][j] + GQGt[i][j];
        }
    }
    ClampCovariance();
}

/* -------------------------------------------------------------------------- */
/* EKF 更新: 标量航向测量                                                      */
/* -------------------------------------------------------------------------- */
static void Odom_EKF_UpdateHeading(float yaw_meas_ros, float r_heading)
{
    float y = AngleDiffRad(yaw_meas_ros, X[2]);

    float s = P[2][2] + r_heading;
    if (s < 1e-9f) s = 1e-9f;

    float k0 = P[0][2] / s;
    float k1 = P[1][2] / s;
    float k2 = P[2][2] / s;

    X[0] += k0 * y;
    X[1] += k1 * y;
    X[2]  = NormalizeRad(X[2] + k2 * y);

    /* P = (I - K H) P  (H = [0 0 1]) */
    float p00 = P[0][0], p01 = P[0][1], p02 = P[0][2];
    float p11 = P[1][1], p12 = P[1][2], p22 = P[2][2];

    P[0][0] = p00 - k0 * p02;
    P[0][1] = p01 - k0 * p12;
    P[0][2] = p02 - k0 * p22;
    P[1][1] = p11 - k1 * p12;
    P[1][2] = p12 - k1 * p22;
    P[2][2] = p22 - k2 * p22;
    P[1][0] = P[0][1];
    P[2][0] = P[0][2];
    P[2][1] = P[1][2];

    ClampCovariance();
}

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

    /* EKF 初始位置为 0, 航向取 AHRS 当前航向并转换为 ROS yaw */
    float heading_deg = AHRS_GetHeading();
    X[0] = 0.0f;
    X[1] = 0.0f;
    X[2] = NormalizeRad(-heading_deg * DEG2RAD);

    memset(P, 0, sizeof(P));
    P[0][0] = 0.1f;
    P[1][1] = 0.1f;
    P[2][2] = 0.1f;

    g_data.pose.heading = heading_deg;
    g_data.diag.r_heading = 0.02f;
    g_data.diag.q_xy = 0.01f;
    g_data.diag.q_yaw = 0.01f;

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

        /* 线速度用已含方向符号的 RPM 换算 */
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

    /* ==== 3. 麦轮正运动学: 轮速 → Vx, Vy, ωz (机器人坐标系) ==== */
    float R  = g_cfg.wheel_radius;
    float L  = g_cfg.wheel_base_x + g_cfg.wheel_base_y;
    float vx_enc, vy_enc, omega_enc;

    if (R > 0.0f && L > 0.0f) {
        float w[4];
        for (int i = 0; i < 4; i++) {
            w[i] = g_data.wheel_mps[i] / R;
        }
        vx_enc   = (R / 4.0f) * ( w[0] + w[1] + w[2] + w[3]);  /* 前 */
        vy_enc   = (R / 4.0f) * (-w[0] + w[1] + w[2] - w[3]);  /* 左 */
        omega_enc = (R / (4.0f * L)) * (-w[0] + w[1] - w[2] + w[3]);
    } else {
        vx_enc = 0.0f;
        vy_enc = 0.0f;
        omega_enc = 0.0f;
    }

    /* 直线行驶时抑制微小侧向漂移 (AI 建议: 麦轮侧滑难检测) */
    if (fabsf(vy_enc) < ODOM_VY_DEADBAND_MPS &&
        fabsf(vx_enc) >= ODOM_VY_DEADBAND_MPS) {
        vy_enc = 0.0f;
    }

    /* ==== 4. 数据质量监督 ==== */
    float speed = sqrtf(vx_enc * vx_enc + vy_enc * vy_enc);
    bool stationary = (speed < ODOM_STATIONARY_SPEED_MPS &&
                       fabsf(omega_enc) < ODOM_STATIONARY_OMEGA_RPS);

    /* 陀螺仪 z 轴 (rad/s) */
    float gyro_z_dps  = g_data.gyro_z;
    float gyro_z_radps = gyro_z_dps * DEG2RAD;

    /* 水平加速度 (m/s^2): 加速度计 ax/ay 单位为 g */
    float accel_horiz = sqrtf(g_data.accel_x * g_data.accel_x +
                              g_data.accel_y * g_data.accel_y) * 9.80665f;
    bool accel_slip = accel_horiz > ODOM_ACCEL_SLIP_MS2;

    /* 编码器转向 vs 陀螺仪转向 */
    float omega_diff = fabsf(omega_enc - gyro_z_radps);
    bool enc_slip = omega_diff > ODOM_GYRO_SLIP_RADPS &&
                    (fabsf(gyro_z_radps) > ODOM_GYRO_SLIP_RADPS ||
                     fabsf(omega_enc) > ODOM_GYRO_SLIP_RADPS);

    /* ==== 5. 动态协方差 ==== */
    float q_xy_base = 0.01f;
    /* 加速度越大/越可疑，编码器位置预测越不可信 */
    float dynamic_q_xy = q_xy_base + accel_horiz * 0.10f;
    dynamic_q_xy = Clampf(dynamic_q_xy, 0.01f, 0.50f);

    float q_yaw_base = 0.01f;
    float dynamic_q_yaw = q_yaw_base + omega_diff * 0.15f;
    dynamic_q_yaw = Clampf(dynamic_q_yaw, 0.01f, 0.30f);

    /* ==== 6. 航向测量：统一使用 AHRS 航向 ====
     * 说明：AHRS 内部已根据 AHRSMAG 开关选择“纯陀螺”或“陀螺+磁力计互补”，
     * 它才是导航/控制实际使用的航向参考。EKF 若再用原始磁力计直接做测量，
     * 在磁力计轴映射/标定不理想时会造成轮式 yaw 偏大。
     * 因此这里始终以 AHRS_GetHeading() 作为 yaw 测量，并把 R 设得很低，
     * 让轮式里程计航向紧密跟随 AHRS，避免编码器 yaw 增益误差导致“转不到一圈”。
     */
    bool heading_valid = true;
    float yaw_meas_ros = NormalizeRad(-AHRS_GetHeading() * DEG2RAD);
    float r_heading = AHRS_IsMagFusionEnabled()
                          ? (stationary ? 0.0005f : 0.002f)
                          : (stationary ? 0.0002f : 0.001f);

    /* ==== 7. EKF 预测 + 更新 ==== */
    Odom_EKF_Predict(vx_enc, vy_enc, omega_enc, dt,
                     dynamic_q_xy, dynamic_q_yaw);

    if (heading_valid) {
        Odom_EKF_UpdateHeading(yaw_meas_ros, r_heading);
    }

    /* ==== 8. ZUPT: 静止时锁死协方差，防止停车坐标乱飘 ==== */
    if (stationary) {
        P[0][0] = fminf(P[0][0], 0.0001f);
        P[1][1] = fminf(P[1][1], 0.0001f);
        P[2][2] = fminf(P[2][2], 0.0001f);
        ClampCovariance();
    }

    /* ==== 9. 输出融合位姿 ==== */
    float heading_deg = NormalizeRad(-X[2]) * RAD2DEG;
    if (heading_deg < 0.0f) heading_deg += 360.0f;

    g_data.pose.x = X[0];
    g_data.pose.y = X[1];
    g_data.pose.heading = heading_deg;
    g_data.pose.vx = stationary ? 0.0f : vx_enc;
    g_data.pose.vy = stationary ? 0.0f : vy_enc;
    g_data.pose.omega = stationary ? 0.0f : omega_enc;

    /* ==== 10. 诊断输出 ==== */
    g_data.diag.flags = 0U;
    if (stationary)     g_data.diag.flags |= ODOM_DIAG_STATIONARY;
    if (heading_valid)  g_data.diag.flags |= ODOM_DIAG_HEADING_VALID;
    if (enc_slip)       g_data.diag.flags |= ODOM_DIAG_ENC_SLIP;
    if (accel_slip)     g_data.diag.flags |= ODOM_DIAG_ACCEL_SLIP;
    if (stationary)     g_data.diag.flags |= ODOM_DIAG_ZUPT_ACTIVE;
    g_data.diag.q_xy = dynamic_q_xy;
    g_data.diag.q_yaw = dynamic_q_yaw;
    g_data.diag.r_heading = r_heading;
    g_data.diag.cov_x = P[0][0];
    g_data.diag.cov_y = P[1][1];
    g_data.diag.cov_yaw = P[2][2];
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
    /* 位置归零，航向保持 */
    X[0] = 0.0f;
    X[1] = 0.0f;
    g_data.pose.x = 0.0f;
    g_data.pose.y = 0.0f;
}

void Odom_SetPose(float x, float y, float heading)
{
    X[0] = x;
    X[1] = y;
    X[2] = NormalizeRad(-heading * DEG2RAD);
    g_data.pose.x = x;
    g_data.pose.y = y;
    g_data.pose.heading = heading;
    /* 人为设置位姿后可小幅重置协方差，便于重新收敛 */
    memset(P, 0, sizeof(P));
    P[0][0] = 0.01f;
    P[1][1] = 0.01f;
    P[2][2] = 0.01f;
}
