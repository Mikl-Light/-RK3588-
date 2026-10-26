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
/* ============ 本文件在做什么（里程计 = 车给自己记账） ============
 * 回答：车怎么算出"我走到哪了"？输出 [x, y, yaw]（北向走了多远、西向走了多远、朝哪）。
 *
 * 本质就一句：每10ms，把"速度×时间"累加到位置上。
 *   X[0] += dt×(vx·cos − vy·sin)   ← 北向累计
 *   X[1] += dt×(vx·sin + vy·cos)   ← 西向累计
 *   X[2] += dt×omega               ← 航向累计
 *
 * 为什么叫"可靠"？因为纯累加会漂（打滑、误差越积越大），
 * 所以上了 EKF（卡尔曼滤波）：让"编码器预测的位置"和"AHRS给的航向"
 * 两个都不太准的数据互相纠错，凑出一个更可信的结果。
 * 你早学过的 ahrs 互补滤波(陀螺98%+磁2%)是它的简化版，EKF只是把
 * "各信多少"从固定比例升级成动态算出来的信任度。 */
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
/* 车的"身材"数字，和 chassis.c 里的一致 */
#define DEFAULT_WHEEL_RADIUS     0.0325f   /**< 32.5mm 半径 (65mm 直径) */
#define DEFAULT_WHEEL_BASE_X     0.075f    /**< 75mm 左右半间距 (150mm/2) */
#define DEFAULT_WHEEL_BASE_Y     0.075f    /**< 75mm 前后半间距 */
#define DEFAULT_ENC_CNT_PER_REV  52.0f     /**< 13PPR × 4 倍频 = 52 */
#define DEFAULT_GEAR_RATIO       30.0f     /**< 30:1 减速比 */

/* -------------------------------------------------------------------------- */
/* 可靠里程计监督层阈值 (现场可微调)                                           */
/* -------------------------------------------------------------------------- */
/* 这些是"各种情况下该判成什么"的门槛值 */
#define ODOM_STATIONARY_SPEED_MPS  0.01f   /**< 线速度低于此值视为静止 (m/s) */
#define ODOM_STATIONARY_OMEGA_RPS  0.01f   /**< 角速度低于此值视为静止 (rad/s) */
#define ODOM_ACCEL_SLIP_MS2        2.5f    /**< 水平加速度超过此值加强位置 Q (m/s^2)，怀疑打滑 */
#define ODOM_GYRO_SLIP_RADPS       0.4f    /**< |编码器转向 - 陀螺转向| 超过此值判定转向打滑 */
#define ODOM_MAG_JUMP_RAD          0.6f    /**< 磁航向与当前 yaw 偏差上限 (rad) — 注：这几个宏目前是死代码 */
#define ODOM_MAG_HIGH_GYRO_RADPS   1.0f    /**< 剧烈旋转时忽略磁力计 — 死代码 */
#define ODOM_MAG_DIFF_WARN_RAD     0.3f    /**< 偏差超过此值增大 R — 死代码 */
#define ODOM_VY_DEADBAND_MPS       0.05f   /**< 直线行驶时抑制微小侧滑漂移（速度低于此值当0） */
#define ODOM_P_MIN                 1e-6f   /**< 协方差下限（协方差="我对这个值的把握有多大"） */
#define ODOM_P_MAX                 10.0f   /**< 协方差上限，防发散 */

/* -------------------------------------------------------------------------- */
/* 内部状态                                                                   */
/* -------------------------------------------------------------------------- */
/* 三个全局变量，存这道题的"题干"和"答案" */
static Odom_Config  g_cfg;      /* 身材配置 */
static Odom_Data    g_data;     /* 输出数据（位姿、传感器读数、诊断） */
static bool         g_initialized = false;   /* 初始化了没 */

/* EKF 状态与协方差
 * X = [x, y, yaw] = 车现在的位置和朝向（答案）
 * P = 协方差矩阵 = "我对这个答案有多大把握"（P 越小越有把握） */
static float X[3];          /* x, y, yaw_ros */
static float P[3][3];       /* 协方差 (row-major) */

/* -------------------------------------------------------------------------- */
/* 小工具函数                                                                 */
/* -------------------------------------------------------------------------- */

/* 【中文名】夹范围：限制 v 在 [lo, hi] 之间 */
static float Clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* 【中文名】角度归一化到弧度 [-π, +π]：和 ahrs.c 里的角度归一化一个道理，只是用弧度 */
static float NormalizeRad(float a)
{
    while (a > (float)M_PI)  a -= 2.0f * (float)M_PI;   /* 太大减一整圈(2π) */
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;   /* 太小加一整圈 */
    return a;
}

/* 【中文名】两个弧度角的差，取最短弧（和 AHRS_AngleDiff 一样，只是单位是弧度） */
static float AngleDiffRad(float a, float b)
{
    return NormalizeRad(a - b);
}

/* 【中文名】矩阵转置：行变列、列变行（矩阵运算的小工具） */
static void Mat3Transpose(float a[3][3], float out[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out[j][i] = a[i][j];   /* 第i行第j列 → 第j行第i列 */
        }
    }
}

/* 【中文名】矩阵乘法：out = a × b（3×3矩阵相乘，先算到临时数组再拷回，避免覆盖出错） */
static void Mat3Mul(float a[3][3], float b[3][3], float out[3][3])
{
    float tmp[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;   /* 累加器：第i行×第j列的结果 */
            for (int k = 0; k < 3; k++) {
                s += a[i][k] * b[k][j];   /* 相乘累加（标准矩阵乘法） */
            }
            tmp[i][j] = s;
        }
    }
    memcpy(out, tmp, sizeof(tmp));   /* 把结果拷回 out */
}

/* 【中文名】协方差限幅：把"把握程度 P"夹在合理范围，防止它跑到无穷大(发散)或无穷小 */
static void ClampCovariance(void)
{
    /* 对角线（自己对自己的把握）：夹在 [P_MIN, P_MAX] 之间 */
    for (int i = 0; i < 3; i++) {
        if (P[i][i] < ODOM_P_MIN) P[i][i] = ODOM_P_MIN;
        if (P[i][i] > ODOM_P_MAX) P[i][i] = ODOM_P_MAX;
    }
    /* 非对角项（两个量之间的关联）：简单限幅，保持数值稳定不爆炸 */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (i == j) continue;   /* 跳过对角线（上面处理过了） */
            float bound = sqrtf(fmaxf(P[i][i] * P[j][j], 0.0f));   /* 用对角项算个安全上限 */
            P[i][j] = Clampf(P[i][j], -bound, bound);
        }
    }
}

/* -------------------------------------------------------------------------- */
/* EKF 预测: 编码器速度作为控制输入                                            */
/* -------------------------------------------------------------------------- */
/* 【中文名】EKF预测（第一步）：用"编码器算出的车速"预测"车下一时刻的位置"。
 * 它做两件事：① 累积位置（速度和时间的积分）② 更新"把握程度 P"（预测后把握下降一点，因为更不确定） */
static void Odom_EKF_Predict(float vx, float vy, float omega, float dt,
                             float q_xy, float q_yaw)
{
    float psi = X[2];     /* 当前航向（弧度），记成 ψ 方便写公式 */
    float cp = cosf(psi); /* cos(ψ) */
    float sp = sinf(psi); /* sin(ψ) */

    /* ① 状态预测：这是里程计的核心！
     * 车的速度(vx向前,vy向左)是"车自己坐标系"的，但账本记的是"世界坐标(北,西)"。
     * 所以要靠 cos/sin 把"车前进的方向"拆成"北向+西向"两个分量。
     *   X[0](北) += 前进的北分量 − 左移的北分量
     *   X[1](西) += 前进的西分量 + 左移的西分量
     *   X[2](航向) += 角速度×时间 */
    X[0] += dt * (vx * cp - vy * sp);
    X[1] += dt * (vx * sp + vy * cp);
    X[2]  = NormalizeRad(X[2] + dt * omega);   /* 航向累加并归一化 */

    /* ② 下面整段是"协方差 P 的预测更新"（标准 EKF 公式），
     * 大意：位置预测得越不确定，P 就该变大（表示"我越没把握"）。
     * 你不用逐行懂矩阵，只要记住：预测完 = 把握下降一点。
     * 下面这些 F、G、Q 都是公式里的矩阵，Mat3Mul 是给它们做乘法。 */
    float F[3][3] = {{1.0f, 0.0f, 0.0f},
                     {0.0f, 1.0f, 0.0f},
                     {0.0f, 0.0f, 1.0f}};   /* 状态转移矩阵（约等于单位阵） */
    F[0][2] = -dt * (vx * sp + vy * cp);    /* 位置对航向的依赖（航向变了，位置投影也变） */
    F[1][2] =  dt * (vx * cp - vy * sp);

    float G[3][3] = {{0.0f, 0.0f, 0.0f},
                     {0.0f, 0.0f, 0.0f},
                     {0.0f, 0.0f, 0.0f}};   /* 输入转移矩阵（速度怎么影响状态） */
    G[0][0] =  dt * cp;
    G[0][1] = -dt * sp;
    G[1][0] =  dt * sp;
    G[1][1] =  dt * cp;
    G[2][2] =  dt;

    /* 过程噪声 Q（表示"车速测量本身有多不可信"） */
    float Qv[3][3] = {{0.0f, 0.0f, 0.0f},
                      {0.0f, 0.0f, 0.0f},
                      {0.0f, 0.0f, 0.0f}};
    Qv[0][0] = q_xy;   /* 位置噪声（q_xy 是动态算出来的，打滑时更大） */
    Qv[1][1] = q_xy;
    Qv[2][2] = q_yaw;  /* 航向噪声 */

    /* 标准 EKF 协方差预测公式：P = F·P·Fᵀ + G·Q·Gᵀ（矩阵形式） */
    float Ft[3][3], FP[3][3], FPFt[3][3];
    Mat3Transpose(F, Ft);
    Mat3Mul(F, P, FP);
    Mat3Mul(FP, Ft, FPFt);   /* FPFt = F·P·Fᵀ */

    float Gt[3][3], GQ[3][3], GQGt[3][3];
    Mat3Transpose(G, Gt);
    Mat3Mul(G, Qv, GQ);
    Mat3Mul(GQ, Gt, GQGt);   /* GQGt = G·Q·Gᵀ */

    /* P = FPFt + GQGt（把两次"把握下降"叠加） */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P[i][j] = FPFt[i][j] + GQGt[i][j];
        }
    }
    ClampCovariance();   /* 限幅，防止发散 */
}

/* -------------------------------------------------------------------------- */
/* EKF 更新: 标量航向测量                                                      */
/* -------------------------------------------------------------------------- */
/* 【中文名】EKF更新（第二步）：拿"AHRS给的权威航向"来纠正预测的位置和航向。
 * 核心：算出"权威航向"和"我预测的航向"差多少，用一个"信任度 k"把账本往回纠。 */
static void Odom_EKF_UpdateHeading(float yaw_meas_ros, float r_heading)
{
    /* y = 权威航向 − 我预测的航向 = 差多少（取最短弧） */
    float y = AngleDiffRad(yaw_meas_ros, X[2]);

    /* s = 我对航向的把握(P) + 测量本身的噪声(r)。
     * 相当于"两边的误差加一起"，作为分母（后面拿它算信任度） */
    float s = P[2][2] + r_heading;
    if (s < 1e-9f) s = 1e-9f;   /* 防止除以0 */

    /* 卡尔曼增益 k：三个数分别对应 x、y、航向的"纠正力度"。
     * 含义：我越有把握(P小)，k越小（少纠）；越没把握(P大)，k越大（多纠） */
    float k0 = P[0][2] / s;
    float k1 = P[1][2] / s;
    float k2 = P[2][2] / s;

    /* 用增益把"差距 y"按比例加到账本上，做纠偏 */
    X[0] += k0 * y;
    X[1] += k1 * y;
    X[2]  = NormalizeRad(X[2] + k2 * y);   /* 航向也纠，并归一化 */

    /* P 的更新：纠正完之后，把握变高了（P 变小）。这是标准 EKF 的 P=(I-K·H)·P。
     * 消掉 H=[0 0 1] 后的手写展开，比调矩阵函数更快、更省算力 */
    float p00 = P[0][0], p01 = P[0][1], p02 = P[0][2];
    float p11 = P[1][1], p12 = P[1][2], p22 = P[2][2];

    P[0][0] = p00 - k0 * p02;
    P[0][1] = p01 - k0 * p12;
    P[0][2] = p02 - k0 * p22;
    P[1][1] = p11 - k1 * p12;
    P[1][2] = p12 - k1 * p22;
    P[2][2] = p22 - k2 * p22;
    P[1][0] = P[0][1];   /* 对称，补全下三角（协方差矩阵是对称的） */
    P[2][0] = P[0][2];
    P[2][1] = P[1][2];

    ClampCovariance();
}

/* -------------------------------------------------------------------------- */
/* 初始化                                                                     */
/* -------------------------------------------------------------------------- */
/* 【中文名】拿默认身材配置（轮径、轴距、编码器、减速比） */
void Odom_GetDefaultConfig(Odom_Config *cfg)
{
    if (cfg == NULL) return;
    cfg->wheel_radius        = DEFAULT_WHEEL_RADIUS;
    cfg->wheel_base_x        = DEFAULT_WHEEL_BASE_X;
    cfg->wheel_base_y        = DEFAULT_WHEEL_BASE_Y;
    cfg->encoder_cnt_per_rev = DEFAULT_ENC_CNT_PER_REV;
    cfg->gear_ratio          = DEFAULT_GEAR_RATIO;
}

/* 【中文名】初始化里程计：位置清零、航向取当前AHRS航向、协方差给个初始值 */
void Odom_Init(const Odom_Config *cfg)
{
    memset(&g_data, 0, sizeof(g_data));   /* 输出数据结构体清零 */

    if (cfg != NULL) {
        g_cfg = *cfg;   /* 用传进来的配置 */
    } else {
        Odom_GetDefaultConfig(&g_cfg);   /* 没传就用默认 */
    }

    /* EKF 初始位置为 0, 航向取 AHRS 当前航向。
     *
     * ⚠ 2026-10-11 修正（此前这里是 `-heading_deg`，导致 Y 轴镜像）：
     *   AHRS_GetHeading() 返回的 heading 本身就是"逆时针为正"的角
     *   （ahrs.c 内部按 gyro_yaw = yaw + gz*dt 积分），
     *   而 EKF 的状态 X[2] 在投影里被当作 **标准 ROS yaw ψ** 使用：
     *       X[0] += dt*(vx*cos ψ - vy*sin ψ)
     *       X[1] += dt*(vx*sin ψ + vy*cos ψ)
     *   所以 X[2] 必须**直接等于 +heading**，不能再取负。
     *   取负会让投影变成"用 -ψ"，等价于世界 Y 轴镜像：
     *       实车 HDG=+47° 前进, 实测世界方向 = -49.1°（应为 +47°）。
     *   该 bug 在 heading≈0 时不可见（-0==+0），必须非零初朝向才能测出。
     */
    float heading_deg = AHRS_GetHeading();   /* 读当前航向 */
    X[0] = 0.0f;   /* 初始位置 x=0（北向起点） */
    X[1] = 0.0f;   /* 初始位置 y=0（西向起点） */
    /* 初始航向 = 当前 AHRS 航向（弧度）。
     * 注意：X[2] 存的是"ROS yaw（逆时针正）"，和 AHRS 的 heading 同号，
     * 所以直接 = +heading，不能取负（取负会导致世界 Y 轴镜像，这是 2026-10-11 修复的 bug） */
    X[2] = NormalizeRad(heading_deg * DEG2RAD);

    /* 协方差初始值 0.1：表示"刚开机，对位置的把握一般般"，之后 EKF 会动态调整 */
    memset(P, 0, sizeof(P));
    P[0][0] = 0.1f;
    P[1][1] = 0.1f;
    P[2][2] = 0.1f;

    /* 对外输出的航向，归一到 0~360 */
    g_data.pose.heading = heading_deg;
    if (g_data.pose.heading < 0.0f) g_data.pose.heading += 360.0f;
    if (g_data.pose.heading >= 360.0f) g_data.pose.heading -= 360.0f;
    /* 初始的诊断参数 */
    g_data.diag.r_heading = 0.02f;
    g_data.diag.q_xy = 0.01f;
    g_data.diag.q_yaw = 0.01f;

    g_initialized = true;   /* 标记已初始化 */
}

/* -------------------------------------------------------------------------- */
/* 主更新                                                                     */
/* -------------------------------------------------------------------------- */
/* 【中文名】里程计主更新（每10ms调一次，本文件总入口）：读传感器→算速度→EKF预测+更新→输出位姿 */
void Odom_Update(float dt)
{
    if (!g_initialized || dt <= 0.0f) return;   /* 没初始化或时间非法就退出 */

    /* ==== 1. 读取编码器原始数据（四个轮子） ==== */
    for (int i = 0; i < 4; i++) {
        g_data.enc_raw[i]   = Motor_GetEncoderCount((uint8_t)i);  /* 原始计数 */
        g_data.enc_delta[i] = Motor_GetEncoderDelta((uint8_t)i);  /* 本周期增量 */
        g_data.wheel_rpm[i] = Motor_GetSpeed((uint8_t)i);         /* 轮子转速(RPM) */

        /* 把轮子转速(RPM)换成线速度(米/秒)：RPM×(2π/60) 是弧度/秒，再×轮半径=米/秒 */
        g_data.wheel_mps[i] = g_data.wheel_rpm[i] * (2.0f * M_PI / 60.0f)
                              * g_cfg.wheel_radius;
    }

    /* ==== 2. 读取 IMU（加速度/陀螺/磁力计） ==== */
    MPU9250_Data imu;
    if (MPU9250_ReadAll(&imu) == 0) {
        g_data.accel_x = imu.accel_x;   /* 加速度计 x（g） */
        g_data.accel_y = imu.accel_y;
        g_data.accel_z = imu.accel_z;
        g_data.gyro_x  = imu.gyro_x;    /* 陀螺 x（°/s） */
        g_data.gyro_y  = imu.gyro_y;
        g_data.gyro_z  = imu.gyro_z;
        g_data.mag_x   = imu.mag_x;     /* 磁力计 x（µT） */
        g_data.mag_y   = imu.mag_y;
        g_data.mag_z   = imu.mag_z;
        g_data.imu_temp = imu.temperature;
        g_data.accel_raw[0] = imu.accel_raw[0];   /* 原始LSB，也存一份 */
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

    /* ==== 3. 麦轮正运动学：轮速 → Vx,Vy,ωz（机器人坐标系） ====
     * 这是逆运动学的"反过程"：由四个轮子实际转速，反推车在前进/横移/转多快 */
    float R  = g_cfg.wheel_radius;
    float L  = g_cfg.wheel_base_x + g_cfg.wheel_base_y;
    float vx_enc, vy_enc, omega_enc;

    if (R > 0.0f && L > 0.0f) {
        float w[4];
        for (int i = 0; i < 4; i++) {
            w[i] = g_data.wheel_mps[i] / R;   /* 轮子线速度 → 轮子角速度(rad/s) */
        }
        vx_enc   = (R / 4.0f) * ( w[0] + w[1] + w[2] + w[3]);   /* 前进=四轮求和 */
        vy_enc   = (R / 4.0f) * (-w[0] + w[1] + w[2] - w[3]);   /* 横移=对角组合 */
        omega_enc = (R / (4.0f * L)) * (-w[0] + w[1] - w[2] + w[3]);  /* 旋转=左右差 */
    } else {
        vx_enc = 0.0f;
        vy_enc = 0.0f;
        omega_enc = 0.0f;
    }

    /* 直线行驶时抑制微小侧向漂移：横移速度很小(<0.05)但前进速度不小 → 当横移=0。
     * 因为麦轮直行时编码器可能有一点侧向噪声，直接当0避免世界坐标乱漂 */
    if (fabsf(vy_enc) < ODOM_VY_DEADBAND_MPS &&
        fabsf(vx_enc) >= ODOM_VY_DEADBAND_MPS) {
        vy_enc = 0.0f;
    }

    /* ==== 4. 数据质量监督 ====
     * 判断车现在是什么状态（静止？打滑？），后面据此调整信任度 */
    float speed = sqrtf(vx_enc * vx_enc + vy_enc * vy_enc);   /* 合速度大小 */
    bool stationary = (speed < ODOM_STATIONARY_SPEED_MPS &&      /* 又慢又几乎不转 = 静止 */
                       fabsf(omega_enc) < ODOM_STATIONARY_OMEGA_RPS);

    float gyro_z_dps  = g_data.gyro_z;              /* 陀螺 z（°/s） */
    float gyro_z_radps = gyro_z_dps * DEG2RAD;      /* 转成弧度/秒 */

    /* 水平加速度(m/s²)：加速度计读数单位是g，×9.8 换成米/秒² */
    float accel_horiz = sqrtf(g_data.accel_x * g_data.accel_x +
                              g_data.accel_y * g_data.accel_y) * 9.80665f;
    bool accel_slip = accel_horiz > ODOM_ACCEL_SLIP_MS2;   /* 加速度太猛 → 怀疑打滑 */

    /* 编码器算的"转向" vs 陀螺测的"转向"：差太多 → 怀疑轮子打滑 */
    float omega_diff = fabsf(omega_enc - gyro_z_radps);
    bool enc_slip = omega_diff > ODOM_GYRO_SLIP_RADPS &&
                    (fabsf(gyro_z_radps) > ODOM_GYRO_SLIP_RADPS ||
                     fabsf(omega_enc) > ODOM_GYRO_SLIP_RADPS);

    /* ==== 5. 动态协方差 ====
     * 核心：运动越"可疑"（加速度大/打滑），就越不信编码器预测(P加大按)。
     * 这是"动态"的含义——信任度不是写死的，是实时算出来的 */
    float q_xy_base = 0.01f;
    float dynamic_q_xy = q_xy_base + accel_horiz * 0.10f;   /* 加速度越大，位置预测越不可信 */
    dynamic_q_xy = Clampf(dynamic_q_xy, 0.01f, 0.50f);      /* 夹在合理范围 */

    float q_yaw_base = 0.01f;
    float dynamic_q_yaw = q_yaw_base + omega_diff * 0.15f;  /* 转向矛盾越大，航向预测越不可信 */
    dynamic_q_yaw = Clampf(dynamic_q_yaw, 0.01f, 0.30f);

    /* ==== 6. 航向测量：统一用 AHRS 航向 ====
     * 不管磁融合开没开，都用 AHRS_GetHeading() 当"权威航向"去纠正 EKF。
     * 这样 EKF 的航向会紧紧跟着 AHRS，不会被原始磁力计带偏 */
    bool heading_valid = true;
    float yaw_meas_ros = NormalizeRad(AHRS_GetHeading() * DEG2RAD);  /* 权威航向(弧度) */
    /* r_heading = 对这个测量的"信任程度"，越小越信它。
     * 开了磁融合→更信；静止时→更信（因为航向最干净） */
    float r_heading = AHRS_IsMagFusionEnabled()
                          ? (stationary ? 0.0005f : 0.002f)
                          : (stationary ? 0.0002f : 0.001f);

    /* ==== 7. EKF 预测 + 更新（核心两步） ==== */
    Odom_EKF_Predict(vx_enc, vy_enc, omega_enc, dt,
                     dynamic_q_xy, dynamic_q_yaw);   /* 先预测位置 */

    if (heading_valid) {
        Odom_EKF_UpdateHeading(yaw_meas_ros, r_heading);   /* 再用航向纠偏 */
    }

    /* ==== 8. ZUPT：静止时锁死协方差，防止停车坐标乱飘 ==== */
    if (stationary) {
        /* 车停了就把它"锁死"：协方差压到极小，表示"我很确定它没动" */
        P[0][0] = fminf(P[0][0], 0.0001f);
        P[1][1] = fminf(P[1][1], 0.0001f);
        P[2][2] = fminf(P[2][2], 0.0001f);
        ClampCovariance();
    }

    /* ==== 9. 输出融合位姿（把 EKF 算的结果打包出去） ==== */
    float heading_deg = NormalizeRad(X[2]) * RAD2DEG;   /* 航向弧度→度 */
    if (heading_deg < 0.0f) heading_deg += 360.0f;

    g_data.pose.x = X[0];            /* 填北向坐标 */
    g_data.pose.y = X[1];            /* 填西向坐标 */
    g_data.pose.heading = heading_deg;  /* 填航向(度) */
    g_data.pose.vx = stationary ? 0.0f : vx_enc;     /* 静止时速度归零 */
    g_data.pose.vy = stationary ? 0.0f : vy_enc;
    g_data.pose.omega = stationary ? 0.0f : omega_enc;

    /* ==== 10. 诊断输出（供上位机看车现在的健康状态） ==== */
    g_data.diag.flags = 0U;   /* 标志位清零，然后逐个置位 */
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
/* 【中文名】拿当前位姿（只读，返回指针） */
const Odom_Pose *Odom_GetPose(void)
{
    return &g_data.pose;
}

/* 【中文名】拿完整数据（位姿+传感器+诊断，只读） */
const Odom_Data *Odom_GetData(void)
{
    return &g_data;
}

/* 【中文名】重置里程计：位置(x,y)归零，但航向保持（不改朝向） */
void Odom_Reset(void)
{
    X[0] = 0.0f;
    X[1] = 0.0f;
    g_data.pose.x = 0.0f;
    g_data.pose.y = 0.0f;
    /* 注意：故意不改 X[2]（航向保持），因为重置位置不等于重新定朝向 */
}

/* 【中文名】手动设置位姿：直接指定车在哪、朝哪（比如 SLAM 重定位时用） */
void Odom_SetPose(float x, float y, float heading)
{
    X[0] = x;
    X[1] = y;
    X[2] = NormalizeRad(heading * DEG2RAD);   /* 航向同号，不取负（同 Odom_Init 说明） */
    g_data.pose.x = x;
    g_data.pose.y = y;
    g_data.pose.heading = heading;
    /* 人为设了位姿后，协方差重置小一点，让 EKF 重新收敛 */
    memset(P, 0, sizeof(P));
    P[0][0] = 0.01f;
    P[1][1] = 0.01f;
    P[2][2] = 0.01f;
}
