/**
  ******************************************************************************
  * @file    ahrs.c
  * @brief   航向姿态参考系统实现
  *          互补滤波: 陀螺仪(短期准) + 磁力计(长期准,绝对方向)
  *          滤波系数 0.98 表示: 98% 信任陀螺仪积分, 2% 信任磁力计修正
  ******************************************************************************
  */
/* ============ 本文件在做什么 ============
 * 回答一个问题：车怎么知道自己是朝哪的？
 * 答案可能反直觉：它其实"不知道"，是自己"记"出来的。
 *
 * 步骤只有两步：
 *   1. 陀螺仪测的是"转得多快"（°/s），不是"朝哪"
 *   2. 每10ms把"这一圈转了多少 = 转速×时间"累加到航向上
 *
 * 但陀螺仪有个零偏（车不动，它也报个很小的非零数），这个零偏会被积分放大。
 * 所以本文件还有两个"纠偏"手段：
 *   - 磁力计（绝对方向，不漂，但受电机/手机干扰，默认关）
 *   - ZARU（车停下来时，重新量一次零偏）
 */
#include "ahrs.h"
#include "debug.h"
#include "mpu9250.h"
#include <math.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/* 配置                                                                       */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* 互补滤波时间常数 τ (v3.4.6-MAGPRED 新增)                                   */
/*                                                                            */
/* 旧写法：固定 AHRS_ALPHA=0.98，等效 τ = dt/(1-α) = 50·dt                     */
/*   → 主循环 dt 一抖动，τ 就跟着变，滤波特性（以及下面要补的滞后量）不再确定   */
/* 新写法：反过来用 τ 求 α，保证 τ 恒定                                       */
/*    α_eff = τ/(τ+dt)   ⇒   修正增益 k = 1-α_eff = dt/(τ+dt)                 */
/*    dt=10ms 时 α_eff=0.9804，与旧的 0.98 几乎一致，行为不变                  */
/* -------------------------------------------------------------------------- */
#define AHRS_TAU         0.5f         /**< 互补滤波时间常数 (s)；匀速下滞后量 = ω·τ */
#define AHRS_ALPHA       0.98f        /**< 旧固定系数，仅作标称参考，实际由 AHRS_TAU 反算 */

/* 前馈补偿系数 (s) —— 注意这**不等于** AHRS_TAU，是实测标定的 (v3.4.7-MAGPRED2)
 * v3.4.6 直接用 AHRS_TAU=0.5 补，实测顺时针补了 15.02°，而真实滞后只有 10.17°，
 * → 有效 τ 相当于 0.73s，超发 48%。反解应补 10.17° 需要 0.34s，
 * 取 0.35 作折中（多出来的滞后可能来自 odometry.c 的 EKF 平滑，未单独拆分）。 */
#define AHRS_TAU_COMP    0.35f
#define MAG_DECLINATION  0.0f         /**< 磁偏角 (度)。改绝对磁航向后未来可设邯郸约 -5.7°（先留0，硬铁校准未持久化前意义不大） */

/* -------------------------------------------------------------------------- */
/* ZARU 零速零偏再估计 (2026-08-18)                                            */
/*   原理: 车静止时 (编码器全零) 陀螺读数≈当前真实零偏; 慢速低通把            */
/*         gyro_z_offset 拉向实测值 → 消除开机校准后零偏漂移 (HDG 匀速漂)     */
/*   门控: 静止≥1s 且 |残余角速度|≤3°/s, 防止把真实旋转误当零偏               */
/* -------------------------------------------------------------------------- */
#define ZARU_SETTLE_MS        1000u   /**< 静止持续多久后开始校正 (ms)，即1秒 */
#define ZARU_TAU_S            2.0f    /**< 零偏校正时间常数 (s)，越小收敛越快 */
#define ZARU_MAX_RESIDUAL_DPS 8.0f    /**< 仅当 |残余角速度|≤此值才校正 (°/s)，注释说的3°旧了，代码是8 */
#define ZARU_MAX_OFFSET_DPS   10.0f   /**< 零偏限幅 (°/s)，防异常值拉飞 */

/* -------------------------------------------------------------------------- */
/* 航向计算直接用 libm 的 atan2f (CMake 已链接 -lm, F407 硬浮点)              */
/* 注: 旧版自写 FastAtan 在 |x|>1 且 x<0 时符号处理错误, 误差最大 ~85°,      */
/*     导致磁航向在部分象限严重偏置 — 已废弃 (2026-08-10 修复)                   */
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* 磁力计航向 (考虑 IMU 安装旋转)                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief IMU 水平面内旋转 90° (xy 交换) 后的磁航向计算
 * @note  2026-08-12 硬件变更: IMU 绕 z 转了 90°, 磁力计 x/y 分量需换轴
 *        顺时针 90° (IMU x→车体y): 车体系磁分量 (北,东) = (my, -mx)
 *        逆时针 90° (IMU x→车体-y): 车体系磁分量 (北,东) = (-my, mx)
 *        若换轴后航向差 180° 或 90°, 调整本函数符号
 */
/* 【中文名】用磁力计的两个水平分量算出"磁航向"（车头相对磁北的角度0~360） */
static float MagHeadingDeg(float mx, float my)
{
#if 1   /* 1=顺时针90°: (北,东)=(my,-mx); 0=逆时针90°: (北,东)=(-my,mx) */
    /* 因为 IMU 装的时候绕 z 轴转了 90°，磁力计的 x 轴和 y 轴对调了，
     * 所以要交换一下：把 my 当"北分量"，mx 当"东分量"
     *
     * 2026-10-20 修正 (v3.4.4-MAGSIGN)：east 由 -mx 改为 +mx。
     *   实测证据（车左转/逆时针，wz=+0.2 持续10s）：
     *     陀螺积分 +143°、rf2o 激光 +137.5°、本函数航向 -72.7° → 方向全反。
     *   推演：deg = atan2(east, north)，
     *     原式 atan2(-mx, my)  -> 航向 = α - φ - 90°，随车体转角 φ 增大而"减小"；
     *     现式 atan2(+mx, my)  -> 航向 = 90° - α + φ，随 φ 增大而"增大"，与陀螺同向。
     *   注意：#else 分支(atan2(mx,-my) = α - φ + 90°)同样随 φ 增大而减小，
     *     即两个分支的方向都是反的 —— 所以这不是"装反了换错分支"。 */
    float north = my;
    float east  = mx;
#else
    float north = -my;
    float east  = mx;
#endif
    /* atan2(东,北) 算出"朝哪个方向"的角度，×57.29578 把弧度转成"度" */
    float deg = atan2f(east, north) * 57.29578f;
    if (deg < 0.0f) deg += 360.0f;   /* atan2 返回 -180~180，负数+360 变成 0~360 */
    return deg;
}


/* 供可靠里程计 EKF 直接调用：由磁力计水平分量计算磁航向 (度)
 * 与 AHRS 互补滤波使用的磁航向保持一致，包含磁偏角修正 */
/* 【中文名】带磁偏角修正的磁航向（给EKF用），就是 MagHeadingDeg 结果再 + 磁偏角 */
float AHRS_ComputeMagHeadingDeg(float mx, float my)
{
    float deg = MagHeadingDeg(mx, my) + MAG_DECLINATION;  /* 加磁偏角（本项目=0） */
    if (deg < 0.0f) deg += 360.0f;      /* 归一到 0~360 */
    if (deg >= 360.0f) deg -= 360.0f;
    return deg;
}

/* -------------------------------------------------------------------------- */
/* 内部变量                                                                   */
/* -------------------------------------------------------------------------- */
static float yaw_deg = 0.0f;          /**< 当前融合航向角（本文件的核心结果） */
static float yaw_rate_dps = 0.0f;     /**< 最近一周期「去零偏后」的陀螺 z 角速度 (°/s)，供前馈补偿用 */
static float mag_gyro_diff = 0.0f;    /**< 最近一周期的 mag_yaw − gyro_yaw (°)，判断"是否真的落后"用 */
static float gyro_z_offset = 0.0f;    /**< 陀螺仪零偏（车不动时它报的那个假速度） */
static bool  mag_fusion_enabled = true;  /**< 磁力计融合开关 (默认开: 绝对磁航向做锚, 不漂) */

/* ZARU 零速零偏再估计状态 (2026-08-18) */
static bool     stationary = false;          /**< 当前是否静止 (由 main 循环按编码器判定) */
static uint32_t stationary_since_ms = 0;      /**< 进入静止状态的时刻 (ms)，用于算"静了多久" */

/* I2C 故障容错 (电机运行时 I2C 易受 EMI 干扰, 不可阻塞主循环) */
static uint32_t i2c_fail_cnt = 0;     /**< 连续 I2C 失败次数 */
#define I2C_FAIL_THRESHOLD  5         /**< 连续失败超过此次数后降级: 纯陀螺仪积分 */
#define I2C_RETRY_INTERVAL  10        /**< 降级时每 N 个周期尝试恢复 I2C 通讯 */

/* -------------------------------------------------------------------------- */
/* 初始化                                                                     */
/* -------------------------------------------------------------------------- */
/* 【中文名】初始化航向：航向归零、加载陀螺零偏、决定初始航向从哪来 */
void AHRS_Init(void)
{
    yaw_deg = 0.0f;           /* 航向从 0 开始（0=开机朝向=北），下面若磁力计可用会覆盖成绝对磁航向 */
    gyro_z_offset = 0.0f;     /* 零偏先归零 */

    /* 磁力计融合默认开启：用绝对磁航向做长期锚，抑制陀螺积分漂移。
     * 电机干扰实测 ~4% 磁场(~3.6°航向)可忽略。野值(手机/铁磁体)剔除
     * 统一挪到上位机 odom_fusion 用激光变化量当裁判，固件里不做门控。 */
    mag_fusion_enabled = true;

    /* 加载 MPU9250 里存好的校准数据（就是开机时 CalibrateGyro 采出来的陀螺零偏） */
    const MPU9250_Calib *c = MPU9250_GetCalibration();
    if (c != NULL) {
        gyro_z_offset = c->gyro_offset_z;   /* 把校准好的零偏拿出来用 */
        DBG_DEBUG(TAG_AHRS, "Loaded gyro Z offset: %.3f dps", gyro_z_offset);
    } else {
        DBG_WARN(TAG_AHRS, "No MPU9250 calibration data, gyro offset = 0");
    }

    if (mag_fusion_enabled) {
        /* 首次用磁力计初始化航向：给一个绝对磁航向基准（之后 yaw 一直在磁北坐标系里） */
        float mx, my, mz;
        if (MPU9250_ReadMag(&mx, &my, &mz) == 0) {
            float mag_deg = MagHeadingDeg(mx, my);
            yaw_deg = mag_deg;
            DBG_INFO(TAG_AHRS, "Initial heading from magnetometer: %.1f°", yaw_deg);
        } else {
            DBG_WARN(TAG_AHRS, "Magnetometer read failed, initial heading = 0°");
        }
    } else {
        DBG_INFO(TAG_AHRS, "Mag fusion OFF: yaw starts at 0° (gyro only)");
    }
}

/* -------------------------------------------------------------------------- */
/* 更新航向 (互补滤波) — 带 I2C 故障容错                                      */
/* -------------------------------------------------------------------------- */
/* 【中文名】更新航向（本文件核心，每10ms调一次）：读陀螺→减零偏→积分→(可选磁融合)→归一化→ZARU纠偏 */
void AHRS_Update(float dt)
{
    if (dt <= 0.0f) return;   /* 时间间隔非法就退出 */

    /* ====== 1. 读陀螺仪 Z 轴角速度 (快速通道, 仅 6 字节 I2C) ====== */
    float gx, gy, gz;
    if (MPU9250_ReadGyro(&gx, &gy, &gz) != 0) {
        /* 陀螺仪读取失败 → 保持上次航向、跳过本周期。绝不能阻塞主循环 */
        i2c_fail_cnt++;   /* 失败计数+1 */
        /* 每1秒才打印一次警告（避免疯狂刷屏） */
        static uint32_t last_warn_ms = 0;
        uint32_t now_ms = HAL_GetTick();
        if (now_ms - last_warn_ms > 1000U) {
            last_warn_ms = now_ms;
            DBG_WARN(TAG_AHRS, "Gyro read FAILED (%lu), yaw frozen!",
                     (unsigned long)i2c_fail_cnt);
        }
        yaw_rate_dps = 0.0f;   /* 本周期无有效角速度 → 出口补偿归零，避免拿旧值一直往前推 */
        return;   /* 这一周期不更新航向 */
    }
    float gz_raw = gz;    /* 原始值（含零偏），先存一份，ZARU 后面要用 */
    gz -= gyro_z_offset;  /* 减去零偏！这样车静止时 gz 就≈0了 */
    yaw_rate_dps = gz;    /* 存一份去零偏后的角速度，AHRS_GetHeading 的前馈补偿要用 */

    i2c_fail_cnt = 0;   /* 读成功了，失败计数清0 */

    /* ====== 2. 陀螺仪积分预测航向 ======
     * 这一行的本质：新航向 = 旧航向 + 这一圈转了多少
     * 其中"这一圈转了多少" = 角速度(gz, °/s) × 时间(dt, s) */
    float gyro_yaw = yaw_deg + gz * dt;
    /* 归一化到 0~360：角度是一圈循环的，超出360就减一圈，小于0就加一圈 */
    while (gyro_yaw < 0.0f)    gyro_yaw += 360.0f;
    while (gyro_yaw >= 360.0f) gyro_yaw -= 360.0f;

    /* ====== 3. 读磁力计得到绝对航向 (失败时沿用陀螺仪预测值) ====== */
    float mx, my, mz;
    float mag_yaw = gyro_yaw;  /* 默认"磁航向"先用陀螺预测值兜底 */
    if (mag_fusion_enabled) {   /* 默认开启：绝对磁航向做长期锚 */
        if (MPU9250_ReadMag(&mx, &my, &mz) == 0) {
            float raw_mag = MagHeadingDeg(mx, my);   /* 磁力计算出绝对航向 */
            raw_mag += MAG_DECLINATION;              /* 加磁偏角（本项目=0） */
            /* 归一到 0~360 */
            if (raw_mag < 0.0f)    raw_mag += 360.0f;
            if (raw_mag >= 360.0f) raw_mag -= 360.0f;

            /* 直接采纳磁力计，不做任何门控。
             * 野值(手机/铁磁体)剔除统一挪到上位机 odom_fusion，用激光变化量当裁判——
             * 固件里没有独立于磁场的参照，任何门控都会撞上"磁绝对 vs 陀螺相对"的 offset。 */
            mag_yaw = raw_mag;
        }
        /* 读取失败 → mag_yaw 保持 = gyro_yaw → 本周期等于纯陀螺 */
    }
    /* 融合关闭 → 纯陀螺仪积分（磁力计被电机干扰时更稳） */

    /* ====== 4. 互补滤波融合 ======
     * 核心思想：陀螺短期准但会漂，磁力计长期准但有噪声。
     * 用"98%信陀螺 + 2%信磁力计"的方式，各取所长。 */
    /* 先算两个航向的差，处理跨 0°/360°（350 vs 10 之类） */
    float diff = mag_yaw - gyro_yaw;
    if (diff > 180.0f)       diff -= 360.0f;
    else if (diff < -180.0f) diff += 360.0f;
    mag_gyro_diff = diff;   /* 存起来供出口判断"真落后 / 磁自身超前" (v3.4.7) */

    /* 互补滤波：结果 = 陀螺预测 + 2% × (磁力计与陀螺的差)。
     * 陀螺短期准(积分量的主要贡献者)，磁力计长期准(绝对、不漂)。
     * 默认磁融合开启时，这 2% 的磁修正会持续把航向往绝对磁北方向锚定，
     * 从而抑制陀螺零偏积分带来的长期漂移；磁力计读到野值时 mag_yaw==gyro_yaw，
     * diff=0，自动退化为纯陀螺，不破坏本周期。 */
    /* 用 τ 反算 α：保证时间常数恒为 AHRS_TAU，不随主循环 dt 抖动而变 (v3.4.6) */
    float alpha_eff = AHRS_TAU / (AHRS_TAU + dt);
    yaw_deg = gyro_yaw + (1.0f - alpha_eff) * diff;

    /* 归一化 */
    while (yaw_deg < 0.0f)    yaw_deg += 360.0f;
    while (yaw_deg >= 360.0f) yaw_deg -= 360.0f;

    /* ====== 5. ZARU: 静止时慢速重估陀螺零偏 ======
     * 为什么需要？开机只校准一次零偏，但零偏会随温度慢慢漂。
     * 车每次真正停下来，就趁机重新量一次零偏，把漂移纠回来。
     * 关键前提（物理事实）：车静止时，真实转速=0，所以陀螺报的数全是零偏 */
    if (stationary) {
        uint32_t now_ms = HAL_GetTick();
        if (stationary_since_ms == 0) {
            stationary_since_ms = now_ms;   /* 记录"开始静止"的时刻 */
        }
        /* 三个门槛必须同时满足才校正：
         * ① 静止持续 ≥ 1秒（刚停时车还在晃）
         * ② 残余角速度 ≤ 8°/s（兜底：确定真的没在转）
         * （注：①②都在"车确实停住"上卡，③其实用的是陀螺自己） */
        if ((now_ms - stationary_since_ms) >= ZARU_SETTLE_MS &&
            gz <= ZARU_MAX_RESIDUAL_DPS && gz >= -ZARU_MAX_RESIDUAL_DPS) {
            /* 慢速低通：每次只把零偏朝"实测值"挪一小步，不直接赋值。
             * k = dt / 2秒 = 0.005，即每周期只补 0.5% 的差距 */
            float k = dt / ZARU_TAU_S;
            if (k > 0.1f) k = 0.1f;   /* 若某次 dt 异常大，限制单次最多改10% */
            gyro_z_offset += (gz_raw - gyro_z_offset) * k;

            /* 限幅：零偏若超过 ±10°/s 就夹住，防止异常值把零偏拉飞 */
            if (gyro_z_offset >  ZARU_MAX_OFFSET_DPS) gyro_z_offset =  ZARU_MAX_OFFSET_DPS;
            if (gyro_z_offset < -ZARU_MAX_OFFSET_DPS) gyro_z_offset = -ZARU_MAX_OFFSET_DPS;
        }
    } else {
        stationary_since_ms = 0;   /* 车动了，重置静止计时 */
    }
}

/* -------------------------------------------------------------------------- */
/* 获取当前航向                                                               */
/* -------------------------------------------------------------------------- */
/* 【中文名】取航向角：返回当前融合出来的航向（0~360度），别人都从这个函数拿航向 */
float AHRS_GetHeading(void)
{
    /* ====== 前馈预测补偿 (v3.4.6 提出，v3.4.7 加符号判据) ======
     * 【问题】第 4 步的互补滤波本质是一阶低通，对匀速转动有恒定相位滞后 = ω·τ。
     *         τ=0.5s、ω=20.5°/s 时滞后 10.25°，实测顺时针 10.17°，完全吻合。
     *
     * 【办法】一阶低通 τ·dy/dt + y = u 的逆模型恰好是 u = y + τ·dy/dt，
     *         而 dy/dt 在匀速时就是角速度 ω（陀螺 gz 直接可测）。
     *         所以把 τ·ω 加回输出，就把这段滞后补掉了。
     *
     * 【为什么补在出口、不改 yaw_deg】
     *   第 4 步是 yaw_deg 自己的递推式，若补偿塞进去会被下一周期再积一次，
     *   形成正反馈（越补越偏）。补在出口只影响对外输出，内部状态干净。
     *
     * 【v3.4.7 新增：符号判据 —— 只在"真的落后"时才补】
     *   一阶低通的滞后是**左右对称**的（不管往哪转都落后）。
     *   但 v3.4.6 全域硬补后，逆时针反而从 6.3° 恶化到 16.2°
     *   —— 因为逆时针那 6.3° 根本不是滞后，而是磁航向自己跑得比陀螺快。
     *   判据（纯几何推理，不含任何经验参数）：
     *     滞后 ⇒ yaw 追不上 mag ⇒ mag 领先 yaw
     *       ω>0（逆时针）：mag 往正走、yaw 落后 ⇒ yaw<mag ⇒ diff=mag−yaw>0 ⇒ diff 与 ω 同号
     *       ω<0（顺时针）：mag 往负走、yaw 落后 ⇒ yaw>mag ⇒ diff=mag−yaw<0 ⇒ diff 与 ω 同号
     *     即 **diff × ω > 0 ⇔ 真落后**；反号说明 yaw 超前于 mag（磁自身超前），补了只会更偏。
     *
     * 【为什么用 AHRS_TAU_COMP 而不是 AHRS_TAU】
     *   实测用 0.5 补会补出 15.02°（超发 48%），标定后改 0.35。
     *
     * 【两个自动性】
     *   1) 补偿量随 ω 自动缩放：快转补多、慢转补少、静止 ω=0 补 0，不需要模式判定
     *   2) 静止时补偿自动消失 → 磁锚的长期零点不受影响
     */
    if (!mag_fusion_enabled) {
        return yaw_deg;      /* 纯积分路径：无低通、无滞后，不补偿 */
    }

    /* |ω| 小于 1°/s 视为静止/极慢：此时补偿量本就微小，且 diff 符号受噪声支配，
     * 直接跳过，避免静止时输出被噪声来回拽（HDG 抖动） */
    if (yaw_rate_dps > -1.0f && yaw_rate_dps < 1.0f) {
        return yaw_deg;
    }

    /* 符号判据：diff 与 ω 同号才是"真落后"，才补 */
    if (mag_gyro_diff * yaw_rate_dps <= 0.0f) {
        return yaw_deg;      /* 磁本来就超前 → 再往前推只会更偏，放弃补偿 */
    }

    float heading = yaw_deg + AHRS_TAU_COMP * yaw_rate_dps;   /* 补偿量单位：s × °/s = ° */
    /* 补偿后可能越界，重新归一化到 0~360 */
    while (heading < 0.0f)    heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;
    return heading;
}

/* 【中文名】手动重新校准陀螺零偏（对应 GYROCAL 命令）：静止时采几百个样平均，得到零偏 */
void AHRS_RecalibrateGyro(uint16_t samples)
{
    if (samples == 0) {
        samples = 500;    /* 没指定就采500个 */
    }
    if (samples > 5000) {
        samples = 5000;   /* 最多5000个（上限保护） */
    }

    float sum = 0.0f;     /* 累加器：把每次读到的 gz 加起来 */
    uint16_t ok = 0;      /* 成功读到的次数 */
    for (uint16_t i = 0; i < samples; i++) {
        float gx, gy, gz;
        if (MPU9250_ReadGyro(&gx, &gy, &gz) == 0) {
            sum += gz;    /* 累加陀螺 z 轴读数 */
            ok++;
        }
        HAL_Delay(1);     /* 每次隔1ms，让传感器有时间出新数据 */
    }

    if (ok == 0) {
        DBG_ERROR(TAG_AHRS, "GYROCAL failed: no gyro samples");
        return;
    }

    /* 平均值就是零偏！原理：车静止时真实转速=0，所以读到的全是零偏 */
    float new_offset = sum / (float)ok;
    gyro_z_offset = new_offset;
    DBG_INFO(TAG_AHRS, "GYROCAL done: samples=%u, gyro_z_offset=%.3f dps",
             (unsigned int)ok, (double)new_offset);
}

/* -------------------------------------------------------------------------- */
/* 获取完整数据                                                               */
/* -------------------------------------------------------------------------- */
/* 【中文名】打包取一份完整数据（航向+磁航向+陀螺），给外部模块用 */
void AHRS_GetData(AHRS_Data *data)
{
    if (data == NULL) return;   /* 传进来的指针为空就退出 */
    data->yaw = yaw_deg;        /* 填当前航向 */

    float mx, my, mz;
    if (MPU9250_ReadMag(&mx, &my, &mz) == 0) {
        data->mag_heading = MagHeadingDeg(mx, my);  /* 填磁航向 */
    }

    float gx, gy, gz;
    if (MPU9250_ReadGyro(&gx, &gy, &gz) == 0) {
        data->gyro_z = gz - gyro_z_offset;   /* 填减过零偏的陀螺角速度 */
    }
}

/* -------------------------------------------------------------------------- */
/* 计算两个角度之间的最短差值                                                 */
/* -------------------------------------------------------------------------- */
/* 【中文名】角度差（取最短弧）：为什么不能直接 target-current？
 * 因为角度是一圈循环的，350° 和 10° 直接减是 340°，但实际只差 20°。
 * 这个函数把"绕远的结果"扳回 -180~+180 之间那个最短值 */
float AHRS_AngleDiff(float target, float current)
{
    float diff = target - current;   /* 先直接减 */

    /* 处理角度环绕, 确保差值在 [-180, 180]：差太大减一圈，差太小加一圈 */
    while (diff > 180.0f)  diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;

    return diff;
}

/* -------------------------------------------------------------------------- */
/* 外部航向覆盖 (可靠里程计 EKF 使用)                                         */
/* -------------------------------------------------------------------------- */
/* 【中文名】强制覆盖航向：EKF 有时想直接指定航向是多少，就调这个 */
void AHRS_OverrideHeadingDeg(float heading_deg)
{
    /* 把传进来的航向归一到 0~360，然后直接覆盖 */
    while (heading_deg < 0.0f)    heading_deg += 360.0f;
    while (heading_deg >= 360.0f) heading_deg -= 360.0f;
    yaw_deg = heading_deg;
}

/* -------------------------------------------------------------------------- */
/* 磁力计融合开关                                                             */
/* -------------------------------------------------------------------------- */
/* 【中文名】开关磁力计融合：开=加入磁力计校正航向，关=纯陀螺 */
void AHRS_SetMagFusion(bool enable)
{
    mag_fusion_enabled = enable;
    if (enable) {
        /* 开启时立即用磁力计重置航向（把累积的陀螺漂移一次性清掉） */
        float mx, my, mz;
        if (MPU9250_ReadMag(&mx, &my, &mz) == 0) {
            yaw_deg = MagHeadingDeg(mx, my);
            DBG_INFO(TAG_AHRS, "Mag fusion ON, heading reset to %.1f°", yaw_deg);
        } else {
            DBG_WARN(TAG_AHRS, "Mag fusion ON but mag read failed");
        }
    } else {
        DBG_INFO(TAG_AHRS, "Mag fusion OFF (gyro only)");
    }
}

/* 【中文名】查询磁力计融合开没开 */
bool AHRS_IsMagFusionEnabled(void)
{
    return mag_fusion_enabled;
}

/* -------------------------------------------------------------------------- */
/* ZARU 零速零偏再估计: 静止状态通知 (2026-08-18)                              */
/* -------------------------------------------------------------------------- */
/* 【中文名】通知"车静止了/车动了"：主循环根据编码器判断后，调这个告诉 AHRS */
void AHRS_SetStationary(bool is_stationary)
{
    stationary = is_stationary;
    if (!is_stationary) {
        stationary_since_ms = 0;   /* 一动就重置静止计时，保证下次静止重新从0开始计 */
    }
}
