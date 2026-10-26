/**
 ******************************************************************************
 * @file    ota.h
 * @brief   RK3588 OTA 升级支持 (App 侧)
 ******************************************************************************
 * 配合 "V1 - 副本" Bootloader 使用:
 *   RK 发 CMD_GET_VERSION -> App 回 "VERSION:<ver> <date>\r\n"
 *   RK 发 CMD_OTA_START   -> App 在 OTA_FLAG_ADDR 写升级标记并软复位,
 *                            Bootloader 检测到标记进入升级模式,
 *                            随后 RK 用 UPDATE/ADDR/DATA/RUN 协议传固件。
 *
 * 升级标记约定 (与 bootloader.h 保持一致):
 *   地址 0x0800FC00 (扇区3末页, Bootloader 镜像只占扇区0, 互不影响)
 *   魔数 0x31544F41 ("AOT1" 小端)
 ******************************************************************************
 */
#ifndef __OTA_H
#define __OTA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 升级标记 (与 bootloader.h 的 BL_FLAG_ADDR / BL_FLAG_MAGIC 一致) */
#define OTA_FLAG_ADDR       0x0800FC00U
#define OTA_FLAG_MAGIC      0x31544F41U  /* "AOT1" 小端 */

/* 固件版本 (RK 侧对比用, 格式: VERSION:v3.1-YFIX 26.10.11)
 * v3.0-ODOMTEST (2026-08-29): 可靠里程计 ODOM 测试版 —
 *   自适应 EKF + 数据质量监督 + 动态 Q/R + ZUPT; 串口遥测保留原始数据,
 *   并新增 EKF 诊断字段; odom_compare_web 增加 /odom_diag 实时显示
 * v3.1-YFIX (2026-10-11): 修正世界投影 Y 轴镜像 —
 *   EKF 状态 X[2] 原按"-heading"存储, 与世界投影公式
 *   `X[1] += dt*(vx*sin ψ + vy*cos ψ)` 符号不匹配, 导致车头非 0° 时
 *   位移方向镜像(实车 HDG=47° 前进, 世界方向实测 -49° 应为 +47°)。
 *   现将 X[2] 统一为 **+heading (标准 ROS yaw)**, 同步修正
 *   Init / Odom_Update 的 yaw 测量 / 输出 heading / Odom_SetPose 四处。
 *   ⚠ 该 bug 在 heading≈0 时不可见(-0==+0), 必须非零初朝向才能验证。
 * v3.2-CFIX (2026-10-18): 修正底盘槽位 — FWD/BACK 与导航直行段把
 *   前进速度错填进 vy(横移)槽, 实车发 FWD,0.3 实测为"左移"而非"前进"。
 *   现将 Chassis_MoveManual(264) / TURNING 直行(334) / MOVING 直行(366)
 *   三处统一为 Vx=前进, 并同步修正 ROS 口径注释(vx 前正 / vy 左正)。
 * v3.3-MAGFIX (2026-10-18): 修复磁力计数据冻结 — AK8963 读取序列缺 ST2(0x09)
 *   寄存器, 导致读走 6 字节后未解除内部锁存, DRDY 不再置位, 磁力计恒返回
 *   上电首帧旧值(实车实测三轴一个 LSB 不变)。现一次读 HXL~ST2 共 7 字节。
 * v3.4-MAGDEF (2026-10-20): 默认启用磁力计融合 — 用绝对磁航向做长期锚，
 *   抑制陀螺零偏积分漂移(电机干扰实测仅~3.6°航向可忽略)；新增磁航向
 *   野值门控: 单帧跳变>15° 判为外部铁磁体(手机)干扰, 本周期退化为纯陀螺。
 *   开机 yaw 初始化为绝对磁航向(非0)，配合上位机 odom_fusion 增量加权融合。
 * v3.4.1-GATEFIX (2026-10-20): 修复野值门控两缺陷 — ①改用"增量一致性"判据
 *   (磁帧间增量 vs 陀螺帧间增量分歧>10°判干扰)，对基准平移免疫，杜绝渐进
 *   干扰"温水煮青蛙"式绑架基准；②干扰期间基准冻结不更新，干扰解除后磁航向
 *   回到正确值即自动恢复采纳。旧版绝对跳变判据在持续手机靠近时会锁死错航向。
 * v3.4.2-GATE2 (2026-10-20): 修复增量一致性门控"误伤真实旋转" — 实测车转
 *   2圈+10°(≈730°)但 HDG 只累计 371°，因磁力计与陀螺采样不同步，帧间增量
 *   频繁分歧>10°被误杀，旋转量被丢掉一半。现改用"磁航向 vs 陀螺预测航向"
 *   绝对差判据(阈值30°)，基准 gyro_yaw 不受磁干扰影响、无历史状态无正反馈，
 *   真实旋转时两路一起转差保持小值通过，手机脉冲差暴增拦截。
 * v3.4.3-MAGSIM (2026-10-20): 撤回固件内野值门控 — 连续三版门控(v3.4/v3.4.1/v3.4.2)
 *   各自死在不同地方(锁死/误伤旋转/永久拦截磁)，根因是固件内无"独立于磁场的参照"，
 *   任何"拿磁比陀螺"的门控都会撞上"磁绝对磁北 vs 陀螺相对积分"的恒定 offset。
 *   现固件只做最简"磁融合ON+互补滤波直接采纳磁力计"，野值剔除全部挪到上位机
 *   odom_fusion 用激光变化量(光学测距不受磁场影响、增量天然消 offset)当裁判。
 * v3.4.4-MAGSIGN (2026-10-20): 修正磁航向旋转方向反向 — MagHeadingDeg 的 east
 *   原为 -mx，导致航向随车体逆时针旋转而"减小"，与陀螺积分/rf2o 方向相反。
 *   实测(左转 wz=+0.2×10s)：陀螺 +143°、激光 +137.5°、磁 -72.7°。
 *   改为 east=+mx 使磁航向与陀螺同向（绝对值镜像为 180°-old，零点变化不影响导航，
 *   世界系仍以开机朝向为基准）。 */
#define FIRMWARE_VERSION    "v3.4.4-MAGSIGN"
#define FIRMWARE_DATE       "26.10.20"

/**
 * @brief  处理 OTA 相关串口命令
 * @param  cmd 行命令 (已去除行尾符)
 * @retval true 已处理 (调用方直接 return), false 不是 OTA 命令
 *
 * 命令:
 *   CMD_GET_VERSION -> 回 "VERSION:v3.1-YFIX 26.10.11\r\n"
 *   CMD_OTA_START   -> 写升级标记, 回 "OK OTA reboot", 软复位 (不返回)
 */
bool OTA_HandleCommand(const char *cmd);

#ifdef __cplusplus
}
#endif

#endif /* __OTA_H */
