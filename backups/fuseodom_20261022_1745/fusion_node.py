#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
odom_fusion/fusion_node.py — 轮式 + 激光 + IMU 里程计融合节点

核心思路（与用户讨论确定）：
  1. 轮式 yaw 漂移的根源 = 陀螺零偏误差（一个恒定残差一直在积分累加）
  2. 用激光里程计 yaw 作为"真值参考"，在特征丰富区在线估计这个零偏 b
  3. 开阔区激光退化时，用估计出的 b 补偿轮式 yaw，靠陀螺+编码器撑住
  4. 动态加权：特征区信激光（纠偏+防打滑），开阔区信轮式+陀螺（不退化）

输入话题：
  /odom              nav_msgs/Odometry   轮式里程计（STM32 EKF，50Hz）
  /odom_laser        nav_msgs/Odometry   激光里程计（rf2o，20Hz）
  /imu/imu407/data_raw  sensor_msgs/Imu  IMU 原始（gyro_z 角速度）
  /scan               sensor_msgs/LaserScan  激光扫描（退化检测用）

输出话题：
  /odom_fused        nav_msgs/Odometry   融合后里程计（50Hz）
  /fusion_diag       std_msgs/String     诊断 JSON（漂移率 b、权重 w、yaw 对比等）
  TF: odom_fused -> base_link

完全独立，不修改任何原有包。
"""
import json
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import TransformStamped, Quaternion
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


# ───────────────────────── 工具函数 ─────────────────────────

def yaw_from_quat(q) -> float:
    """从四元数提取 yaw (rad)。"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw: float) -> Quaternion:
    """yaw (rad) -> 四元数。"""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def wrap_pi(a: float) -> float:
    """角度归一化到 [-pi, pi]。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def linreg_slope(points) -> float:
    """
    对 [(t, y), ...] 做最小二乘线性回归，返回斜率 dy/dt。
    点数 < 2 时返回 0。
    """
    n = len(points)
    if n < 2:
        return 0.0
    sx = sum(p[0] for p in points)
    sy = sum(p[1] for p in points)
    sxx = sum(p[0] * p[0] for p in points)
    sxy = sum(p[0] * p[1] for p in points)
    denom = n * sxx - sx * sx
    if abs(denom) < 1e-12:
        return 0.0
    return (n * sxy - sx * sy) / denom


# ───────────────────────── 融合节点 ─────────────────────────

class OdomFusionNode(Node):
    def __init__(self):
        super().__init__('odom_fusion')

        # ── 参数（可通过 launch 覆盖）──
        self.declare_parameter('wheel_topic', '/odom')
        self.declare_parameter('laser_topic', '/odom_laser')
        self.declare_parameter('imu_topic', '/imu/imu407/data_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('out_topic', '/odom_fused')
        self.declare_parameter('out_frame', 'odom_fused')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        # 退化检测
        self.declare_parameter('scan_valid_ratio_threshold', 0.25)  # 有效点占比低于此值→激光退化
        self.declare_parameter('laser_stale_sec', 0.5)               # 激光 odom 超过此时间未更新→退化
        # 零偏估计
        self.declare_parameter('bias_window_sec', 5.0)                # 滑动窗口长度
        self.declare_parameter('bias_min_trust', 0.5)                 # 权重低于此值不估计零偏
        self.declare_parameter('bias_jump_reject_rad', 0.3)          # 单帧 diff 跳变超过此值→丢弃（激光跳变），0.3rad≈17°
        self.declare_parameter('bias_lowpass_alpha', 0.05)            # 零偏低通系数（越小越平滑）
        # 融合
        self.declare_parameter('laser_correction_gain', 0.1)          # 特征区激光 yaw 慢速修正增益
        self.declare_parameter('trust_smoothing', 0.2)                 # 权重 w 低通系数
        # 磁野值剔除（激光变化量当裁判，2026-10-20）
        self.declare_parameter('mag_outlier_thresh_rad', 0.35)         # 轮式 dyaw 与激光 dyaw 分歧>20°(0.35rad)→判磁野值，本帧丢弃轮式增量、纯激光兜底
        self.declare_parameter('mag_outlier_max_gate', 0.5)            # 连续判野值超过此秒数→怀疑激光本身漂了，强制恢复采纳轮式(防止激光退化时误杀)
        # 轮式(磁绝对)权重下限 + 静止绝对锚（2026-10-20 修"静止 110s 漂 8.5°"）
        # 2026-10-22 实测下调 0.3 → 0.05。
        # 原理由（10-20）：防止"纯激光增量累加继承 rf2o 漂移"。
        #   该担心只在 **静止** 时成立（rf2o 静止随机游走 110s 漂 8.5°），
        #   而 **行驶** 时 rf2o 实测偏差仅 0.03°（`fusion_line_test.py` 两轮）。
        # 另一方面固件磁航向因环境磁场梯度 21µT/m，行驶 3s 偏了 −7.99°。
        # 0.3 的磁权重把误差放大成 −2.68°（验算 0.3×(−7.99)+0.7×(−0.03)=−2.42，实测 −2.68，
        # 模型吻合），比纯激光差 90 倍。详见 .workbuddy/memory/2026-10-22.md 结论 7。
        self.declare_parameter('wheel_weight_floor', 0.05)
        self.declare_parameter('still_lin_thresh', 0.01)               # 静止判据：轮式线速度(m/s)低于此值
        self.declare_parameter('still_ang_thresh', 0.02)               # 静止判据：IMU 角速度(rad/s)低于此值，0.02rad/s≈1.15°/s
        self.declare_parameter('still_hold_sec', 0.5)                  # 连续满足静止多久才认定（防运动中的瞬时低速误触发）
        self.declare_parameter('still_anchor_gain', 0.02)              # 静止时向磁绝对航向收敛的每帧增益(50Hz 下 τ≈1s)
        # 2026-10-22：静止时是否"冻结航向"（True=冻结，不再向磁航向收敛）。
        # 置 False 可回到 10-20 的旧行为（向磁收敛），用于对比实验。
        self.declare_parameter('still_anchor_freeze', True)
        # 2026-10-22 新增：位置增量累加的跳变保护。
        # 融合按轮式 50Hz 跑，激光只有 8Hz。正常单帧增量上界 = 最大速度 0.6m/s ÷ 8Hz ≈ 7.5cm。
        # rf2o 在特征退化/重定位时偶发给出米级增量，若不拦住会把位置整个拽飞。
        self.declare_parameter('pos_jump_reject', 0.30)                # 单帧激光增量超过此值(m)丢弃

        # ── 状态 ──
        self.wheel_latest = None      # 最新轮式 odom
        self.laser_latest = None      # 最新激光 odom
        self.laser_stamp = 0.0        # 激光 odom 时间戳
        self.gyro_z = 0.0             # 最新 IMU 角速度 z (rad/s)
        self.scan_valid_ratio = 0.0   # 最新 scan 有效点占比

        # 零偏估计
        self.diff_window = deque()    # [(t, diff_rad), ...] 滑动窗口
        self.b_smooth = 0.0           # 平滑后的零偏估计 (rad/s)
        self.b_raw = 0.0              # 本次回归原始值
        self.bias_samples = 0          # 累计有效估计次数
        self.b_cumulative = 0.0        # 累计补偿量 (rad)，yaw_corrected = yaw_wheel - b_cumulative
        self.last_bias_t = None        # 上次累计补偿的时间

        # 权重
        self.w_smooth = 0.0            # 平滑后的激光可信度 [0,1]

        # 增量加权融合状态（2026-10-20：yaw 融合从"激光绝对值修正"改为"增量加权"）
        # 背景：固件默认开磁融合后，轮式 yaw 是"绝对磁航向"(不漂)，激光 rf2o yaw 是"相对初始帧"，
        #       两者基准差一整段常数(如 310°)。若仍用绝对差修正，会每帧把磁绝对航向拉回激光相对基准。
        # 解法：只用两者的"帧间增量 dyaw"做加权融合，基准常数在增量里自然消掉。
        self.last_yaw_wheel = None     # 上一帧轮式 yaw (绝对磁航向)
        self.last_yaw_laser = None     # 上一帧激光 yaw (相对初始帧)
        self.last_yaw_fused = None     # 上一帧融合 yaw (绝对锚=轮式磁航向)
        # 2026-10-22 新增：融合位置的增量累加状态
        #   旧写法 `x = w*laser + (1-w)*wheel` 属跨坐标系加权（见 _fuse_and_publish 第 6 步注释），
        #   没有物理意义，行驶/静止切换会跳变量级。改为只累加激光帧间增量。
        self.last_x_fused = None       # 融合系当前位置 (m)
        self.last_y_fused = None
        self.laser_prev_xy = None      # 上一次读到的激光位置（求增量用）
        self.pos_frozen = False        # 本帧位置是否被冻结（激光退化/静止）
        self.laser_dyaw = 0.0          # 最近一个激光消息的帧间增量 (rad)，激光未更新期间保持不变
        # 2026-10-20 修「时间尺度不匹配」：
        #   激光 8Hz、轮式 50Hz。若直接把 laser_dyaw(每 0.125s 的增量) 加到每个轮式帧(每 0.02s)，
        #   同一个增量会被重复计入约 6 次 → 融合航向跑飞（实测转 360° 被累计成 4254°）。
        #   正解：把激光增量换算成角速度 laser_rate(rad/s)，再乘本轮式帧的真实 dt，才同尺度。
        self.laser_rate = 0.0          # 激光角速度 (rad/s)
        self.last_laser_t = 0.0        # 上次激光帧时间戳
        self.last_fuse_t = None        # 上次融合时间戳（求轮式帧 dt）
        # 磁野值剔除状态
        self.mag_outlier = False       # 当前帧轮式增量是否被判为磁野值
        self.mag_outlier_since = None  # 连续判野值的起始时间
        # 静止检测状态
        self.still_since = None        # 本轮"开始静止"的时间
        self.still_active = False      # 是否已认定静止（用于切到绝对锚模式）

        # 输出
        self.out_seq = 0

        # ── 订阅 ──
        # /scan 由 scan_filter 以 Best-Effort 发布，订阅必须也用 best_effort，
        # 否则 QoS 不兼容（sub Reliable + pub BestEffort=不兼容），收不到 scan
        # → scan_valid_ratio 恒 0、trust_w 恒 0、野值剔除的激光裁判失效。
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)
        self.create_subscription(
            Odometry, self.get_parameter('wheel_topic').value,
            self._wheel_cb, 10)
        self.create_subscription(
            Odometry, self.get_parameter('laser_topic').value,
            self._laser_cb, 10)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value,
            self._imu_cb, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._scan_cb, scan_qos)

        # ── 发布 ──
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter('out_topic').value, 10)
        self.diag_pub = self.create_publisher(String, '/fusion_diag', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            'odom_fusion started: wheel=%s laser=%s imu=%s scan=%s -> %s'
            % (self.get_parameter('wheel_topic').value,
               self.get_parameter('laser_topic').value,
               self.get_parameter('imu_topic').value,
               self.get_parameter('scan_topic').value,
               self.get_parameter('out_topic').value))

    # ────────────── 回调 ──────────────

    def _wheel_cb(self, msg: Odometry):
        """轮式 odom 回调（50Hz）—— 触发融合输出。"""
        self.wheel_latest = msg
        self._fuse_and_publish()

    def _laser_cb(self, msg: Odometry):
        """激光 odom 回调：更新激光 yaw 的帧间增量 dyaw 与角速度 rate（增量加权融合用）。"""
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        t_now = self._now_sec()
        if self.last_yaw_laser is not None:
            # 激光帧间增量：wrap_pi 处理跨 ±π
            self.laser_dyaw = wrap_pi(yaw - self.last_yaw_laser)
            # 换算成角速度（rad/s），供 50Hz 的轮式帧按自己的 dt 取用
            dt = t_now - self.last_laser_t
            if 0.01 < dt < 1.0:
                self.laser_rate = self.laser_dyaw / dt
            else:
                # 时间戳异常（首帧/停顿/回绕）→ 本帧不给速度，避免污染
                self.laser_rate = 0.0
        self.last_yaw_laser = yaw
        self.last_laser_t = t_now
        self.laser_latest = msg
        self.laser_stamp = t_now

    def _imu_cb(self, msg: Imu):
        self.gyro_z = msg.angular_velocity.z

    def _scan_cb(self, msg: LaserScan):
        """退化检测：计算 scan 有效点占比。"""
        total = len(msg.ranges)
        if total == 0:
            self.scan_valid_ratio = 0.0
            return
        valid = 0
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                valid += 1
        self.scan_valid_ratio = valid / total

    # ────────────── 核心融合 ──────────────

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _compute_trust(self) -> float:
        """
        计算激光可信度 w ∈ [0,1]。
        两个条件同时满足才可信：
          1. scan 有效点占比 >= 阈值（特征丰富）
          2. 激光 odom 不陈旧（rf2o 在正常输出）
        """
        ratio_thresh = self.get_parameter('scan_valid_ratio_threshold').value
        stale_sec = self.get_parameter('laser_stale_sec').value

        ratio_ok = self.scan_valid_ratio >= ratio_thresh
        fresh_ok = (self._now_sec() - self.laser_stamp) < stale_sec if self.laser_stamp > 0 else False

        w = 1.0 if (ratio_ok and fresh_ok) else 0.0
        # 低通平滑，避免抖动
        alpha = self.get_parameter('trust_smoothing').value
        self.w_smooth = self.w_smooth + alpha * (w - self.w_smooth)
        return self.w_smooth

    def _update_still(self, t: float, wheel) -> bool:
        """
        静止判定。返回 True 表示"已认定静止"。
        三个量同时低于阈值才算这一帧静止：轮式 vx、轮式 vy、IMU gyro_z。
        必须连续满足 still_hold_sec 才真正认定，避免运动途中的瞬时低速把
        "绝对锚"误打开（那样会把磁力计的扰动硬拽进融合航向）。
        """
        lin_th = self.get_parameter('still_lin_thresh').value
        ang_th = self.get_parameter('still_ang_thresh').value
        hold = self.get_parameter('still_hold_sec').value

        vx = abs(wheel.twist.twist.linear.x)
        vy = abs(wheel.twist.twist.linear.y)
        wz = abs(self.gyro_z)

        if vx < lin_th and vy < lin_th and wz < ang_th:
            if self.still_since is None:
                self.still_since = t
            return (t - self.still_since) >= hold

        self.still_since = None
        return False

    def _estimate_bias(self, t: float, yaw_wheel: float, yaw_laser: float, w: float):
        """
        在线估计陀螺零偏 b (rad/s)。
        只在激光可信时（w >= bias_min_trust）采集 diff 样本，
        用滑动窗口线性回归求斜率。
        """
        min_trust = self.get_parameter('bias_min_trust').value
        if w < min_trust or self.laser_latest is None:
            return

        diff = wrap_pi(yaw_wheel - yaw_laser)

        # 跳变剔除：与上一帧 diff 差过大 → 认为激光跳变，清空窗口重新采
        jump_reject = self.get_parameter('bias_jump_reject_rad').value
        if len(self.diff_window) > 0:
            last_diff = self.diff_window[-1][1]
            if abs(wrap_pi(diff - last_diff)) > jump_reject:
                self.diff_window.clear()

        self.diff_window.append((t, diff))

        # 清理过期窗口
        window_sec = self.get_parameter('bias_window_sec').value
        while len(self.diff_window) > 0 and (t - self.diff_window[0][0]) > window_sec:
            self.diff_window.popleft()

        # 窗口足够长才回归（至少 2 秒数据）
        if len(self.diff_window) >= 2 and (self.diff_window[-1][0] - self.diff_window[0][0]) > 1.0:
            self.b_raw = linreg_slope(list(self.diff_window))
            alpha = self.get_parameter('bias_lowpass_alpha').value
            self.b_smooth = self.b_smooth + alpha * (self.b_raw - self.b_smooth)
            self.bias_samples += 1

    def _fuse_and_publish(self):
        """融合并发布 /odom_fused + TF + diag。"""
        if self.wheel_latest is None:
            return

        t = self._now_sec()
        wheel = self.wheel_latest
        yaw_wheel = yaw_from_quat(wheel.pose.pose.orientation)

        # 本轮式帧的真实 dt（把激光角速度换算成"这一帧该走多少度"用）
        if self.last_fuse_t is not None:
            dt_wheel = t - self.last_fuse_t
            if not (0.001 < dt_wheel < 0.5):
                dt_wheel = 0.02     # 时间戳异常兜底（正常 50Hz=0.02s）
        else:
            dt_wheel = 0.0
        self.last_fuse_t = t
        # 激光本帧应贡献的航向增量：rate × 本帧 dt（与轮式增量同尺度）
        dyaw_laser_eff = self.laser_rate * dt_wheel

        # 1. 计算激光可信度
        w = self._compute_trust()

        # 2. 零偏估计（特征区）
        yaw_laser = 0.0
        if self.laser_latest is not None:
            yaw_laser = yaw_from_quat(self.laser_latest.pose.pose.orientation)
            self._estimate_bias(t, yaw_wheel, yaw_laser, w)

        # 3. 累计补偿量（b_cumulative 是"已经漂了多少"，从估计开始累计）
        #    注：磁绝对航向下轮式 yaw 已不漂，b_smooth≈0，此累计也≈0，属上一代兜底逻辑，
        #    仍保留是为兼容 AHRSMAG,OFF 后轮式退回陀螺积分的场景。
        if self.last_bias_t is not None:
            dt = t - self.last_bias_t
            if 0 < dt < 1.0:  # 防止时间跳变
                self.b_cumulative += self.b_smooth * dt
        self.last_bias_t = t

        # 4. 补偿后轮式 yaw（磁绝对航向，不漂；是融合的"绝对锚"）
        yaw_wheel_corrected = wrap_pi(yaw_wheel - self.b_cumulative)

        # 4.5 静止检测（2026-10-20）
        #     目的：车静止时 rf2o 没有真实观测，激光 yaw 会做随机游走（实测 110s 漂 8.5°）。
        #     若此时仍按"增量加权"累加，就等于把激光的静止漂移完整抄进融合航向。
        #     判据：轮式线速度(vx,vy) 与 IMU 角速度同时接近 0，且持续 still_hold_sec。
        self.still_active = self._update_still(t, wheel)

        # 5. 融合 yaw（增量加权，2026-10-20）：
        #    轮式 yaw = 绝对磁航向(不漂)，激光 yaw = 相对初始帧(有基准差)，
        #    两者基准差一大段常数。直接用绝对差修正会把磁绝对航向拉回激光相对基准。
        #    正确做法：只用两者"帧间增量 dyaw"加权，基准常数在增量里自动消掉。
        #      yaw_fused = 上一帧融合 yaw + (w_wheel*dyaw_wheel + w_laser*dyaw_laser)
        #    其中 w_laser = 激光可信度 w（但轮式至少保留 wheel_weight_floor），w_wheel = 1 - w。
        #    特征区(激光可信高) → 旋转量以激光增量为主(短期准)；
        #    开阔区(激光退化)   → 旋转量退回轮式增量(磁绝对兜底)。
        if self.last_yaw_wheel is not None:
            dyaw_wheel = wrap_pi(yaw_wheel_corrected - self.last_yaw_wheel)
        else:
            dyaw_wheel = 0.0
        self.last_yaw_wheel = yaw_wheel_corrected

        # 5.5 磁野值剔除：用激光变化量当裁判（2026-10-20）
        #    只有当"激光可信"（有扫描、不退化）时才有资格当裁判。
        #    判据：|轮式dyaw - 激光dyaw| 超阈值 → 轮式这帧的航向增量不可信(磁被手机扰动)
        #    因为手机只扰磁场、不扰光学测距，所以激光dyaw是干净的"真实旋转"参考。
        #    两路增量做差时，磁(绝对)和激光(相对)的基准offset天然抵消，无需对齐初始值。
        #    静止时不判：静止时激光 dyaw 本身只是噪声，拿噪声当裁判会误杀。
        thresh = self.get_parameter('mag_outlier_thresh_rad').value
        self.mag_outlier = False
        if (not self.still_active) and w > 0.1 and self.laser_latest is not None \
                and abs(dyaw_laser_eff) > 1e-6:
            disagree = abs(wrap_pi(dyaw_wheel - dyaw_laser_eff))
            if disagree > thresh:
                self.mag_outlier = True
                if self.mag_outlier_since is None:
                    self.mag_outlier_since = t
        else:
            self.mag_outlier_since = None  # 激光不可信 / 车静止，不判野值

        # 连续判野值太久（超过 mag_outlier_max_gate 秒）→ 怀疑是激光自己漂了，
        # 强制解除野值标记，恢复采纳轮式（防止激光退化时把轮式永久当野值）。
        max_gate = self.get_parameter('mag_outlier_max_gate').value
        if self.mag_outlier_since is not None and (t - self.mag_outlier_since) > max_gate:
            self.mag_outlier = False
            self.mag_outlier_since = None

        # 权重：激光可信度 w，但轮式至少保留 wheel_weight_floor
        # （2026-10-20 修：原来 w_wheel = 1-w，w=1.0 时轮式权重为 0，
        #   融合航向就变成"纯激光增量累加"，完全丢掉了磁绝对锚，继承了 rf2o 的漂移）
        floor = self.get_parameter('wheel_weight_floor').value
        w_wheel = max(floor, 1.0 - w)
        w_laser = 1.0 - w_wheel

        # 判为磁野值 → 丢弃轮式增量，纯激光增量兜底（保持航向不被磁污染带跑）
        dyaw_wheel_eff = 0.0 if self.mag_outlier else dyaw_wheel
        # 重新归一化：野值时全部权重给激光
        if self.mag_outlier:
            dyaw_fused = dyaw_laser_eff
        else:
            dyaw_fused = w_wheel * dyaw_wheel_eff + w_laser * dyaw_laser_eff

        if self.last_yaw_fused is None:
            self.last_yaw_fused = yaw_wheel_corrected   # 首次用轮式磁绝对做锚

        if self.still_active:
            # 5.6 静止处理（2026-10-22 改：默认冻结航向）
            #   原逻辑（10-20）：向 yaw_wheel_corrected（磁绝对航向）按 gain 收敛。
            #     前提是"磁航向是可绝对信任的锚"。
            #   但实测环境磁场梯度 21µT/m：车前进 30cm，magx 24.64→18.28µT，磁矢量方向转 8°，
            #     手推回原位（零电流）又回到 23.78µT → 变化与**位置**绑定，不是电机电流/剩磁。
            #   ⇒ **车停下后，该位置的磁航向本身就是错的**，向它收敛等于锚在错地方。
            #     实测：停车 1.5s 额外拉偏 −2.85°，把行驶段的 −2.68° 恶化到 −5.53°。
            #   物理上：静止时车并不转动，航向本不该变化 → 直接冻结。
            #   （still_anchor_freeze=False 可回退旧行为做对比）
            if self.get_parameter('still_anchor_freeze').value:
                yaw_fused = self.last_yaw_fused        # 冻结：静止时航向不修正
            else:
                err = wrap_pi(yaw_wheel_corrected - self.last_yaw_fused)
                yaw_fused = wrap_pi(self.last_yaw_fused
                                    + self.get_parameter('still_anchor_gain').value * err)
        else:
            yaw_fused = wrap_pi(self.last_yaw_fused + dyaw_fused)
        self.last_yaw_fused = yaw_fused

        # 6. 融合位置：激光帧间增量累加（2026-10-22 重写）
        #    旧写法 x = w*laser + (1-w)*wheel 是**跨坐标系加权**，没有物理意义：
        #      /odom_laser 在激光系（+x = 车头）；/odom 在固件世界系（X=北 / Y=西）。
        #      实测同一段 2m 直线（2026-10-22 走廊）：
        #        laser 位移 (+2.013, +0.057)，wheel 位移 (−0.071, −1.997)  → 相差约 90°，
        #        且固件 (北,西) 与 ROS (北,东) 的 Y 轴手性未做换算。
        #      ⇒ 行驶时 w=1 走激光、静止时走轮式，切换瞬间位置会跳，建图时车走走停停 → 地图被拽断。
        #    改法：只累加激光的**帧间增量**（增量与坐标系无关，只要方向一贯）；
        #      静止或激光退化时冻结（静止时车不动，位置本来就不该变）。
        #    轮式位置不再参与 —— 它的标定已被 2m 双传感器互证（wheel −0.09% / laser +0.70%），
        #      不需要再靠加权来"借"它的精度。
        if self.last_x_fused is None:
            # 首帧：融合系原点 = 激光系原点（rf2o 从 0 起算，天然对齐）
            self.last_x_fused = 0.0
            self.last_y_fused = 0.0

        laser_pos_ok = (self.laser_latest is not None) and (w > 0.1)
        if self.laser_prev_xy is not None and laser_pos_ok and not self.still_active:
            lp = self.laser_latest.pose.pose.position
            dx = lp.x - self.laser_prev_xy[0]
            dy = lp.y - self.laser_prev_xy[1]
            if math.hypot(dx, dy) <= self.get_parameter('pos_jump_reject').value:
                self.last_x_fused += dx
                self.last_y_fused += dy
                self.pos_frozen = False
            else:
                # 单帧增量离谱 → 判 rf2o 跳变，丢弃本帧（位置停在原处）
                self.pos_frozen = True
        else:
            self.pos_frozen = True          # 静止或激光退化：位置冻结
        # 增量基准始终跟随最新激光读数。同一帧激光被 50Hz 的融合重复读到时会算出 0 增量，
        # 不会重复累加（这是"每帧更新 prev"而不是"只在激光帧更新"的原因）。
        if self.laser_latest is not None:
            lp = self.laser_latest.pose.pose.position
            self.laser_prev_xy = (lp.x, lp.y)

        x_fused = self.last_x_fused
        y_fused = self.last_y_fused

        # 7. 速度：用轮式（控制更可靠），wz 用 IMU 原始角速度（更准）
        vx = wheel.twist.twist.linear.x
        vy = wheel.twist.twist.linear.y
        wz = self.gyro_z if abs(self.gyro_z) > 1e-6 else wheel.twist.twist.angular.z

        # ── 发布 /odom_fused ──
        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.get_parameter('out_frame').value
        out.child_frame_id = self.get_parameter('base_frame').value
        out.pose.pose.position.x = x_fused
        out.pose.pose.position.y = y_fused
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation = quat_from_yaw(yaw_fused)
        # 协方差：融合后比单一来源更可信，给较小值
        out.pose.covariance[0] = 0.01    # x
        out.pose.covariance[7] = 0.01    # y
        out.pose.covariance[35] = 0.005  # yaw
        out.twist.twist.linear.x = vx
        out.twist.twist.linear.y = vy
        out.twist.twist.angular.z = wz
        out.twist.covariance[0] = 0.01
        out.twist.covariance[7] = 0.01
        out.twist.covariance[35] = 0.01
        self.odom_pub.publish(out)

        # ── 发布 TF ──
        if self.get_parameter('publish_tf').value:
            tf = TransformStamped()
            tf.header.stamp = out.header.stamp
            tf.header.frame_id = self.get_parameter('out_frame').value
            tf.child_frame_id = self.get_parameter('base_frame').value
            tf.transform.translation.x = x_fused
            tf.transform.translation.y = y_fused
            tf.transform.translation.z = 0.0
            tf.transform.rotation = quat_from_yaw(yaw_fused)
            self.tf_broadcaster.sendTransform(tf)

        # ── 发布诊断（供 Web 可视化）──
        diag = {
            't': round(t, 3),
            'trust_w': round(w, 3),                    # 激光可信度 [0,1]
            'scan_valid_ratio': round(self.scan_valid_ratio, 3),
            'bias_b_rad_s': round(self.b_smooth, 6),   # 估计的零偏 (rad/s)
            'bias_b_deg_s': round(math.degrees(self.b_smooth), 4),
            'bias_b_raw': round(math.degrees(self.b_raw), 4),
            'bias_cumulative_deg': round(math.degrees(self.b_cumulative), 2),
            'bias_samples': self.bias_samples,
            'yaw_wheel_deg': round(math.degrees(yaw_wheel), 2),
            'yaw_wheel_corrected_deg': round(math.degrees(yaw_wheel_corrected), 2),
            'yaw_laser_deg': round(math.degrees(yaw_laser), 2) if self.laser_latest else None,
            'yaw_fused_deg': round(math.degrees(yaw_fused), 2),
            'gyro_z_rad_s': round(self.gyro_z, 4),
            'dyaw_wheel_deg_s': round(math.degrees(dyaw_wheel), 3),
            'dyaw_laser_deg_s': round(math.degrees(dyaw_laser_eff), 3),   # 已换算到本轮式帧尺度
            'laser_rate_deg_s': round(math.degrees(self.laser_rate), 2),  # 激光原始角速度
            'mag_outlier': self.mag_outlier,
            'still': self.still_active,                # 是否已认定静止（切绝对锚模式）
            'w_wheel': round(w_wheel, 3),              # 轮式(磁)增量权重（有下限）
            'w_laser': round(w_laser, 3),              # 激光增量权重
            'x_fused': round(x_fused, 3),
            'y_fused': round(y_fused, 3),
        }
        diag_msg = String()
        diag_msg.data = json.dumps(diag, ensure_ascii=False)
        self.diag_pub.publish(diag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
