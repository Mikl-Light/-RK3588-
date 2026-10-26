#!/usr/bin/env python3
"""激光自回波过滤：/scan_raw -> /scan

为什么必须有这个节点（本项目最重要的一条教训）：
  雷达装在车体最前方，扫描平面离底盘上表面只有 3cm，它必然会扫到自身
  安装支架与车体结构。这些近距离回波会被当成「真障碍」写进 SLAM 地图，
  形成**落在车体足迹内部的幽灵占据像素**。一旦幽灵像素距车体中心小于
  footprint 的内切半径，机器人自身所在栅格就会被判为 INSCRIBED
  （规划器视为不可通行），NavFn 从起点一步都走不出去——表现为
  "Failed to create plan" 或「原地打转不前进」。

  实测本车 /scan：laser 自身 -40°..0° 整段无有效回波（< range_min 被
  遮挡），+0° 附近另有 0.225m 近距回波，都是自回波特征。

两道互相独立的过滤：
  A. 距离下限 min_range —— 任何比它更近的回波一律置无效。与方位无关，
     主要用来挡掉车体结构。这是主力手段。
  B. 角度扇区 mask_sectors —— 在指定方位区间内把回波全部置无效。
     用来挡掉「只在某个方位出现」的安装支架干扰。

参数编码说明：
  mask_sectors 用**扁平的双精度数组**表示，两两成对：
      [-45.0, 10.0, 120.0, 140.0]  即屏蔽 [-45°, 10°] 与 [120°, 140°]
  （rclpy 的参数类型不支持嵌套数组，所以拍平表示。）
  角度以 laser 自身坐标系为准，已归一化到 (-180, 180]。

无效值取值（invalid_value）：
  被过滤掉的光束写什么值。接受以下几种写法，大小写不敏感：
      'inf' / '+inf' / 'infinity'   无回波（默认，ROS 惯例）
      'nan'                          无回波（另一种惯例）
      'max' / 'range_max' / 'max_range'   写成 max_range（有限但超量程）
      任意数值字符串，如 '0.0' / '15.0'
  无法识别的写法会记一条 error 日志并回退为 inf，**不会让节点崩溃**。

  ⚠ 该参数必须写成**带引号的字符串**（见 config/scan_filter_params.yaml）。
  历史故障：它原先写在 launch 的参数字典里，launch_ros 会用 yaml.dump()
  序列化字符串参数（launch_ros/utilities/normalize_parameters.py:151-152），
  而 PyYAML 不认为裸 inf 是数字，于是输出不带引号的 `inf`；接收端 rcl 按
  YAML 1.1 解析成 DOUBLE 无穷，与 STRING 声明冲突，节点在 __init__ 里
  抛 InvalidParameterTypeException 直接崩溃。本节点现在对两种类型都做了
  容错（_declare_invalid_value / _resolve_invalid_value），但参数来源仍以
  独立 params 文件为准，避免类型漂移。

自校准：
  把 report_period 设为 >0（如 3.0），节点会周期性地打印「每 10° 扇区的
  最小有效回波」。把车停在空旷处跑一次，就能看出自身结构出现在哪个方位。
"""

import math

import rclpy
from rclpy.exceptions import (
    InvalidParameterTypeException,
    ParameterAlreadyDeclaredException,
)
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from . import run_node

SENSOR_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class ScanFilter(Node):

    def __init__(self):
        super().__init__('scan_filter')

        self.declare_parameter('scan_in', '/scan_raw')
        self.declare_parameter('scan_out', '/scan')
        self.declare_parameter('min_range', 0.28)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('mask_sectors', [-45.0, 10.0])
        self._declare_invalid_value()
        self.declare_parameter('report_period', 0.0)

        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.invalid_value, self.invalid_label = self._resolve_invalid_value(
            self.get_parameter('invalid_value').value)
        self.report_period = float(self.get_parameter('report_period').value)

        flat = [float(v) for v in self.get_parameter('mask_sectors').value]
        if len(flat) % 2 != 0:
            self.get_logger().error(
                'mask_sectors 必须是成对的角度，收到 %d 个数，已忽略全部扇区' % len(flat))
            flat = []
        self.sectors = [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]

        # 每条光束是否落在屏蔽扇区内：扫描角度是固定的，只在几何变化时重算
        self._beam_mask = None
        self._geom_key = None
        self._masked = 0
        self._floored = 0
        self._scans = 0
        self._last_report = None

        self.pub = self.create_publisher(LaserScan, self.get_parameter('scan_out').value, SENSOR_QOS)
        self.stats_pub = self.create_publisher(String, '~/stats', 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_in').value, self._on_scan, SENSOR_QOS)

        self.get_logger().info(
            '自回波过滤就绪: %s -> %s, 距离下限 %.2fm, 上限 %.1fm, 屏蔽扇区 %s, 无效值 %s' % (
                self.get_parameter('scan_in').value, self.get_parameter('scan_out').value,
                self.min_range, self.max_range,
                self.sectors if self.sectors else '无',
                self.invalid_label))
        if self.report_period > 0.0:
            self.get_logger().info(
                '自校准模式已开启：每 %.1fs 打印一次各方位最小回波，请把车停在空旷处' % self.report_period)

    # ------------------------------------------- invalid_value 的声明与类型归一化

    def _declare_invalid_value(self):
        """声明 invalid_value，并容忍「值被以非字符串类型传入」的情况。

        实测根因（2026-09-13）：
          launch_ros 对「Python 字符串」参数走 yaml.dump() 序列化
          （launch_ros/utilities/normalize_parameters.py:151-152）。
          PyYAML 的隐式解析器要求无穷必须写成 `.inf`（带前导点），因此它
          不认为裸 `inf` 是浮点数，dump('inf') 输出的就是**不带引号**的 inf。
          而接收端 rcl 按 YAML 1.1 解析，裸 `inf` 正是 **DOUBLE 无穷**。
          结果：节点按 STRING 声明该参数时立刻抛
          InvalidParameterTypeException，在 __init__ 里直接崩溃。
          （对比：dump('460800') 会加引号，所以「数字样式」的字符串反而安全，
          这也解释了为什么只有 invalid_value 出问题。）

        兜底策略：退化到「动态类型」声明，把字符串与浮点两种来源都接住，
        取值统一交给 _resolve_invalid_value() 归一化 —— 无论哪种来源，
        节点都不会崩，且语义一致。
        """
        try:
            self.declare_parameter('invalid_value', 'inf')
            return
        except InvalidParameterTypeException:
            pass
        try:
            self.declare_parameter('invalid_value')
        except ParameterAlreadyDeclaredException:
            pass
        self.get_logger().warn(
            'invalid_value 被以非字符串类型传入（典型原因：launch 把裸 inf 序列化成了浮点）。'
            '已按动态类型接收，取值将被归一化。')

    def _resolve_invalid_value(self, raw):
        """把 invalid_value 归一化成 (float, 用于日志的标签)。兼容字符串与数值两种来源。"""
        if raw is None:
            return math.inf, 'inf'
        # bool 是 int 的子类，必须先挡掉，否则 True 会被当成 1.0
        if isinstance(raw, bool):
            self.get_logger().error('invalid_value 收到布尔值 %r，无法解释，回退为 inf' % raw)
            return math.inf, 'inf'
        if isinstance(raw, (int, float)):
            value = float(raw)
            if math.isnan(value):
                self.get_logger().warn('invalid_value = nan：消费端应把非有限值视为「无回波」')
                return value, 'nan'
            if value == math.inf:
                return value, 'inf'
            if value == -math.inf:
                self.get_logger().warn('invalid_value = -inf 不是合法取值，回退为 inf')
                return math.inf, 'inf'
            return value, '%.3f' % value

        token = str(raw).strip().strip('\'"').strip().lower()
        if token in ('inf', '+inf', 'infinity', '+infinity', 'infinite'):
            return math.inf, 'inf'
        if token in ('-inf', '-infinity'):
            self.get_logger().warn('invalid_value = %r 不是合法取值，回退为 inf' % raw)
            return math.inf, 'inf'
        if token in ('nan', '+nan', '-nan'):
            self.get_logger().warn('invalid_value = nan：消费端应把非有限值视为「无回波」')
            return math.nan, 'nan'
        if token in ('max', 'range_max', 'max_range'):
            return self.max_range, 'range_max(%.1f)' % self.max_range
        try:
            number = float(token)
        except ValueError:
            self.get_logger().error(
                'invalid_value 无法识别: %r（可用 inf / nan / max / range_max / 数值），回退为 inf'
                % raw)
            return math.inf, 'inf'
        return number, '%.3f' % number

    # ------------------------------------------------------------ 屏蔽表

    def _build_beam_mask(self, scan: LaserScan):
        key = (len(scan.ranges), round(scan.angle_min, 6), round(scan.angle_increment, 8))
        if key == self._geom_key and self._beam_mask is not None:
            return
        mask = [False] * len(scan.ranges)
        for i in range(len(scan.ranges)):
            if not self.sectors:
                break
            deg = math.degrees(scan.angle_min + i * scan.angle_increment)
            deg = (deg + 180.0) % 360.0 - 180.0        # 归一化到 (-180, 180]
            for lo, hi in self.sectors:
                if lo <= deg <= hi:
                    mask[i] = True
                    break
        self._beam_mask = mask
        self._geom_key = key
        self.get_logger().info('光束屏蔽表已生成: %d/%d 条被屏蔽' % (sum(mask), len(mask)))

    # --------------------------------------------------------------- 主逻辑

    def _on_scan(self, scan: LaserScan):
        self._build_beam_mask(scan)

        ranges = list(scan.ranges)
        masked = 0
        floored = 0

        for i, r in enumerate(ranges):
            if self._beam_mask[i]:
                ranges[i] = self.invalid_value
                masked += 1
                continue
            if not math.isfinite(r):
                continue
            # 超出量程的回波同样置为无效，避免下游把 0.0 当成"正前方有障碍"
            if r < self.min_range:
                ranges[i] = self.invalid_value
                floored += 1
            elif r > self.max_range:
                ranges[i] = self.invalid_value

        out = LaserScan()
        out.header = scan.header
        out.angle_min = scan.angle_min
        out.angle_max = scan.angle_max
        out.angle_increment = scan.angle_increment
        out.time_increment = scan.time_increment
        out.scan_time = scan.scan_time
        out.range_min = max(scan.range_min, self.min_range)
        out.range_max = min(scan.range_max, self.max_range)
        out.ranges = ranges
        out.intensities = scan.intensities
        self.pub.publish(out)

        self._masked += masked
        self._floored += floored
        self._scans += 1

        msg = String()
        msg.data = 'masked=%d floored=%d total=%d' % (masked, floored, len(ranges))
        self.stats_pub.publish(msg)

        if self.report_period > 0.0:
            self._maybe_report(scan)

    # --------------------------------------------------- 自校准：方位最小回波

    def _maybe_report(self, scan: LaserScan):
        now = self.get_clock().now()
        if self._last_report is not None:
            if (now - self._last_report).nanoseconds / 1e9 < self.report_period:
                return
        self._last_report = now

        bins = {}
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            deg = math.degrees(scan.angle_min + i * scan.angle_increment)
            deg = (deg + 180.0) % 360.0 - 180.0
            b = int((deg + 180.0) // 10) * 10 - 180
            if b not in bins or r < bins[b]:
                bins[b] = r

        parts = []
        for b in range(-180, 180, 10):
            r = bins.get(b)
            parts.append('%+4d:%s' % (b, ('%.2f' % r) if r is not None else ' -- '))
        self.get_logger().info('方位最小回波(m) ' + ' '.join(parts))


def main(args=None):
    rclpy.init(args=args)
    run_node(ScanFilter())


if __name__ == '__main__':
    main()
