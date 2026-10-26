#!/usr/bin/env python3
"""圆形路径跟随 —— 车头始终指向圆心的**连续**跟随（自写控制律）。

交互（在 RViz 里用「Publish Point」工具点，它发 geometry_msgs/PointStamped 到 /clicked_point）：

    第 1 个点 = 圆心
    第 2 个点 = 圆周上任意一点（两点距离即半径），起点的方位角也由它确定

收满两点后几何立即锁定并用 Marker 画出来（圆周 / 圆心 / 半径 / 起点 / 两个方向箭头）。
确认画面无误后调 ~/start，车沿圆连续走**一圈**后自动停；~/stop 随时中止；~/clear 清空几何。

姿态与运动学（车头始终指向圆心）：
    车位于 P(ψ) = C + R·(cosψ, sinψ) 时车头 yaw = ψ + π（指向圆心），
    此时**切线方向正好落在车体横向**，于是：
        vx = 0            稳态下不需要前后走
        vy = -v_t         纯侧移（麦轮独有），v_t 为逆时针切向速度
        ω  = +v_t / R     维持"始终指向圆心"
    控制律在此基础上加两项修正：
        · 径向 P 修正 v_in = k_r·(d - R)：把车拉回正确的半径（走内/走外都会自动纠正）
        · 朝向 P 修正 k_θ·e：让车头持续对准圆心（误差大时切向速度按 cos(e) 自动降速）
    ⇒ ω = k_θ·e + v_t·cos(e)/d

⚠ 安全：本节点直接发 /cmd_vel，**绕开了 nav2 的安全层**。因此自带：
    ① /scan 切向扇区分级减速 + 急停（半径急停距离 stop_distance）
    ② 20 Hz 定频喂住 chassis_bridge 的 0.5 s 看门狗
    ③ 启动前 /cmd_vel 空闲探测 + 运行中"这条速度不是我发的"检测 —— 防止与 nav2 抢底盘
    ④ dry_run=true 时只打印不发，用于验证指令序列
"""

import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Point, PointStamped, Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time as RclTime
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from . import run_node

MARKER_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)

# scan_filter 是 Best Effort 发的，订阅必须对齐，否则一条都收不到
SENSOR_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)

SEGMENTS = 72          # 圆周用多少段折线画（5° 一段）
CLOCKWISE = -1         # 内部记号
COUNTER_CLOCKWISE = 1

TWO_PI = 2.0 * math.pi


def _wrap(a):
    """把角度归一到 (-π, π]。"""
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def _yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def plan_circle_command(px, py, yaw, center, radius, linear_vel, counter_clockwise,
                        k_theta, k_radial, max_forward_vel, max_lateral_vel,
                        max_angular_vel):
    """车头始终指向圆心的圆周跟随控制律（**纯函数**，便于离线验算）。

    稳态（车在圆上、车头正对圆心）时应有：
        vx = 0        vy = ∓v        ω = ±v/R
    其中逆时针为 vy = -v、ω = +v/R。

    返回 (vx, vy, wz, info)。
    """
    cx, cy = center
    dx, dy = cx - px, cy - py
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return 0.0, 0.0, 0.0, {'d': 0.0, 'e': 0.0, 'vt': 0.0, 'degenerate': True}

    phi = math.atan2(dy, dx)              # 车 → 圆心 的方向
    e = _wrap(phi - yaw)                  # 朝向误差：>0 需逆时针转

    # 切向速度：朝向误差大时按 cos(e) 自动降速（clamp 保证 cos > 0，不会倒着走）
    v_t = linear_vel if counter_clockwise else -linear_vel
    vt_eff = v_t * math.cos(_clamp(e, -1.2, 1.2))

    # 径向修正：d > R 时往圆心走（车头指向圆心 ⇒ 车体 +x 即向内）
    v_in = _clamp(k_radial * (d - radius), -max_forward_vel, max_forward_vel)

    # 世界系 → 车体系。
    #   u_in = (cosφ, sinφ) 指向圆心     u_t = (sinφ, -cosφ) 逆时针切向
    wx = v_in * math.cos(phi) + vt_eff * math.sin(phi)
    wy = v_in * math.sin(phi) - vt_eff * math.cos(phi)
    vx = wx * math.cos(yaw) + wy * math.sin(yaw)
    vy = -wx * math.sin(yaw) + wy * math.cos(yaw)

    # 角速度 = 朝向 P 修正 + 圆周运动前馈（用实际距离 d，收敛更快）
    omega = k_theta * e + vt_eff / max(d, 0.10)

    vx = _clamp(vx, -max_forward_vel, max_forward_vel)
    vy = _clamp(vy, -max_lateral_vel, max_lateral_vel)
    omega = _clamp(omega, -max_angular_vel, max_angular_vel)
    return vx, vy, omega, {'d': d, 'e': e, 'vt': vt_eff, 'degenerate': False}


class PathFollower(Node):

    def __init__(self):
        super().__init__('path_follower')

        # ---------------- 几何 / 交互
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('click_topic', '/clicked_point')
        self.declare_parameter('marker_topic', 'path_markers')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # ---------------- 速度与增益
        # 切向线速度（m/s）。实测底盘限幅 vx/vy<=0.60、wz<=2.00，很宽裕。
        self.declare_parameter('linear_vel', 0.20)
        self.declare_parameter('max_lateral_vel', 0.60)
        self.declare_parameter('max_forward_vel', 0.15)   # 径向修正的限幅，故意压小
        self.declare_parameter('max_angular_vel', 2.00)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('k_theta', 2.0)            # 朝向 P 增益
        self.declare_parameter('k_radial', 1.0)           # 径向 P 增益
        self.declare_parameter('counter_clockwise', True)
        self.declare_parameter('min_radius', 0.15)        # 比这更小的圆拒绝执行

        # ---------------- 安全
        self.declare_parameter('stop_distance', 0.35)     # 切向障碍急停
        self.declare_parameter('slow_distance', 0.80)     # 降到 35%
        self.declare_parameter('sector_inner_deg', 35.0)  # 切向扇区内边界（车体角）
        self.declare_parameter('sector_outer_deg', 145.0)  # 切向扇区外边界
        self.declare_parameter('laser_yaw_fallback', math.pi)  # TF 查不到时的兜底（本车反装）
        self.declare_parameter('idle_check_sec', 1.5)     # 启动前听 /cmd_vel 的时长
        self.declare_parameter('dry_run', False)

        self.frame_id = self.get_parameter('frame_id').value
        self.click_topic = self.get_parameter('click_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.linear_vel = float(self.get_parameter('linear_vel').value)
        self.max_lateral_vel = float(self.get_parameter('max_lateral_vel').value)
        self.max_forward_vel = float(self.get_parameter('max_forward_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.k_theta = float(self.get_parameter('k_theta').value)
        self.k_radial = float(self.get_parameter('k_radial').value)
        self.counter_clockwise = bool(self.get_parameter('counter_clockwise').value)
        self.min_radius = float(self.get_parameter('min_radius').value)

        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.slow_distance = float(self.get_parameter('slow_distance').value)
        self.sector_inner = float(self.get_parameter('sector_inner_deg').value)
        self.sector_outer = float(self.get_parameter('sector_outer_deg').value)
        self.laser_yaw_fallback = float(self.get_parameter('laser_yaw_fallback').value)
        self.idle_check_sec = float(self.get_parameter('idle_check_sec').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)

        # ---------------- 几何状态
        self._center = None          # (x, y)
        self._edge = None            # (x, y) 圆周上那一点
        self._radius = None
        self._theta0 = None
        self._line_locked = False    # 收满两点后为 True

        # ---------------- 运行状态
        self._executing = False
        self._progress = 0.0         # 已绕过的角度（rad）
        self._last_psi = None
        self._blocked = False
        self._miss = 0               # 连续查不到 TF 的次数
        self._last_info = None
        self._last_marker_t = 0.0

        # ---------------- 传感器 / 互斥
        self._scan = None
        self._laser_yaw_cached = None
        self._own_msgs = deque(maxlen=80)     # (t, vx, vy, wz) 自己刚发过的
        # 收到过的"不是我发的" /cmd_vel 时刻 —— 用来判断底盘是不是被别人开着。
        # ⚠ 必须非阻塞地判断（不能在服务回调里 sleep，那会把 executor 卡住，
        #   订阅回调不执行，探测等于失效）。
        self._ext_times = deque(maxlen=400)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, MARKER_QOS)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(PointStamped, self.click_topic, self._on_click, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, SENSOR_QOS)
        self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_seen, 50)
        self.create_service(Trigger, '~/clear', self._on_clear)
        self.create_service(Trigger, '~/start', self._on_start)
        self.create_service(Trigger, '~/stop', self._on_stop)

        self.create_timer(1.0 / max(self.control_rate, 1.0), self._control_tick)

        self._publish_markers()
        self.get_logger().info(
            '圆形跟随就绪: 在 RViz 用「Publish Point」点两下（话题 %s）—— '
            '第 1 点是圆心，第 2 点是圆周上一点（距离即半径）。\n'
            '  线速度 %.2f m/s，%s，急停距离 %.2f m，控制 %g Hz%s\n'
            '  几何确认后用 ~/start 走一圈（~/stop 中止，~/clear 清空）。\n'
            '  ⚠ 本节点只认 %s；画航点（直线）用的是另一个话题 %s，不会互相干扰。'
            % (self.click_topic, self.linear_vel,
               '逆时针' if self.counter_clockwise else '顺时针',
               self.stop_distance, self.control_rate,
               '，【DRY-RUN 不真正发 /cmd_vel】' if self.dry_run else '',
               self.click_topic, '/waypoint_point'))

    # ------------------------------------------------------------------ TF

    def _lookup_pose(self):
        """车在 map 系下的 (x, y, yaw)；查不到返回 None。"""
        try:
            t = self.tf_buffer.lookup_transform(
                self.frame_id, self.base_frame, RclTime(),
                timeout=Duration(seconds=0.05))
        except Exception:                                  # noqa: BLE001
            return None
        tr = t.transform.translation
        return float(tr.x), float(tr.y), _yaw_of(t.transform.rotation)

    def _laser_yaw(self):
        """laser_frame 相对 base_frame 的 yaw；TF 查不到就用兜底值。"""
        if self._laser_yaw_cached is not None:
            return self._laser_yaw_cached
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.laser_frame, RclTime(),
                timeout=Duration(seconds=0.05))
        except Exception:                                  # noqa: BLE001
            return self.laser_yaw_fallback
        self._laser_yaw_cached = _yaw_of(t.transform.rotation)
        self.get_logger().info(
            '已从 TF 读到 %s -> %s 的 yaw = %.3f rad（%.1f°），'
            '用于把扫描角换算到车体系。'
            % (self.base_frame, self.laser_frame, self._laser_yaw_cached,
               math.degrees(self._laser_yaw_cached)))
        return self._laser_yaw_cached

    # ------------------------------------------------------------------ 输入

    def _on_click(self, msg: PointStamped):
        frame = msg.header.frame_id or '(空)'
        if frame != self.frame_id:
            self.get_logger().error(
                '点的坐标系是 %r，与 %r 不一致，已忽略。'
                '请把 RViz 的 Fixed Frame 设成 %s 再点。'
                % (frame, self.frame_id, self.frame_id))
            return
        if self._executing:
            self.get_logger().warn('正在执行，忽略这个点。先 ~/stop。')
            return

        pt = (float(msg.point.x), float(msg.point.y))

        if self._line_locked:
            # 已经有一个圆了 → 这一个点当作新圆的圆心，重新开始
            self._center, self._edge, self._radius, self._theta0 = pt, None, None, None
            self._line_locked = False
            self.get_logger().info(
                '已开始一个新的圆：圆心 = (%.3f, %.3f)。再点圆周上一点定半径。' % pt)
        elif self._center is None:
            self._center = pt
            self.get_logger().info(
                '圆心已记下 = (%.3f, %.3f)。再点圆周上一点（距离即半径）。' % pt)
        else:
            self._edge = pt
            dx, dy = pt[0] - self._center[0], pt[1] - self._center[1]
            self._radius = math.hypot(dx, dy)
            self._theta0 = math.atan2(dy, dx)
            self._line_locked = True
            if self._radius < self.min_radius:
                self.get_logger().error(
                    '半径只有 %.3f m，小于下限 %.2f m，~/start 会被拒绝。'
                    % (self._radius, self.min_radius))

            omega = self.linear_vel / max(self._radius, 1e-6)
            self.get_logger().info(
                '圆已就绪: 圆心=(%.3f, %.3f) 半径=%.3f m 起点方位=%.1f° 方向=%s\n'
                '  稳态运动: vx=0, vy=%+.2f m/s, ω=%+.3f rad/s（绕一圈约 %.1f s）\n'
                '  确认画面上的圆无误后调 ~/start。'
                % (self._center[0], self._center[1], self._radius,
                   math.degrees(self._theta0),
                   '逆时针' if self.counter_clockwise else '顺时针',
                   (-self.linear_vel if self.counter_clockwise else self.linear_vel),
                   (omega if self.counter_clockwise else -omega),
                   TWO_PI * self._radius / self.linear_vel))

        self._publish_markers()

    def _on_scan(self, msg: LaserScan):
        self._scan = msg

    def _on_cmd_seen(self, msg: Twist):
        """记录 /cmd_vel 上**不是自己发的**消息 —— 用来发现"底盘被别人开着"。

        办法：自己每次发布都把 (时刻, vx, vy, wz) 记进 _own_msgs；回环回来的消息
        如果能对上就是自己的，对不上就说明有第二个发布者（多半是 nav2）。
        """
        now = time.time()
        if self._executing:
            vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
            for t, a, b, c in self._own_msgs:
                if now - t > 1.0:
                    continue
                if (abs(a - vx) < 1e-6 and abs(b - vy) < 1e-6 and abs(c - wz) < 1e-6):
                    return                            # 是我自己发的
        self._ext_times.append(now)

    def _external_cmd_vel_recent(self, window):
        now = time.time()
        return sum(1 for t in self._ext_times if now - t < window)

    # ------------------------------------------------------------------ 服务

    def _on_clear(self, _request, response):
        if self._executing:
            response.success = False
            response.message = '正在执行，先 ~/stop 再清空'
            return response
        self._center = self._edge = self._radius = self._theta0 = None
        self._line_locked = False
        response.success = True
        response.message = '已清空圆形几何'
        self.get_logger().info(response.message)
        self._publish_markers()
        return response

    def _on_stop(self, _request, response):
        if not self._executing:
            response.success = False
            response.message = '当前没有在执行'
            return response
        self._abort('收到 ~/stop')
        response.success = True
        response.message = '已停止并停车'
        return response

    def _on_start(self, _request, response):
        if self._executing:
            response.success = False
            response.message = '已经在执行了'
            return response
        if not self._line_locked or self._radius is None:
            response.success = False
            response.message = '还没有完整的圆：先用「Publish Point」点圆心 + 圆周上一点'
            return response
        if self._radius < self.min_radius:
            response.success = False
            response.message = ('半径 %.3f m 小于下限 %.2f m，拒绝执行'
                                % (self._radius, self.min_radius))
            return response

        # ① TF 必须可用
        if self._lookup_pose() is None:
            response.success = False
            response.message = ('查不到 TF %s -> %s，先把定位跑起来（slam_toolbox + rf2o）'
                                % (self.frame_id, self.base_frame))
            return response

        # ② /cmd_vel 互斥探测（非阻塞）：nav2 空闲时不发消息，导航中会持续发。
        #    判据 = 最近 idle_check_sec 秒内收到过"不是我发的"速度。
        n_ext = self._external_cmd_vel_recent(self.idle_check_sec)
        if n_ext > 0:
            response.success = False
            response.message = (
                '最近 %.1f 秒内 /cmd_vel 上有 %d 条速度消息，说明有别的节点正在驱动底盘'
                '（多半是 nav2 在导航）。请先取消所有导航目标，或改用 ~/stop 停掉它，'
                '然后再调 ~/start。' % (self.idle_check_sec, n_ext))
            self.get_logger().error(response.message)
            return response

        self._executing = True
        self._progress = 0.0
        self._last_psi = None
        self._blocked = False
        self._miss = 0
        self._own_msgs.clear()

        r = self._radius
        stop_d = float(self.get_parameter('stop_distance').value)
        slow_d = float(self.get_parameter('slow_distance').value)
        response.success = True
        response.message = ('开始沿圆跟随：R=%.3f m，线速 %.2f m/s，一圈约 %.1f s'
                            % (r, self.linear_vel, TWO_PI * r / self.linear_vel))
        self.get_logger().info(
            '=== 开始圆形跟随 ===\n'
            '  R=%.3f m  线速度=%.2f m/s  %s  预计一圈 %.1f s\n'
            '  控制律: vy = %+.3f m/s(稳态), ω = k_θ·e + v_t·cos(e)/d, vx = k_r·(d-R)\n'
            '  急停距离 %.2f m，减速距离 %.2f m%s'
            % (r, self.linear_vel,
               '逆时针' if self.counter_clockwise else '顺时针',
               TWO_PI * r / self.linear_vel,
               (-self.linear_vel if self.counter_clockwise else self.linear_vel),
               stop_d, slow_d,
               '  【DRY-RUN】' if self.dry_run else ''))
        return response

    def _abort(self, why):
        self._executing = False
        for _ in range(6):                 # 连发几拍零速，确保底盘收到
            self._publish_cmd(0.0, 0.0, 0.0, force=True)
            time.sleep(0.02)
        self.get_logger().warn('已停止圆形跟随（%s）' % why)
        self._publish_markers()

    # ------------------------------------------------------------------ 控制

    def _control_tick(self):
        if not self._executing:
            return

        # ---- 运行中互斥：如果别的节点也在发速度，立刻让出底盘
        n_ext = self._external_cmd_vel_recent(0.5)
        if n_ext >= 5:
            self.get_logger().error(
                '执行期间最近 0.5 秒内 /cmd_vel 上出现 %d 条**不是我发的**速度 —— '
                '有第二个发布者（多半是 nav2 的 controller_server / behavior_server）'
                '在抢底盘。已停车。' % n_ext)
            self._abort('检测到第二个速度发布者，防止抢底盘')
            return

        pose = self._lookup_pose()
        if pose is None:
            self._miss += 1
            if self._miss == 20:
                self.get_logger().error('连续 1 秒查不到 TF，已停车。')
                self._abort('TF 丢失')
            return
        self._miss = 0

        px, py, yaw = pose
        cx, cy = self._center
        r_target = self._radius

        vx, vy, omega, cinfo = plan_circle_command(
            px, py, yaw, (cx, cy), r_target, self.linear_vel,
            self.counter_clockwise, self.k_theta, self.k_radial,
            self.max_forward_vel, self.max_lateral_vel, self.max_angular_vel)
        if cinfo['degenerate']:
            self._abort('车与圆心重合，几何退化')
            return
        d, e = cinfo['d'], cinfo['e']

        # ---- 进度：按车绕圆心的方位角累积
        psi = math.atan2(py - cy, px - cx)
        if self._last_psi is None:
            self._last_psi = psi
        else:
            dpsi = _clamp(_wrap(psi - self._last_psi), -0.5, 0.5)   # 防跳变
            self._last_psi = psi
            self._progress += dpsi if self.counter_clockwise else -dpsi

        # ---- 安全层
        factor, obs = self._obstacle_factor()
        stop_d = float(self.get_parameter('stop_distance').value)
        if factor <= 0.0:
            self._publish_cmd(0.0, 0.0, 0.0)
            if not self._blocked:
                self.get_logger().warn(
                    '运动前沿 %.2f m 处有障碍（阈值 %.2f m），已急停。'
                    '清障后会自动继续；要终止请 ~/stop。'
                    '想临时放宽：ros2 param set %s stop_distance 0.25'
                    % (obs, stop_d, self.get_fully_qualified_name()))
                self._blocked = True
        else:
            if self._blocked:
                self.get_logger().info('障碍已清除，继续跟随。')
                self._blocked = False
            self._publish_cmd(vx * factor, vy * factor, omega * factor)

        info = {'d': d, 'r': r_target, 'e': e, 'vx': vx, 'vy': vy, 'wz': omega,
                'factor': factor, 'obs': obs, 'blocked': self._blocked,
                'progress': self._progress}

        # ---- 一圈走完？
        if self._progress >= TWO_PI:
            self._executing = False
            for _ in range(6):
                self._publish_cmd(0.0, 0.0, 0.0, force=True)
                time.sleep(0.02)
            self.get_logger().info(
                '=== 圆形跟随完成 === 已绕满 %.1f°，用时约 %.1f s，已停车。'
                % (math.degrees(self._progress),
                   TWO_PI * r_target / self.linear_vel))
            self._publish_markers(info)
            return

        # ---- Marker 限到 5 Hz，别在 20 Hz 控制环里重建整幅图
        now = time.time()
        if now - self._last_marker_t > 0.2:
            self._last_marker_t = now
            self._publish_markers(info)
        self._last_info = info

    # ------------------------------------------------------------------ 安全

    def _obstacle_factor(self):
        """只看**切向扇区**（车体 ±y 方向）—— 那是真正会撞上的方向。

        故意排除车头方向：车头指向圆心，绕柱巡检时圆心就是柱子，
        把 0° 方向算进来会导致一启动就急停。
        返回 (因子, 最近距离)；因子 0 表示急停。

        ⚠ 阈值**每拍热读参数**，现场可 `ros2 param set /path_follower stop_distance 0.25`
        立即生效，不必重编译重启。
        """
        scan = self._scan
        if scan is None or not scan.ranges:
            return 1.0, float('inf')

        stop_d = float(self.get_parameter('stop_distance').value)
        slow_d = float(self.get_parameter('slow_distance').value)
        lo = float(self.get_parameter('sector_inner_deg').value)
        hi = float(self.get_parameter('sector_outer_deg').value)

        yaw_off = self._laser_yaw()
        amin, ainc = scan.angle_min, scan.angle_increment
        best = float('inf')

        for i, raw in enumerate(scan.ranges):
            dd = raw
            if not math.isfinite(dd) or dd < scan.range_min or dd > scan.range_max:
                continue
            a_base = math.degrees(_wrap(amin + i * ainc + yaw_off))
            deg = abs(a_base)
            if lo <= deg <= hi and dd < best:
                best = dd

        if best < stop_d:
            return 0.0, best
        if best < slow_d:
            span = max(slow_d - stop_d, 1e-6)
            return 0.35 + 0.65 * (best - stop_d) / span, best
        return 1.0, best

    # ------------------------------------------------------------------ 输出

    def _publish_cmd(self, vx, vy, wz, force=False):
        if self.dry_run and not force:
            now = time.time()
            if now - getattr(self, '_last_dry', 0.0) > 0.5:
                self._last_dry = now
                self.get_logger().info(
                    '[DRY-RUN] 会发 /cmd_vel: vx=%+.3f vy=%+.3f wz=%+.3f'
                    % (vx, vy, wz))
            return
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(vx), float(vy), float(wz)
        self._own_msgs.append((time.time(), msg.linear.x, msg.linear.y, msg.angular.z))
        self.cmd_pub.publish(msg)

    # ------------------------------------------------------------------ 可视化

    def _publish_markers(self, info=None):
        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()

        wipe = Marker()
        wipe.header.frame_id = self.frame_id
        wipe.header.stamp = stamp
        wipe.action = Marker.DELETEALL
        array.markers.append(wipe)

        if self._center is None:
            self._add_text(array, stamp, 0,
                           '圆形路径：点 1 = 圆心，点 2 = 圆周上一点',
                           (0.0, 0.4, 0.6), 0.22, (1.0, 0.9, 0.2))
            self.marker_pub.publish(array)
            return

        cx, cy = self._center
        cline = (0.2, 0.5, 1.0)

        self._add_sphere(array, stamp, 1, cx, cy, 0, 0.07, cline)
        self._add_text(array, stamp, 2, '圆心', (cx, cy, 0.22), 0.16, (0.7, 0.85, 1.0))

        if self._radius is None:
            self._add_text(array, stamp, 3, '再点圆周上一点定半径',
                           (cx, cy + 0.42, 0.22), 0.18, (1.0, 0.9, 0.2))
            self.marker_pub.publish(array)
            return

        r, t0 = self._radius, self._theta0
        ccw = self.counter_clockwise
        direction = COUNTER_CLOCKWISE if ccw else CLOCKWISE

        # 圆周
        ring = Marker()
        ring.header.frame_id = self.frame_id
        ring.header.stamp = stamp
        ring.ns = 'circle'
        ring.id = 10
        ring.type = Marker.LINE_STRIP
        ring.action = Marker.ADD
        ring.scale.x = 0.02
        ring.color.r, ring.color.g, ring.color.b, ring.color.a = cline[0], cline[1], cline[2], 1.0
        ring.pose.orientation.w = 1.0
        for i in range(SEGMENTS + 1):
            th = t0 + direction * TWO_PI * i / SEGMENTS
            ring.points.append(_pt(cx + r * math.cos(th), cy + r * math.sin(th)))
        array.markers.append(ring)

        # 半径线
        radial = Marker()
        radial.header.frame_id = self.frame_id
        radial.header.stamp = stamp
        radial.ns = 'radius'
        radial.id = 11
        radial.type = Marker.LINE_STRIP
        radial.action = Marker.ADD
        radial.scale.x = 0.01
        radial.color.r, radial.color.g, radial.color.b, radial.color.a = 0.6, 0.6, 0.6, 0.9
        radial.pose.orientation.w = 1.0
        radial.points.append(_pt(cx, cy))
        radial.points.append(_pt(cx + r * math.cos(t0), cy + r * math.sin(t0)))
        array.markers.append(radial)
        self._add_text(array, stamp, 12, 'R = %.3f m' % r,
                       (cx + 0.5 * r * math.cos(t0), cy + 0.5 * r * math.sin(t0), 0.14),
                       0.15, (0.85, 0.85, 0.85))

        # 起点 + 车头指向圆心
        sx, sy = cx + r * math.cos(t0), cy + r * math.sin(t0)
        self._add_sphere(array, stamp, 13, sx, sy, 0.02, 0.06, (1.0, 0.6, 0.1))
        self._add_text(array, stamp, 14, '起点', (sx, sy, 0.20), 0.15, (1.0, 0.8, 0.4))
        self._add_arrow(array, stamp, 15, sx, sy, 0.06, t0 + math.pi,
                        min(0.35, 0.6 * r), (1.0, 0.6, 0.1))
        self._add_text(array, stamp, 16, '车头 → 圆心', (sx, sy, 0.34), 0.13, (1.0, 0.8, 0.4))

        # 运动方向（切线）
        self._add_arrow(array, stamp, 17, sx, sy, 0.10,
                        (t0 + math.pi / 2.0) if ccw else (t0 - math.pi / 2.0),
                        min(0.40, 0.7 * r), (0.1, 0.9, 0.9))

        # 状态文字
        omega = self.linear_vel / r
        self._add_text(array, stamp, 18,
                       '圆 R=%.3f m · %s · 线速 %.2f m/s · ω %.2f rad/s · 一圈约 %.1f s'
                       % (r, '逆时针' if ccw else '顺时针', self.linear_vel, omega,
                          TWO_PI * r / self.linear_vel),
                       (cx, cy + r + 0.35, 0.60), 0.20, (0.75, 0.95, 0.75))

        # ---- 运行状态
        if info is None and self._executing:
            info = self._last_info
        if info is not None:
            pct = _clamp(info['progress'] / TWO_PI, 0.0, 1.0) * 100.0
            color = (1.0, 0.3, 0.3) if info.get('blocked') else (0.3, 1.0, 0.4)
            state = '已急停（障碍）' if info.get('blocked') else '跟随中'
            self._add_text(
                array, stamp, 20,
                '%s · 已绕 %.0f%% · 半径偏差 %+.3f m · 朝向误差 %+.1f° · 障碍 %.2f m'
                % (state, pct, info['d'] - info['r'], math.degrees(info['e']),
                   info['obs'] if math.isfinite(info['obs']) else -1.0),
                (cx, cy - r - 0.30, 0.60), 0.18, color)
            self._add_text(
                array, stamp, 21,
                '指令 vx=%+.3f  vy=%+.3f  ω=%+.3f  （降速系数 %.2f）'
                % (info['vx'], info['vy'], info['wz'], info['factor']),
                (cx, cy - r - 0.50, 0.60), 0.15, color)
        elif not self._executing:
            self._add_text(array, stamp, 22, '待命：~/start 开始走一圈',
                           (cx, cy - r - 0.30, 0.60), 0.16, (1.0, 0.9, 0.2))

        self.marker_pub.publish(array)

    def _add_sphere(self, array, stamp, mid, x, y, z, scale, rgb):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp
        m.ns = 'geom'
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = float(scale)
        m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], 1.0
        array.markers.append(m)

    def _add_text(self, array, stamp, mid, text, pos, height, rgb):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp
        m.ns = 'geom_text'
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (
            float(pos[0]), float(pos[1]), float(pos[2]))
        m.pose.orientation.w = 1.0
        m.scale.z = float(height)
        m.text = text
        m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], 1.0
        array.markers.append(m)

    def _add_arrow(self, array, stamp, mid, x, y, z, yaw, length, rgb):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp
        m.ns = 'geom_arrow'
        m.id = mid
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        m.pose.orientation.z = math.sin(yaw * 0.5)
        m.pose.orientation.w = math.cos(yaw * 0.5)
        m.scale.x, m.scale.y, m.scale.z = float(length), 0.05, 0.05
        m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], 1.0
        array.markers.append(m)


def _pt(x, y, z=0.0):
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def main(args=None):
    rclpy.init(args=args)
    run_node(PathFollower())


if __name__ == '__main__':
    main()
