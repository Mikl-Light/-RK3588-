#!/usr/bin/env python3
"""航点巡航 + 到点拍照 + 巡检报告

为什么自己实现而不是直接用 nav2 的 waypoint_follower + PhotoAtWaypoint：
  1. 需要**逐航点**的失败策略（重试 N 次后「跳过」还是「终止」），nav2 只有全局的
     stop_on_failure 开关，颗粒度不够。
  2. 需要把照片与航点、尝试次数、耗时、剩余距离**关联成一份可归档的巡检报告**，
     而不是拍完就散落在目录里。
  3. 需要在到点后等待若干秒再拍（车体停稳、自动曝光收敛），否则照片常是糊的。

航点文件格式（YAML）：
    loop: false
    waypoints:
      - name: shelf_a          # 必填，会进文件名与报告
        x: 1.20                # 必填
        y: -0.35               # 必填
        yaw: 1.57              # 可选，到点朝向（弧度）；不填则不对朝向提要求
        settle: 1.5            # 可选，覆盖全局 settle_time
        note: "货架A正面"       # 可选，进报告

航点来源（两种，二选一，**都是"一次性给出全部点"**）：
  A. 文件：把全部航点预先写进 route_file（默认 config/waypoints.yaml），一次触发跑完。
     适合已知固定路径、要版本管理的场合。
  B. RViz 连点：在 RViz 里用「2D Goal Pose」工具**连续点多个点**（每个点可以拖出朝向），
     节点按点击顺序把它们收进一个列表；点完再调一次 ~/start 就**依次**跑完：
     每个点单独发一个 NavigateToPose 目标 → 阻塞等到 SUCCEEDED → 停 settle 秒 → 拍照
     → 才去下一个。绝不是"直接开到最后一个点"。
     - 收集开关 `collect_mode` 可以**运行中随时切**：
         ros2 param set /inspection_route collect_mode true
       不必重启。关着的时候点击会被忽略，并在日志里提示你打开。
     - 已收集的点会实时画到 RViz 的 `inspection_waypoints` 话题上（橙色箭头+编号，
       正在前往的那个变绿）。加 Marker 显示即可看到，免得盲点。
     ⚠ 收集期间不会自己动 —— 本节点是这个话题唯一的消费者；但如果你同时跑着旧包的
       goal_pose_bridge，它会立刻把点转成导航目标，务必先停掉它。
     收集结果会写进 collect_file，可复查、也可当 route_file 复用。

  两种来源同时存在时**以已收集的列表为准**（更符合"我刚点的点"的直觉），
  用 ~/clear 清空列表即回落到文件。

  C. **直线/连点 + 车头自动对齐行驶方向**（`auto_heading:=true`）
     走的是同一套"逐点阻塞导航 + 到点拍照"，但**朝向不用你指定**，开跑前自动算：
        第 1 个点 ← 从"车当前位姿"指向它的方向（就是我开过去的方向）
        第 i 个点 ← 从第 i-1 个点指向它的方向（"我怎么到这里的"）
     于是可以直接用**「Publish Point」连点**（只发点、不带朝向，点起来最快），
     也可以继续用「2D Goal Pose」（拖出的朝向会被覆盖）。
     若 `auto_heading=false`（默认）则沿用你拖出的朝向，行为与原来一致。

运行方式：
  * 服务 ~/start 触发一轮巡检，~/stop 中止（会取消当前导航目标）
  * 服务 ~/clear 清空已收集的航点（不影响 route_file）
  * autostart 参数设为 true 则启动 start_delay 秒后自动开始
"""

import json
import math
import os
import threading
import time
import traceback

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion, Twist
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import FollowPath, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time as RclTime
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from . import run_node

SENSOR_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)

# 代价地图是 latched（TRANSIENT_LOCAL）发布的，晚订阅者也能立刻拿到最后一张
COSTMAP_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

STATUS_TEXT = {
    GoalStatus.STATUS_UNKNOWN: 'unknown',
    GoalStatus.STATUS_ACCEPTED: 'accepted',
    GoalStatus.STATUS_EXECUTING: 'executing',
    GoalStatus.STATUS_CANCELING: 'canceling',
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_CANCELED: 'canceled',
    GoalStatus.STATUS_ABORTED: 'aborted',
}


def quat_from_yaw(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


def build_straight_path(x0, y0, x1, y1, yaw, step=0.05):
    """把从 (x0,y0) 到 (x1,y1) 的直线离散成路径点（**纯函数**，可离线验算）。

    每一点的朝向都用同一个 `yaw`（即"沿行驶方向"）。
    **末点严格等于目标点**（不因步长取整而停在半路）。
    起点终点重合时返回单点列表。

    返回 `[(x, y, yaw), ...]`。
    """
    dx, dy = float(x1) - float(x0), float(y1) - float(y0)
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return [(float(x1), float(y1), float(yaw))]
    n = max(int(math.ceil(dist / max(float(step), 0.01))), 1)
    return [(float(x0) + dx * (i / float(n)),
             float(y0) + dy * (i / float(n)),
             float(yaw)) for i in range(n + 1)]


def wrap_to_pi(a):
    """把角度归一到 (-pi, pi]。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


# ---------------------------------------------------------------- 障碍判定

FOLLOW_OBSTACLE = 'obstacle'
FOLLOW_STUCK = 'stuck'
FOLLOW_OTHER = 'other'

# nav2_msgs/action/FollowPath 的 error_code 常量（Jazzy）
ERR_PATIENCE = 104            # PATIENCE_EXCEEDED：控制环很久给不出有效控制
ERR_NO_PROGRESS = 105         # FAILED_TO_MAKE_PROGRESS
ERR_NO_VALID_CONTROL = 106    # NO_VALID_CONTROL：RPP 碰撞检测抛的就是这个

_OBSTACLE_WORDS = ('collision', 'obstacle', 'blocked', 'inscribed', 'lethal')


def classify_follow_failure(error_code, error_msg):
    """把 FollowPath 的失败归类（**纯函数**，可离线验算）。

    为什么要分类：严格直线模式**不会绕行**，所以"前方有东西"必须变成"停车"，
    而不能像多点巡航那样重试/跳过 —— 否则车会一次次朝同一个障碍冲过去。

    - `106 (NO_VALID_CONTROL)`：RPP 碰撞检测抛出的，文案是
      `RegulatedPurePursuitController detected collision ahead!` ⇒ **前方有障碍**。
    - `104 / 105`：控制环长时间产生不出有效控制。可能被挡住（也可能定位问题），
      严格直线下按"被挡住"处理更安全。
    - 文案里出现 collision/obstacle 等词时也归为障碍（对版本差异容错）。

    返回 `(kind, 说明)`，kind ∈ `{'obstacle', 'stuck', 'other'}`。
    """
    code = int(error_code) if error_code else 0
    text = (error_msg or '').lower()
    looks_blocked = any(w in text for w in _OBSTACLE_WORDS)
    # 文案里明确提到碰撞/障碍 ⇒ 就是障碍（**不看 code**：code 可能缺失或为 0，
    # 而这句话本身就是最直接的证据）。
    if code == ERR_NO_VALID_CONTROL or looks_blocked:
        return FOLLOW_OBSTACLE, '前方有障碍，控制器无法产生有效控制'
    if code in (ERR_PATIENCE, ERR_NO_PROGRESS):
        return FOLLOW_STUCK, '走不动（控制器长时间产生不出有效控制，可能被挡住）'
    if code:
        return FOLLOW_OTHER, '控制器返回错误 %d' % code
    return FOLLOW_OTHER, 'FollowPath 未成功（无 error_code）'


def nearest_scan_obstacle(ranges, angle_min, angle_increment, laser_yaw,
                          half_deg=60.0):
    """在车头 ±`half_deg` 内找最近的有效回波（**纯函数**，可离线验算）。

    只求一个"障碍大概在哪边、多远"的数字，写进日志让人能立刻定位现场，
    所以不做扇区分级。坐标系是**车体系**（用 laser→base 的 yaw 把扫描角转过来）。
    返回 `(距离, 方位角(度))`；没有有效回波时返回 `(None, None)`。
    """
    best = None
    best_deg = None
    for i, raw in enumerate(ranges):
        try:
            d = float(raw)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(d) or d <= 0.0:
            continue
        deg = math.degrees(wrap_to_pi(angle_min + i * angle_increment + laser_yaw))
        if abs(deg) <= half_deg and (best is None or d < best):
            best, best_deg = d, deg
    return best, best_deg


def path_blocked(snapshot, clear_distance, scan_missing=False):
    """判断"前方现在还不能走吗"（**纯函数**，可离线验算）。

    返回 `True` = 还堵着；`False` = **已经清空，可以走**；`None` = **无法判断**。

    三种情况要分清楚，混起来会出两种坏事：
    - `scan_missing=True`（压根没收到激光）⇒ `None`。
      千万别当成"已清空" —— 那会让调用方疯狂重发目标（忙等、日志爆掉）。
    - 有激光、但视野内**没有有效回波** ⇒ `False`：这是明确的"前方没东西"。
    - 有回波且比 `clear_distance` 近 ⇒ `True`。

    `clear_distance` 故意取得比触发点大，形成**迟滞**：否则会
    "清一点 → 走两步 → 又被拦"反复抖。
    """
    if scan_missing:
        return None
    if snapshot is None:
        return False
    return float(snapshot['distance']) <= float(clear_distance)


INSCRIBED_COST = 99        # nav2 costmap 翻译表：>=99 就是内切/致命
UNKNOWN_COST = 255         # OccupancyGrid 的 -1 读成 int8 再取绝对值就是 255


def corridor_blocked(data, width, height, resolution, origin_x, origin_y,
                     robot_x, robot_y, robot_yaw, length, half_width,
                     fatal_cost=INSCRIBED_COST):
    """检查"车正前方一条窄走廊"里有没有致命/内切代价（**纯函数**，可离线验算）。

    ★ 为什么不用"最近的激光回波"做判据：那件事**无法区分两种情况** ——
      · **障碍物挡在路中间**（该停下等）
      · **环境本来就窄**（比如两侧货架只有 0.4 m，但车头要去的方向是通的，**不该停**）
    用"±60° 内最近回波"的话，第二种会永远不满足 ⇒ 无限等待。

    这里只看**车要走的那条窄带**（前向 `length`、横向 ±`half_width`），
    语义与控制器一致，也就不会被侧面的东西误伤。

    `data/width/height/resolution/origin_*` 直接来自 `nav_msgs/OccupancyGrid`；
    机器人位姿是**在该代价地图坐标系下**的位姿。
    **未知格（255）不算障碍** —— 否则地图边缘会被永远判成"堵"。
    返回 `(是否堵, 走廊内最坏代价, 最坏位置(前向, 侧向))`。
    """
    worst = -1
    worst_at = None
    if not data or width <= 0 or height <= 0 or resolution <= 0:
        return False, worst, None
    n_along = max(int(math.ceil(float(length) / resolution)), 1)
    n_across = max(int(math.ceil(float(half_width) / resolution)), 1)
    cos_y, sin_y = math.cos(robot_yaw), math.sin(robot_yaw)
    for i in range(n_along + 1):
        along = min(i * resolution, float(length))
        for j in range(-n_across, n_across + 1):
            across = j * resolution
            if abs(across) > half_width:
                continue
            wx = robot_x + along * cos_y - across * sin_y
            wy = robot_y + along * sin_y + across * cos_y
            cx = int((wx - origin_x) / resolution)
            cy = int((wy - origin_y) / resolution)
            if not (0 <= cx < width and 0 <= cy < height):
                continue
            v = int(data[cy * width + cx])
            if v < 0:
                v = UNKNOWN_COST
            if v == UNKNOWN_COST:
                continue                    # 未知不是障碍
            if v > worst:
                worst = v
                worst_at = (round(along, 3), round(across, 3))
    return (worst >= fatal_cost), worst, worst_at


def compute_headings(points, pose=None):
    """按"车头与行驶方向一致"给每个待定的航点算朝向（**纯函数**，可离线验算）。

    points: `[{'name','x','y','yaw'}, ...]`；`yaw is None` 表示这个点待算
            （RViz「Publish Point」点的位就没有朝向）。
    pose:   车的当前位姿 `(x, y, yaw)`，只用于第 1 个点；None 表示 TF 查不到。

    规则：
        第 1 个点 ← 从"车当前位姿"指向它（就是我开过去的方向）
        第 i 个点 ← 从第 i-1 个点指向它（"我怎么到这里的"）
    用户自己给过朝向的点**原样保留**，不覆盖。

    返回 `(yaws, notes)`：yaws 与 points 等长；notes 是 `[(level, text)]`，
    交给调用方打日志（这样这里不依赖 rclpy，能单独跑）。
    """
    yaws = [None if p.get('yaw') is None else float(p['yaw']) for p in points]
    notes = []

    for i, wp in enumerate(points):
        name = wp.get('name') or 'p%02d' % (i + 1)
        if yaws[i] is not None:
            continue

        if i == 0 and pose is None and len(points) <= 1:
            # 只有一个点、又拿不到车位姿 ⇒ 根本无从谈"行驶方向"
            yaws[i] = 0.0
            notes.append(('warn',
                          '航点 %s 是唯一的点、且 TF 查不到车位姿，无法确定行驶方向，'
                          '朝向取 0°（朝东）。建议先让定位跑起来，'
                          '或再点一个点来定方向。' % name))
            continue

        if i == 0:
            if pose is not None:
                dx = float(wp['x']) - pose[0]
                dy = float(wp['y']) - pose[1]
                src = '车当前位姿 → 该点'
            elif len(points) > 1:
                dx = float(points[1]['x']) - float(wp['x'])
                dy = float(points[1]['y']) - float(wp['y'])
                src = '该点 → 下一个点（TF 不可用，退回这个方向）'
            else:
                dx = dy = 0.0
                src = '无参考'
        else:
            prev = points[i - 1]
            dx = float(wp['x']) - float(prev['x'])
            dy = float(wp['y']) - float(prev['y'])
            src = '上一个点 → 该点'

        if math.hypot(dx, dy) < 1e-3:
            # 两点重合 ⇒ 路径方向不存在，不能硬算 atan2(0,0)
            if i > 0 and yaws[i - 1] is not None:
                yaws[i] = yaws[i - 1]
                notes.append(('warn', '%s 与上一个点重合，朝向无法由路径确定，'
                                      '沿用 %.1f°' % (name, math.degrees(yaws[i]))))
            elif pose is not None:
                yaws[i] = float(pose[2])
                notes.append(('warn', '%s 与上一个点重合，改用当前车头朝向 %.1f°'
                                      % (name, math.degrees(yaws[i]))))
            else:
                yaws[i] = 0.0
                notes.append(('warn', '%s 与上一个点重合且无参考，朝向取 0°' % name))
        else:
            yaws[i] = math.atan2(dy, dx)
            notes.append(('info', '航点 %s 朝向自动对齐行驶方向: %.1f°（来源：%s）'
                                  % (name, math.degrees(yaws[i]), src)))
    return yaws, notes


class InspectionRoute(Node):

    def __init__(self):
        super().__init__('inspection_route')

        self.declare_parameter('route_file', '')
        self.declare_parameter('frame_id', 'map')
        # auto_heading 要靠 TF 查车位姿；base_frame 一起做成参数，便于测试时隔离
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('nav_action', 'navigate_to_pose')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('save_dir', os.path.expanduser('~/inspection_shots'))
        self.declare_parameter('jpeg_quality', 90)
        self.declare_parameter('settle_time', 1.0)
        self.declare_parameter('max_retries', 1)
        self.declare_parameter('navigate_timeout', 120.0)
        self.declare_parameter('on_failure', 'skip')
        self.declare_parameter('loop', False)
        self.declare_parameter('autostart', False)
        self.declare_parameter('start_delay', 5.0)
        # ---- RViz 连点收集 ----
        self.declare_parameter('collect_mode', False)
        self.declare_parameter('collect_topic', '/goal_pose')
        # auto_heading=true 时可以只用「Publish Point」连点（不带朝向，点得快）
        # ⚠ 必须与 path_follower 的 click_topic 分开！两者都订阅的话，
        #   点直线时 path_follower 会把第 1 点当圆心、第 2 点当圆周点 → 画出一个圆。
        self.declare_parameter('point_topic', '/waypoint_point')
        # ★ 车头自动对齐行驶方向：开跑前按"我怎么到这里的"算每点朝向，
        #   第 1 个点用"车当前位姿 -> 该点"的方向。false 则沿用拖出的朝向。
        self.declare_parameter('auto_heading', False)
        self.declare_parameter('collect_file',
                               os.path.join(os.path.expanduser('~/inspection_shots'),
                                            'collected_waypoints.yaml'))
        self.declare_parameter('marker_topic', 'inspection_waypoints')
        # 与 nav2_params.yaml 里 goal_checker.xy_goal_tolerance 保持一致：
        # 相邻航点间距小于它时，nav2 会直接判定"已到达"，车不会真的过去。
        self.declare_parameter('aim_tolerance', 0.20)
        # 标点期间监测底盘速度：有别的客户端在发目标（RViz 的 Nav2 Goal 工具 /
        # Navigation 2 面板）时，这里会立刻报出来。
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_epsilon', 0.01)
        # 到点后除了 settle 秒，最多再等多久去拿一帧"到点之后"的图像
        self.declare_parameter('capture_timeout', 10.0)

        # ★ 逐点之间"怎么走"：
        #   'navigate' = nav2 全局规划（NavigateToPose）——会绕障，但路径会被膨胀层推偏
        #   'straight' = 严格沿你连的直线（/follow_path，**不过全局规划器**）
        self.declare_parameter('segment_mode', 'navigate')
        self.declare_parameter('straight_step', 0.05)          # 直线离散步长 m
        self.declare_parameter('follow_action', 'follow_path')
        self.declare_parameter('follow_controller_id', '')     # 空 = 用默认 controller

        # ★ 严格直线段"撞上东西"时怎么办 —— 只作用于 segment_mode='straight'：
        #   该模式**不会绕行**（这正是它存在的意义）。
        #   'wait' = **停车等障碍消失，清空后自动继续这一段**（默认）
        #   'stop' = 停车 + 中止整轮巡检
        #   'skip' = 按旧逻辑重试，然后跳过该点
        self.declare_parameter('on_obstacle', 'wait')
        self.declare_parameter('scan_topic', '/scan')          # 判断"清空了吗" + 报告障碍方位
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('obstacle_fov_deg', 60.0)       # 看车头 ±多少度
        # 判"障碍已消失"的门槛。**要比触发点大**（迟滞），否则会"清一点→走两步→又停"。
        self.declare_parameter('clear_distance', 0.55)
        self.declare_parameter('obstacle_poll_period', 0.3)    # 等待时多久看一次
        self.declare_parameter('obstacle_log_period', 10.0)    # 等待进度日志的节流
        self.declare_parameter('obstacle_wait_timeout', 0.0)   # 0 = 一直等（~/stop 可中止）
        self.declare_parameter('blind_retry_period', 3.0)      # 拿不到激光时，隔多久让控制器试一次
        # ★ 判"能不能走了"的**主判据是代价地图**（与控制器同源），激光只用来区分
        #   "真障碍"和"残留像素"。只看车前方一条窄走廊 —— 否则窄通道里会永远等。
        self.declare_parameter('use_costmap', True)
        self.declare_parameter('costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('clear_costmap_service',
                               '/local_costmap/clear_entirely_local_costmap')
        self.declare_parameter('corridor_len', 0.60)           # 走廊长度 m
        self.declare_parameter('corridor_half_width', 0.16)    # 走廊半宽 m
        self.declare_parameter('wait_fov_deg', 25.0)           # 激光只看车头 ± 多少度
        self.declare_parameter('max_futile_probes', 6)         # 清图后仍走不通的次数上限
        self.declare_parameter('clear_settle', 0.4)            # 清图后等多久再读

        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame').value
        self.image_topic = self.get_parameter('image_topic').value
        self.save_dir = self.get_parameter('save_dir').value
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.max_retries = int(self.get_parameter('max_retries').value)
        self.navigate_timeout = float(self.get_parameter('navigate_timeout').value)
        self.on_failure = str(self.get_parameter('on_failure').value).lower()
        self.loop = bool(self.get_parameter('loop').value)
        self.route_file = self.get_parameter('route_file').value
        self.collect_mode = bool(self.get_parameter('collect_mode').value)
        self.collect_topic = self.get_parameter('collect_topic').value
        self.point_topic = self.get_parameter('point_topic').value
        self.auto_heading = bool(self.get_parameter('auto_heading').value)
        self.collect_file = os.path.expanduser(self.get_parameter('collect_file').value)
        self.marker_topic = self.get_parameter('marker_topic').value
        self.aim_tolerance = float(self.get_parameter('aim_tolerance').value)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_vel_epsilon = float(self.get_parameter('cmd_vel_epsilon').value)
        self.capture_timeout = float(self.get_parameter('capture_timeout').value)
        self.straight_step = float(self.get_parameter('straight_step').value)
        self.follow_controller_id = self.get_parameter('follow_controller_id').value
        self.on_obstacle = str(self.get_parameter('on_obstacle').value).lower()
        self.scan_topic = self.get_parameter('scan_topic').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.obstacle_fov_deg = float(self.get_parameter('obstacle_fov_deg').value)
        self.clear_distance = float(self.get_parameter('clear_distance').value)
        self.obstacle_poll_period = float(self.get_parameter('obstacle_poll_period').value)
        self.obstacle_log_period = float(self.get_parameter('obstacle_log_period').value)
        self.obstacle_wait_timeout = float(self.get_parameter('obstacle_wait_timeout').value)
        self.blind_retry_period = float(self.get_parameter('blind_retry_period').value)
        self.use_costmap = bool(self.get_parameter('use_costmap').value)
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.clear_costmap_service = self.get_parameter('clear_costmap_service').value
        self.corridor_len = float(self.get_parameter('corridor_len').value)
        self.corridor_half_width = float(self.get_parameter('corridor_half_width').value)
        self.wait_fov_deg = float(self.get_parameter('wait_fov_deg').value)
        self.max_futile_probes = int(self.get_parameter('max_futile_probes').value)
        self.clear_settle = float(self.get_parameter('clear_settle').value)

        self._bridge = CvBridge()
        self._latest_frame = None
        self._latest_frame_time = None
        self._frame_lock = threading.Lock()

        self._running = False
        self._cancel_requested = False
        self._current_goal = None
        self._goal_lock = threading.Lock()
        self._last_distance = None
        self._worker = None
        self._collected = []
        self._collect_lock = threading.Lock()
        self._waypoints = []
        self._active_index = None
        self._last_ignore_warn = 0.0
        self._last_cmdvel_warn = 0.0
        # "不是我发的"非零速度的时刻 —— 用来判断开跑前底盘是不是正被别人开着
        # （典型：圆跟随器正在跑、或 nav2 正在导航）。必须非阻塞地判断，
        # 不能在服务回调里 sleep：那会把回调卡住，哨兵根本收不到消息。
        self._ext_cmd_times = []

        # 障碍短路：一旦置位，本轮巡检立刻收尾（不再重试、不再去下一个点）。
        self._obstacle_hit = False
        self._obstacle_info = None
        # 本段"停车等障碍"的累计（供报告用；每次 _visit 前重置）
        self._seg_wait = {'count': 0, 'seconds': 0.0, 'clears': 0}
        # 最近一帧代价地图 + 最近一次走廊判读（供日志/报告用）
        self._costmap = None
        self._costmap_last = None
        self._costmap_frame_warned = False
        # 最近一帧激光，只用于"障碍在哪边"的报告
        self._scan = None
        self._laser_yaw_cached = None
        self._zero_cmd = Twist()          # 停车用（全 0）

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._client = ActionClient(self, NavigateToPose,
                                    self.get_parameter('nav_action').value)
        self._follow_client = ActionClient(self, FollowPath,
                                           self.get_parameter('follow_action').value)

        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        # 主动停车用：检测到障碍 / 中止时连发几条零速。
        # （chassis_bridge 的 0.5s 看门狗本来也会兜底停车，这里让它更干脆、更可预期。）
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Image, self.image_topic, self._on_image, SENSOR_QOS)
        self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, SENSOR_QOS)
        if self.use_costmap:
            self.create_subscription(OccupancyGrid, self.costmap_topic,
                                     self._on_costmap, COSTMAP_QOS)
            self._clear_client = self.create_client(ClearEntireCostmap,
                                                    self.clear_costmap_service)
        else:
            self._clear_client = None
        self.create_service(Trigger, '~/start', self._on_start)
        self.create_service(Trigger, '~/stop', self._on_stop)
        self.create_service(Trigger, '~/clear', self._on_clear)

        # 收集订阅**始终**建立，是否真的收集在回调里按 collect_mode 现场判断。
        # 这样运行中可以用 `ros2 param set ... collect_mode true` 直接切换，不必重启栈。
        self.create_subscription(PoseStamped, self.collect_topic,
                                 self._on_collect_pose, 10)
        # 不带朝向的「Publish Point」也收 —— auto_heading 时点起来最快
        self.create_subscription(PointStamped, self.point_topic,
                                 self._on_collect_point, 10)

        os.makedirs(self.save_dir, exist_ok=True)

        self.get_logger().info(
            '航点巡航就绪: 航点文件=%s, 照片目录=%s, 图像=%s' % (
                self.route_file or '(未设置)', self.save_dir, self.image_topic))
        self.get_logger().info(
            'RViz 连点收集[%s]: 话题=%s%s，收集到的点画在 %s 上。'
            '要用连点就 `ros2 param set /inspection_route collect_mode true`，'
            '然后在 RViz 连点多个点，点完调一次 ~/start 依次跑完（每点停 %.1fs 拍照）。'
            % ('已启用' if self.collect_mode else '待启用',
               self.collect_topic,
               ('，也可用「Publish Point」发 %s' % self.point_topic)
               if self.auto_heading else '',
               self.marker_topic, self.settle_time))
        if self.auto_heading:
            self.get_logger().info(
                '车头自动对齐行驶方向[已启用]：开跑前按"我怎么到这里的"算每点朝向，'
                '第 1 个点用"车当前位姿 → 该点"的方向。'
                '在 RViz 里用「Publish Point」连点即可（不用拖朝向）。')
        else:
            self.get_logger().info(
                '车头自动对齐行驶方向[关闭]：沿用你在「2D Goal Pose」里拖出的朝向。'
                '想改成自动：ros2 param set /inspection_route auto_heading true')

        if bool(self.get_parameter('autostart').value):
            self.create_timer(
                max(0.1, float(self.get_parameter('start_delay').value)), self._autostart_once)

    # ------------------------------------------------------------------ 触发

    def _autostart_once(self):
        self._autostart_timer = None
        self.get_logger().info('autostart: 开始巡检')
        ok, reason = self._begin()
        if not ok:
            self.get_logger().error('autostart 启动失败：%s' % reason)

    def _on_start(self, _request, response):
        if self._running:
            response.success = False
            response.message = '巡检已在进行中'
            return response
        ok, reason = self._begin()
        response.success = ok
        # 把**具体原因**回给调用方 —— 只说"看日志"的话，用户在命令行上啥也看不到
        response.message = '已开始' if ok else ('启动失败：%s' % reason)
        return response

    def _on_stop(self, _request, response):
        if not self._running:
            response.success = False
            response.message = '当前没有在巡检'
            return response
        # ★ 兜底自救：worker 线程若已死（内部异常），`_running` 会卡在 True，
        #   于是 stop 回"已请求中止"、start 却一直回"巡检已在进行中" —— 死结。
        #   这里一发现线程已死就强制复位状态（配合 `_run_route_guarded` 双保险）。
        if self._worker is not None and not self._worker.is_alive():
            self._running = False
            self._active_index = None
            self._cancel_requested = True
            try:
                self._stop_now()
                self._publish_status('aborted_unexpected',
                                     extra={'note': '巡检线程已停止，状态已复位'})
            except Exception:                              # noqa: BLE001
                pass
            response.success = True
            response.message = ('巡检线程此前已异常退出 —— 已强制复位状态，'
                                '现在可以重新 ~/start（详情见日志）')
            self.get_logger().error(response.message)
            return response
        self._cancel_requested = True
        self._cancel_current_goal()
        response.success = True
        response.message = '已请求中止'
        return response

    def _begin(self):
        # 优先用 RViz 连点收集到的点：更符合"我刚点的那几个点"的直觉。
        with self._collect_lock:
            collected = list(self._collected)

        if collected:
            self._waypoints = collected
            self._loop_route = self.loop
            self.get_logger().info('本轮使用 RViz 收集的 %d 个航点（源=%s）' % (
                len(collected), self.collect_file))
        else:
            try:
                route = self._load_route()
            except Exception as exc:                  # noqa: BLE001
                self.get_logger().error('航点文件读取失败: %s' % exc)
                return False, '航点文件读取失败: %s' % exc
            self._waypoints = route['waypoints']
            self._loop_route = route['loop']

        if not self._waypoints:
            msg = ('没有任何航点：既没有收集到的点，%s 里也没有内容'
                   % (self.route_file or '(未设置 route_file)'))
            self.get_logger().error(msg)
            return False, msg

        # 开跑前把"与行驶方向一致"的朝向算出来（auto_heading=false 时原样返回）
        self._waypoints = self._resolve_headings(self._waypoints)

        # 底盘不能被别人开着：圆跟随器也直接发 /cmd_vel，两边同时发会互相踩。
        recent = self._external_cmd_recent(1.0)
        if recent > 0:
            msg = ('最近 1 秒内 %s 上有 %d 条非零速度 —— 有别的节点正在驱动底盘：'
                   '多半是「圆形跟随」正在跑（它也直接发 /cmd_vel），'
                   '或上一轮导航刚结束还没停稳。'
                   '先停掉它（ros2 service call /path_follower/stop std_srvs/srv/Trigger "{}"）'
                   '或等 2 秒再试。'
                   % (self.cmd_vel_topic, recent))
            self.get_logger().error(msg)
            return False, msg

        # 开跑前把"这些点太近、车不会真的过去"讲清楚，别等用户看见车只走一个点再猜。
        too_close = self._spacing_report(self._waypoints)
        for name_a, name_b, gap in too_close:
            self.get_logger().warn(
                '航点 %s 与 %s 相距仅 %.3f m < 到达容差 %.2f m：'
                '车到 %s 后会直接判定 %s 已到达而不移动。'
                % (name_a, name_b, gap, self.aim_tolerance, name_a, name_b))
        if too_close:
            self.get_logger().warn(
                '共 %d 对航点过近（上述）。若需要每点都真的停过去，'
                '请拉开间距或调小 goal_checker.xy_goal_tolerance。' % len(too_close))

        self._cancel_requested = False
        self._running = True
        self._active_index = None
        self._obstacle_hit = False
        self._obstacle_info = None
        if self._segment_mode_now() == 'straight':
            self.get_logger().info(
                '段间走法 = 严格直线（/follow_path，不过全局规划器，步长 %.2f m）。\n'
                '  ⚠ 遇障**不会绕行**。遇障动作 on_obstacle=%s（wait=停车等障碍消失后自动继续；'
                'stop=中止整轮；skip=跳过该点）。\n'
                '  判"已清空"的门槛 clear_distance=%.2f m（只看车头 ±%.0f°），'
                '等待上限 %.0f s（0=一直等）。\n'
                '  改回 nav2 规划：ros2 param set /inspection_route segment_mode navigate'
                % (self.straight_step, self._on_obstacle_now(), self.clear_distance,
                   self.wait_fov_deg, self.obstacle_wait_timeout))
        else:
            self.get_logger().info(
                '段间走法 = nav2 全局规划（NavigateToPose）：会绕障，但路径由规划器决定，'
                '靠近墙时可能被膨胀层推偏。\n'
                '  想走严格直线：ros2 param set /inspection_route segment_mode straight')
        self._publish_markers()
        self._publish_status('started', extra={
            'waypoints': len(self._waypoints),
            'source': 'rviz_collect' if collected else 'route_file'})
        self._worker = threading.Thread(target=self._run_route_guarded, daemon=True)
        self._worker.start()
        return True, ''

    def _run_route_guarded(self):
        """跑一轮巡检，**保证任何异常都不会把节点卡在"运行中"**。

        ⚠ 这一层是必需的，不是保险起见：worker 线程里任何未捕获的异常都会让它
        **静默死掉**，而 `_running` 会永远停在 True ⇒ 之后 `~/stop`、`~/start`
        全部失效 —— 用户看到的现象就是"**终止都终止不了**"。
        2026-09-14 真踩过：残留像素分支里一个 `KeyError: 'clears'`（本轮重置字典时
        漏了一个键）导致整整卡死，日志停在"等待判据初值"后再无输出。
        """
        try:
            self._run_route()
        except Exception:                                  # noqa: BLE001
            self.get_logger().error(
                '巡检线程异常退出（已自动复位，可以重新 ~/start）：\n%s'
                % traceback.format_exc())
        finally:
            if self._running:
                self._running = False
                self._active_index = None
                try:
                    self._stop_now()                       # 兜底停车
                except Exception:                          # noqa: BLE001
                    pass
                try:
                    self._publish_markers()
                    self._publish_status(
                        'aborted_unexpected',
                        extra={'note': '巡检线程异常退出：已停车并复位，可重新 ~/start'})
                except Exception:                          # noqa: BLE001
                    pass
                self.get_logger().error(
                    '已复位：现在 ~/stop 与 ~/start 都恢复可用。')

    # --------------------------------------------------------- RViz 连点收集

    @staticmethod
    def _yaw_from_quat(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _auto_heading_now(self):
        """实时读参数，让 auto_heading 也能运行中热切。"""
        return bool(self.get_parameter('auto_heading').value)

    def _warn_collect_off(self, topic):
        now = time.time()
        if now - self._last_ignore_warn > 5.0:
            self._last_ignore_warn = now
            self.get_logger().warn(
                '收到 %s 上的点，但 collect_mode=false，已忽略。'
                '要收集请先执行：ros2 param set /inspection_route collect_mode true'
                % topic)

    def _warn_frame(self, frame):
        self.get_logger().error(
            '收集到的点坐标系是 %r，与航点坐标系 %r 不一致，已忽略。'
            '请把 RViz 的 Fixed Frame 设成 %s 再点。'
            % (frame, self.frame_id, self.frame_id))

    def _on_collect_pose(self, msg: PoseStamped):
        """记录一个来自 RViz「2D Goal Pose」的航点。只收集、不导航 —— 车不会动。"""
        if not bool(self.get_parameter('collect_mode').value):
            self._warn_collect_off(self.collect_topic)
            return
        if self._running:
            self.get_logger().warn('巡检进行中，忽略这个新点。先调 ~/stop 再收集。')
            return
        frame = msg.header.frame_id or '(空)'
        if frame != self.frame_id:
            self._warn_frame(frame)
            return
        yaw = (None if self._auto_heading_now()
               else self._yaw_from_quat(msg.pose.orientation))
        self._append_waypoint(float(msg.pose.position.x), float(msg.pose.position.y),
                              yaw, 'RViz 2D Goal Pose')

    def _on_collect_point(self, msg: PointStamped):
        """记录一个 RViz「Publish Point」的点（不带朝向）。

        点一下就行、不用拖方向 ⇒ 连点最快；朝向交给开跑前自动算。
        """
        if not bool(self.get_parameter('collect_mode').value):
            self._warn_collect_off(self.point_topic)
            return
        if self._running:
            self.get_logger().warn('巡检进行中，忽略这个新点。先调 ~/stop 再收集。')
            return
        frame = msg.header.frame_id or '(空)'
        if frame != self.frame_id:
            self._warn_frame(frame)
            return
        self._append_waypoint(float(msg.point.x), float(msg.point.y), None,
                              'RViz Publish Point')

    def _append_waypoint(self, x, y, yaw, source):
        """把一个点收进列表（两种输入工具共用）。yaw=None 表示"开跑前自动算"。"""
        wp = {
            'name': 'p%02d' % (len(self._collected) + 1),
            'x': x,
            'y': y,
            'yaw': yaw,
            'settle': self.settle_time,
            'note': source,
        }
        with self._collect_lock:
            previous = self._collected[-1] if self._collected else None
            self._collected.append(wp)
            count = len(self._collected)

        if yaw is None:
            self.get_logger().info(
                '已收集第 %d 个航点: %s (%.3f, %.3f)，朝向开跑前自动算 '
                '—— 继续点，点完调 ~/start' % (count, wp['name'], x, y))
        else:
            self.get_logger().info(
                '已收集第 %d 个航点: %s (%.3f, %.3f, yaw=%.1f°) —— 继续点，'
                '点完调 ~/start' % (count, wp['name'], x, y, math.degrees(yaw)))

        # 相邻两点太近 ⇒ nav2 会直接判定"已到达"，车根本不会过去。
        # 这是"看着标了好几个点、实际只走了一个"的第二大原因（第一是 RViz 工具误发目标）。
        if previous is not None:
            gap = math.hypot(x - float(previous['x']),
                             y - float(previous['y']))
            if gap < self.aim_tolerance:
                self.get_logger().warn(
                    '%s 与上一个点 %s 只差 %.3f m，小于 nav2 的到达容差 %.2f m '
                    '—— 到时车会判定"已到达"而不移动。要分别停稳请把两点拉到 %.2f m 以上，'
                    '或把 config/nav2_params.yaml 里 goal_checker.xy_goal_tolerance 调小后重编译。'
                    % (wp['name'], previous['name'], gap, self.aim_tolerance,
                       self.aim_tolerance * 1.5))

        self._save_collected()
        self._publish_markers()
        self._publish_status('collected', count - 1, count, wp['name'],
                             extra={'x': x, 'y': y,
                                    'yaw': yaw if yaw is not None else 'auto',
                                    'collected': count})

    # ------------------------------------------------- 开跑前：解析每点朝向

    def _lookup_pose(self, frame=None):
        """车在 `frame`（默认 frame_id=map）下的 (x, y, yaw)；查不到返回 None。"""
        try:
            t = self._tf_buffer.lookup_transform(
                frame or self.frame_id, self.base_frame, RclTime(),
                timeout=Duration(seconds=0.05))
        except Exception:                                  # noqa: BLE001
            return None
        tr = t.transform.translation
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return float(tr.x), float(tr.y), yaw

    # ------------------------------------------------------------ 障碍停车

    def _laser_yaw(self):
        """base_frame → laser 的 yaw（缓存）。本车雷达是反装的（≈π）。"""
        if self._laser_yaw_cached is None:
            try:
                t = self._tf_buffer.lookup_transform(
                    self.base_frame, self.laser_frame, RclTime(),
                    timeout=Duration(seconds=0.05))
                q = t.transform.rotation
                self._laser_yaw_cached = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            except Exception:                              # noqa: BLE001
                return 0.0
        return self._laser_yaw_cached

    def _on_scan(self, msg: LaserScan):
        self._scan = msg

    def _on_costmap(self, msg: OccupancyGrid):
        self._costmap = msg

    def _obstacle_snapshot(self, fov_deg=None):
        """障碍大概在哪边、多远 —— 只为了让人一眼知道去现场该看哪里。"""
        scan = self._scan
        if scan is None or not scan.ranges:
            return None
        d, deg = nearest_scan_obstacle(
            scan.ranges, scan.angle_min, scan.angle_increment, self._laser_yaw(),
            self.obstacle_fov_deg if fov_deg is None else fov_deg)
        if d is None:
            return None
        return {'distance': round(d, 3), 'bearing_deg': round(deg, 1)}

    # -------------------------------------------------- 代价地图：判断"能不能走了"

    def _corridor_blocked(self):
        """代价地图上"车正前方走廊"是否被堵。返回 `True` / `False` / `None`（不可用）。

        ★ 这是判"能不能走"的**主判据**：数据源与控制器（RPP）完全一致，
        且只看车要走的那条窄带，所以**窄通道里不会被侧面的东西卡住**。
        """
        if not self.use_costmap:
            return None
        grid = self._costmap
        if grid is None or not grid.data:
            return None
        frame = grid.header.frame_id or self.frame_id
        pose = self._lookup_pose(frame)
        if pose is None:
            if not self._costmap_frame_warned:
                self._costmap_frame_warned = True
                self.get_logger().warn(
                    '查不到 TF %s -> %s，暂时无法用代价地图判断前方是否可走'
                    % (frame, self.base_frame))
            return None
        blocked, worst, at = corridor_blocked(
            grid.data, grid.info.width, grid.info.height, grid.info.resolution,
            grid.info.origin.position.x, grid.info.origin.position.y,
            pose[0], pose[1], pose[2],
            self.corridor_len, self.corridor_half_width)
        self._costmap_last = {'blocked': blocked, 'worst': worst, 'at': at,
                              'frame': frame}
        return blocked

    def _clear_local_costmap(self):
        """清一次 local costmap。

        ★ 这是"障碍物移开后残留像素"的**确定性修复**：
        障碍移开时激光在那些方向打不到东西（返回 inf），而 nav2 的 obstacle_layer
        对 inf 读数**不做 raycast clearing** ⇒ 旧障碍标记会**永远留在代价地图里**，
        控制器就一直判"前方要撞"。主动清一次，残留就没了（真障碍下一拍会被重新标记）。
        """
        if not self.use_costmap or self._clear_client is None:
            return False
        try:
            if not self._clear_client.service_is_ready() and \
                    not self._clear_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('清图服务 %s 不可用（nav2 起了吗？）'
                                       % self.clear_costmap_service)
                return False
            self._clear_client.call_async(ClearEntireCostmap.Request())
            return True
        except Exception as exc:                           # noqa: BLE001
            self.get_logger().warn('清 %s 失败: %s' % (self.costmap_topic, exc))
            return False

    def _stop_now(self):
        """连发几条零速让底盘立刻停住（比单纯等 chassis_bridge 的看门狗更干脆）。

        这里是 worker 线程，不是服务回调 —— 所以 sleep 是安全的
        （**服务回调里绝不能 sleep**，那会把 executor 卡住）。
        """
        for _ in range(5):
            self.cmd_vel_pub.publish(self._zero_cmd)
            time.sleep(0.02)

    def _flag_obstacle(self, kind, detail, wp):
        """标记"撞上东西了"：**停车 + 中止整轮**（不重试、不去下一个点）。"""
        self._obstacle_hit = True
        snap = self._obstacle_snapshot()
        self._obstacle_info = {'kind': kind, 'detail': detail,
                               'waypoint': wp.get('name'), 'scan': snap}
        self._stop_now()
        msg = ('⚠ 检测到障碍，已停车，本轮巡检中止 —— 航点 %s 的直线段：%s。'
               % (wp.get('name', '?'), detail))
        if snap is not None:
            msg += '车头 ±%.0f° 内最近回波 %.2f m（方位 %+.0f°）。' % (
                self.obstacle_fov_deg, snap['distance'], snap['bearing_deg'])
        msg += ('严格直线模式**不会绕行**（这正是它存在的意义），所以这里选择停车，'
                '而不是像多点巡航那样重试/跳过。清障后重新 ~/start 即可；'
                '也可以 `ros2 param set /inspection_route on_obstacle skip` 改成跳过继续。')
        self.get_logger().error(msg)

    def _resolve_headings(self, waypoints):
        """把 yaw=None 的点补成"与行驶方向一致"（规则见 compute_headings）。"""
        if not self._auto_heading_now():
            return waypoints

        pose = self._lookup_pose()
        if pose is None:
            self.get_logger().warn(
                'auto_heading 需要 TF %s -> %s 来定第 1 个点的朝向，'
                '但查不到（定位没在跑？）。第 1 个点退回用"该点 → 下一个点"的方向。'
                % (self.frame_id, self.base_frame))

        yaws, notes = compute_headings(waypoints, pose)
        for i, wp in enumerate(waypoints):
            if yaws[i] is not None:
                wp['yaw'] = yaws[i]
        for level, text in notes:
            getattr(self.get_logger(), level)(text)
        return waypoints

    def _save_collected(self):
        """把收集到的点存成 YAML，既是留档也方便直接当 route_file 复用。

        `yaw: null` 表示"朝向开跑前自动算"（auto_heading 模式下收集的点）。
        """
        with self._collect_lock:
            points = list(self._collected)
        payload = {
            'loop': self.loop,
            'auto_heading': self._auto_heading_now(),
            'note': '由 inspection_route 的 RViz 连点收集自动生成，可直接当 route_file 用'
                    '；yaw: null 表示朝向在开跑前按行驶方向自动计算',
            'waypoints': [{'name': p['name'], 'x': round(p['x'], 4),
                           'y': round(p['y'], 4),
                           'yaw': (None if p.get('yaw') is None
                                   else round(float(p['yaw']), 4)),
                           'settle': p['settle'], 'note': p['note']}
                          for p in points],
        }
        try:
            os.makedirs(os.path.dirname(self.collect_file), exist_ok=True)
            tmp = self.collect_file + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as handle:
                yaml.safe_dump(payload, handle, allow_unicode=True,
                               sort_keys=False, default_flow_style=False)
            os.replace(tmp, self.collect_file)         # 原子替换，避免读到半截文件
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().error('收集文件写入失败: %s' % exc)

    def _on_cmd_vel(self, msg: Twist):
        """底盘速度哨兵。两个用途：

        ① 标点阶段车必须**一动不动**。这时 /cmd_vel 上出现非零速度，说明有别的
           客户端在驱动底盘 —— 实测最常见的就是 RViz 的「Nav2 Goal」工具
           （nav2_rviz_plugins/GoalTool）或「Navigation 2」面板：它们在你点击的瞬间
           就把目标直接发给 nav2 action，于是"标点"变成"边走边标"，后点的还会抢先前点的，
           最后只去最后一个点。这里主动喊出来，避免用户以为是自己程序的问题。
        ② 开跑前的互斥判断：记录"有人在驱动底盘"的时刻，供 `_begin()` 非阻塞地检查
           （比如圆跟随器正在跑 —— 它也直接发 /cmd_vel）。
        """
        effort = abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z)
        if effort <= self.cmd_vel_epsilon:
            return
        # 自己巡检时 nav2 必然在发速度，那不是"别人"，不记
        if self._running:
            return

        now = time.time()
        self._ext_cmd_times.append(now)
        if len(self._ext_cmd_times) > 400:
            del self._ext_cmd_times[:200]

        if not bool(self.get_parameter('collect_mode').value):
            return
        if now - self._last_cmdvel_warn <= 5.0:
            return
        self._last_cmdvel_warn = now
        self.get_logger().error(
            '标点期间 %s 上出现非零速度指令（|vx|+|vy|+|wz| = %.3f）—— '
            '有别的客户端在驱动底盘，标点结果不可信！'
            '最常见来源：RViz 的「Nav2 Goal」工具栏 / 「Navigation 2」面板，'
            '它们会在点击瞬间把目标发给 nav2。'
            '请改用只发话题的「2D Goal Pose」工具，或用本包自带的 '
            'rviz/inspection_waypoints.rviz 启动 RViz。'
            % (self.cmd_vel_topic, effort))

    def _external_cmd_recent(self, window):
        now = time.time()
        return sum(1 for t in self._ext_cmd_times if now - t < window)

    def _spacing_report(self, points):
        """列出相邻间距小于到达容差的航点对。"""
        bad = []
        for i in range(1, len(points)):
            a, b = points[i - 1], points[i]
            gap = math.hypot(float(b['x']) - float(a['x']),
                             float(b['y']) - float(a['y']))
            if gap < self.aim_tolerance:
                bad.append((a.get('name', '?'), b.get('name', '?'), gap))
        return bad

    def _on_clear(self, _request, response):
        if self._running:
            response.success = False
            response.message = '巡检进行中，先 ~/stop 再清空'
            return response
        with self._collect_lock:
            count = len(self._collected)
            self._collected = []
        response.success = True
        response.message = '已清空 %d 个收集到的航点（route_file 未受影响）' % count
        self.get_logger().info(response.message)
        self._publish_markers()
        self._publish_status('cleared', extra={'collected': 0, 'removed': count})
        return response

    # ------------------------------------------------------- RViz 可视化标记

    def _publish_markers(self):
        """把「已收集 / 本轮要跑的」航点画到 RViz，免得连点全靠日志核对。

        橙色箭头 = 待执行，绿色箭头 = 正在前往的那个；箭头方向就是到点朝向。
        先发一个 DELETEALL，保证清空后不会留下旧标记。
        """
        # 巡检进行中画本轮真正在跑的列表；空闲时画已收集的列表。
        if self._running and self._waypoints:
            points = list(self._waypoints)
        else:
            with self._collect_lock:
                points = list(self._collected)

        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()

        wipe = Marker()
        wipe.header.frame_id = self.frame_id
        wipe.header.stamp = stamp
        wipe.action = Marker.DELETEALL
        array.markers.append(wipe)

        # 路径连线：按点击顺序串起来，一眼看清车会怎么走
        if len(points) >= 2:
            line = Marker()
            line.header.frame_id = self.frame_id
            line.header.stamp = stamp
            line.ns = 'route_line'
            line.id = 900
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.02
            line.color.r, line.color.g, line.color.b, line.color.a = 0.2, 0.9, 1.0, 0.85
            line.pose.orientation.w = 1.0
            for wp in points:
                p = Point()
                p.x, p.y, p.z = float(wp['x']), float(wp['y']), 0.02
                line.points.append(p)
            array.markers.append(line)

        for index, wp in enumerate(points):
            active = (self._active_index is not None
                      and index == self._active_index)
            yaw = self._display_yaw(points, index)
            orientation = quat_from_yaw(0.0 if yaw is None else yaw)

            if yaw is None:
                # 朝向还没法确定（auto 模式下的第 1 个点）：画个球，别画箭头误导
                arrow = Marker()
                arrow.header.frame_id = self.frame_id
                arrow.header.stamp = stamp
                arrow.ns = 'waypoints'
                arrow.id = index
                arrow.type = Marker.SPHERE
                arrow.action = Marker.ADD
                arrow.pose.position.x = float(wp['x'])
                arrow.pose.position.y = float(wp['y'])
                arrow.pose.position.z = 0.05
                arrow.pose.orientation.w = 1.0
                arrow.scale.x = arrow.scale.y = arrow.scale.z = 0.14
                if active:
                    arrow.color.r, arrow.color.g, arrow.color.b = 0.0, 1.0, 0.0
                else:
                    arrow.color.r, arrow.color.g, arrow.color.b = 1.0, 0.55, 0.0
                arrow.color.a = 1.0
                array.markers.append(arrow)
            else:
                arrow = Marker()
                arrow.header.frame_id = self.frame_id
                arrow.header.stamp = stamp
                arrow.ns = 'waypoints'
                arrow.id = index
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.pose.position.x = float(wp['x'])
                arrow.pose.position.y = float(wp['y'])
                arrow.pose.position.z = 0.05
                arrow.pose.orientation = orientation
                arrow.scale.x = 0.45          # 箭杆长度 = 朝向指示
                arrow.scale.y = 0.07
                arrow.scale.z = 0.07
                if active:
                    arrow.color.r, arrow.color.g, arrow.color.b = 0.0, 1.0, 0.0
                else:
                    arrow.color.r, arrow.color.g, arrow.color.b = 1.0, 0.55, 0.0
                arrow.color.a = 1.0
                array.markers.append(arrow)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = stamp
            label.ns = 'waypoint_labels'
            label.id = 1000 + index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(wp['x'])
            label.pose.position.y = float(wp['y'])
            label.pose.position.z = 0.40
            label.pose.orientation.w = 1.0
            label.scale.z = 0.18          # 文字高度（米）
            if yaw is None:
                label.text = '%d %s (朝向待算)' % (index + 1, wp.get('name', ''))
            else:
                label.text = '%d %s %.0f°' % (index + 1, wp.get('name', ''),
                                              math.degrees(yaw))
            # 黄字在深色/浅色 RViz 主题下都看得见
            label.color.r, label.color.g, label.color.b = 1.0, 0.9, 0.2
            label.color.a = 1.0
            array.markers.append(label)

        self.marker_pub.publish(array)

    def _display_yaw(self, points, index):
        """画箭头用的朝向。

        收集阶段（auto_heading）各点 yaw 是 None；第 i(>0) 个点的朝向开跑时会
        等于"从第 i-1 个点指向它"，所以这里**直接按同一规则预览**，让用户在点的时候
        就能看到车到点后会朝哪边。第 1 个点的朝向取决于开跑那一刻车在哪 ⇒ 返回 None。
        """
        wp = points[index]
        yaw = wp.get('yaw')
        if yaw is not None:
            return float(yaw)
        if index > 0:
            a = points[index - 1]
            dx = float(wp['x']) - float(a['x'])
            dy = float(wp['y']) - float(a['y'])
            if math.hypot(dx, dy) > 1e-3:
                return math.atan2(dy, dx)
        return None

    def _load_route(self):
        path = os.path.expanduser(self.route_file)
        if not path or not os.path.isfile(path):
            raise FileNotFoundError('找不到航点文件: %r' % path)
        with open(path, encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}

        raw = data.get('waypoints') or []
        waypoints = []
        for idx, item in enumerate(raw):
            if 'x' not in item or 'y' not in item:
                raise ValueError('第 %d 个航点缺 x/y' % (idx + 1))
            waypoints.append({
                'name': str(item.get('name', 'wp%02d' % (idx + 1))),
                'x': float(item['x']),
                'y': float(item['y']),
                'yaw': float(item['yaw']) if item.get('yaw') is not None else None,
                'settle': float(item['settle']) if item.get('settle') is not None else self.settle_time,
                'note': str(item.get('note', '')),
            })
        return {'waypoints': waypoints, 'loop': bool(data.get('loop', self.loop))}

    # ------------------------------------------------------------------ 主流程

    def _run_route(self):
        started = time.time()
        results = []
        total = len(self._waypoints)
        aborted = False

        self.get_logger().info('巡检开始: 共 %d 个航点, 失败策略=%s, 单点最多 %d 次' % (
            total, self.on_failure, self.max_retries + 1))

        while True:
            for index, wp in enumerate(self._waypoints):
                if self._cancel_requested:
                    break
                results.append(self._visit(index, total, wp))
                if self._cancel_requested:
                    break
                if self._obstacle_hit:
                    aborted = True
                    break
                if not results[-1]['ok'] and self.on_failure == 'abort':
                    self.get_logger().error('航点 %s 失败且策略为 abort，终止本轮' % wp['name'])
                    aborted = True
                    break
            if self._cancel_requested or aborted or not self._loop_route:
                break
            self.get_logger().info('loop 模式：重新开始一轮')
            for r in results:
                r['attempts'] = r['attempts']      # 保留历史，仅继续累加
            break

        report = self._write_report(results, started, total)
        self._running = False
        self._active_index = None
        self._publish_markers()
        self._publish_status('finished', None, None,
                             extra={'report': report,
                                    'ok': sum(1 for r in results if r['ok']),
                                    'total': total,
                                    'aborted_reason': ('obstacle'
                                                       if self._obstacle_hit else None),
                                    'obstacle': (dict(self._obstacle_info)
                                                 if self._obstacle_info else None)})
        if self._obstacle_hit:
            self.get_logger().error(
                '巡检因**障碍**中止: 已走完 %d/%d, 报告 %s。'
                '现场清障后重新 ~/start 即可（不会自动继续）。'
                % (sum(1 for r in results if r['ok']), total, report))
        else:
            self.get_logger().info('巡检结束: 成功 %d/%d, 报告 %s' % (
                sum(1 for r in results if r['ok']), total, report))

    def _visit(self, index, total, wp):
        entry = {
            'index': index + 1, 'name': wp['name'], 'x': wp['x'], 'y': wp['y'],
            'yaw': wp['yaw'], 'note': wp['note'], 'attempts': 0, 'ok': False,
            'status': 'not_started', 'image': None, 'distance_remaining': None,
            'duration_s': None, 'segment_mode': self._segment_mode_now(),
        }
        t_start = time.time()

        for attempt in range(1, self.max_retries + 2):
            if self._cancel_requested:
                entry['status'] = 'canceled'
                break
            entry['attempts'] = attempt
            self._active_index = index
            self._publish_markers()
            self._publish_status('navigating', index, total, wp['name'],
                                 extra={'attempt': attempt, 'max_attempts': self.max_retries + 1})
            self.get_logger().info('[%d/%d] %s 第 %d 次前往 (%.2f, %.2f)' % (
                index + 1, total, wp['name'], attempt, wp['x'], wp['y']))

            self._seg_wait = {'count': 0, 'seconds': 0.0, 'clears': 0}
            ok, status = self._navigate(wp)
            entry['status'] = status
            entry['distance_remaining'] = self._last_distance
            if self._seg_wait['count']:
                entry['obstacle_waits'] = self._seg_wait['count']
                entry['obstacle_wait_s'] = round(self._seg_wait['seconds'], 1)
                if self._seg_wait['clears']:
                    entry['costmap_clears'] = self._seg_wait['clears']
            if ok:
                entry['ok'] = True
                break
            if self._cancel_requested:
                entry['status'] = 'canceled'
                break
            if self._obstacle_hit:
                # 撞上东西了：不再重试（重试 = 再冲一次），也不去下一个点。
                entry['status'] = 'blocked_by_obstacle'
                entry['obstacle'] = dict(self._obstacle_info or {})
                break
            self.get_logger().warn('[%d/%d] %s 第 %d 次失败: %s' % (
                index + 1, total, wp['name'], attempt, status))

        if entry['ok']:
            self._publish_status('capturing', index, total, wp['name'])
            entry['image'] = self._capture(index, wp)
            if entry['image'] is None:
                entry['ok'] = False
                entry['status'] = 'arrived_but_no_image'
                self.get_logger().warn('[%d/%d] %s 已到达但未取到图像' % (index + 1, total, wp['name']))

        entry['duration_s'] = round(time.time() - t_start, 2)
        self.get_logger().info('[%d/%d] %s 结束: ok=%s status=%s 耗时 %.1fs 照片=%s' % (
            index + 1, total, wp['name'], entry['ok'], entry['status'],
            entry['duration_s'], entry['image'] or '-'))
        return entry

    # ------------------------------------------------------------------ 导航

    def _navigate(self, wp):
        """逐点之间"怎么走"按 segment_mode 分派（**每点热读参数**，可运行中切换）。

        `navigate` → nav2 全局规划（多点巡航用这个）
        `straight` → 严格沿直线（/follow_path，不过全局规划器）
        """
        if self._segment_mode_now() == 'straight':
            return self._nav_straight(wp)
        return self._nav_planner(wp)

    def _segment_mode_now(self):
        """实时读参数，让 segment_mode 也能运行中热切。"""
        text = str(self.get_parameter('segment_mode').value).lower()
        return text if text in ('navigate', 'straight') else 'navigate'

    def _nav_planner(self, wp):
        """nav2 全局规划（NavigateToPose）：会绕障，但路径由规划器决定。"""
        if not self._client.wait_for_server(timeout_sec=5.0):
            return False, '导航 action server 不可用'
        self._last_distance = None

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wp['x']
        goal.pose.pose.position.y = wp['y']
        goal.pose.pose.position.z = 0.0
        yaw = wp.get('yaw')
        if yaw is None:
            # 正常路径下 _resolve_headings 已经填好了；这里是兜底 ——
            # 比如从 route_file 读入 yaw: null 但 auto_heading 没开。
            # 硬用 0（朝东）会让车到点后乱转，所以现场按"车 → 该点"的方向算。
            pose = self._lookup_pose()
            if pose is not None:
                yaw = math.atan2(float(wp['y']) - pose[1], float(wp['x']) - pose[0])
            else:
                yaw = 0.0
            self.get_logger().warn(
                '航点 %s 没有指定朝向，按"车当前位姿指向该点"兜底为 %.1f°'
                % (wp.get('name', '?'), math.degrees(yaw)))
        goal.pose.pose.orientation = quat_from_yaw(float(yaw))

        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        if not self._await(send_future, 10.0):
            return False, 'goal 发送超时'
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False, 'goal 被拒绝（目标可能在障碍里或坐标系不对）'

        with self._goal_lock:
            self._current_goal = handle

        result_future = handle.get_result_async()
        if not self._await(result_future, self.navigate_timeout):
            self._cancel_current_goal()
            return False, '导航超时 %.0fs' % self.navigate_timeout

        with self._goal_lock:
            self._current_goal = None

        wrapped = result_future.result()
        status = wrapped.status if wrapped is not None else GoalStatus.STATUS_UNKNOWN
        text = STATUS_TEXT.get(status, 'status_%s' % status)
        return status == GoalStatus.STATUS_SUCCEEDED, text

    # -------------------------------------------------- 严格直线段（follow_path）

    def _nav_straight(self, wp):
        """严格沿直线走到 wp（**遇障可等待**）。

        做法：把"车当前实际位姿 → 目标点"离散成一条 `nav_msgs/Path`，直接交给
        **`/follow_path`**（nav2 的 controller_server），**完全不过全局规划器**，
        所以路径就是这条直线，不会被膨胀层推开、也不会自己绕行。

        ⚠ 遇到障碍**不会绕**（这正是它存在的意义）。怎么办由 `on_obstacle` 决定：

        | 值 | 行为 |
        |---|---|
        | `wait`（默认） | **停车等障碍消失**，清空后自动继续这一段 |
        | `stop` | 停车 + 中止整轮巡检 |
        | `skip` | 按旧逻辑重试，然后跳过该点 |

        路径起点取"车当前实际位姿"而不是上一个航点，这样控制器起点和车一致
        （两者相差在 goal_checker 容差内，不影响"走直线"）。
        """
        if not self._follow_client.wait_for_server(timeout_sec=5.0):
            return False, ('%s action 不可用（controller_server 起了吗？）'
                           % self.get_parameter('follow_action').value)

        while True:
            pose = self._lookup_pose()
            if pose is None:
                return False, ('查不到 TF %s -> %s，无法构造直线段'
                               % (self.frame_id, self.base_frame))

            yaw = wp.get('yaw')
            if self._auto_heading_now() or yaw is None:
                # ★ 用**这条直线的实际方向**（路径起点 → 终点），而不是 wp['yaw']
                #   （wp['yaw'] 是"上一个航点 → 该航点"）。因为路径起点取的是
                #   "车当前位置"，第一段、或中途车没停在上一个点上时两者方向不同，
                #   照搬 wp['yaw'] 会让车**斜着走**。
                # 用户手动指定过朝向（auto_heading=false 且拖过）则尊重他。
                yaw = math.atan2(float(wp['y']) - pose[1], float(wp['x']) - pose[0])
            yaw = float(yaw)

            pts = build_straight_path(pose[0], pose[1], wp['x'], wp['y'],
                                      yaw, self.straight_step)
            if len(pts) < 2:
                self.get_logger().info(
                    '已经在 %s 上了（距离 < 1 mm），本段跳过' % wp.get('name', '?'))
                return True, 'succeeded'

            ok, text, kind, why = self._follow_segment(pose, wp, yaw, pts)
            if ok:
                return True, 'succeeded'
            if self._cancel_requested:
                return False, 'canceled'

            mode = self._on_obstacle_now()
            if kind not in (FOLLOW_OBSTACLE, FOLLOW_STUCK):
                return False, text

            # ---- 到这里说明"这段走不通"（多半是被挡住）
            if mode == 'skip':
                self.get_logger().warn(
                    '直线段走不通（%s），但 on_obstacle=skip，按旧逻辑重试/跳过。' % why)
                return False, text

            if mode == 'stop':
                self._flag_obstacle(kind, why, wp)
                return False, 'blocked_%s: %s' % (kind, why)

            # mode == 'wait'：停车等障碍消失，清空后重算路径再走
            self._stop_now()
            self.get_logger().warn(
                '⏸ %s 这一段走不通（%s）—— 严格直线不会绕行，**停车等待障碍消失**。'
                '人把东西挪开后会自动继续这一段（判据是代价地图 + 激光，'
                '还会自动清掉障碍移开后残留的格子）。'
                % (wp.get('name', '?'), why))
            resumed, waited, note = self._wait_until_clear(wp)
            self._seg_wait['count'] += 1
            self._seg_wait['seconds'] += waited
            if not resumed:
                self.get_logger().error(
                    '等待障碍结束但**没有继续**：%s（已等 %.0f s）' % (note, waited))
                return False, 'obstacle_wait_%s' % note
            self.get_logger().info(
                '▶ 障碍处置完成（%s，等了 %.1f s），重新前往 %s。' % (note, waited, wp.get('name', '?')))
            # 循环回去：重算路径（车可能被人挪动过）+ 重发

    def _follow_segment(self, pose, wp, yaw, pts):
        """发一次 FollowPath 并等结果。返回 `(ok, text, kind, why)`。"""
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for (x, y, yw) in pts:
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = path.header.stamp
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation = quat_from_yaw(yw)
            path.poses.append(ps)

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.follow_controller_id   # 空 = 用默认 controller
        goal.goal_checker_id = ''
        goal.progress_checker_id = ''

        seg_len = math.hypot(float(wp['x']) - pose[0], float(wp['y']) - pose[1])
        self.get_logger().info(
            '[%s] 严格直线段: (%.2f, %.2f) → (%.2f, %.2f)，%.3f m / %d 个路径点 / 朝向 %.0f°'
            % (wp.get('name', '?'), pose[0], pose[1], float(wp['x']), float(wp['y']),
               seg_len, len(path.poses), math.degrees(yaw)))

        self._last_distance = None
        send_future = self._follow_client.send_goal_async(
            goal, feedback_callback=self._on_follow_feedback)
        if not self._await(send_future, 10.0):
            return False, '直线段 goal 发送超时', FOLLOW_OTHER, '发送超时'
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False, '直线段 goal 被拒绝（路径无效或坐标系不对）', FOLLOW_OTHER, '目标被拒'

        with self._goal_lock:
            self._current_goal = handle

        result_future = handle.get_result_async()
        if not self._await(result_future, self.navigate_timeout):
            self._cancel_current_goal()
            return (False, '直线段超时 %.0fs' % self.navigate_timeout,
                    FOLLOW_OTHER, '超时')

        with self._goal_lock:
            self._current_goal = None

        wrapped = result_future.result()
        status = wrapped.status if wrapped is not None else GoalStatus.STATUS_UNKNOWN
        if status == GoalStatus.STATUS_SUCCEEDED:
            return True, 'succeeded', None, ''

        inner = getattr(wrapped, 'result', None)
        code = getattr(inner, 'error_code', None)
        emsg = getattr(inner, 'error_msg', '')
        text = STATUS_TEXT.get(status, 'status_%s' % status)
        if code:
            text += ' [follow_path error_code=%s %s]' % (code, emsg)

        # RPP 的碰撞检测抛的是 NoValidControl(106)，文案
        # "RegulatedPurePursuitController detected collision ahead!"。
        kind, why = classify_follow_failure(code, emsg)
        return False, text, kind, why

    def _wait_until_clear(self, wp):
        """**停车等障碍消失**。返回 `(是否已清空, 等待秒数, 说明)`。

        ★ 判据的主次（2026-09-14 修过一个"永远等下去"的 bug）：

        | 代价地图走廊 | 激光车头窄扇区 | 判定 | 动作 |
        |---|---|---|---|
        | 空 | — | **可以走了** | 重发目标 |
        | 有障碍 | 也认为有东西 | **真障碍** | 继续等 |
        | 有障碍 | **但已经空了** | **残留像素** | **清一次 local costmap** 再判 |

        **为什么主判据是代价地图而不是激光**：控制器（RPP）就是拿代价地图判的，
        两边同源才不会出现"我判能走、它不认"。而且只看车正前方一条**窄走廊**，
        所以**窄通道里不会被两侧货架卡死** —— 用"±60° 内最近回波"就会（旧版本的毛病）。

        **为什么需要清图这一步**：障碍物被移开时，激光在那些方向打不到东西（返回 inf），
        而 nav2 的 obstacle_layer 对 inf 读数**不做 raycast clearing** ⇒ 旧标记永远留着，
        控制器一直判"要撞"，于是永远等下去。清一次 costmap 就把残留抹掉了
        （真障碍下一拍会被激光重新标记，所以不会误放行）。

        本函数跑在 worker 线程里（不是服务回调），所以 `sleep` 是安全的。
        等待期间 `~/stop` 可以随时中止。
        """
        t0 = time.time()
        last_log = 0.0
        futile = 0
        last_futile = 0.0
        first = True
        self._publish_status('waiting_obstacle',
                             extra={'waypoint': wp.get('name'), 'waited_s': 0.0})

        while not self._cancel_requested:
            waited = time.time() - t0
            narrow = self._obstacle_snapshot(fov_deg=self.wait_fov_deg)
            gap = path_blocked(narrow, self.clear_distance,
                               scan_missing=(self._scan is None))
            # gap 三态：None = 拿不到激光（不知道）；True = 还堵着；False = 已空
            cost_blocked = self._corridor_blocked()

            if first:
                first = False
                # 这条对现场排错很关键：一眼看出"三个判据各自看到什么"
                self.get_logger().info(
                    '等待判据初值: 激光(车头 ±%.0f°) %s；代价地图走廊(%s) %s；'
                    '判空阈值 %.2f m'
                    % (self.wait_fov_deg,
                       ('还堵 %.2f m' % narrow['distance']) if gap is True
                       else ('已空' if gap is False else '拿不到激光'),
                       self.costmap_topic,
                       {True: '还堵', False: '已空', None: '不可用'}[cost_blocked],
                       self.clear_distance))

            # ① 代价地图说前方走廊是空的 ⇒ 可以走了（最终仍由控制器裁决）
            if cost_blocked is False:
                worst = (self._costmap_last or {}).get('worst')
                return True, waited, ('代价地图上正前方 %.2f m 走廊内已无障碍'
                                      '（最坏代价 %s）'
                                      % (self.corridor_len, worst))
            # ② 代价地图用不了 ⇒ 退化成"激光窄扇区"判断；连激光也没有就更退化
            if cost_blocked is None:
                if gap is False:
                    return True, waited, ('（未启用/收不到代价地图）激光车头 ±%.0f° 内已空'
                                          % self.wait_fov_deg)
                if gap is None and waited >= self.blind_retry_period:
                    # 什么判据都没有 ⇒ 隔一会儿交给控制器自己试，别死等
                    return True, waited, '既没有代价地图也没有激光，交给控制器自己裁决'
            # ③ 代价地图说堵、但激光车头已经是空的 ⇒ **残留像素**，清一次
            elif gap is False:
                now = time.time()
                # 计数按**时间窗**累加（不是每拍都加）：
                # 清图后 costmap 要 1~2 拍才反映出来，若按拍计数会在 2 秒内就"放弃"。
                if now - last_futile >= 1.5:
                    futile += 1
                    last_futile = now
                    self._seg_wait['clears'] += 1
                    if futile > self.max_futile_probes:
                        return False, waited, (
                            '反复清图后代价地图仍显示前方有障碍（已试 %d 次）—— '
                            '激光明明已经空了，所以这多半不是真障碍，而是**代价地图里的残留'
                            '像素**。请打开 RViz 看 local_costmap（红色格子是否还在原地）；'
                            '若红格在 **global_costmap** 上，那是 SLAM 静态地图的幽灵，'
                            '需要重新建图或清除该区域。' % futile)
                    self.get_logger().warn(
                        '激光车头 ±%.0f° 内已空，但代价地图仍显示有障碍 —— 判定为'
                        '**残留像素**（障碍移开时激光返回 inf，nav2 不会据此 raycast 清除）。'
                        '清一次 %s 后重试（第 %d/%d 次）。'
                        % (self.wait_fov_deg, self.costmap_topic, futile,
                           self.max_futile_probes))
                if self._clear_local_costmap():
                    time.sleep(self.clear_settle)
                else:
                    time.sleep(self.blind_retry_period)
                continue

            # ④ 两边都说有东西 ⇒ 真障碍，继续等（+ 进度日志）
            if waited - last_log >= self.obstacle_log_period:
                last_log = waited
                where = ('车头 ±%.0f° 内最近回波 %.2f m（方位 %+.0f°）'
                         % (self.obstacle_fov_deg, narrow['distance'],
                            narrow['bearing_deg'])) if narrow else \
                        ('车头 ±%.0f° 内暂无有效回波（尚未收到 %s？）'
                         % (self.obstacle_fov_deg, self.scan_topic))
                extra = ''
                if self._costmap_last and self._costmap_last.get('at'):
                    extra = ('；代价地图上前方 %.2f m 处最坏代价 %s'
                             % (self._costmap_last['at'][0],
                                self._costmap_last['worst']))
                self.get_logger().warn(
                    '⏸ 停车等待障碍消失 —— %s 已等 %.0f s；%s%s。'
                    '~/stop 可中止本轮' % (wp.get('name', '?'), waited, where, extra))
                self._publish_status('waiting_obstacle',
                                     extra={'waypoint': wp.get('name'),
                                            'waited_s': round(waited, 1)})

            if self.obstacle_wait_timeout > 0 and waited >= self.obstacle_wait_timeout:
                return (False, waited, '等待障碍超时 %.0f s（obstacle_wait_timeout）'
                        % self.obstacle_wait_timeout)
            time.sleep(self.obstacle_poll_period)

        return False, time.time() - t0, '等待期间被 ~/stop 中止'

    def _on_obstacle_now(self):
        """实时读参数，让 on_obstacle 也能运行中热切。"""
        text = str(self.get_parameter('on_obstacle').value).lower()
        return text if text in ('wait', 'stop', 'skip') else 'wait'

    def _on_follow_feedback(self, feedback_msg):
        try:
            self._last_distance = round(float(feedback_msg.feedback.distance_to_goal), 3)
        except Exception:                             # noqa: BLE001
            pass

    def _await(self, future, timeout):
        """在 worker 线程里等 future（executor 在别的线程，回调会被正常处理）。"""
        deadline = time.time() + timeout
        while not future.done():
            if self._cancel_requested:
                return False
            if time.time() > deadline:
                return False
            time.sleep(0.05)
        return True

    def _cancel_current_goal(self):
        with self._goal_lock:
            handle = self._current_goal
        if handle is not None:
            try:
                handle.cancel_goal_async()
                self.get_logger().info('已请求取消当前导航目标')
            except Exception as exc:                  # noqa: BLE001
                self.get_logger().warn('取消目标失败: %s' % exc)

    def _on_feedback(self, feedback_msg):
        try:
            self._last_distance = round(float(feedback_msg.feedback.distance_remaining), 3)
        except Exception:                             # noqa: BLE001
            pass

    # ------------------------------------------------------------------ 拍照

    def _on_image(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn('图像转换失败: %s' % exc, throttle_duration_sec=5.0)
            return
        with self._frame_lock:
            self._latest_frame = frame
            self._latest_frame_time = time.time()

    def _capture(self, index, wp):
        import cv2

        # 等车体停稳、自动曝光收敛，同时确保拿到的是到点之后的帧
        arrived = time.time()
        deadline = arrived + max(wp['settle'], 0.0)
        while time.time() < deadline and not self._cancel_requested:
            time.sleep(0.1)

        # ★ 必须拿到"到点之后"的帧。图像话题可能很稀疏（大图走 DDS 时尤其明显），
        #   只等 settle 秒往往会一帧都没有 ⇒ 记成 arrived_but_no_image。
        #   这里再多等 capture_timeout 秒，专门等一帧新的。
        hard_deadline = arrived + max(wp['settle'], 0.0) + self.capture_timeout
        while True:
            with self._frame_lock:
                frame = None if self._latest_frame is None else self._latest_frame.copy()
                frame_time = self._latest_frame_time
            if frame is not None and frame_time is not None and frame_time >= arrived:
                break
            if self._cancel_requested or time.time() > hard_deadline:
                break
            time.sleep(0.05)

        if frame is None:
            self.get_logger().warn(
                '等 %.1fs 仍没收到任何图像（话题 %s）—— 相机没起？'
                '用 `ros2 topic info %s` 看 Publisher count 是否为 1'
                % (self.capture_timeout, self.image_topic, self.image_topic))
            return None
        if frame_time is not None and frame_time < arrived:
            self.get_logger().warn(
                '等 %.1fs 只拿到到点之前的旧帧（旧 %.1fs）。图像话题可能太稀疏，'
                '或大图消息在 DDS 里被丢（见 config/fastdds_large_shm.xml）。'
                % (self.capture_timeout, arrived - frame_time))

        stamp = time.strftime('%Y%m%d_%H%M%S')
        safe = ''.join(ch if ch.isalnum() or ch in '-_' else '_' for ch in wp['name'])
        filename = '%03d_%s_%s.jpg' % (index + 1, safe, stamp)
        path = os.path.join(self.save_dir, filename)
        ok = cv2.imwrite(path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            self.get_logger().error('照片写入失败: %s' % path)
            return None
        self.get_logger().info('照片已保存: %s' % path)
        return path

    # ------------------------------------------------------------------ 报告

    def _write_report(self, results, started, total):
        stamp = time.strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self.save_dir, 'report_%s.json' % stamp)
        payload = {
            'started_at': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(started)),
            'finished_at': time.strftime('%Y-%m-%d %H:%M:%S'),
            'duration_s': round(time.time() - started, 2),
            'route_file': os.path.expanduser(self.route_file),
            'frame_id': self.frame_id,
            'on_failure': self.on_failure,
            'on_obstacle': self._on_obstacle_now(),
            'segment_mode': self._segment_mode_now(),
            'aborted_reason': ('obstacle' if self._obstacle_hit else None),
            'obstacle': (dict(self._obstacle_info) if self._obstacle_info else None),
            # 遇障"停车等待"的统计（on_obstacle=wait 时才有值）
            'obstacle_waits_total': sum(r.get('obstacle_waits', 0) for r in results),
            'obstacle_wait_s_total': round(
                sum(float(r.get('obstacle_wait_s', 0.0) or 0.0) for r in results), 1),
            'costmap_clears_total': sum(r.get('costmap_clears', 0) for r in results),
            'total': total,
            'succeeded': sum(1 for r in results if r['ok']),
            'results': results,
            'note': '坐标为 map 坐标系；image 为绝对路径；status 取值为导航 action 的终态',
        }
        try:
            with open(path, 'w', encoding='utf-8') as handle:
                json.dump(payload, handle, ensure_ascii=False, indent=2)
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().error('报告写入失败: %s' % exc)
            return ''
        return path

    def _publish_status(self, state, index=None, total=None, name='', extra=None):
        payload = {'state': state}
        if index is not None:
            payload.update({'index': index + 1, 'total': total, 'name': name})
        if extra:
            payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    # ------------------------------------------------------------------ 退出

    def destroy_node(self):
        self._cancel_requested = True
        self._cancel_current_goal()
        if self._worker is not None and self._worker.is_alive():
            self._worker.join(timeout=3.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InspectionRoute()
    # 本节点既要跑 action 的异步回调，又要在独立线程里顺序推进航点，
    # 必须用多线程 executor，否则 worker 线程等 future 时会死锁。
    executor = MultiThreadedExecutor()
    run_node(node, executor)


if __name__ == '__main__':
    main()
