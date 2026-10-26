#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
route_follower.py — 画线巡航 / 航点巡航后端

订阅:
  /route/follow_path   nav_msgs/Path   严格沿给定路径走，发给 Nav2 controller_server 的 /follow_path action
  /route/waypoints     nav_msgs/Path   依次到达每个离散航点，逐个发给 bt_navigator 的 /navigate_to_pose action
  /route/stop          std_srvs/Trigger 停止当前路径/航点任务并取消 action

发布:
  /route/status        std_msgs/String 当前状态，便于 Web/RViz 查看

设计:
  - 画线巡航不经过全局 planner，直接把整条 Path 交给 controller_server 的 FollowPath action。
  - 航点巡航使用 Nav2 的 NavigateToPose action，每个航点规划一次，到达后再去下一个。
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import FollowPath, NavigateToPose
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


def yaw_to_quat(yaw):
    """由 yaw(rad) 生成 geometry_msgs/Quaternion"""
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class RouteFollower(Node):
    def __init__(self):
        super().__init__('route_follower')

        self.declare_parameter('follow_path_action', '/follow_path')
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('waypoint_frame', 'map')
        self.declare_parameter('auto_continue_waypoints', True)

        # 避障安全检查参数
        self.declare_parameter('route_safety_check', True)
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('blocked_threshold', 65)
        self.declare_parameter('block_unknown', False)

        self.follow_path_action = self.get_parameter('follow_path_action').value
        self.navigate_action = self.get_parameter('navigate_action').value
        self.waypoint_frame = self.get_parameter('waypoint_frame').value
        self.auto_continue = self.get_parameter('auto_continue_waypoints').value
        self.safety_check = self.get_parameter('route_safety_check').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.blocked_threshold = self.get_parameter('blocked_threshold').value
        self.block_unknown = self.get_parameter('block_unknown').value
        self.costmap = None

        self.follow_path_client = ActionClient(self, FollowPath, self.follow_path_action)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, self.navigate_action)

        self.status_pub = self.create_publisher(String, '/route/status', 10)

        self.create_subscription(Path, '/route/follow_path', self.follow_path_cb, 10)
        self.create_subscription(Path, '/route/waypoints', self.waypoints_cb, 10)
        self.create_service(Trigger, '/route/stop', self.stop_cb)

        # 本地代价地图：用于检查画线路径/航点是否落在障碍上
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, self.costmap_topic, self.costmap_cb, map_qos)

        self.lock = threading.Lock()
        self.current_goal_handle = None
        self.active_mode = None          # 'path' | 'waypoints'
        self.waypoint_list = []
        self.waypoint_index = 0
        self.waypoint_frame = self.waypoint_frame

        self.get_logger().info(
            'route_follower ready: /route/follow_path (strict) or /route/waypoints (sequential)')
        self.publish_status('idle')

    # ================================================================ 状态
    def publish_status(self, text=None):
        if text is None:
            with self.lock:
                mode = self.active_mode or 'idle'
                idx = self.waypoint_index
                total = len(self.waypoint_list)
            text = f'mode={mode} waypoint={idx}/{total}'
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    # ================================================================ 避障安全检查
    def costmap_cb(self, msg: OccupancyGrid):
        self.costmap = msg

    def _world_blocked(self, x, y):
        """返回 (blocked, reason)；costmap 未就绪时返回 None"""
        cm = self.costmap
        if cm is None:
            return None, 'costmap not received'
        info = cm.info
        gx = int((x - info.origin.position.x) / info.resolution)
        gy = int((y - info.origin.position.y) / info.resolution)
        if gx < 0 or gy < 0 or gx >= info.width or gy >= info.height:
            return None, 'point outside costmap'
        v = cm.data[gy * info.width + gx]
        if v < 0:
            return (True, 'unknown') if self.block_unknown else (False, 'unknown')
        if v >= self.blocked_threshold:
            return True, f'occupied({v})'
        return False, 'free'

    def _pose_blocked(self, pose):
        if self.costmap is not None:
            pf = pose.header.frame_id
            cf = self.costmap.header.frame_id
            if pf and cf and pf != cf:
                return None, f'frame mismatch: waypoint={pf}, costmap={cf}'
        p = pose.pose.position
        return self._world_blocked(p.x, p.y)

    def path_is_safe(self, msg: Path):
        """画线路径安全检查：沿每段路径每 5cm 采样，若穿过障碍则拒绝。"""
        if not self.safety_check:
            return True, 'safety check disabled'
        if self.costmap is None:
            return False, f'costmap not ready: {self.costmap_topic}'
        if msg.header.frame_id and self.costmap.header.frame_id and msg.header.frame_id != self.costmap.header.frame_id:
            return False, (f'path frame={msg.header.frame_id} but costmap frame='
                           f'{self.costmap.header.frame_id}; cannot safety-check')

        step = 0.05
        for i in range(len(msg.poses) - 1):
            a = msg.poses[i].pose.position
            b = msg.poses[i + 1].pose.position
            dx, dy = b.x - a.x, b.y - a.y
            dist = math.hypot(dx, dy)
            n = max(1, int(dist / step))
            for k in range(n + 1):
                t = k / n
                x = a.x + dx * t
                y = a.y + dy * t
                blocked, reason = self._world_blocked(x, y)
                if blocked is True:
                    return False, f'path crosses obstacle at ({x:.2f},{y:.2f}): {reason}'
        return True, 'safe'

    # ================================================================ 严格画线
    def follow_path_cb(self, msg: Path):
        if len(msg.poses) < 2:
            self.get_logger().warn('follow_path ignored: path has < 2 poses')
            return

        safe, reason = self.path_is_safe(msg)
        if not safe:
            self.publish_status(f'path REJECTED: {reason}')
            self.get_logger().warn(f'画线路径未执行: {reason}')
            return

        if not self.follow_path_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(
                f'FollowPath action server not available: {self.follow_path_action}')
            self.publish_status('error: /follow_path not available')
            return

        goal = FollowPath.Goal()
        goal.path = msg
        try:
            goal.controller_id = ''
        except Exception:
            pass

        with self.lock:
            self.active_mode = 'path'
            self.waypoint_list = []
            self.waypoint_index = 0
        self.publish_status(f'path follow start: {len(msg.poses)} poses, frame={msg.header.frame_id}')

        future = self.follow_path_client.send_goal_async(
            goal, feedback_callback=self.path_feedback_cb)
        future.add_done_callback(self.path_response_cb)

    def path_response_cb(self, future):
        try:
            handle = future.result()
        except Exception as e:
            self.publish_status(f'path send failed: {e}')
            return
        if handle is None or not handle.accepted:
            with self.lock:
                self.active_mode = None
            self.publish_status('path rejected by controller_server')
            return
        with self.lock:
            self.current_goal_handle = handle
        self.publish_status('path accepted')
        handle.get_result_async().add_done_callback(self.path_result_cb)

    def path_feedback_cb(self, feedback_msg):
        # 限频，避免刷日志
        fb = feedback_msg.feedback
        try:
            self.get_logger().debug(
                f'path feedback: speed={fb.speed:.2f}, distance_to_goal={fb.distance_to_goal:.2f}')
        except Exception:
            pass

    def path_result_cb(self, future):
        try:
            status = future.result().status
        except Exception as e:
            self.publish_status(f'path result error: {e}')
            return
        with self.lock:
            self.current_goal_handle = None
            self.active_mode = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_status('path follow SUCCEEDED')
        elif status == GoalStatus.STATUS_CANCELED:
            self.publish_status('path follow CANCELED')
        else:
            self.publish_status(f'path follow finished status={status}')

    # ================================================================ 航点
    def waypoints_cb(self, msg: Path):
        if len(msg.poses) == 0:
            self.get_logger().warn('waypoints ignored: empty path')
            return
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(
                f'NavigateToPose action server not available: {self.navigate_action}')
            self.publish_status('error: /navigate_to_pose not available')
            return

        with self.lock:
            self.active_mode = 'waypoints'
            self.waypoint_list = list(msg.poses)
            self.waypoint_index = 0
            self.waypoint_frame = msg.header.frame_id or self.waypoint_frame
        self.publish_status(f'waypoints start: {len(self.waypoint_list)} points')
        self._send_next_waypoint()

    def _send_next_waypoint(self):
        pose = None
        while True:
            with self.lock:
                if self.active_mode != 'waypoints':
                    return
                idx = self.waypoint_index
                if idx >= len(self.waypoint_list):
                    self.active_mode = None
                    self.publish_status('waypoints ALL DONE')
                    return
                pose = self.waypoint_list[idx]

            if not pose.header.frame_id:
                pose.header.frame_id = self.waypoint_frame

            # 航点落在障碍里时，Nav2 的规划器通常会拒绝；这里提前跳过/中止。
            if self.safety_check:
                blocked, reason = self._pose_blocked(pose)
                if blocked is True:
                    self.publish_status(f'waypoint {idx} SKIPPED (obstacle: {reason})')
                    if self.auto_continue:
                        with self.lock:
                            self.waypoint_index = idx + 1
                        continue
                    with self.lock:
                        self.active_mode = None
                    self.publish_status('waypoints ABORTED: waypoint in obstacle')
                    return
            break

        goal = NavigateToPose.Goal()
        goal.pose = pose
        # 不用 behavior tree 的默认 recoveries? 保留默认即可
        future = self.nav_to_pose_client.send_goal_async(
            goal, feedback_callback=self.waypoint_feedback_cb)
        future.add_done_callback(lambda f, i=idx: self._waypoint_response_cb(f, i))

    def _waypoint_response_cb(self, future, idx):
        try:
            handle = future.result()
        except Exception as e:
            self.publish_status(f'waypoint {idx} send failed: {e}')
            return
        if handle is None or not handle.accepted:
            self.publish_status(f'waypoint {idx} rejected')
            if self.auto_continue:
                with self.lock:
                    self.waypoint_index += 1
                self._send_next_waypoint()
            else:
                with self.lock:
                    self.active_mode = None
            return

        with self.lock:
            self.current_goal_handle = handle
        self.publish_status(f'waypoint {idx} accepted')
        handle.get_result_async().add_done_callback(
       
            lambda f, i=idx: self._waypoint_result_cb(f, i))

    def _waypoint_result_cb(self, future, idx):
        try:
            status = future.result().status
        except Exception as e:
            self.publish_status(f'waypoint {idx} result error: {e}')
            status = None

        success = (status == GoalStatus.STATUS_SUCCEEDED)
        if success:
            self.publish_status(f'waypoint {idx} SUCCEEDED')
        else:
            self.publish_status(f'waypoint {idx} finished status={status}')

        if not success and not self.auto_continue:
            with self.lock:
                self.active_mode = None
            return

        with self.lock:
            self.waypoint_index = idx + 1
        self._send_next_waypoint()

    def waypoint_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        try:
            self.get_logger().debug(
                f'waypoint feedback: remaining={fb.distance_remaining:.2f}')
        except Exception:
            pass

    # ================================================================ 停止
    def stop_cb(self, request, response):
        with self.lock:
            handle = self.current_goal_handle
            self.active_mode = None
            self.waypoint_list = []
            self.waypoint_index = 0
            self.current_goal_handle = None
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
        self.publish_status('stopped by /route/stop')
        response.success = True
        response.message = 'route stopped'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RouteFollower()
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
