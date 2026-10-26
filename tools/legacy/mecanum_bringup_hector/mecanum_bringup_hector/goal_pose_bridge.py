#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
goal_pose_bridge.py — 让 rviz2 的 2D Goal Pose 能触发 Nav2 导航

问题:
  rviz2 默认 2D Goal Pose 只发布 /goal_pose (geometry_msgs/PoseStamped)
  Nav2 bt_navigator 只提供 /navigate_to_pose action
  两者不直接连接，所以有时 rviz 点目标点没反应。

方案:
  本节点订阅 /goal_pose，收到后转成 NavigateToPose action 发给 Nav2。

用法:
  ros2 run mecanum_bringup_hector goal_pose_bridge
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class GoalPoseBridge(Node):
    def __init__(self):
        super().__init__('goal_pose_bridge')
        self.sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.get_logger().info('goal_pose_bridge ready: /goal_pose -> /navigate_to_pose')

    def goal_cb(self, msg: PoseStamped):
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('navigate_to_pose action server not available')
            return

        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.get_logger().info(
            f'Sending goal from rviz: x={msg.pose.position.x:.2f}, '
            f'y={msg.pose.position.y:.2f}')
        self.action_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseBridge()
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
