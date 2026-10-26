#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
map_trigger.py — 获取 Nav2 map_server 地图并持续重发 /map

Jazzy 的 map_server 激活后不会主动发 /map，
必须调用 /map_server/map 服务才会发布一次地图。
但一次 transient local 发布在 Fast DDS 下可能不是所有节点都能收到。

本节点:
  1. 调用 /map_server/map 服务拿回完整地图
  2. 以 1Hz 持续发布 /map
  3. 让 planner_server / rviz2 等都能稳定收到地图

用法:
  ros2 run mecanum_bringup_hector map_trigger
"""

import rclpy
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from std_srvs.srv import Empty


class MapTrigger(Node):
    def __init__(self):
        super().__init__('map_trigger')
        # Jazzy 的服务名是 /map_server/map，不是 /map_server/get_map
        self.client = self.create_client(GetMap, '/map_server/map')
        # 与 map_server 一致：transient_local + reliable，才能被 planner/AMCL 的
        # static_layer / map 订阅者匹配。同时我们以 1Hz 持续发，避免只发一次收不到。
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.latest_map = None
        self.global_loc_client = self.create_client(Empty, '/reinitialize_global_localization')
        self.get_logger().info('Waiting for /map_server/map service...')

    def fetch_map(self):
        if not self.client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error('map_server/map service not available')
            return False

        for i in range(30):
            if not self.client.service_is_ready():
                time.sleep(1.0)
                continue
            req = GetMap.Request()
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.done() and future.result() is not None:
                self.latest_map = future.result().map
                self.get_logger().info(
                    f'Got map {self.latest_map.info.width}x{self.latest_map.info.height}, '
                    f'start republishing /map at 1Hz')
                return True
            self.get_logger().warn(f'map service call failed, retry {i + 1}/30')
            time.sleep(1.0)

        self.get_logger().error('Failed to get map from map_server')
        return False

    def timer_cb(self):
        if self.latest_map is not None:
            now = self.get_clock().now().to_msg()
            msg = self.latest_map
            msg.header.stamp = now
            msg.header.frame_id = 'map'
            self.map_pub.publish(msg)

    def global_loc_done(self, future):
        if future.done() and future.result() is not None:
            self.get_logger().info('AMCL global localization triggered (no initial pose needed)')
        else:
            self.get_logger().warn('AMCL global localization call failed')

    def call_global_localization(self):
        """AMCL 全局重定位：粒子撒满自由地图，不需要手动给初始位姿。
        注意：这里处于 timer 回调中，rclpy 已在 spin，不能再用 spin_until_future_complete。"""
        if not self.global_loc_client.service_is_ready():
            self.get_logger().warn('reinitialize_global_localization service not available yet')
            return
        req = Empty.Request()
        future = self.global_loc_client.call_async(req)
        future.add_done_callback(self.global_loc_done)

    def run(self):
        if not self.fetch_map():
            return False
        self.create_timer(1.0, self.timer_cb)
        # 地图稳定后触发一次全局重定位
        self.create_timer(3.0, self.call_global_localization)
        rclpy.spin(self)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = MapTrigger()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
