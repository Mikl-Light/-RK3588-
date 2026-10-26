#!/usr/bin/env python3
"""栅格地图去散斑：/map_raw -> /map

背景（实测数据）：
  在线 SLAM 生成的栅格图里存在大量**孤立占据像素**——本车地图里占全部
  占据像素的 17.6%（510 格中 90 格周围占据邻居 <= 1）。在代价地图里，
  每个孤立像素都会长出一个半径 inflation_radius 的高代价圆盘。散斑越密，
  自由空间被切得越碎，狭窄空间里就会出现「明明能过却规划不出路」的假象。

做法：
  把「8 邻域内占据邻居数 < min_neighbors」的占据像素判为噪点，改写为空闲。
  可重复 passes 次，逐轮收敛。
  可选 fill_holes：把被占据像素完全包围的小块空闲填成占据，避免墙体被
  掏空成「可通行」。

接法（重要）：
  slam_toolbox 的 map 话题重映射为 /map_raw，本节点输出 /map，
  nav2 的 global_costmap static_layer 订阅 /map —— 这样规划器用的是清洗
  过的地图，而原始地图仍可在 RViz 里对照查看。
"""

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from . import run_node

LATCHED_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class MapDespeckle(Node):

    def __init__(self):
        super().__init__('map_despeckle')

        self.declare_parameter('map_in', '/map_raw')
        self.declare_parameter('map_out', '/map')
        self.declare_parameter('occupied_thresh', 65)
        self.declare_parameter('min_neighbors', 2)
        self.declare_parameter('passes', 1)
        self.declare_parameter('fill_holes', False)
        self.declare_parameter('min_hole_neighbors', 8)

        self.occupied_thresh = int(self.get_parameter('occupied_thresh').value)
        self.min_neighbors = int(self.get_parameter('min_neighbors').value)
        self.passes = max(1, int(self.get_parameter('passes').value))
        self.fill_holes = bool(self.get_parameter('fill_holes').value)
        self.min_hole_neighbors = int(self.get_parameter('min_hole_neighbors').value)

        self._last_payload = None
        self._published = 0

        self.pub = self.create_publisher(OccupancyGrid, self.get_parameter('map_out').value, LATCHED_QOS)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_in').value, self._on_map, LATCHED_QOS)

        self.get_logger().info(
            '地图去散斑就绪: %s -> %s, 占据阈值 %d, 孤立判据 < %d 个邻居, %d 轮, 填洞 %s' % (
                self.get_parameter('map_in').value, self.get_parameter('map_out').value,
                self.occupied_thresh, self.min_neighbors, self.passes, self.fill_holes))

    # --------------------------------------------------------------- 邻居统计

    @staticmethod
    def _neighbor_count(mask: np.ndarray) -> np.ndarray:
        """统计每个格子的 8 邻域中 mask 为真的个数。"""
        h, w = mask.shape
        count = np.zeros((h, w), dtype=np.uint8)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                shifted = np.zeros_like(mask)
                src_y = slice(max(0, dy), h + min(0, dy))
                dst_y = slice(max(0, -dy), h + min(0, -dy))
                src_x = slice(max(0, dx), w + min(0, dx))
                dst_x = slice(max(0, -dx), w + min(0, -dx))
                shifted[dst_y, dst_x] = mask[src_y, src_x]
                count += shifted.astype(np.uint8)
        return count

    # ------------------------------------------------------------------ 主逻辑

    def _on_map(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            self.get_logger().warn('收到空地图，忽略')
            return

        raw = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
        unknown = raw < 0
        occupied = raw >= self.occupied_thresh

        original_occupied = int(occupied.sum())
        occupied = occupied.copy()

        for _ in range(self.passes):
            count = self._neighbor_count(occupied)
            speckle = occupied & (count < self.min_neighbors)
            if not speckle.any():
                break
            occupied = occupied & ~speckle

        removed = original_occupied - int(occupied.sum())

        filled = 0
        if self.fill_holes:
            free = ~occupied & ~unknown
            count = self._neighbor_count(occupied)
            holes = free & (count >= self.min_hole_neighbors)
            filled = int(holes.sum())
            occupied = occupied | holes

        out_data = np.where(unknown, -1, np.where(occupied, 100, 0)).astype(np.int8)

        payload = (msg.info.width, msg.info.height, out_data.tobytes())
        if payload == self._last_payload:
            return                                  # 地图没变，不必重发
        self._last_payload = payload

        out = OccupancyGrid()
        out.header.frame_id = msg.header.frame_id or 'map'
        out.header.stamp = self.get_clock().now().to_msg()
        out.info = msg.info
        out.data = out_data.flatten().tolist()
        self.pub.publish(out)

        self._published += 1
        total = max(1, w * h)
        self.get_logger().info(
            '地图已清洗(第%d次): 占据 %d -> %d (去掉 %d 个噪点, 占全部占据 %.1f%%), 填洞 %d, 尺寸 %dx%d' % (
                self._published, original_occupied, int(occupied.sum()), removed,
                100.0 * removed / max(1, original_occupied), filled, w, h),
            throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    run_node(MapDespeckle())


if __name__ == '__main__':
    main()
