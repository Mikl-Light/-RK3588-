#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lifecycle_manager.py — 等待 slam_toolbox 就绪并完成 configure + activate

为什么需要它:
  原 launch 用固定延时(4s/6s)的 TimerAction 调 `ros2 lifecycle set` CLI,
  在 RK3588 上 DDS 发现慢于 4s → configure 报 "Node not found";
  activate 又在 configure 未成功时执行 → 节点仍在 unconfigured, 报
  "Unknown transition requested (available: configure/shutdown)"。
  slam_toolbox 停在 unconfigured = 不订阅 /scan、不发布 /map, 建图不工作。

本脚本:
  1. 轮询等待目标节点的 lifecycle 服务可查询 (超时 30s, DDS 慢自动重试)
  2. 若已 active 直接退出 (幂等, 重复 launch 不报错)
  3. 依次 configure → activate (各最多重试 3 次)
  4. 校验最终状态为 active, 成功退出 0 / 失败退出 1

用法:
  ros2 run mecanum_bringup lifecycle_manager                       # 默认 /slam_toolbox
  ros2 run mecanum_bringup lifecycle_manager --ros-args -p node_name:=/another_node

依赖: rclpy lifecycle_msgs (ROS2 自带)
"""

import rclpy
import sys
import time

from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetAvailableTransitions, GetState
from lifecycle_msgs.msg import Transition


class LifecycleManager(Node):
    def __init__(self):
        super().__init__('lifecycle_manager')
        self.declare_parameter('node_name', '/slam_toolbox')
        self.declare_parameter('timeout', 30.0)

        name = self.get_parameter('node_name').value
        self.timeout = self.get_parameter('timeout').value
        self.target = name if name.startswith('/') else '/' + name
        self.target = self.target.rstrip('/')

        self.get_logger().info(f'LifecycleManager: 目标 {self.target}, '
                               f'超时 {self.timeout:.0f}s')

        self.change_cli = self.create_client(
            ChangeState, f'{self.target}/change_state')
        self.avail_cli = self.create_client(
            GetAvailableTransitions, f'{self.target}/get_available_transitions')
        self.state_cli = self.create_client(
            GetState, f'{self.target}/get_state')

    # ---------------- 工具 ----------------
    def call_srv(self, cli, req, timeout_sec=10.0):
        """同步调用服务, 成功返回响应, 失败/超时返回 None"""
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)
        if fut.done() and fut.result() is not None:
            return fut.result()
        return None

    def wait_ready(self):
        """轮询等待节点 lifecycle 服务可查询 (DDS 发现慢时自动重试)"""
        deadline = time.time() + self.timeout
        while time.time() < deadline:
            if self.avail_cli.wait_for_service(timeout_sec=2.0):
                resp = self.call_srv(self.avail_cli,
                                     GetAvailableTransitions.Request())
                if resp is not None:
                    # 兼容不同 lifecycle_msgs 版本的 available_transitions 元素类型
                    # (labels 仅用于日志, 用 hasattr 探测避免版本差异导致崩溃):
                    #   Humble 及更早: 元素是 TransitionDescription
                    #     {transition(Transition), start_state(State), goal_state(State)}
                    #   Jazzy 2.0.x: 元素是 Transition, 只有 id/label
                    #     (start_state/goal_state 是后续版本才加进 Transition 的)
                    #   Rolling/最新: Transition 自带 start_state/goal_state
                    labels = []
                    for t in resp.available_transitions:
                        if hasattr(t, 'start_state'):
                            labels.append(
                                f'{t.start_state.label}->{t.goal_state.label}')
                        elif hasattr(t, 'transition'):
                            labels.append(t.transition.label)
                        else:
                            labels.append(t.label)
                    self.get_logger().info(
                        f'{self.target} 已就绪, 可用转换: {labels}')
                    return resp
            self.get_logger().info(f'等待 {self.target} lifecycle 服务就绪...')
        return None

    def get_state_label(self):
        resp = self.call_srv(self.state_cli, GetState.Request())
        return resp.current_state.label if resp is not None else None

    def do_transition(self, transition_id, label, retries=3):
        for attempt in range(1, retries + 1):
            if not self.change_cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(f'{label}: 服务不可用 (第 {attempt} 次)')
                continue
            req = ChangeState.Request()
            req.transition.id = transition_id
            req.transition.label = label
            resp = self.call_srv(self.change_cli, req, timeout_sec=15.0)
            if resp is not None and resp.success:
                self.get_logger().info(f'OK {label}')
                return True
            self.get_logger().warn(f'{label}: 失败 (第 {attempt}/{retries} 次)')
            time.sleep(1.0)
        return False

    # ---------------- 主流程 ----------------
    def run(self):
        if self.wait_ready() is None:
            self.get_logger().error(
                f'超时: {self.target} 的 lifecycle 服务一直不可用 '
                f'(节点没起来? 名字不对? 当前目标是 {self.target})')
            return False

        # 幂等: 已激活直接成功返回
        state = self.get_state_label()
        if state == 'active':
            self.get_logger().info(f'{self.target} 已 active, 无需操作')
            return True
        self.get_logger().info(f'{self.target} 当前状态: {state}')

        if not self.do_transition(Transition.TRANSITION_CONFIGURE, 'configure'):
            return False
        if not self.do_transition(Transition.TRANSITION_ACTIVATE, 'activate'):
            return False

        state = self.get_state_label()
        if state == 'active':
            self.get_logger().info(f'{self.target} 已激活 (active), 建图就绪')
            return True
        self.get_logger().error(f'激活后状态异常: {state}')
        return False


def main(args=None):
    rclpy.init(args=args)
    mgr = LifecycleManager()
    ok = False
    try:
        ok = mgr.run()
    except Exception as e:  # 兜底: 任何异常都让 launch 输出可见
        mgr.get_logger().error(f'异常: {e}')
    finally:
        mgr.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
