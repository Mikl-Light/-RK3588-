"""仓库巡检小车功能包。

所有节点均为独立实现。包含：
  chassis_bridge   底盘串口桥（/cmd_vel -> 串口 V 命令，带失控看门狗）
  scan_filter      激光自回波过滤（角度扇区 + 距离门限）
  map_despeckle    栅格地图去散斑（/map_raw -> /map）
  camera_source    USB 相机图像源（cv2/V4L2）
  inspection_route 航点巡航 + 到点拍照 + 巡检报告
"""

import rclpy

try:                                                  # 新版 rclpy 有，旧版没有
    from rclpy.executors import ExternalShutdownException
except ImportError:                                   # pragma: no cover
    class ExternalShutdownException(Exception):
        """兼容占位：旧版 rclpy 没有这个异常类型。"""


__version__ = '0.1.0'


def run_node(node, executor=None):
    """统一的 spin 入口。

    Ctrl-C（KeyboardInterrupt）与外部 kill/SIGTERM（ExternalShutdownException）
    都要收敛成干净退出，否则节点被正常关停时会打一整屏 traceback，
    现场排障时非常容易误判成"节点崩了"。
    """
    if executor is not None:
        executor.add_node(node)
    try:
        if executor is None:
            rclpy.spin(node)
        else:
            executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            if executor is not None:
                try:
                    executor.shutdown()
                except Exception:                     # noqa: BLE001
                    pass
            if rclpy.ok():
                rclpy.shutdown()
