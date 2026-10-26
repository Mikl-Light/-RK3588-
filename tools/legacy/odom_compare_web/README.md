# odom_compare_web — 里程计对比 + 串口命令 Web 控制

独立 ROS2 包，不修改 mecanum_bringup / mecanum_bringup_hector。

## 功能

- 显示轮式 `/odom` 与激光 `/odom_laser` 轨迹、Yaw、速度、误差
- 设置基准点，显示 ΔX / ΔY / ΔYaw
- Web 按钮控制小车：W/S/Q/E/A/D/STOP
- Web 发送固件串口命令：STATUS / VERSION / TELEM / AHRSMAG / MAGCAL / ODORST / WALL 等
- 所有串口命令通过同一个串口桥进程发送，不重复打开串口

## 部署

```bash
# 拷贝到 ~/ros2_ws/src/odom_compare_web
cd ~/ros2_ws
colcon build --packages-select odom_compare_web
source ~/ros2_ws/install/setup.bash

ros2 launch odom_compare_web odom_compare.launch.py
```

## 浏览器

```text
http://<RK3588_IP>:8765/
```

## 端口

8765

防火墙：

```bash
sudo ufw allow 8765/tcp
```
