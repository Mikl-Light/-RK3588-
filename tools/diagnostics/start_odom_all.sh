#!/bin/bash
# 启动里程计融合全套（serial + rplidar + scan_filter + rf2o + fusion）
# 用法：bash /tmp/start_odom_all.sh
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

# 清理旧进程
pkill -f 'serial_bridge'        2>/dev/null
pkill -f 'rplidar_node'         2>/dev/null
pkill -f 'warehouse_inspector/lib/warehouse_inspector/scan_filter' 2>/dev/null
pkill -f 'rf2o_laser_odometry_node' 2>/dev/null
pkill -f 'odom_fusion/fusion_node'  2>/dev/null
sleep 2

# 1) 串口桥：STM32 <-> ROS
setsid nohup ros2 launch mecanum_bringup mecanum_control.launch.py \
    > /tmp/n_serial.log 2>&1 < /dev/null &
sleep 3

# 2) 雷达 C1 -> /scan_raw
#    ⚠ 不能用 rplidar_c1_launch.py：它没有暴露 topic_name，雷达会发默认的 /scan，
#    于是 /scan 出现两个发布者（雷达 + scan_filter），scan_filter 订阅的 /scan_raw
#    永远是空的 —— 2026-10-22 实测踩过，min_range/mask_sectors 全部形同虚设。
setsid nohup ros2 run rplidar_ros rplidar_node --ros-args \
    -p channel_type:=serial \
    -p serial_port:=/dev/ttyUSB0 \
    -p serial_baudrate:=460800 \
    -p frame_id:=laser \
    -p inverted:=false \
    -p angle_compensate:=true \
    -p scan_mode:=Standard \
    -p topic_name:=scan_raw \
    > /tmp/n_lidar.log 2>&1 < /dev/null &
sleep 5

# 3) scan_filter: scan_raw -> /scan (BEST_EFFORT)
setsid nohup ros2 run warehouse_inspector scan_filter --ros-args \
    -r __node:=scan_filter \
    --params-file "$HOME/ros2_ws/src/warehouse_inspector/config/scan_filter_params.yaml" \
    > /tmp/n_filter.log 2>&1 < /dev/null &
sleep 3

# 4) rf2o -> /odom_laser (frame odom_laser)
setsid nohup ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args \
    -r __node:=rf2o_laser_odometry \
    -p laser_scan_topic:=/scan \
    -p odom_topic:=/odom_laser \
    -p base_frame_id:=base_link \
    -p odom_frame_id:=odom_laser \
    -p publish_tf:=true \
    -p init_pose_from_topic:="" \
    -p freq:=8.0 \
    > /tmp/n_rf2o.log 2>&1 < /dev/null &
sleep 3

# 5) 融合
setsid nohup ros2 launch odom_fusion odom_fusion.launch.py \
    > /tmp/n_fusion.log 2>&1 < /dev/null &
sleep 4

echo '=== 进程 ==='
pgrep -af 'serial_bridge|rplidar_node|scan_filter|rf2o_laser|fusion_node' \
    | grep -v pgrep | sed 's/--ros-args.*//' | sed 's/^/  /'
echo '=== 完成 ==='
