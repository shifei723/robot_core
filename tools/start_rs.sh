#!/bin/bash
# start_rs.sh — 启动 RealSense D435i（供测试/建图使用）
#
# 用法: start_rs.sh [imu]     加 imu 参数则尝试启用 IMU（见下方说明）
#
# 说明: D435i 的 IMU 走 HID→IIO 路径，librealsense 需要写
#   /sys/bus/iio/devices/iio:deviceN/{trigger/current_trigger,buffer/*}
# 这些 sysfs 文件默认是 root:root 0644，普通用户写不了。
# 因此启用 IMU 需要 root 权限或对应的 udev 权限规则。
# RGB-D 建图不需要 IMU，默认不启用。
set +u
source /opt/ros/humble/setup.bash

LOG=${LOG:-/tmp/rs_camera.log}
ARGS="enable_depth:=true enable_color:=true align_depth.enable:=true"

if [ "$1" = "imu" ]; then
    ARGS="$ARGS enable_gyro:=true enable_accel:=true unite_imu_method:=2"
fi

pkill -9 -f realsense2_camera_node 2>/dev/null
sleep 3

nohup ros2 launch realsense2_camera rs_launch.py $ARGS > "$LOG" 2>&1 &
echo "已启动，日志: $LOG"
