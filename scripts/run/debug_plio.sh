#!/bin/bash
# 单独重启 point_lio（保持雷达驱动运行），验证 lose lidar 是否为启动竞态
set +u
source /opt/ros/humble/setup.bash
source /data/sf_code/ros_ws/install/setup.bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0
export PYTHONUNBUFFERED=1

echo "=== 重启前: 雷达驱动是否还在 ==="
pgrep -f unitree_lidar_ros2_node >/dev/null && echo "驱动运行中" || echo "驱动已退出"

pkill -f pointlio_mapping 2>/dev/null
sleep 3

echo "=== 单独重启 point_lio ==="
ros2 launch point_lio mapping_unilidar_l2.launch.py rviz:=false > /tmp/plio_restart.log 2>&1 &
PID=$!
sleep 30

echo "=== point_lio 日志尾部 ==="
grep -vE "WARNING" /tmp/plio_restart.log | tail -8

echo ""
echo "=== 建图输出话题 ==="
for t in /cloud_registered /path; do
    printf "%-20s " "$t"
    timeout 10 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" || echo "无数据"
done

kill $PID 2>/dev/null
