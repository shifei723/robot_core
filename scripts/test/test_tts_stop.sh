#!/bin/bash
# test_tts_stop.sh — 验证改造后的 hobot_tts 真正取消（而非只是静音）
# 关键验证点：打断后再发新内容，念的必须是新内容，不能接着念旧的
set +u
source /opt/ros/humble/setup.bash
source /data/sf_code/tts/hobot_ws/install/setup.bash
export TROS_DISTRO=humble
export GLOG_minloglevel=1
export PYTHONUNBUFFERED=1

echo "=== 停掉所有 hobot_tts 实例 ==="
pkill -f "lib/hobot_tts/hobot_tts" 2>/dev/null
pkill -f "run hobot_tts" 2>/dev/null
sleep 3

echo "=== 启动改造版 hobot_tts ==="
ros2 run hobot_tts hobot_tts --ros-args -p playback_device:=default \
    > /tmp/newtts.log 2>&1 &
PID=$!
sleep 15
echo "--- 启动日志 ---"
grep -E "Interrupt topic|Sample rate|Max seconds" /tmp/newtts.log

echo ""
echo "=== 执行打断测试 (用 Python 发布器，避免 topic pub --once 丢消息) ==="
timeout 60 python3 /data/sf_code/agent_ws/src/omni_node/scripts/probe_tts_stop.py 2>&1

echo ""
echo "--- 打断日志 ---"
grep -E "\[Interrupt\]" /tmp/newtts.log || echo "(未出现打断日志 = 打断未生效)"

kill $PID 2>/dev/null
pkill -f "lib/hobot_tts/hobot_tts" 2>/dev/null
echo ""
echo "=== 测试结束 ==="
