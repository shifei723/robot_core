#!/bin/bash
# --- robot_core 可移植自定位 ---
ROBOT_CORE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")/../.." && pwd)"; export ROBOT_CORE
# test_toolcall.sh — Tool-Call 链路联调测试
# 用假导航服务 + mock 底盘（复刻 wl_base_node 的接收校验逻辑），不需真实 Nav2/硬件
# 用法: bash test_toolcall.sh
set +u
SCRIPTS=$ROBOT_CORE/voice/agent/src/omni_node/scripts

source "$ROBOT_CORE/setup.sh"
true
export PYTHONUNBUFFERED=1

echo "=== 清理旧实例 ==="
pkill -f tts_bridge.py 2>/dev/null
pkill -f tool_agent.py 2>/dev/null
pkill -f fake_nav_server.py 2>/dev/null
pkill -f mock_wl_base.py 2>/dev/null
sleep 2

cd "$SCRIPTS" || exit 1
FAKE_NAV_SEC=3 python3 fake_nav_server.py > /tmp/fakenav.log 2>&1 &
PID_NAV=$!
python3 mock_wl_base.py > /tmp/mockbase.log 2>&1 &
PID_BASE=$!
python3 tool_agent.py > /tmp/toolagent.log 2>&1 &
PID_AGENT=$!
python3 tts_bridge.py --ros-args -p backend:=hobot > /tmp/ttsbridge.log 2>&1 &
PID_TTS=$!
sleep 10

echo "=== 启动情况 ==="
echo "--- tool_agent ---"; head -4 /tmp/toolagent.log
echo "--- mock_wl_base ---"; head -2 /tmp/mockbase.log
echo "--- tts_bridge ---"; head -2 /tmp/ttsbridge.log
echo "--- fake_nav ---";   head -2 /tmp/fakenav.log

echo ""
echo "=== 注入多任务指令并观察 ==="
timeout 60 python3 probe_toolcall.py 2>&1

echo ""
echo "=== tool_agent 执行轨迹 ==="
grep -E "解析出|执行:|播报|完成|未知" /tmp/toolagent.log | head -20

echo ""
echo "=== 假导航服务收到的请求 ==="
grep -E "收到导航请求|已到达" /tmp/fakenav.log | head -6

echo ""
echo "=== tts_bridge 是否拦下 JSON ==="
grep -E "跳过工具调用|TTS#" /tmp/ttsbridge.log | head -8

# 先结束 mock 底盘，让它打印统计报告（校验关节名/限幅/频率/回中位）
kill -INT $PID_BASE 2>/dev/null
sleep 3
echo ""
echo "=== mock 底盘校验结果 ==="
grep -E "丢弃|限幅|超时保护" /tmp/mockbase.log | head -6
sed -n '/MockWlBase 统计报告/,$p' /tmp/mockbase.log

kill $PID_NAV $PID_AGENT $PID_TTS $PID_BASE 2>/dev/null
echo ""
echo "=== 测试结束，已清理 ==="
