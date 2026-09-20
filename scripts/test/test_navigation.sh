#!/usr/bin/env bash
# --- robot_core 可移植自定位 ---
ROBOT_CORE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")/../.." && pwd)"; export ROBOT_CORE
# test_navigation.sh — 本地验证「发布目标 → 规划路径」链路(不依赖 Foxglove/rviz)
#
# 做的事:
#   1. 前置检查: 定位链路(/odometry_base)与规划器(/planning/bspline 有发布者)是否就绪
#   2. 从 TF map->base_link 取当前位姿, 在"正前方 DIST 米"生成一个目标点
#   3. 发到 /goal_pose (map 系) → goal_frame_bridge 转 world → 规划器
#   4. 在时间窗内监听三段话题, 逐段报 PASS/FAIL:
#        /planner/goal_world  (目标点已桥接到 world)
#        /planning/bspline    (规划器已产出轨迹 = 规划成功)
#        /cmd_vel             (控制器已输出速度指令)
#
# ★ 安全警告: 真机模式下 /planning/bspline 一出, 控制器就会往 /cmd_vel 发速度,
#   机器人会真的动! 首次测试请把机器人架空 / 断开底盘 / 手按急停。
#
# 用法:
#   ./test_navigation.sh              # 正前方 1.0m 处放目标
#   ./test_navigation.sh 1.5          # 正前方 1.5m
#   ./test_navigation.sh --at 2.0 0.5 # 直接指定 map 系绝对坐标 (x=2.0, y=0.5)
#   ./test_navigation.sh --yes 1.0    # 跳过 5 秒倒计时确认
set -u

source "$ROBOT_CORE/setup.sh"
true 2>/dev/null
true 2>/dev/null

# ── 解析参数 ──
SKIP_CONFIRM=0
MODE="forward"; DIST="1.0"; ABS_X=""; ABS_Y=""
args=()
for a in "$@"; do
    case "$a" in
        --yes) SKIP_CONFIRM=1 ;;
        --at)  MODE="abs" ;;
        *)     args+=("$a") ;;
    esac
done
if [ "$MODE" = "abs" ]; then
    ABS_X="${args[0]:-}"; ABS_Y="${args[1]:-0.0}"
    [ -z "$ABS_X" ] && { echo "用法: $0 --at <x> <y>"; exit 1; }
else
    [ -n "${args[0]:-}" ] && DIST="${args[0]}"
fi

echo "============================================================"
echo " 导航链路本地测试 (发布目标 → 规划路径)"
echo "============================================================"

# ── 安全倒计时 ──
if [ "$SKIP_CONFIRM" != "1" ]; then
    echo ""
    echo " ★★★ 规划成功后机器人会开始移动! 确保已架空/急停在手 ★★★"
    echo "     5 秒后开始, Ctrl-C 取消..."
    for i in 5 4 3 2 1; do printf "\r     %d " "$i"; sleep 1; done
    echo ""
fi

MODE="$MODE" DIST="$DIST" ABS_X="$ABS_X" ABS_Y="$ABS_Y" python3 - <<'PY'
import math, os, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = rclpy.create_node('nav_link_test')

# ---- 前置检查: 定位与规划器就绪吗 ----
def spin(sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(node, timeout_sec=0.1)

print("\n[1/4] 前置检查 ...")
ok_loc = node.count_publishers('/odometry_base') > 0
ok_plan = node.count_publishers('/planning/bspline') > 0
print(f"      定位 /odometry_base 发布者: {'有 ✔' if ok_loc else '无 ✗'}")
print(f"      规划器 /planning/bspline 发布者: {'有 ✔' if ok_plan else '无 ✗'}")
if not ok_loc:
    print("      [中止] 定位链路没在跑, 先: ./run_vins_localization.sh")
    node.destroy_node(); rclpy.shutdown(); raise SystemExit(1)
if not ok_plan:
    print("      [中止] 规划器没在跑, 先: ./run_vins_navigation.sh --planner-only --no-rviz")
    node.destroy_node(); rclpy.shutdown(); raise SystemExit(1)

# ---- 取当前位姿 map->base_link ----
buf = Buffer(); TransformListener(buf, node)
print("\n[2/4] 读取当前位姿 (TF map->base_link) ...")
cur = None
t0 = time.time()
while time.time() - t0 < 5.0:
    rclpy.spin_once(node, timeout_sec=0.1)
    try:
        tf = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        cur = tf.transform; break
    except Exception:
        continue
if cur is None:
    print("      [中止] 拿不到 map->base_link TF, 定位可能未锁定")
    node.destroy_node(); rclpy.shutdown(); raise SystemExit(1)

px, py = cur.translation.x, cur.translation.y
q = cur.rotation
yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
print(f"      当前位姿: x={px:.2f} y={py:.2f} yaw={math.degrees(yaw):.1f}°")

# ---- 计算目标点 ----
mode = os.environ['MODE']
if mode == 'abs':
    gx, gy = float(os.environ['ABS_X']), float(os.environ['ABS_Y'])
    gyaw = math.atan2(gy - py, gx - px)
else:
    d = float(os.environ['DIST'])
    gx, gy = px + d*math.cos(yaw), py + d*math.sin(yaw)
    gyaw = yaw
print(f"      目标点(map): x={gx:.2f} y={gy:.2f} yaw={math.degrees(gyaw):.1f}°")

# ---- 监听三段话题 ----
got = {'goal_world': 0, 'bspline': 0, 'cmd_vel': 0}
rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
node.create_subscription(PoseStamped, '/planner/goal_world',
                         lambda m: got.__setitem__('goal_world', got['goal_world']+1), rel)
node.create_subscription(Twist, '/cmd_vel',
                         lambda m: got.__setitem__('cmd_vel', got['cmd_vel']+1), 10)
try:
    from scan_planner_msgs.msg import Bspline
    node.create_subscription(Bspline, '/planning/bspline',
                             lambda m: got.__setitem__('bspline', got['bspline']+1), rel)
    have_bspline_type = True
except Exception as e:
    have_bspline_type = False
    print(f"      (注: 未能导入 Bspline 类型, 将用发布者活跃度间接判断: {e})")

# ---- 发布目标 (连发 3 次确保送达 queue=1 的订阅者) ----
pub = node.create_publisher(PoseStamped, '/goal_pose', rel)
def make_goal():
    g = PoseStamped()
    g.header.frame_id = 'map'
    g.header.stamp = node.get_clock().now().to_msg()
    g.pose.position.x, g.pose.position.y = gx, gy
    g.pose.orientation.z = math.sin(gyaw/2); g.pose.orientation.w = math.cos(gyaw/2)
    return g
print("\n[3/4] 发布目标到 /goal_pose ...")
for _ in range(3):
    pub.publish(make_goal()); spin(0.3)

print("\n[4/4] 监听 8 秒, 观察链路各段 ...")
spin(8.0)

print("\n============================================================")
print(" 结果")
print("============================================================")
def line(name, topic, n):
    print(f"  {'PASS ✔' if n>0 else 'FAIL ✗'}  {name:22s} {topic:22s} 收到 {n} 帧")
line("目标已桥接到 world", "/planner/goal_world", got['goal_world'])
if have_bspline_type:
    line("规划器产出轨迹",   "/planning/bspline",  got['bspline'])
else:
    print(f"  ----   规划器产出轨迹        /planning/bspline      (类型未加载, 未统计)")
line("控制器输出速度",     "/cmd_vel",           got['cmd_vel'])
print("------------------------------------------------------------")
if got['goal_world'] == 0:
    print("  诊断: 目标没到 world → goal_frame_bridge 未运行, 或 map->world TF 缺失")
elif have_bspline_type and got['bspline'] == 0:
    print("  诊断: 目标已收到但没出轨迹 → 目标落在障碍/未知区, 或规划器 planGlobalTraj 失败")
    print("        换个近一点、在已建图空旷区的目标再试; 看规划器终端日志的 FSM 状态")
elif got['cmd_vel'] == 0:
    print("  诊断: 有轨迹但没速度 → 控制器(closed_loop_controller)未起或未订阅到轨迹")
else:
    print("  链路走通 ✔  发布目标 → 桥接 → 规划轨迹 → 速度指令 全部有输出")
print("============================================================")

node.destroy_node(); rclpy.shutdown()
PY
