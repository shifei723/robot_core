#!/bin/bash
# --- robot_core 可移植自定位 ---
ROBOT_CORE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")/../.." && pwd)"; export ROBOT_CORE
# ============================================================
# 建图一键启动脚本（SSH 启动 + VNC 远程可视化）
#
# 原理: 算法节点在后台跑，rviz2 显示到设备本机 X 桌面(:0)，
#       Mac 端通过 VNC (vnc://192.168.2.50:5900) 看画面。
#       rviz2 使用设备本地 GPU 渲染，只传像素，比 X11 转发流畅得多。
#
# 用法:
#   ./run_mapping.sh l2          # Unitree L2 雷达 + Point-LIO
#   ./run_mapping.sh mid360      # Mid360 + Point-LIO
#   ./run_mapping.sh fastlio2    # FAST-LIO2
#   ./run_mapping.sh livo        # FAST-LIVO2 (mid70 + 相机)
#   ./run_mapping.sh l2 --no-rviz  # 只跑算法不开界面
#   ./run_mapping.sh stop        # 停止建图（并自动停转降噪）
#   ./run_mapping.sh status      # 查看状态
#   ./run_mapping.sh save        # 保存当前地图为 pcd
#
# L2 噪声控制: 平时雷达保持停转静音，本脚本启动时自动启转、停止时自动停转。
#   手动控制: $ROBOT_CORE/scripts/run/setup_l2_lidar.sh quiet  (停转)
#             $ROBOT_CORE/scripts/run/setup_l2_lidar.sh spin   (启转)
#
# 日志: /tmp/mapping_logs/*.log
# ============================================================
# 注意: 不能用 set -u，ROS setup.bash 内部有未定义变量引用

ROS_WS="$ROBOT_CORE/install/ros_ws"
LOG_DIR=/tmp/mapping_logs
PIDFILE=$LOG_DIR/mapping.pids
# rviz2 显示到本机 X 桌面，Mac 通过 VNC 观看
export DISPLAY=${DISPLAY:-:0}
export XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}
# Livox/Unitree SDK 依赖，缺失会导致雷达驱动起不来
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0

stop_all() {
    # 传入 no-quiet 时跳过停转（启动前的清理场景，马上还要启转）
    local quiet_lidar="${1:-yes}"
    echo "=== 停止建图 ==="
    if [ -f "$PIDFILE" ]; then
        while read -r name pid; do
            kill -0 "$pid" 2>/dev/null && { kill "$pid" 2>/dev/null; echo "  [停止] $name"; }
        done < "$PIDFILE"
        rm -f "$PIDFILE"
    fi
    pkill -f pointlio_mapping 2>/dev/null
    pkill -f "fastlio2/lio_node" 2>/dev/null
    pkill -f fastlivo 2>/dev/null
    pkill -f unitree_lidar_ros2 2>/dev/null
    pkill -f livox_ros2_driver 2>/dev/null
    pkill -f hesai_ros_driver 2>/dev/null
    pkill -f "rviz2 -d" 2>/dev/null

    # L2 停转降噪: 建图结束后让雷达安静下来。
    # 必须在驱动退出后执行，否则两边争抢串口数据
    if [ "$quiet_lidar" = "yes" ] && \
       { [ -e /dev/unilidar ] || ls /dev/ttyACM* >/dev/null 2>&1; }; then
        sleep 2
        echo "  [停转] 让 L2 安静下来..."
        $ROBOT_CORE/scripts/run/setup_l2_lidar.sh quiet >/dev/null 2>&1 \
            && echo "  [停转] 已完成" \
            || echo "  [停转] 未成功（雷达可能未连接）"
    fi
    echo "=== 已停止 ==="
}

show_status() {
    echo "=== 建图组件状态 ==="
    for pat in unitree_lidar_ros2 livox_ros2_driver hesai_ros_driver \
               pointlio_mapping "fastlio2/lio_node" fastlivo rviz2; do
        pid=$(pgrep -f "$pat" | head -1)
        [ -n "$pid" ] && echo "  [运行中] $(basename "$pat") (pid=$pid)" \
                      || echo "  [未运行] $(basename "$pat")"
    done
    echo ""
    echo "=== 关键话题频率 (3秒采样) ==="
    source /opt/ros/humble/setup.bash 2>/dev/null
    true 2>/dev/null
    # 雷达原始数据 + 建图输出。注: Point-LIO 发布 /path 而非 /Odometry
    # 每个 ros2 topic hz 都是独立进程，各自要重做 DDS 发现，
    # 列表里第一个话题还要承担 CLI 首次加载开销，单次 4s 容易误报无数据，
    # 所以失败时用更长窗口重试一次再下结论
    for t in /unilidar/cloud /unilidar/imu /livox/lidar /cloud_registered /path; do
        hz=$(timeout 5 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
        if [ -z "$hz" ]; then
            hz=$(timeout 8 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
        fi
        [ -n "$hz" ] && echo "  $t : ${hz} Hz" || echo "  $t : 无数据"
    done
}

save_map() {
    source /opt/ros/humble/setup.bash
    true
    OUT=${1:-$ROS_WS/map_$(date +%m%d_%H%M).pcd}
    echo "正在保存点云地图到: $OUT"
    # 从建图输出的累积点云话题抓取一帧完整地图
    timeout 30 ros2 run pcl_ros pointcloud_to_pcd --ros-args \
        -r input:=/cloud_registered -p prefix:="${OUT%.pcd}_" 2>/dev/null \
        || echo "提示: 若失败，多数建图算法退出时会自动存 pcd，请查看算法配置的 map_file_path"
    ls -la "${OUT%.pcd}"* 2>/dev/null
}

MODE="${1:-}"
case "$MODE" in
    stop)   stop_all;   exit 0 ;;
    status) show_status; exit 0 ;;
    save)   save_map "$2"; exit 0 ;;
    l2|mid360|fastlio2|livo) ;;
    *) sed -n '4,20p' "$0" | sed 's/^# \?//'; exit 1 ;;
esac

USE_RVIZ=true
[ "$2" = "--no-rviz" ] && USE_RVIZ=false

# 启动前清理旧进程（no-quiet: 不停转，后面马上要用雷达）
stop_all no-quiet >/dev/null 2>&1
mkdir -p "$LOG_DIR"
: > "$PIDFILE"

source "$ROBOT_CORE/setup.sh"
true

echo "============================================"
echo " 启动建图: $MODE   $(date '+%F %T')"
echo " 可视化: $([ "$USE_RVIZ" = true ] && echo "rviz2 -> DISPLAY=$DISPLAY (VNC 观看)" || echo "关闭")"
echo "============================================"

# ── 1. 雷达驱动 ──
case "$MODE" in
    l2)
        # L2 的 ROS2 驱动配置为串口模式(initialize_type=1)，
        # 上电后雷达可能处于 UDP 模式，先自动确保切到串口模式
        echo "[0/2] 检查 L2 工作模式..."
        # 注意用 PIPESTATUS 取配置脚本的退出码，而非管道末端 sed 的
        $ROBOT_CORE/scripts/run/setup_l2_lidar.sh auto 2>&1 | grep -vE "WARNING|^$" | sed 's/^/      /'
        if [ "${PIPESTATUS[0]}" -ne 0 ]; then
            echo "      [警告] L2 模式配置未成功，仍尝试启动驱动"
        fi
        ros2 launch unitree_lidar_ros2 launch.py > "$LOG_DIR/lidar.log" 2>&1 &
        echo "lidar $!" >> "$PIDFILE"; echo "[1/2] Unitree L2 驱动已启动"
        ;;
    mid360|livo)
        ros2 launch livox_ros2_driver livox_lidar_msg_launch.py > "$LOG_DIR/lidar.log" 2>&1 &
        echo "lidar $!" >> "$PIDFILE"; echo "[1/2] Livox 驱动已启动"
        if [ "$MODE" = "livo" ]; then
            # FAST-LIVO2 需要相机，走 eth1 的 GigE 网口
            ros2 launch mvs_ros2_pkg mvs_camera_trigger.py > "$LOG_DIR/camera.log" 2>&1 &
            echo "camera $!" >> "$PIDFILE"; echo "      GigE 相机已启动"
        fi
        ;;
    fastlio2)
        ros2 launch hesai_ros_driver start.py > "$LOG_DIR/lidar.log" 2>&1 &
        echo "lidar $!" >> "$PIDFILE"; echo "[1/2] Hesai 驱动已启动"
        ;;
esac

# 等驱动出点云再启算法。L2 刚启转时转速未稳，头几帧可能是空点云，
# Point-LIO 拿到空帧会报 "lose lidar"，故给 L2 多留些时间
if [ "$MODE" = "l2" ]; then
    sleep 12
else
    sleep 5
fi

# ── 2. 建图算法（launch 内含 rviz2，用 rviz:= 控制）──
case "$MODE" in
    l2)       LAUNCH="point_lio mapping_unilidar_l2.launch.py" ;;
    mid360)   LAUNCH="point_lio mapping_mid360.launch.py" ;;
    fastlio2) LAUNCH="fastlio2 lio_launch.py" ;;
    livo)     LAUNCH="fast_livo mapping_mid70.launch.py" ;;
esac

if [ "$MODE" = "fastlio2" ] || [ "$MODE" = "livo" ]; then
    # 这两个 launch 无 rviz 开关参数，rviz2 随 launch 一起起
    ros2 launch $LAUNCH > "$LOG_DIR/mapping.log" 2>&1 &
else
    ros2 launch $LAUNCH rviz:=$USE_RVIZ > "$LOG_DIR/mapping.log" 2>&1 &
fi
echo "mapping $!" >> "$PIDFILE"
echo "[2/2] 建图算法已启动 ($LAUNCH)"

echo ""
echo "============================================"
echo " 建图已启动!"
echo ""
echo " 在 Mac 上查看可视化界面:"
echo "   Finder 按 Cmd+K -> vnc://192.168.2.50:5900 -> 输入 VNC 密码"
echo "   (rviz2 窗口就在该桌面上，点云实时渲染)"
echo ""
echo " 监控命令:"
echo "   $0 status                     # 组件状态 + 话题频率"
echo "   tail -f $LOG_DIR/mapping.log  # 建图日志"
echo "   tail -f $LOG_DIR/lidar.log    # 雷达日志"
echo "   $0 stop                       # 停止建图"
echo "============================================"
