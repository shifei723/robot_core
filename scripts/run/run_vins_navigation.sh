#!/bin/bash
# run_vins_navigation.sh — VINS+rtabmap 定位 之上叠加 SCAN-Planner 局部导航
#
# 关系:  run_vins_mapping.sh   → 建图(写库)
#        run_vins_localization.sh → 定位(只读库) + 发布 /odometry_base
#        run_vins_navigation.sh   → 在定位链路之上再拉起局部规划器(本脚本)
#
# 为什么几乎不用改接口: SCAN-Planner 的真机模式(is_real_world:=true)本就是
#   按这条 VINS+rtabmap 链路设计的(见 run.launch.py 的 is_real 分支)——
#     body_pose/sensor_pose = /odometry_base   ← odom_to_tf.py 发布的"连续局部"里程计
#     depth                = /camera/camera/aligned_depth_to_color/image_raw
#     目标点 /goal_pose(map) → goal_frame_bridge.py 用 tf 转到 world → 规划器
#     输出   /cmd_vel + /cmd_posture            → 交给底盘驱动(外部)
#
# 位姿连续性(关键): 规划全程跑在 VINS 的 world 系(连续、不跳)。rtabmap 重定位
#   只修正 map→world 这段 TF，绝不直接喂给规划器——否则重定位瞬间的位姿跳变
#   会把正在跟踪的轨迹拽断。全局一致性由 goal_frame_bridge 把 map 系目标点
#   实时转到 world 系来保证(见 development_practice_specification: 规划器位姿输入安全架构)。
#
# 用法:
#   run_vins_navigation.sh                 (定位未起则自动拉起) 起规划器 + 导航专用 rviz
#   run_vins_navigation.sh --no-rviz       同上但不开 rviz(用主机 Foxglove 时选这个)
#   run_vins_navigation.sh --planner-only  只起规划器(定位须已在跑, 适合你当前工作流)
#   run_vins_navigation.sh status          查看状态
#   run_vins_navigation.sh stop            停止规划器 + 定位(整体下电)
#   run_vins_navigation.sh stop --keep-loc 只停规划器, 保留定位继续跑
#
# 在 rviz 里用 "2D Goal Pose" 工具点目标即可开始导航(navi_mode=1)。
set +u

LOG_DIR=/tmp/vins_navigation_logs
LOC=/data/sf_code/run_vins_localization.sh
WS_NAV=/data/sf_code/ws_nav
MAP_DIR=/data/sf_code/rtabmap_maps
# 导航专用 rviz(只放导航相关: 位姿/2D地图/障碍点云/规划轨迹/目标点 + 2D Goal Pose 工具)
RVIZ_CFG=${RVIZ_CFG:-$MAP_DIR/vins_nav.rviz}
# 规划器真机参数(可用环境变量覆盖)。默认取历史验证值。
ROBOT_MODEL=${ROBOT_MODEL:-slash}
SENSOR_TYPE=${SENSOR_TYPE:-depth}
NAVI_MODE=${NAVI_MODE:-1}
CONTROLLER_MODE=${CONTROLLER_MODE:-closed_loop}
USE_GPU=${USE_GPU:-false}
# 真机首跑限速(深度相机有效视距只有 ~3m, 视野外全 unknown, 速度必须压低)
REAL_MAX_VEL=${REAL_MAX_VEL:-0.3}
REAL_MAX_ACC=${REAL_MAX_ACC:-0.3}
REAL_MAX_VYAW=${REAL_MAX_VYAW:-0.5}
# navi_mode=2(预设路径点)时需要的 keypoints 参数文件
KEYPOINTS_FILE=${KEYPOINTS_FILE:-}
# 定位链路依赖的连续局部里程计话题(规划器的位姿输入)
ODOM_BASE_TOPIC=/odometry_base
# PCL 报 libusb_set_option undefined 的老坑(同建图/定位脚本)
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0

need_source() {
    source /opt/ros/humble/setup.bash
    if [ -f "$WS_NAV/install/setup.bash" ]; then
        source "$WS_NAV/install/setup.bash"
    else
        echo "[错误] $WS_NAV/install/setup.bash 不存在，请先编译 ws_nav 工作空间!"
        echo "       cd $WS_NAV && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
        exit 1
    fi
}

wait_log() {
    local f=$1 pat=$2 tmo=$3 i=0
    while [ "$i" -lt "$tmo" ]; do
        grep -q "$pat" "$f" 2>/dev/null && return 0
        sleep 0.5; i=$((i+1))
    done
    return 1
}

wait_node() {
    local node=$1 tmo=$2 i=0
    while [ "$i" -lt "$tmo" ]; do
        ros2 node list 2>/dev/null | grep -qx "$node" && return 0
        sleep 0.5; i=$((i+1))
    done
    return 1
}

wait_topic() {
    local t=$1 tmo=$2 i=0
    while [ "$i" -lt "$tmo" ]; do
        ros2 topic list 2>/dev/null | grep -qx "$t" && return 0
        sleep 0.5; i=$((i+1))
    done
    return 1
}

wait_gone() {
    local pat=$1 tmo=$2 i=0
    while [ "$i" -lt "$((tmo*2))" ]; do
        pgrep -f "$pat" >/dev/null 2>&1 || return 0
        sleep 0.5; i=$((i+1))
    done
    return 1
}

# 定位链路是否已在跑: /rtabmap 节点在 且 /odometry_base 有发布
loc_running() {
    ros2 node list 2>/dev/null | grep -qx /rtabmap && \
    ros2 topic list 2>/dev/null | grep -qx "$ODOM_BASE_TOPIC"
}

# 规划器组件清单(stop/status 共用; 模式首字符用字符类包住, 防 pgrep 匹配到自己)
NAV_COMPONENTS=("scan_planner run.launch.py" scan_planner_node closed_loop_controller open_loop_controller goal_frame_bridge.py leg_height_to_posture.py robot_state_publisher "rviz2.*vins_nav.rviz")

case "$1" in
status)
    need_source
    echo "=== 定位链路 ==="
    if loc_running; then
        echo "  [运行中] /rtabmap + $ODOM_BASE_TOPIC"
    else
        echo "  [未运行] 定位链路未就绪，规划器缺少位姿/深度输入"
    fi
    echo ""
    echo "=== 规划器组件 ==="
    for p in "${NAV_COMPONENTS[@]}"; do
        pid=$(pgrep -f "[${p:0:1}]${p:1}" | head -1)
        [ -n "$pid" ] && echo "  [运行中] $p (pid=$pid)" || echo "  [未运行] $p"
    done
    echo ""
    echo "=== 关键话题 ==="
    for t in "$ODOM_BASE_TOPIC" /goal_pose /planner/goal_world /cmd_vel /cmd_posture /planning/bspline; do
        printf "  %-24s " "$t"
        ros2 topic list 2>/dev/null | grep -qx "$t" && echo "存在" || echo "缺失"
    done
    echo ""
    echo "=== 规划器节点状态 ==="
    if wait_node /scan_planner_node 1; then
        echo "  scan_planner_node 已注册"
    else
        echo "  scan_planner_node 未注册（查看 $LOG_DIR/planner.log）"
    fi
    grep -E "goal|Goal|replan|FSM|发散|定位异常" "$LOG_DIR/planner.log" 2>/dev/null | tail -3 | sed 's/^/  /'
    exit 0
    ;;
stop)
    KEEP_LOC=0
    for a in "$@"; do [ "$a" = "--keep-loc" ] && KEEP_LOC=1; done
    echo "停止规划器..."
    for pat in "${NAV_COMPONENTS[@]}"; do
        pkill -f "$pat" 2>/dev/null
    done
    for pat in "${NAV_COMPONENTS[@]}"; do
        if ! wait_gone "[${pat:0:1}]${pat:1}" 10; then
            pkill -9 -f "[${pat:0:1}]${pat:1}" 2>/dev/null
        fi
    done
    echo "  规划器已停止"
    if [ "$KEEP_LOC" = "1" ]; then
        echo "  定位链路保留继续运行（--keep-loc）"
    else
        echo "停止定位链路..."
        bash "$LOC" stop
    fi
    exit 0
    ;;
esac

# ── 解析启动参数 ──
USE_RVIZ=1
PLANNER_ONLY=0
for a in "$@"; do
    [ "$a" = "--no-rviz" ] && USE_RVIZ=0
    [ "$a" = "--planner-only" ] && PLANNER_ONLY=1
done

mkdir -p "$LOG_DIR"
rm -f "$LOG_DIR"/*.log
need_source

echo "============================================"
echo " VINS+rtabmap 定位 + SCAN-Planner 局部导航"
echo " 机器人模型: $ROBOT_MODEL   传感器: $SENSOR_TYPE   导航模式: $NAVI_MODE"
echo " 限速: vel=$REAL_MAX_VEL acc=$REAL_MAX_ACC vyaw=$REAL_MAX_VYAW"
echo " 日志目录: $LOG_DIR"
echo "============================================"

# ── 1. 确保定位链路就绪 ──
if loc_running; then
    echo "[1/2] 定位链路已在运行，直接复用（$ODOM_BASE_TOPIC 已发布）"
elif [ "$PLANNER_ONLY" = "1" ]; then
    echo "[1/2] [错误] --planner-only 但未检测到定位链路"
    echo "      请先运行: $LOC" && exit 1
else
    echo "[1/2] 定位链路未运行，自动拉起 run_vins_localization.sh ..."
    # 定位一律不开自己的 rviz, 导航专用 rviz 由本脚本统一拉起(避免双开抢 CPU)
    LOC_ARGS="--no-rviz"
    # 注意: 真机上此步会等待 VINS 初始化(需机器人/相机移动激励), 定位脚本内部已处理
    if ! bash "$LOC" $LOC_ARGS; then
        echo "      [失败] 定位链路启动失败，中止导航（详见 /tmp/vins_localization_logs）"
        exit 1
    fi
fi

# 规划器强依赖连续局部里程计, 没有它 grid_map 无法投影深度、FSM 拿不到位姿
if ! wait_topic "$ODOM_BASE_TOPIC" 30; then
    echo "      [失败] 30s 内未见 $ODOM_BASE_TOPIC，规划器无位姿输入，中止"
    exit 1
fi
echo "      位姿输入就绪：$ODOM_BASE_TOPIC"

# ── 2. 拉起 SCAN-Planner 真机导航 ──
# is_real_world:=true 会把上面这些话题重映射好, 并额外拉起:
#   goal_frame_bridge.py  (目标点 map→world + 里程计发散看门狗)
#   leg_height_to_posture.py (腿高指令转 /cmd_posture)
#   closed_loop_controller   (轨迹→ /cmd_vel)
echo "[2/2] 启动 SCAN-Planner (is_real_world:=true)..."
LAUNCH_ARGS=(
    is_real_world:=true
    robot_model:="$ROBOT_MODEL"
    sensor_type:="$SENSOR_TYPE"
    navi_mode:="$NAVI_MODE"
    controller_mode:="$CONTROLLER_MODE"
    use_gpu:="$USE_GPU"
    use_sim_time:=false
    real_max_vel:="$REAL_MAX_VEL"
    real_max_acc:="$REAL_MAX_ACC"
    real_max_vyaw:="$REAL_MAX_VYAW"
)
if [ "$NAVI_MODE" = "2" ]; then
    if [ -z "$KEYPOINTS_FILE" ] || [ ! -f "$KEYPOINTS_FILE" ]; then
        echo "      [失败] navi_mode=2 需要 KEYPOINTS_FILE 指向有效的路径点参数 YAML"
        exit 1
    fi
    LAUNCH_ARGS+=(keypoints_file:="$KEYPOINTS_FILE")
fi

nohup ros2 launch scan_planner run.launch.py "${LAUNCH_ARGS[@]}" \
    > "$LOG_DIR/planner.log" 2>&1 &

if ! wait_node /scan_planner_node 40; then
    echo "      [失败] scan_planner_node 未注册，查看 $LOG_DIR/planner.log"
    exit 1
fi
echo "      规划器已启动"

# 确认目标点桥接就绪(真机模式才有), 它同时是里程计发散看门狗
if wait_node /goal_frame_bridge 15; then
    echo "      目标点桥接就绪：/goal_pose(map) → world → 规划器（含发散看门狗）"
else
    echo "      [警告] goal_frame_bridge 未就绪，rviz 点的目标可能无法转到 world（查看 $LOG_DIR/planner.log）"
fi

# ── 3. 导航专用可视化(--no-rviz 可关; 用主机 Foxglove 时建议关掉本地 rviz) ──
if [ "$USE_RVIZ" = "1" ]; then
    export XAUTHORITY=${XAUTHORITY:-/home/sunrise/.Xauthority}
    # 设备重启后桌面 X 显示号不固定(实测会跑到 :1001)，从 /tmp/.X11-unix 探测能连上的
    _disp=""
    for _sock in /tmp/.X11-unix/X*; do
        [ -e "$_sock" ] || continue
        _n=":${_sock##*/X}"
        if DISPLAY="$_n" timeout 3 xset q >/dev/null 2>&1; then _disp="$_n"; break; fi
    done
    export DISPLAY=${_disp:-:0}
    echo "[+] 启动导航 rviz2 (DISPLAY=$DISPLAY，设备桌面/VNC 查看)..."
    nohup rviz2 -d "$RVIZ_CFG" > "$LOG_DIR/rviz.log" 2>&1 &
    if wait_node /rviz 30 || wait_node /rviz2 3; then
        echo "      导航 rviz2 已就绪 (配置: $RVIZ_CFG)"
    else
        echo "      [警告] rviz2 未就绪，查看 $LOG_DIR/rviz.log（不影响导航）"
    fi
fi

echo ""
echo "导航已就绪! 在 rviz 里用 [2D Goal Pose] 点选目标即可开始规划"
echo "  目标点会经 goal_frame_bridge 从 map 转到 world 再喂给规划器"
echo "  规划输出: /cmd_vel(底盘) + /cmd_posture(腿高)"
echo "  查看状态: $0 status"
echo "  只停规划器(保留定位): $0 stop --keep-loc"
echo "  整体停止: $0 stop"
