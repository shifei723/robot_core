#!/bin/bash
# --- robot_core 可移植自定位 ---
ROBOT_CORE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")/../.." && pwd)"; export ROBOT_CORE
source "$ROBOT_CORE/setup.sh"
# run_vins_localization.sh — VINS-Fusion 里程计 + rtabmap 重定位（不建图）
#
# 与 run_vins_mapping.sh 共用同一套链路（相机→VINS→odom_to_tf→rtabmap），
# 唯一本质区别是 rtabmap 切到定位模式:
#     Mem/IncrementalMemory=false   → 不扩展地图，只定位
#     Mem/LocalizationReadOnly=true → 数据库只读，绝不写回污染地图
#     去掉 delete_db_on_start        → 加载已有地图
#
# 重定位原理:
#   rtabmap 用地图 DB 里的视觉词库做全局 place recognition（不是点云匹配，
#   .ply 只是可视化导出，重定位用不到它）。相机看到熟悉场景时恢复全局位姿，
#   并发布 map→world 修正 TF，把 VINS 的局部漂移轨迹拽回全局坐标。
#   所以"开机在哪不知道也没关系"——走着走着匹配上就自动归位。
#   若大致知道开机位置，可用 INITIAL_POSE 给初值（见下方变量）。
#
# 前提（已验证当前地图满足）:
#   rtabmap_vins.db 含 79 节点 / 5861 视觉词 / 每帧 ~646 特征 / 8 条回环，
#   视觉匹配数据充分，可直接重定位。
#
# 红外一致性: 建图与定位都必须保持 emitter 关闭（本脚本已关），
#   否则成像外观变化会降低视觉词匹配率。
#
# 用法:
#   run_vins_localization.sh                          启动(带 rviz2)
#   run_vins_localization.sh --no-rviz                纯后台
#   INITIAL_POSE="1.0 0.5 0 0 0 0.5" run_vins_localization.sh   给初始位姿
#   run_vins_localization.sh status                   查看状态
#   run_vins_localization.sh stop                     停止
#
# 运行中也可随时人工重定位(同 rviz 的 AMCL 用法):
#   ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
#     '{header:{frame_id:"map"},pose:{pose:{position:{x:1.0,y:0.5,z:0.0},orientation:{w:1.0}}}}'
set +u

LOG_DIR=/tmp/vins_localization_logs
MAP_DIR=${MAP_DIR:-$ROBOT_CORE/data/rtabmap_maps}
DB=${DB:-$MAP_DIR/rtabmap_vins.db}
# 初始位姿提示（可选）: "x y z roll pitch yaw"。留空则纯靠全局视觉匹配。
INITIAL_POSE=${INITIAL_POSE:-}
WS="$ROBOT_CORE/localization/visual_slam"
# 构建产物统一落在 $ROBOT_CORE/install/<组>（见 build.sh）；visual_slam 属于 rtabmap 组。
# $WS 只是源码目录（VINS 配置文件所在），它下面不会有 install/
RTABMAP_WS="$ROBOT_CORE/install/rtabmap"
VINS_SRC=$WS/vins_fusion
VINS_CONFIG=${VINS_CONFIG:-$VINS_SRC/config/realsense_d435i/d435i_stereo_imu_config.yaml}
# 定位专用 rviz: 显示 RealSense 正方向+实际位置、3D 地面/障碍物区分点云
RVIZ_CFG=${RVIZ_CFG:-$MAP_DIR/vins_localization.rviz}
# PCL 报 libusb_set_option undefined 的老坑（同 fastlio2/point_lio）
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0

CAM_NS=/camera/camera
RGB_TOPIC=$CAM_NS/color/image_raw
DEPTH_TOPIC=$CAM_NS/aligned_depth_to_color/image_raw
INFO_TOPIC=$CAM_NS/color/camera_info
IR1_TOPIC=$CAM_NS/infra1/image_rect_raw
IMU_RAW_TOPIC=$CAM_NS/imu
# 里程计源: 默认用 VINS 的 IMU 前向递推位姿 /imu_propagate(~200Hz, 延迟~1ms)，
# 而非优化后的 /odometry(~15Hz, 带滑窗求解延迟~120ms)。规划闭环对延迟极敏感，
# 低延迟远比优化那点平滑重要；重定位大跳走 map→world TF 与 /odometry_global，不进这路。
# 需回退对比时: ODOM_TOPIC=/odometry ./run_vins_localization.sh
ODOM_TOPIC=${ODOM_TOPIC:-/imu_propagate}
BODY_FRAME=body
BASE_FRAME=base_link
ODOM_BASE_TOPIC=/odometry_base
CAM_ROOT=camera_link
B2C_XYZ="--x 0.011739999987185 --y 0.00552000012248755 --z -0.00510000018402934"
B2C_QUAT="--qx 0 --qy 0 --qz 0 --qw 1"
ODOM_TF_NODE=/odom_to_tf
ODOM_TF_PY=$ROBOT_CORE/tools/odom_to_tf.py
# 地面零点锚定(m): base_link 落地站立高度 = leg_height_default(0.25) + base_z_offset(0.0322)。
# 该值由机器人运动学确定，勿再依赖 ground_anchor 自动标定（分割↔锚定循环依赖会污染基准）。
# ANCHOR_FILE 仅作可选手动覆盖；不存在时用运动学确定值 0.2822。
ANCHOR_FILE=$ROBOT_CORE/data/rtabmap_maps/ground_anchor.txt
if [ -f "$ANCHOR_FILE" ]; then INITIAL_BASE_HEIGHT=$(cat "$ANCHOR_FILE"); else INITIAL_BASE_HEIGHT=0.2822; fi
# 全局位姿发布: 把 map→base_link（含 rtabmap 重定位修正）采样成 odometry。
# 该位姿全局一致但重定位瞬间会跳，与 /odometry_base（局部连续）互补。
TF_ODOM_NODE=/tf_to_odom
TF_ODOM_PY=$ROBOT_CORE/tools/tf_to_odom.py
GLOBAL_ODOM_TOPIC=/odometry_global
# 地面锚定/离地高度节点: 定位侧只读锚定、仅显示离地高度
PROBE_NODE=/ground_anchor
PROBE_PY=$ROBOT_CORE/tools/ground_anchor.py

need_source() {
    source /opt/ros/humble/setup.bash
    if [ -f "$RTABMAP_WS/setup.bash" ]; then
        source "$RTABMAP_WS/setup.bash"
    else
        echo "[错误] $RTABMAP_WS/setup.bash 不存在，请先编译 rtabmap 工作区!"
        echo "       cd $ROBOT_CORE && ./build.sh rtabmap"
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

wait_gone() {
    local pat=$1 tmo=$2 i=0
    while [ "$i" -lt "$((tmo*2))" ]; do
        pgrep -f "$pat" >/dev/null 2>&1 || return 0
        sleep 0.5; i=$((i+1))
    done
    return 1
}

COMPONENTS=("rviz2.*vins_localization.rviz" "rtabmap_slam/rtabmap" ground_anchor.py tf_to_odom.py odom_to_tf.py vins_node static_transform_publisher realsense2_camera_node)

case "$1" in
status)
    need_source
    echo "=== 组件状态 ==="
    for p in "${COMPONENTS[@]}"; do
        pid=$(pgrep -f "[${p:0:1}]${p:1}" | head -1)
        [ -n "$pid" ] && echo "  [运行中] $p (pid=$pid)" || echo "  [未运行] $p"
    done
    echo ""
    echo "=== TF 树桥接 ==="
    for pair in "world $BASE_FRAME" "$BASE_FRAME camera_link" "map $BASE_FRAME"; do
        set -- $pair
        printf "  %-22s " "$1 → $2"
        if timeout 5 ros2 run tf2_ros tf2_echo "$1" "$2" 2>/dev/null | grep -q "Translation"; then
            echo "OK"
        else
            echo "不连通"
        fi
    done
    echo ""
    echo "=== VINS 状态 ==="
    if grep -q "solver costs" "$LOG_DIR/vins.log" 2>/dev/null; then
        echo "  已初始化"
    else
        echo "  未初始化: 请手持相机平稳移动"
    fi
    if grep -q "里程计已发散" "$LOG_DIR/odom_tf.log" 2>/dev/null; then
        echo "  [发散] TF 转发已自保护停止，定位不可靠，请 stop 后重启并手持移动"
    fi
    echo ""
    echo "=== rtabmap 定位状态 ==="
    # 定位成功的关键日志: 匹配上已知地点(回环/重定位)，而非"建图拒绝"
    grep -E "Localizing|localization|Loop closure|relocalization" "$LOG_DIR/rtabmap.log" 2>/dev/null | tail -3 | sed 's/^/  /'
    pos=$(timeout 6 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null \
          | awk -F': ' '/^x:/{x=$2} /^y:/{y=$2} /^z:/{z=$2} END{if(x!="")printf "%.2f, %.2f, %.2f", x,y,z}')
    [ -n "$pos" ] && echo "  当前位姿(world, VINS 局部): $pos"
    echo ""
    echo "地图数据库: $DB"
    [ -f "$DB" ] && echo "  大小: $(du -h "$DB" | cut -f1)"
    exit 0
    ;;
stop)
    echo "停止重定位..."
    for pat in "${COMPONENTS[@]}"; do
        pkill -f "$pat" 2>/dev/null
    done
    for pat in "${COMPONENTS[@]}"; do
        if ! wait_gone "[${pat:0:1}]${pat:1}" 10; then
            pkill -9 -f "[${pat:0:1}]${pat:1}" 2>/dev/null
        fi
    done
    echo "已停止（地图为只读，未做任何修改）"
    exit 0
    ;;
esac

# ── 安全门槛: 定位模式绝不允许无地图/删库 ──
if [ ! -f "$DB" ]; then
    echo "[错误] 地图数据库不存在: $DB"
    echo "       请先用 run_vins_mapping.sh 建图，或用 DB= 指定已有地图"
    exit 1
fi

USE_RVIZ=1
for a in "$@"; do [ "$a" = "--no-rviz" ] && USE_RVIZ=0; done

mkdir -p "$LOG_DIR" $ROBOT_CORE/data/vins_output
rm -f "$LOG_DIR"/*.log
need_source

echo "============================================"
echo " VINS-Fusion 里程计 + rtabmap 重定位(只读)"
echo " 地图数据库:  $DB"
[ -n "$INITIAL_POSE" ] && echo " 初始位姿:    $INITIAL_POSE"
echo " 日志目录:    $LOG_DIR"
echo "============================================"

if [ ! -f "$VINS_CONFIG" ]; then
    echo "[错误] VINS 配置不存在: $VINS_CONFIG"
    exit 1
fi

# ── 1. 相机（与建图完全一致: 红外外观一致是重定位匹配率的前提）──
echo "[1/6] 启动 D435i (infra1+infra2+color+depth+IMU, 640x360@30)..."
pkill -9 -f realsense2_camera_node 2>/dev/null
wait_gone "[r]ealsense2_camera_node" 5
nohup ros2 launch realsense2_camera rs_launch.py \
    enable_depth:=true enable_color:=true align_depth.enable:=true \
    enable_infra1:=true enable_infra2:=true \
    enable_sync:=true \
    enable_gyro:=true enable_accel:=true unite_imu_method:=2 \
    rgb_camera.color_profile:=640x360x30 \
    depth_module.depth_profile:=640x360x30 \
    depth_module.infra_profile:=640x360x30 \
    > "$LOG_DIR/camera.log" 2>&1 &

if ! wait_log "$LOG_DIR/camera.log" "Node Is Up" 60; then
    echo "      [失败] 相机未就绪，查看 $LOG_DIR/camera.log"
    exit 1
fi
echo "      相机就绪"

if timeout 15 ros2 param set /camera/camera depth_module.emitter_enabled 0 2>/dev/null | grep -q "successful"; then
    echo "      红外投影器已关闭（与建图时一致，保证视觉词匹配外观不变）"
else
    echo "      [警告] 关闭投影器失败，成像外观与建图时不一致会降低匹配率"
    echo "             手动执行: ros2 param set /camera/camera depth_module.emitter_enabled 0"
fi

# ── 2. static TF: base_link → camera_link ──
echo "[2/6] 发布 static TF: $BASE_FRAME → $CAM_ROOT ..."
nohup ros2 run tf2_ros static_transform_publisher \
    --frame-id "$BASE_FRAME" --child-frame-id "$CAM_ROOT" \
    $B2C_XYZ $B2C_QUAT \
    --ros-args -r __node:=vins_body_tf \
    > "$LOG_DIR/tf_body.log" 2>&1 &
if ! wait_node /vins_body_tf 15; then
    echo "      [失败] static TF 未就绪"
    exit 1
fi
echo "      TF 桥接就绪"

# ── 3. VINS 里程计（局部，重定位的输入）──
if ! timeout 8 ros2 topic list 2>/dev/null | grep -qx "$IMU_RAW_TOPIC"; then
    echo "      [失败] 无 IMU 话题 $IMU_RAW_TOPIC"
    exit 1
fi
echo "[3/6] 启动 vins_node (双目红外 + IMU)..."
nohup ros2 run vins vins_node "$VINS_CONFIG" \
    > "$LOG_DIR/vins.log" 2>&1 &
# 注意: 本 VINS 构建注册的节点名为空(日志前缀 []:)，ros2 node list 里查不到
# /vins_estimator，故不能用 wait_node 判就绪——改由下方 "solver costs" 日志确认。
echo "      VINS 已启动（等待初始化）"

echo "      ★ 请手持相机缓慢移动，VINS 初始化后即可开始移动寻找定位 ★"
if wait_log "$LOG_DIR/vins.log" "solver costs" 90; then
    echo "      ★ VINS 初始化成功（$ODOM_TOPIC）"
else
    echo "      [失败] 90 秒内 VINS 未初始化，中止"
    exit 1
fi

# ── 4. odom→TF 转发（与建图一致，含静止发散保护）──
echo "[4/6] 启动 odom→TF 转发 (world → $BASE_FRAME)..."
nohup python3 "$ODOM_TF_PY" --ros-args \
    -p odom_topic:="$ODOM_TOPIC" \
    -p base_frame:="$BASE_FRAME" \
    -p out_topic:="$ODOM_BASE_TOPIC" \
    -p initial_base_height:="$INITIAL_BASE_HEIGHT" \
    > "$LOG_DIR/odom_tf.log" 2>&1 &
if ! wait_node "$ODOM_TF_NODE" 20; then
    echo "      [失败] odom_to_tf 未就绪，查看 $LOG_DIR/odom_tf.log"
    exit 1
fi
if ! wait_log "$LOG_DIR/odom_tf.log" "首帧已转发" 15; then
    echo "      [失败] 未收到 $ODOM_TOPIC"
    exit 1
fi
grep "首帧已转发" "$LOG_DIR/odom_tf.log" | tail -1 | sed 's/.*: /      /'

# ── 5. rtabmap 定位模式 ──
# 与建图的三处核心差异:
#   Mem/IncrementalMemory=false    定位而非建图
#   Mem/LocalizationReadOnly=true  数据库只读，定位结果不写回（保护地图）
#   无 delete_db_on_start          加载已有地图
# 定位模式下匹配已知地点时发布 map→world 修正 TF（odom_frame_id=world）。
#
# 3D 地面/障碍物区分: Grid\3D=true 让 rtabmap 输出 3D 占据，配合法向量分割
# 把点云分成 /cloud_ground(地面) 与 /cloud_obstacles(障碍)，rviz 里分别用绿/红显示。
# 注意: rtabmap 定位模式用的是建图时缓存进 DB 的局部栅格。若 DB 是用旧的
# Grid\3D=false 建的，这里改 3D 只对新建的地图生效——要真正得到 3D 分割，
# 需用同样开 Grid\3D=true 的 run_vins_mapping.sh 重新建图（见对话说明）。
cat > "$MAP_DIR/vins_loc_grid_params.ini" <<'EOF'
# RTAB-Map 定位可视化参数（3D 地面/障碍分割）
# 注意 rtabmap 退出会写回本文件，故每次启动都重建，不要手工编辑
[Core]
Grid\3D=true
Grid\RayTracing=true
Grid\CellSize=0.05
Grid\RangeMax=8.0
# 法向量分割地面: 手持/移动时按点云法向判地面，比固定高度阈值更鲁棒
Grid\NormalsSegmentation=true
# 地面锚到 z=0 后按真实高度收紧(与建图脚本一致)
Grid\MaxGroundHeight=0.10
Grid\MaxObstacleHeight=2.0
# 法向量分割细化(与建图脚本一致)
Grid\MaxGroundAngle=45
Grid\MinClusterSize=10
Grid\NormalK=20
# 在 map 帧按位姿 roll/pitch 把点云摆平后再分地面（前提 frame_id 是 REP-103 base_link）
Grid\MapFrameProjection=true
EOF
RTABMAP_ARGS=(
    -p frame_id:="$BASE_FRAME"
    -p odom_frame_id:=world
    -p subscribe_depth:=true
    -p approx_sync:=true
    -p wait_for_transform:=0.3
    -p database_path:="$DB"
    -p config_path:="$MAP_DIR/vins_loc_grid_params.ini"
    -p "Mem/IncrementalMemory:='false'"
    -p "Mem/LocalizationReadOnly:='true'"
    -p "Rtabmap/DetectionRate:='1.0'"
    -p cloud_output_voxelized:=true
    -r odom:="$ODOM_BASE_TOPIC"
    -r rgb/image:=$RGB_TOPIC
    -r depth/image:=$DEPTH_TOPIC
    -r rgb/camera_info:=$INFO_TOPIC
)
if [ -n "$INITIAL_POSE" ]; then
    # initial_pose 期望 "x y z roll pitch yaw"（Transform::fromString 格式）
    RTABMAP_ARGS+=(-p initial_pose:="'$INITIAL_POSE'")
fi
echo "[5/6] 启动 rtabmap 定位模式..."
nohup ros2 run rtabmap_slam rtabmap --ros-args "${RTABMAP_ARGS[@]}" \
    > "$LOG_DIR/rtabmap.log" 2>&1 &
if ! wait_node /rtabmap 30; then
    echo "      [失败] rtabmap 节点未注册，查看 $LOG_DIR/rtabmap.log"
    exit 1
fi
echo "      定位已启动"

if wait_log "$LOG_DIR/rtabmap.log" "Rate=.*RTAB-Map=" 30; then
    echo "      已确认 rtabmap 在处理数据"
else
    echo "      [警告] 30s 内未见 rtabmap 处理数据，可能是同步问题"
    echo "             排查: grep -i 'did not receive\|sync' $LOG_DIR/rtabmap.log"
fi

# ── 5b. 全局位姿发布（map→base_link 采样成 odometry）──
# rtabmap 只把重定位修正挂在 map→world TF 上，没有对应话题。这里采样成
# $GLOBAL_ODOM_TOPIC，供需要全局位姿话题的下游（监控 / 将目标点锤定到 map 系）。
# 注: 该位姿重定位瞬间会跳，不要直接当局部规划器位姿输入（见 tf_to_odom.py 头）。
echo "[5b] 启动全局位姿发布 (map → $BASE_FRAME → $GLOBAL_ODOM_TOPIC)..."
nohup python3 "$TF_ODOM_PY" --ros-args \
    -p parent_frame:=map \
    -p child_frame:="$BASE_FRAME" \
    -p out_topic:="$GLOBAL_ODOM_TOPIC" \
    > "$LOG_DIR/tf_to_odom.log" 2>&1 &
if ! wait_node "$TF_ODOM_NODE" 15; then
    echo "      [警告] tf_to_odom 未就绪（不影响定位，仅无全局位姿话题），查看 $LOG_DIR/tf_to_odom.log"
else
    echo "      全局位姿发布已就绪（首次匹配上地图前会提示等待重定位）"
fi

# ── 5c. 离地高度显示（只读锚定，从 /cloud_ground 算 RealSense 离地高度）──
echo "[5c] 启动离地高度显示 (只读锚定)..."
nohup python3 "$PROBE_PY" --ros-args \
    -p base_frame:="$BASE_FRAME" \
    -p world_frame:=map \
    -p ground_topic:=/cloud_ground \
    -p current_anchor:="$INITIAL_BASE_HEIGHT" \
    -p write_anchor:=false \
    > "$LOG_DIR/probe.log" 2>&1 &
if ! wait_node "$PROBE_NODE" 15; then
    echo "      [警告] 地面节点未就绪（不影响定位），查看 $LOG_DIR/probe.log"
else
    echo "      离地高度显示已就绪：/realsense/height_above_ground 与 rviz /realsense/height_viz"
fi

# ── 6. 可视化 ──
if [ "$USE_RVIZ" = "1" ]; then
    export XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}
    # 设备重启后桌面 X 显示号不固定(实测重启后跑到 :1001，不再是 :0)。
    # 从 /tmp/.X11-unix 探测真正能连上的显示号，都连不上再回退 :0。
    _disp=""
    for _sock in /tmp/.X11-unix/X*; do
        [ -e "$_sock" ] || continue
        _n=":${_sock##*/X}"
        if DISPLAY="$_n" timeout 3 xset q >/dev/null 2>&1; then _disp="$_n"; break; fi
    done
    export DISPLAY=${_disp:-:0}
    echo "[6/6] 启动 rviz2 (DISPLAY=$DISPLAY，显示在设备桌面，请用 VNC 查看)..."
    nohup rviz2 -d "$RVIZ_CFG" > "$LOG_DIR/rviz.log" 2>&1 &
    if wait_node /rviz 30 || wait_node /rviz2 3; then
        echo "      rviz2 已就绪"
    else
        echo "      [警告] rviz2 未就绪，查看 $LOG_DIR/rviz.log（不影响定位）"
    fi
fi

echo ""
echo "重定位已启动! 移动到建图时去过的地方即可自动恢复全局位姿"
echo "  查看状态: $0 status"
echo "  人工重定位: 发布 /initialpose（见脚本头部示例）"
echo "  停止:     $0 stop"
