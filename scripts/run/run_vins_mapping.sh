#!/bin/bash
# --- robot_core 可移植自定位 ---
ROBOT_CORE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")/../.." && pwd)"; export ROBOT_CORE
source "$ROBOT_CORE/setup.sh"
# run_vins_mapping.sh — VINS-Fusion 里程计 + rtabmap 建图（联合方案）
#
# 链路: realsense2_camera ─┬→ infra1/infra2 + IMU → vins_node → /odometry
#                          └→ color + aligned_depth → rtabmap ← /odometry
#
# 与 run_rgbd_mapping.sh 的区别: 用 VINS 的紧耦合 VIO 替代 rgbd_odometry。
#   rgbd_odometry 是帧间 RANSAC 配准，快速运动/弱纹理时 inliers 不足就跟丢，
#   触发 Odom/ResetCountdown 重置 → 轨迹被砍断 → 地图段间错位（实测 5 分钟 6 次重置）。
#   VINS 用 IMU 预积分做滑窗紧耦合，运动模型连续，不存在"重置"概念。
#
# ── 三个关键工程问题及处理 ──
# 1. 坐标系桥接（本方案最容易做错的一步）:
#    VINS 的 pubTF() 里第一行是 `return; // tmp.`，即完全不发 TF，
#    只发 odometry 消息(header.frame_id=world, child_frame_id=body)。
#    而 rtabmap 需要完整 TF 树: 既要把里程计位姿插值到图像时刻(odom 15Hz、
#    图像 30Hz，时间戳对不齐)，也要从 base frame 查到相机光学帧反投影深度。
#    目标拓扑:  map ─→ world ─→ base_link ─→ camera_link ─→ 各光学帧
#              (rtabmap)  (转发)     (static)     (realsense)
#    处理两条:
#      a) world→base_link: 起 tools/odom_to_tf.py 把 /odometry 转成 TF。
#         （不能改 VINS 源码放开 pubTF: 其函数体用了未初始化的
#           TransformBroadcaster 指针，去掉 return 会段错误）
#      b) base_link→camera_link: static TF。base_link 与 body 同原点(在 gyro 上)、
#         姿态为 REP-103，而 camera_link 本身就是 REP-103 对齐的，所以这段
#         只有平移、旋转是单位四元数（见下面第 2 点为何要插 base_link）。
#    注意方向: 早前把 body 作为 camera_gyro_optical_frame 的**子**帧是错的——
#    里程计的运动帧必须在相机树之上，否则等于把它钉死在相机上永远不动，
#    且 world 会与相机树断成两棵不连通的树。
#
# 2. VINS 的 body 帧是光学系(x右y下z前)，不能直接当 rtabmap 的 base frame:
#    rtabmap 的 2D 栅格地面分割假定 base frame 符合 REP-103(x前y左z上)——
#    它从位姿里取 roll/pitch 把点云"水平化"后再分地面与障碍
#    (LocalGridMaker.cpp 中 Grid/MapFrameProjection 的分支)。
#    body 当 base frame 时相机水平前视的 roll 恒为 -90°(实测数据库中位数
#    -89.1°)，水平化等于把每帧点云又绕 x 轴转了 89°，2D 栅格图彻底错乱；
#    而 3D 点云不走这段代码，看着完全正常，很容易误判成 TF 绑错。
#    Grid/MapFrameProjection 治不了这个——它只用 pose 的 roll/pitch，
#    并不会把光学系本身转正。
#    处理: odom_to_tf.py 额外发布与 body 同原点、姿态转成 REP-103 的 base_link，
#    并把里程计重发到 /odometry_base；rtabmap 用 frame_id=base_link 配这个话题。
#    (rtabmap 不校验 odom 的 child_frame_id，直接把消息位姿当 map→frame_id，
#     所以话题与 frame_id 必须成对改，只改一个会整体错 90°。)
#    此时 world 帧重力对齐(z 上)，Grid/MapFrameProjection=true 的水平化才有意义。
#
# 3. 红外投影器与深度质量冲突:
#    VINS 用红外图做光流，emitter 的散斑图案随相机移动会污染跟踪 → 必须关。
#    但深度的立体匹配又依赖散斑填补弱纹理区域。
#    处理: 优先保 VIO 精度，emitter 关闭。代价是白墙/纯色平面深度会有空洞，
#    对准有纹理的物体建图即可。若更看重深度完整度，用 run_rgbd_mapping.sh。
#
# 4. 必须全程手持移动，不能放桌上静止（实测踩过的坑）:
#    VINS-Fusion 没有零速约束。相机静止时加速度计 bias 与重力方向不可解耦，
#    bias 估值一漂，位置就沿二次积分爆炸。实测放在桌上不动: 前 29 秒位姿
#    一直是 0.000，之后开始发散，5 分钟跑到 1534 米，把栅格图撑成 7699x10832 格，
#    rviz 内存 2.7GB、整机 load 9.6。
#    处理: odom_to_tf.py 内置发散保护（>5 m/s 即停止转发 TF）。
#    注意这只是防止污染地图的兵线，不能代替手持移动。
#
# 用法:
#   run_vins_mapping.sh            启动(带 rviz2，显示在 VNC 桌面)
#   run_vins_mapping.sh --no-rviz  纯后台
#   run_vins_mapping.sh status     查看状态
#   run_vins_mapping.sh stop       停止
set +u

LOG_DIR=/tmp/vins_mapping_logs
MAP_DIR=${MAP_DIR:-$ROBOT_CORE/data/rtabmap_maps}
DB=${DB:-$MAP_DIR/rtabmap_vins.db}
WS="$ROBOT_CORE/localization/visual_slam"
# 构建产物统一落在 $ROBOT_CORE/install/<组>（见 build.sh）；visual_slam 属于 rtabmap 组。
# $WS 只是源码目录（VINS 配置文件所在），它下面不会有 install/
RTABMAP_WS="$ROBOT_CORE/install/rtabmap"
VINS_SRC=$WS/vins_fusion
VINS_CONFIG=${VINS_CONFIG:-$VINS_SRC/config/realsense_d435i/d435i_stereo_imu_config.yaml}
RVIZ_CFG=${RVIZ_CFG:-$MAP_DIR/vins_map.rviz}
# PCL 报 libusb_set_option undefined 的老坑（同 fastlio2/point_lio）
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0

CAM_NS=/camera/camera
RGB_TOPIC=$CAM_NS/color/image_raw
DEPTH_TOPIC=$CAM_NS/aligned_depth_to_color/image_raw
INFO_TOPIC=$CAM_NS/color/camera_info
IR1_TOPIC=$CAM_NS/infra1/image_rect_raw
IMU_RAW_TOPIC=$CAM_NS/imu
# VINS 的话题都在根命名空间（相对话题名不含节点名）
ODOM_TOPIC=/odometry
# VINS 的 body 帧是光学系，不能直接给 rtabmap 当 base frame（见文件头第 2 点）。
# odom_to_tf.py 把它转成 REP-103 的 base_link，并重发里程计到 ODOM_BASE_TOPIC。
BODY_FRAME=body
BASE_FRAME=base_link
ODOM_BASE_TOPIC=/odometry_base
# base_link 之下挂 realsense 树根。base_link 与 camera_link 姿态一致(都是 REP-103)，
# 只差平移: base_link 落在 gyro 上，而实测 camera_link→camera_gyro_optical_frame
# 的平移是 t=(-0.01174,-0.00552,0.00510)，取反即得下列数值。
CAM_ROOT=camera_link
B2C_XYZ="--x 0.011739999987185 --y 0.00552000012248755 --z -0.00510000018402934"
B2C_QUAT="--qx 0 --qy 0 --qz 0 --qw 1"
ODOM_TF_NODE=/odom_to_tf
ODOM_TF_PY=$ROBOT_CORE/tools/odom_to_tf.py
# 地面零点锚定(m): base_link 落地站立高度 = leg_height_default(0.25) + base_z_offset(0.0322)。
# 该值由机器人运动学确定，勿再依赖 ground_anchor 的自动标定（分割↔锚定循环依赖会污染基准）。
# ANCHOR_FILE 仅作可选手动覆盖；不存在时用运动学确定值 0.2822。
ANCHOR_FILE=$ROBOT_CORE/data/rtabmap_maps/ground_anchor.txt
if [ -f "$ANCHOR_FILE" ]; then INITIAL_BASE_HEIGHT=$(cat "$ANCHOR_FILE"); else INITIAL_BASE_HEIGHT=0.2822; fi
# 地面锚定/离地高度节点: 自动标定地面零点 + rviz 显示离地高度
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

# 等待日志文件出现指定关键字（事件驱动，代替固定 sleep）
# 用法: wait_log <logfile> <pattern> <timeout秒>
wait_log() {
    local f=$1 pat=$2 tmo=$3 i=0
    while [ "$i" -lt "$tmo" ]; do
        grep -q "$pat" "$f" 2>/dev/null && return 0
        sleep 0.5; i=$((i+1))
    done
    return 1
}

# 等待 ROS2 节点注册完成
# 用法: wait_node <node名> <timeout秒>
wait_node() {
    local node=$1 tmo=$2 i=0
    while [ "$i" -lt "$tmo" ]; do
        ros2 node list 2>/dev/null | grep -qx "$node" && return 0
        sleep 0.5; i=$((i+1))
    done
    return 1
}

# 等待进程退出
# 用法: wait_gone <pgrep模式> <timeout秒>
wait_gone() {
    local pat=$1 tmo=$2 i=0
    while [ "$i" -lt "$((tmo*2))" ]; do
        pgrep -f "$pat" >/dev/null 2>&1 || return 0
        sleep 0.5; i=$((i+1))
    done
    return 1
}

# 组件清单（stop/status 共用；模式首字符用字符类包住，防止 pgrep 匹配到自己）
COMPONENTS=("rviz2.*vins_map.rviz" "rtabmap_slam/rtabmap" ground_anchor.py odom_to_tf.py vins_node static_transform_publisher realsense2_camera_node)

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
    # 两段都通才算完整: world→base_link(转发) 与 base_link→camera_link(static)
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
    echo "=== 话题频率 ==="
    # /mapData 仅在地图新增节点时发布，相机静止时无数据属正常
    for t in "$IR1_TOPIC" "$RGB_TOPIC" "$DEPTH_TOPIC" "$IMU_RAW_TOPIC" "$ODOM_TOPIC" "$ODOM_BASE_TOPIC" /mapData; do
        printf "  %-50s " "$t"
        rate=$(timeout 6 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
        if [ -z "$rate" ]; then
            rate=$(timeout 10 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
        fi
        [ -n "$rate" ] && echo "$rate Hz" || echo "无数据"
    done
    echo ""
    echo "=== VINS 状态 ==="
    if grep -q "solver costs" "$LOG_DIR/vins.log" 2>/dev/null; then
        echo "  已初始化"
        grep "solver costs" "$LOG_DIR/vins.log" | tail -1 | sed 's/^/  /'
    else
        echo "  未初始化: 请手持相机平稳移动"
        grep -E "excitation|features" "$LOG_DIR/vins.log" 2>/dev/null | tail -1 | sed 's/^/  /'
    fi
    # 发散检查: 静止放置会让位置二次积分爆炸，此时建出的地图已不可用
    if grep -q "里程计已发散" "$LOG_DIR/odom_tf.log" 2>/dev/null; then
        echo "  [发散] TF 转发已自保护停止，地图不再更新，请 stop 后重启并全程手持"
        grep "里程计已发散" "$LOG_DIR/odom_tf.log" | tail -1 | sed 's/.*\] //;s/^/  /'
    else
        pos=$(timeout 6 ros2 topic echo "$ODOM_TOPIC" --once --field pose.pose.position 2>/dev/null \
              | awk -F': ' '/^x:/{x=$2} /^y:/{y=$2} /^z:/{z=$2} END{if(x!="")printf "%.2f, %.2f, %.2f", x,y,z}')
        [ -n "$pos" ] && echo "  当前位置(world): $pos"
    fi
    echo ""
    echo "=== rtabmap 状态 ==="
    grep -cE "Rejected loop closure" "$LOG_DIR/rtabmap.log" 2>/dev/null \
        | sed 's/^/  回环被拒次数: /'
    grep -E "Loop closure detected|Added .* to map" "$LOG_DIR/rtabmap.log" 2>/dev/null | tail -2 | sed 's/^/  /'
    echo ""
    echo "地图数据库: $DB"
    [ -f "$DB" ] && echo "  大小: $(du -h "$DB" | cut -f1)"
    exit 0
    ;;
stop)
    echo "停止联合建图..."
    for pat in "${COMPONENTS[@]}"; do
        pkill -f "$pat" 2>/dev/null
    done
    # 等优雅退出，超时未退则强杀（GUI 的 rviz2 对 SIGTERM 常不响应）
    for pat in "${COMPONENTS[@]}"; do
        if ! wait_gone "[${pat:0:1}]${pat:1}" 10; then
            echo "  强制结束: $pat"
            pkill -9 -f "[${pat:0:1}]${pat:1}" 2>/dev/null
        fi
    done
    echo "已停止（地图保留在 $DB）"
    exit 0
    ;;
esac

USE_RVIZ=1
for a in "$@"; do [ "$a" = "--no-rviz" ] && USE_RVIZ=0; done

mkdir -p "$LOG_DIR" "$MAP_DIR" $ROBOT_CORE/data/vins_output
rm -f "$LOG_DIR"/*.log   # 清空旧日志，避免残留关键字干扰就绪检测
need_source

echo "============================================"
echo " VINS-Fusion 里程计 + rtabmap 建图"
echo " VINS 配置:   $VINS_CONFIG"
echo " 地图数据库:  $DB"
echo " 日志目录:    $LOG_DIR"
echo "============================================"

if [ ! -f "$VINS_CONFIG" ]; then
    echo "[错误] VINS 配置不存在: $VINS_CONFIG"
    exit 1
fi

# ── 1. 相机（5 路流: infra1/infra2 给 VINS，color/aligned_depth 给 rtabmap，IMU 给 VINS）──
# USB 3.2 实测带宽足够: 5 路 640x360@30 约 48MB/s
# enable_sync=true : 驱动层帧同步，让 infra 与 color 时间戳落在同一帧集合，
#                    rtabmap 的 odom/图像 近似同步才对得上
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

# emitter_enabled 是驱动动态注册的 integer 参数(0关/1开/2自动)，
# launch 参数列表里没声明，只能运行时 set（传 bool 会被驱动报类型错误）
if timeout 15 ros2 param set /camera/camera depth_module.emitter_enabled 0 2>/dev/null | grep -q "successful"; then
    echo "      红外投影器已关闭（保 VIO 精度，代价是弱纹理处深度有空洞）"
else
    echo "      [警告] 关闭投影器失败，散斑会污染 VINS 光流跟踪"
    echo "             手动执行: ros2 param set /camera/camera depth_module.emitter_enabled 0"
fi

# ── 2. TF 桥接之一: base_link → camera_link（static）──
# 把整个 realsense TF 树挂到 base_link 之下（base_link 是父，camera_link 是子）。
# 显式指定节点名: static_transform_publisher 默认会给节点名追加随机后缀
# (如 static_transform_publisher_0OnrnmPJQLOSCf3U)，就绪检测无法精确匹配
echo "[2/6] 发布 static TF: $BASE_FRAME → $CAM_ROOT ..."
nohup ros2 run tf2_ros static_transform_publisher \
    --frame-id "$BASE_FRAME" --child-frame-id "$CAM_ROOT" \
    $B2C_XYZ $B2C_QUAT \
    --ros-args -r __node:=vins_body_tf \
    > "$LOG_DIR/tf_body.log" 2>&1 &
if ! wait_node /vins_body_tf 15; then
    echo "      [失败] static TF 未就绪，rtabmap 将无法反投影深度图"
    exit 1
fi
echo "      TF 桥接就绪"

# ── 3. VINS 里程计 ──
if ! timeout 8 ros2 topic list 2>/dev/null | grep -qx "$IMU_RAW_TOPIC"; then
    echo "      [失败] 无 IMU 话题 $IMU_RAW_TOPIC，VINS 无法启动"
    exit 1
fi
echo "[3/6] 启动 vins_node (双目红外 + IMU)..."
nohup ros2 run vins vins_node "$VINS_CONFIG" \
    > "$LOG_DIR/vins.log" 2>&1 &
# 注意: 本 VINS 构建注册的节点名为空(日志前缀 []:)，ros2 node list 里查不到
# /vins_estimator，故不能用 wait_node 判就绪——改由下方 "solver costs" 日志确认。
echo "      VINS 已启动（等待初始化）"

# 等待初始化: 日志出现 "solver costs"(后端优化启动)即认为成功。
# 双目尺度由 50mm 基线给出，不需要强激励，实测 2~3 秒即可完成。
# 必须等它初始化完再起 rtabmap: 未初始化时 /odometry 无输出，
# rtabmap 会因等不到 odom 而一帧都建不出来。
echo "      ★ 请现在拿起相机并保持缓慢移动，全程不要放下静止 ★"
echo "        （VIO 无零速约束，静止超过半分钟位置就会开始二次积分发散）"
if wait_log "$LOG_DIR/vins.log" "solver costs" 90; then
    echo "      ★ VINS 初始化成功，里程计开始输出（$ODOM_TOPIC）"
else
    echo "      [失败] 90 秒内 VINS 未初始化，rtabmap 收不到里程计，中止"
    echo "             排查: tail $LOG_DIR/vins.log"
    exit 1
fi

# ── 4. TF 桥接之二: world → base_link（动态，从 /odometry 转换后转发）──
# 要等 VINS 初始化完再起，否则没数据可转；也要在 rtabmap 之前起，
# 让 rtabmap 开始查 TF 时缓存里已有数据（否则刷 extrapolation into the past）
echo "[4/6] 启动 odom→TF 转发 (world → $BASE_FRAME，姿态转 REP-103)..."
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
    echo "      [失败] 未收到 $ODOM_TOPIC，TF 树仍会断开"
    exit 1
fi
grep "首帧已转发" "$LOG_DIR/odom_tf.log" | tail -1 | sed 's/.*: /      /'

# ── 5. rtabmap SLAM ──
# 注意: rtabmap 的核心参数（Rtabmap/*、Vis/*、Grid/* 这类）必须用字符串传，
# 写成 double/int 会抛 InvalidParameterTypeException 直接崩溃。
#
# frame_id:=base_link   : 必须是 REP-103 帧，且与 ODOM_BASE_TOPIC 成对（文件头第 2 点）
# approx_sync:=true     : 里程计约 15Hz 而图像 30Hz，帧率不同必须近似同步
#                         （run_rgbd_mapping.sh 用 false 是因为里程计与图像同源同频）
# odom 话题重映射       : rtabmap 默认订阅 "odom"，这里用转换后的 $ODOM_BASE_TOPIC
# wait_for_transform    : static TF 查询放宽，避免启动瞬间 TF 尚未缓存
cat > "$MAP_DIR/vins_grid_params.ini" <<'EOF'
# RTAB-Map 核心参数（VINS 联合建图专用）
# 注意 rtabmap 退出时会把全量参数写回这个文件，所以每次启动都重建，不要手工编辑
# 由 rtabmap 节点 config_path 参数加载（RTAB-Map 的 ini 格式: / 用 \ 分隔）
[Core]
Grid\3D=true
Grid\RayTracing=true
Grid\CellSize=0.05
Grid\RangeMax=8.0
# 法向量分割地面: 按点云法向判地面/障碍，产出 3D 的 /cloud_ground 与 /cloud_obstacles。
# 定位脚本 vins_loc_grid_params.ini 与此保持一致，两边同为 3D 才能匹配缓存栅格。
Grid\NormalsSegmentation=true
# 地面锚到 z=0 后 MaxGroundHeight 才有物理意义: 地面上方 0.10m 内算地面，其余算障碍
Grid\MaxGroundHeight=0.10
Grid\MaxObstacleHeight=2.0
# 法向量分割细化: 地面最大坡度角(度)、最小聚类点数、法向估计邻域点数
Grid\MaxGroundAngle=45
Grid\MinClusterSize=10
Grid\NormalK=20
# 在 map 帧做地面分割: 手持相机 roll/pitch 一直在变，要按位姿把点云先摆平。
# 前提是 frame_id 已经是 REP-103 的 base_link（否则这步会把点云转歪 90°）
Grid\MapFrameProjection=true
# 回环验证的 RANSAC 迭代数（默认300），弱纹理帧更容易找到解
Vis\Iterations=1000
EOF
echo "[5/6] 启动 rtabmap SLAM..."
nohup ros2 run rtabmap_slam rtabmap --ros-args \
    -p frame_id:="$BASE_FRAME" \
    -p subscribe_depth:=true \
    -p approx_sync:=true \
    -p wait_for_transform:=0.3 \
    -p database_path:="$DB" \
    -p config_path:="$MAP_DIR/vins_grid_params.ini" \
    -p "Rtabmap/DetectionRate:='1.0'" \
    -p cloud_output_voxelized:=true \
    -p delete_db_on_start:=true \
    -r odom:="$ODOM_BASE_TOPIC" \
    -r rgb/image:=$RGB_TOPIC \
    -r depth/image:=$DEPTH_TOPIC \
    -r rgb/camera_info:=$INFO_TOPIC \
    > "$LOG_DIR/rtabmap.log" 2>&1 &
if ! wait_node /rtabmap 30; then
    echo "      [失败] rtabmap 节点未注册，查看 $LOG_DIR/rtabmap.log"
    exit 1
fi
echo "      SLAM 已启动"

# 确认 rtabmap 真的收到了同步后的数据: 它每收够一组就会打印处理耗时。
# 只起节点不算成功——话题名或同步配置错时节点照样在，但一帧都不处理。
# 注意日志格式是 "rtabmap (65): Rate=1.00s"，带序号，不能写成 "rtabmap: Rate="
if wait_log "$LOG_DIR/rtabmap.log" "Rate=.*RTAB-Map=" 30; then
    echo "      已确认 rtabmap 在处理数据"
else
    echo "      [警告] 30s 内未见 rtabmap 处理数据，可能是同步问题"
    echo "             排查: grep -i 'did not receive\|sync' $LOG_DIR/rtabmap.log"
fi

# ── 5b. 地面零点自动标定 + 离地高度 ──
# rtabmap 就绪后 /cloud_ground 才有数据。节点取其全局中位数作地面 z，
# 算 RealSense 离地高度发 /realsense/height_above_ground + rviz /realsense/height_viz，
# write_anchor=false: 只做离地高度可视化，不再把分割反算的锚定写回文件
# （分割依赖锚定、锚定又依赖分割的循环依赖会污染基准，锚定改由运动学确定）。
echo "[5b] 启动离地高度可视化 (从 /cloud_ground, 不写锚定)..."
nohup python3 "$PROBE_PY" --ros-args \
    -p base_frame:="$BASE_FRAME" \
    -p world_frame:=map \
    -p ground_topic:=/cloud_ground \
    -p current_anchor:="$INITIAL_BASE_HEIGHT" \
    -p anchor_file:="$ANCHOR_FILE" \
    -p write_anchor:=false \
    > "$LOG_DIR/probe.log" 2>&1 &
if ! wait_node "$PROBE_NODE" 15; then
    echo "      [警告] 地面标定节点未就绪（不影响建图），查看 $LOG_DIR/probe.log"
else
    echo "      地面标定已就绪：自动写入 $ANCHOR_FILE（下次启动自动生效）"
fi

# ── 6. 可视化（rviz2）──
# 显示: /cloud_map 3D彩色点云 + /map 2D栅格 + /mapPath rtabmap优化后轨迹
#       + /path VINS 原始轨迹（两条对比可直观看出图优化修正了多少）
if [ "$USE_RVIZ" = "1" ]; then
    # 走 VNC: 让窗口显示在设备物理桌面上，Mac 通过 VNC 观看
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
    # 注意节点名: rviz2 可执行文件注册的节点名是 /rviz（logger 名才是 rviz2），
    # 两个都接受以防版本差异
    if wait_node /rviz 30 || wait_node /rviz2 3; then
        echo "      rviz2 已就绪"
    else
        echo "      [警告] rviz2 未就绪，查看 $LOG_DIR/rviz.log（不影响建图）"
    fi
fi

echo ""
echo "联合建图已启动! ★ 全程手持相机缓慢移动，不要放下静止 ★"
echo "  里程计: VINS-Fusion 双目红外+IMU（$ODOM_TOPIC）"
echo "  查看状态: $0 status   （会报告是否已发散）"
echo "  停止:     $0 stop"
