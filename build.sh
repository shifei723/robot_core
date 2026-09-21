#!/usr/bin/env bash
# ============================================================
# robot_core 分组构建脚本（保留“多工作区”布局，各组隔离 install）
#
# 用法:
#   ./build.sh [ros_ws|rtabmap|ws_nav|agent_ws|tts|all]
#   默认 all：按依赖顺序依次构建全部工作区。
#
# 依赖关系（构建顺序即加载顺序）:
#   ros_ws  -> rtabmap -> ws_nav -> agent_ws -> tts
#
# 说明:
#   - 每组构建产物落在 install/<组>，setup.sh 会按上面的顺序全部加载。
#   - 不参与 colcon 的第三方/原生库（librealsense、kws、llm_sdk、
#     unitree_lidar_sdk）通过 COLCON_IGNORE 或无 package.xml 自动排除。
#   - rtabmap/realsense2_camera 依赖系统已安装的 rtabmap、librealsense2，
#     请先按 assets/PREREQUISITES.md 装好系统依赖。
# ============================================================
set -e

_RC_SRC="${BASH_SOURCE[0]}"
while [ -h "$_RC_SRC" ]; do _RC_SRC="$(readlink "$_RC_SRC")"; done
export ROBOT_CORE="$(cd "$(dirname "$_RC_SRC")" && pwd)"
cd "$ROBOT_CORE"

: "${ROS_DISTRO:=humble}"
[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ] && source "/opt/ros/$ROS_DISTRO/setup.bash"

# 包内编译并行度（单个包 make -jN）。
# 本机 4 核 / 8G 内存，rtabmap、PCL、Open3D 这类重型包单个编译单元就要 1~2G，
# 默认 2：只用一半 CPU，给系统和桌面留余量，避免拉满 CPU / OOM 卡死。
# 想手动调整: MAKE_JOBS=1 ./build.sh rtabmap （最稳） / MAKE_JOBS=3（更快但更吃内存）
MAKE_JOBS="${MAKE_JOBS:-2}"
# 包级并发（同时编译几个包），默认 1 = 一次只编一个包。
PKG_WORKERS="${PKG_WORKERS:-1}"
# 是否包间串行（sequential）以避免多个重型包(PCL/Open3D/OpenCV)同时编译爆内存。
# 嵌入式/小内存机器保持 1（默认）；大内存机器可设 PKG_SERIAL=0 改回并行。
PKG_SERIAL="${PKG_SERIAL:-1}"
GROUP="${1:-all}"

# ---- 各工作区包含的源码目录（相对 ROBOT_CORE）----
PATHS_ros_ws="localization/lidar_slam localization/lidar_localization \
navigation/zone_interfaces navigation/nav2_task_manager navigation/rviz2_zone_plugin \
navigation/l2_nav_bringup navigation/costmap_intensity robot \
sensors/camera/mvs_ros2_pkg sensors/lidar/livox_ros2_driver sensors/lidar/rslidar_sdk \
sensors/lidar/rslidar_msg sensors/lidar/HesaiLidar_ROS_2.0 sensors/lidar/unitree_lidar \
sensors/imu_serial"
PATHS_rtabmap="localization/visual_slam"
PATHS_ws_nav="navigation/scan_planner"
PATHS_agent_ws="voice/agent"
PATHS_tts="voice/tts"

build_group() {
  local g="$1"
  local var="PATHS_$g"
  local paths
  eval "paths=\$$var"
  echo "==================== 构建工作区: $g ===================="
  echo "  base-paths: $paths"
  # 加载前序已构建工作区，解析跨组依赖
  local s
  for s in ros_ws rtabmap ws_nav agent_ws tts; do
    [ "$s" = "$g" ] && continue
    [ -f "$ROBOT_CORE/install/$s/setup.bash" ] && source "$ROBOT_CORE/install/$s/setup.bash"
  done
  # shellcheck disable=SC2086
  export MAKEFLAGS="-j${MAKE_JOBS}"
  echo "  包内并行: make -j${MAKE_JOBS} | 包级并发: ${PKG_WORKERS} | 包间串行: ${PKG_SERIAL}"
  _exec_arg=""
  [ "${PKG_SERIAL:-1}" = "1" ] && _exec_arg="--executor sequential"
  colcon --log-base "log/$g" build \
    --base-paths $paths \
    --build-base   "build/$g" \
    --install-base "install/$g" \
    --continue-on-error \
    --parallel-workers "${PKG_WORKERS}" \
    $_exec_arg \
    --event-handlers console_direct+
}

case "$GROUP" in
  ros_ws|rtabmap|ws_nav|agent_ws|tts)
    build_group "$GROUP"
    ;;
  all)
    # 注意: colcon 带 --continue-on-error 时，只要有包失败仍返回非 0。
    # 配合文件开头的 set -e，会导致某组失败后后面的组全部被跳过（历史上
    # ros_ws 一失败，rtabmap/ws_nav/agent_ws/tts 就再也没编过）。
    # 这里显式吞掉失败并继续，最后统一汇总。
    _FAILED=""
    for _g in ros_ws rtabmap ws_nav agent_ws tts; do
      build_group "$_g" || _FAILED="$_FAILED $_g"
    done
    if [ -n "$_FAILED" ]; then
      echo ""
      echo "[警告] 以下工作区构建失败:$_FAILED"
      echo "       修复后可单独重编: $0 <组名>"
      unset _FAILED _g
      exit 1
    fi
    unset _FAILED _g
    ;;
  *)
    echo "用法: $0 [ros_ws|rtabmap|ws_nav|agent_ws|tts|all]" >&2
    exit 1
    ;;
esac

echo ""
echo "构建完成。运行前请加载环境:"
echo "  source $ROBOT_CORE/setup.sh"
