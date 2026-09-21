#!/usr/bin/env bash
# ============================================================
# robot_core 统一环境引导脚本（可移植）
#
# 用法:
#   source /path/to/robot_core/setup.sh
#
# 作用:
#   1. 自动定位仓库根目录并导出 ROBOT_CORE（所有脚本/launch/节点据此解析路径）
#   2. 导出 ROBOT_MODELS（模型资产目录，仓库外，可被外部覆盖）
#   3. 加载系统 ROS2（默认 humble）
#   4. 按依赖顺序加载本仓库各工作区已构建的 install/（存在才加载）
#
# 说明: 本仓库保留“多工作区”布局，分组见 build.sh:
#   ros_ws -> rtabmap -> ws_nav -> agent_ws -> tts
# ============================================================

# 定位仓库根（跟随软链接，兼容任意 clone 路径）
_RC_SRC="${BASH_SOURCE[0]:-$0}"
while [ -h "$_RC_SRC" ]; do
  _RC_TARGET="$(readlink "$_RC_SRC")"
  case "$_RC_TARGET" in
    /*) _RC_SRC="$_RC_TARGET" ;;
    *)  _RC_SRC="$(dirname "$_RC_SRC")/$_RC_TARGET" ;;
  esac
done
export ROBOT_CORE="$(cd "$(dirname "$_RC_SRC")" && pwd)"

# 模型资产目录（大二进制不进版本库，默认取仓库同级的 models/，回退 /data/models）
: "${ROBOT_MODELS:=$ROBOT_CORE/../models}"
[ -d "$ROBOT_MODELS" ] || ROBOT_MODELS="/data/models"
export ROBOT_MODELS

# 加载 ROS2
: "${ROS_DISTRO:=humble}"
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "/opt/ros/$ROS_DISTRO/setup.bash"
else
  echo "[setup.sh] 警告: 未找到 /opt/ros/$ROS_DISTRO/setup.bash（请先安装 ROS2 $ROS_DISTRO）" >&2
fi

# 按依赖顺序加载各工作区 install（首次未构建时自动跳过）
for _g in ros_ws rtabmap ws_nav agent_ws tts; do
  _s="$ROBOT_CORE/install/$_g/setup.bash"
  # shellcheck disable=SC1090
  [ -f "$_s" ] && source "$_s"
done

# Open3D 预编译版（不入库）：缺失时提示获取方式
if [ ! -d "$ROBOT_CORE/third/open3d141" ] && [ ! -d "$ROBOT_CORE/third/open3d141_x86" ]; then
  echo "[setup.sh] 提示: Open3D 未就位，构建 open3d_loc 前请先运行: bash assets/fetch_open3d.sh"
fi

echo "[setup.sh] ROBOT_CORE  = $ROBOT_CORE"
echo "[setup.sh] ROBOT_MODELS= $ROBOT_MODELS"
echo "[setup.sh] 已加载的工作区 install: $(ls "$ROBOT_CORE/install" 2>/dev/null | tr '\n' ' ')"

unset _RC_SRC _RC_TARGET _g _s
