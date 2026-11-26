#!/usr/bin/env bash
set -eo pipefail
export ROS_DOMAIN_ID="${DYNAMIC_ROS_DOMAIN_ID:-36}"
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
source "$VISUAL_PERCEPTION_ROOT/scripts/dds_env.sh"
export DISPLAY="${DISPLAY:-:1}"
if [[ -z "${XAUTHORITY:-}" && -r "/run/user/$(id -u)/gdm/Xauthority" ]]; then
  export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
fi
export QT_QPA_PLATFORM=xcb
exec rviz2 -d "$VISUAL_PERCEPTION_ROOT/config/336l_dynamic.rviz" --ros-args -r __node:=dynamic_detection_rviz
