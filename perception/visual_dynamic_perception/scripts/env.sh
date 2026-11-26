#!/usr/bin/env bash
# Dedicated Jazzy overlay. Does not source or reconfigure the navigation/recording stacks.
VISUAL_PERCEPTION_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ROBOT_CORE="$(cd "$VISUAL_PERCEPTION_ROOT/../.." && pwd)"
export PATH="/usr/bin:/bin:$PATH"
source /opt/ros/jazzy/setup.bash
export AMENT_PREFIX_PATH="$VISUAL_PERCEPTION_ROOT/.deps/opt/ros/jazzy:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="$VISUAL_PERCEPTION_ROOT/.deps:$VISUAL_PERCEPTION_ROOT/.deps/opt/ros/jazzy:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$VISUAL_PERCEPTION_ROOT/.deps/lib:$VISUAL_PERCEPTION_ROOT/.deps/opt/ros/jazzy/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$VISUAL_PERCEPTION_ROOT/.deps/opt/ros/jazzy/lib/python3.12/site-packages:${PYTHONPATH:-}"
if [[ -f "$ROBOT_CORE/install/ws_perception/local_setup.bash" ]]; then
  source "$ROBOT_CORE/install/ws_perception/local_setup.bash"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-36}"
