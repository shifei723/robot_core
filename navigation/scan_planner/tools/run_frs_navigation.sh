#!/usr/bin/env bash
# Start the existing Slash simulation with FRS-based spline timing enabled.
set -e
if [[ "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: bash navigation/scan_planner/tools/run_frs_navigation.sh [ROS launch arguments]

Starts Slash ROS kinematic navigation with dynamic_prediction:=frs.
Requires install/ws_nav_frs and a live world-frame pedestrian track on
/dynamic_navigation/pedestrian (nav_msgs/Odometry, child_frame_id=pedestrian_N).
For three targets pass dynamic_min_tracks:=3; each known target must remain fresh.
With no fresh track the controller waits; this launcher does not synthesize one.

For the self-contained crossing experiment use validate_dynamic_navigation.py.
See navigation/scan_planner/FRS_NAVIGATION.md for build and test commands.
EOF
  exit 0
fi
robot_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
setup_file="$robot_root/install/ws_nav_frs/setup.bash"
if [[ ! -f "$setup_file" ]]; then
  echo "Missing $setup_file; build ws_nav_frs as described in FRS_NAVIGATION.md." >&2
  exit 1
fi
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source "$setup_file"
exec ros2 launch scan_planner run.launch.py \
  robot_model:=slash is_real_world:=false controller_mode:=closed_loop \
  use_gpu:=false dynamic_obstacles:=false dynamic_prediction:=frs \
  init_x:=0.0 init_y:=-2.0 init_yaw:=1.5707963267948966 "$@"
