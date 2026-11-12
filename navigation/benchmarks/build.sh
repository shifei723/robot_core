#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
python3 "$BENCH_ROOT/prepare_workspace.py"
bench_container "robot-core-bench-build-$$" 199 bash -c '
set -eo pipefail
mkdir -p "$HOME"
source /opt/ros/jazzy/setup.bash
cd "$BENCH_WS"
MAKEFLAGS=-j2 colcon build --base-paths src --packages-up-to scan_planner geometric_obstacles jackal_helper dynabarn_ros2 \
 --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
'
