#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$here/../common.sh"
bench_require_build
method=${1:-frs_soft};phase=${2:-0};trial=${3:-0};group=${DYNA_GROUP:-final}
[[ "$method" == mppi || "$method" == frs_soft || "$method" == frs_hard ]] || exit 2
[[ "$phase" =~ ^[0-9]+$ && "$trial" =~ ^[0-9]+$ && "$group" =~ ^[a-zA-Z0-9_-]+$ ]] || exit 2
name="${method}_phase_${phase}_trial_${trial}"
[[ ! -e "$here/results/$group/$name" ]] || { echo '结果已存在，请使用新的分组或试验编号';exit 2; }
mkdir -p "$here/logs/$group"
bench_container "robot-core-dynamic-${method}-${phase}-${trial}" 197 bash -c '
set -eo pipefail
mkdir -p "$HOME"
source /opt/ros/jazzy/setup.bash
source "$BENCH_WS/install/setup.bash"
here=/repo/navigation/benchmarks/dynamic_dynabarn
ros2 launch dynabarn_ros2 comparison.launch.py method:="$1" motion_phase:="$2" > "$here/logs/$4/${1}_phase_${2}_trial_${3}_launch.log" 2>&1 &
pid=$!
trap '\''kill -TERM "$pid" 2>/dev/null || true;wait "$pid" || true'\'' EXIT
python3 "$BENCH_WS/src/dynabarn_ros2/scripts/compare_evaluate.py" --method "$1" --motion-phase "$2" --output "$here/results/$4/${1}_phase_${2}_trial_${3}" --ros-args -p use_sim_time:=true
' bash "$method" "$phase" "$trial" "$group" 2>&1 | tee "$here/logs/$group/${name}_evaluation.log"
