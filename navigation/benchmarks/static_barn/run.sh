#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$here/../common.sh"
bench_require_build
method=${1:-slash};world=${2:-0};trial=${3:-0}
[[ "$method" == mppi || "$method" == slash ]] || exit 2
[[ "$world" == 0 || "$world" == 6 || "$world" == 12 || "$world" == 18 || "$world" == 36 || "$world" == 54 ]] || { echo '当前随代码提供的场景为 0、6、12、18、36、54';exit 2; }
[[ "$trial" =~ ^[0-9]+$ ]] || exit 2
if [[ "$method" == mppi ]];then default_group=spatial_mppi;else default_group=spatial_release;fi
group=${BARN_RUN_SET:-$default_group}
[[ "$group" =~ ^[a-zA-Z0-9_-]+$ ]] || exit 2
[[ ! -e "$here/results/$group/${method}_world_${world}_trial_${trial}.json" ]] || { echo '结果已存在，请使用新的分组或试验编号';exit 2; }
mkdir -p "$here/logs/$group" "$here/results/$group"
bench_container "robot-core-static-${method}-${world}-${trial}" 187 bash -c '
set -eo pipefail
mkdir -p "$HOME"
source /opt/ros/jazzy/setup.bash
source "$BENCH_WS/install/setup.bash"
here=/repo/navigation/benchmarks/static_barn
ros2 launch "$here/benchmark.launch.py" method:="$1" world_idx:="$2" > "$here/logs/$4/${1}_${2}_${3}_launch.log" 2>&1 &
pid=$!
trap '\''kill -TERM "$pid" 2>/dev/null || true;wait "$pid" || true'\'' EXIT
python3 "$here/evaluate.py" --method "$1" --world "$2" --trial "$3" --launch-pid "$pid" --output "$here/results/$4" --goal-tolerance "$5" --ros-args -p use_sim_time:=true
' bash "$method" "$world" "$trial" "$group" "${BARN_GOAL_TOLERANCE:-1.0}" 2>&1 | tee "$here/logs/$group/${method}_${world}_${trial}_evaluation.log"
