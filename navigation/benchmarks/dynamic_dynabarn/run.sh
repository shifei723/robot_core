#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$here/../common.sh"
bench_require_build
method=${1:-frs_joint};phase=${2:-0};trial=${3:-0};group=${DYNA_GROUP:-final}
protocol=${DYNA_PROTOCOL:-challenge}
localization=${DYNA_LOCALIZATION:-velocity_imu}
[[ "$localization" == velocity_imu || "$localization" == legacy ]] || exit 2
[[ "$protocol" == challenge || "$protocol" == legacy ]] || exit 2
[[ "$method" == mppi || "$method" == frs_soft || "$method" == frs_joint || "$method" == frs_hard || "$method" == mpc_d_cbf ]] || exit 2
[[ "$phase" =~ ^[0-9]+$ && "$trial" =~ ^[0-9]+$ && "$group" =~ ^[a-zA-Z0-9_-]+$ ]] || exit 2
[[ "${DYNA_ACTIVE_ESCAPE:-false}" == true || "${DYNA_ACTIVE_ESCAPE:-false}" == false ]] || { echo "DYNA_ACTIVE_ESCAPE必须为true或false";exit 2; }
[[ "${DYNA_LOCAL_PLANNER:-false}" == true || "${DYNA_LOCAL_PLANNER:-false}" == false ]] || { echo "DYNA_LOCAL_PLANNER必须为true或false";exit 2; }
[[ "${DYNA_JOINT_ROLLOUT:-false}" == true || "${DYNA_JOINT_ROLLOUT:-false}" == false ]] || exit 2
name="${method}_phase_${phase}_trial_${trial}"
[[ ! -e "$here/results/$group/$name" ]] || { echo '结果已存在，请使用新的分组或试验编号';exit 2; }
mkdir -p "$here/logs/$group"
bench_container "robot-core-dynamic-${method}-${phase}-${trial}" 197 bash -c '
set -eo pipefail
mkdir -p "$HOME"
source /opt/ros/jazzy/setup.bash
source "$BENCH_WS/install/setup.bash"
here=/repo/navigation/benchmarks/dynamic_dynabarn
ros2 launch dynabarn_ros2 comparison.launch.py method:="$1" motion_phase:="$2" active_escape:="$5" local_dynamic_planner:="$6" joint_rollout:="$7" protocol:="$8" localization_profile:="$9" > "$here/logs/$4/${1}_phase_${2}_trial_${3}_launch.log" 2>&1 &
pid=$!
trap '\''kill -TERM "$pid" 2>/dev/null || true;wait "$pid" || true'\'' EXIT
python3 "$BENCH_WS/src/dynabarn_ros2/scripts/compare_evaluate.py" --method "$1" --motion-phase "$2" --active-escape "$5" --local-dynamic-planner "$6" --joint-rollout "$7" --protocol "$8" --localization-profile "$9" --output "$here/results/$4/${1}_phase_${2}_trial_${3}" --ros-args -p use_sim_time:=true
' bash "$method" "$phase" "$trial" "$group" "${DYNA_ACTIVE_ESCAPE:-false}" "${DYNA_LOCAL_PLANNER:-false}" "${DYNA_JOINT_ROLLOUT:-false}" "$protocol" "$localization" 2>&1 | tee "$here/logs/$group/${name}_evaluation.log"
