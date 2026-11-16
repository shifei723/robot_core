#!/usr/bin/env bash
# Indoor Slash simulation with the BARN MPPI speed/yaw caps; keeps height safety.
set -euo pipefail
TASK_CORE_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
TASK_METHOD=${1:-frs_soft}
TASK_OBSTACLE_SPEED=${2:-0.22}
TASK_OUTPUT=${3:-"${TASK_CORE_ROOT}/output/indoor_benchmark_speed/${TASK_METHOD}_${TASK_OBSTACLE_SPEED}"}
case "${TASK_METHOD}" in
  frs_soft) TASK_TIMING=soft; TASK_PREDICTION=frs ;;
  frs_hard) TASK_TIMING=hard; TASK_PREDICTION=frs ;;
  cv_hard) TASK_TIMING=hard; TASK_PREDICTION=cv ;;
  *) echo 'method must be frs_soft, frs_hard or cv_hard' >&2; exit 2 ;;
esac
if [[ -e "${TASK_OUTPUT}/summary.json" ]]; then
  echo "Refusing to overwrite completed run: ${TASK_OUTPUT}" >&2; exit 2
fi
set +u
source /opt/ros/jazzy/setup.bash
source "${TASK_CORE_ROOT}/install/ws_nav_frs/setup.bash"
set -u
exec /usr/bin/python3 "${TASK_CORE_ROOT}/navigation/scan_planner/tools/validate_geometric_pipeline.py" \
  --case indoor_turn_height --static-prior --timing-policy "${TASK_TIMING}" \
  --prediction "${TASK_PREDICTION}" --spatial-weight 0 \
  --encounter-speed "${TASK_OBSTACLE_SPEED}" --coast-timeout 1 --duration 180 \
  --height-spatial-tracking --robot-speed 0.5 --robot-yaw-rate 1.0 --robot-acceleration 0.5 --robot-braking 0.5 \
  --domain "${INDOOR_SIM_DOMAIN:-193}" --output "${TASK_OUTPUT}"
