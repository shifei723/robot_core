#!/usr/bin/env bash
# Full indoor Slash simulation: BARN MPPI speed caps, spatial FRS .5, timing FRS 2, height safety.
set -euo pipefail
TASK_CORE_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
TASK_METHOD=${1:-frs_soft}
TASK_OBSTACLE_SPEED=${2:-0.22}
TASK_OUTPUT=${3:-"${TASK_CORE_ROOT}/output/indoor_full_speed/${TASK_METHOD}_${TASK_OBSTACLE_SPEED}"}
case "${TASK_METHOD}" in
  full|frs_soft) TASK_TIMING=soft; TASK_PREDICTION=frs; TASK_SPATIAL=0.5 ;;
  soft_time) TASK_TIMING=soft; TASK_PREDICTION=frs; TASK_SPATIAL=0.0 ;;
  frs_hard) TASK_TIMING=hard; TASK_PREDICTION=frs; TASK_SPATIAL=0.0 ;;
  cv_hard) TASK_TIMING=hard; TASK_PREDICTION=cv; TASK_SPATIAL=0.0 ;;
  *) echo 'method must be full/frs_soft, soft_time, frs_hard or cv_hard' >&2; exit 2 ;;
esac
if [[ -e "${TASK_OUTPUT}/summary.json" ]]; then
  echo "Refusing to overwrite completed run: ${TASK_OUTPUT}" >&2; exit 2
fi
set +u
source /opt/ros/jazzy/setup.bash
source "${TASK_CORE_ROOT}/install/ws_nav_frs/setup.bash"
set -u
exec /usr/bin/python3 "${TASK_CORE_ROOT}/navigation/scan_planner/tools/validate_geometric_pipeline.py" \
  --case indoor_turn_height --indoor-scenario dual_crossing --beam-clearance .60 \
  --minimum-leg-height .13 --goal-height-tolerance .008 --search-budget .1 --search-heuristic 1.8 \
  --camera-focal 170 --record-period .05 --compress-trace --static-prior --timing-policy "${TASK_TIMING}" \
  --prediction "${TASK_PREDICTION}" --spatial-weight "${TASK_SPATIAL}" \
  --encounter-speed "${TASK_OBSTACLE_SPEED}" --coast-timeout 1 --duration 180 \
  --height-spatial-tracking --robot-speed 0.5 --robot-yaw-rate 1.0 --robot-acceleration 0.5 --robot-braking 0.5 \
  --domain "${INDOOR_SIM_DOMAIN:-193}" --output "${TASK_OUTPUT}"
