#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/../../.."
source /opt/ros/jazzy/setup.bash
source install/ws_nav_frs/setup.bash
prefix=${1:-indoor_turn_height_$(date +%Y%m%d_%H%M%S)}
domain=${2:-201}
/usr/bin/python3 navigation/scan_planner/tools/validate_geometric_pipeline.py \
  --output "data/geometric_validation/${prefix}" --case indoor_turn_height \
  --static-prior --shifted-map-frame --duration 180 --domain "$domain" \
  --timing-policy soft --frs-weight 2 --spatial-weight .5 \
  --measurement-sigma .06 --accel-sigma .2 --camera-focal 170
/usr/bin/python3 navigation/scan_planner/tools/report_indoor_navigation.py \
  "data/geometric_validation/${prefix}" \
  --output "data/geometric_validation/${prefix}/evaluation.json"
