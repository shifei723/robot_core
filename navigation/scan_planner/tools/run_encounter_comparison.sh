#!/usr/bin/env bash
# Matched perception and geometry; only timing policy differs.
set -eo pipefail
cd "$(dirname "$0")/../../.."
source /opt/ros/jazzy/setup.bash
source install/ws_nav_frs/setup.bash
prefix=${1:-encounter_$(date +%Y%m%d_%H%M%S)}
domain=${2:-201}
weight=${3:-2.0}
common=(--case encounter --static-prior --shifted-map-frame --duration 75
  --encounter-trigger-x .6 --encounter-x 1.8 --encounter-side 1.1
  --encounter-end-y .8 --encounter-speed .22 --camera-focal 170
  --measurement-sigma .06 --accel-sigma .2 --spatial-weight 0 --frs-weight "$weight")
for policy in soft hard; do
  /usr/bin/python3 navigation/scan_planner/tools/validate_geometric_pipeline.py \
    --output "data/geometric_validation/${prefix}_${policy}" --domain "$domain" \
    --timing-policy "$policy" "${common[@]}"
  domain=$((domain+1))
done
/usr/bin/python3 navigation/scan_planner/tools/report_encounter_comparison.py \
  "data/geometric_validation/${prefix}_soft" "data/geometric_validation/${prefix}_hard" \
  --output "data/geometric_validation/${prefix}_comparison.json"
