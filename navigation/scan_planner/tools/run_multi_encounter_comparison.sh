#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/../../.."
source /opt/ros/jazzy/setup.bash
source install/ws_nav_frs/setup.bash
prefix=${1:-multi_encounter_$(date +%Y%m%d_%H%M%S)}
domain=${2:-201}
common=(--case multi_encounter --static-prior --shifted-map-frame --duration 110
  --camera-focal 170 --measurement-sigma .06 --accel-sigma .2
  --spatial-weight 0 --frs-weight 2)
for policy in soft hard; do
  /usr/bin/python3 navigation/scan_planner/tools/validate_geometric_pipeline.py \
    --output "data/geometric_validation/${prefix}_${policy}" --domain "$domain" \
    --timing-policy "$policy" "${common[@]}"
  domain=$((domain+1))
done
/usr/bin/python3 navigation/scan_planner/tools/report_multi_encounter.py \
  "data/geometric_validation/${prefix}_soft" "data/geometric_validation/${prefix}_hard" \
  --output "data/geometric_validation/${prefix}_comparison.json"
