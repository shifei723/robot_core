#!/usr/bin/env bash
# All nine predeclared runs, sequential to avoid compute-contention bias.
set -eo pipefail
cd "$(dirname "$0")/../../.."
source /opt/ros/jazzy/setup.bash
source install/ws_nav_frs/setup.bash
prefix=${1:-thesis_indoor_compare_20261023}
mkdir -p "data/geometric_validation/${prefix}"
for speed in 0.18 0.22 0.26; do
 for method in cv_hard frs_hard frs_soft; do
  prediction=${method%_*}; policy=${method#*_}
  run="data/geometric_validation/${prefix}/${method}_${speed}"
  if [[ -e "$run/summary.json" ]]; then
   /usr/bin/python3 -c 'import json,sys; r=json.load(open(sys.argv[1])); assert r.get("duration") is not None and r.get("error") is None' "$run/summary.json"
   continue
  fi
  /usr/bin/python3 navigation/scan_planner/tools/validate_geometric_pipeline.py \
   --output "$run" --case indoor_turn_height --static-prior --shifted-map-frame \
   --duration 180 --domain 211 --prediction "$prediction" --timing-policy "$policy" \
   --frs-weight 2 --spatial-weight 0 --measurement-sigma .06 --accel-sigma .2 \
   --camera-focal 170 --encounter-speed "$speed" > "data/geometric_validation/${prefix}/${method}_${speed}.log" 2>&1
 done
done
