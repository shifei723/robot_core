#!/usr/bin/env bash
# Same production nodes as the PCD/depth headless validation; no learned weights.
set -e
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export GEOMETRIC_PERCEPTION=true
export STATIC_PRIOR=true
export STATIC_PRIOR_TOPIC="${STATIC_PRIOR_TOPIC:-/cloud_obstacles}"
exec bash "$SCRIPT_DIR/run_vins_navigation.sh" "$@"
