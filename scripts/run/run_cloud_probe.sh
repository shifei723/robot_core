#!/bin/bash
# --- robot_core 可移植自定位 ---
ROBOT_CORE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")/../.." && pwd)"; export ROBOT_CORE
set +u
source "$ROBOT_CORE/setup.sh"
true
export PYTHONUNBUFFERED=1
cd $ROBOT_CORE/voice/agent/src/omni_node/scripts || exit 1
python3 probe_cloud_width.py
