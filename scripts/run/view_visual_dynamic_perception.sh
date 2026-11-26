#!/usr/bin/env bash
set -eo pipefail
perception_core_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
exec bash "$perception_core_root/perception/visual_dynamic_perception/scripts/view_rviz.sh" "$@"
