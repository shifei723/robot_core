#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
source "$VISUAL_PERCEPTION_ROOT/scripts/dds_env.sh"
cd "$VISUAL_PERCEPTION_ROOT"
/usr/bin/python3 scripts/test_rgbd_motion.py
/usr/bin/python3 scripts/test_vins_body_odometry.py
/usr/bin/python3 scripts/test_openvins_config.py
/usr/bin/python3 scripts/test_yolo_semantics.py
/usr/bin/python3 scripts/test_openvins_synthetic.py
/usr/bin/python3 scripts/test_dynamic_detector.py --semantic matched
/usr/bin/python3 scripts/test_dynamic_detector.py --semantic unmatched
/usr/bin/python3 scripts/test_dynamic_detector.py --semantic unknown-strong
/usr/bin/python3 scripts/test_dynamic_detector.py --semantic stale
/usr/bin/python3 scripts/test_dynamic_detector.py --motion lateral --pose-loss
