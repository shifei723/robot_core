#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$ROBOT_CORE"
if [[ ! -f "$VISUAL_PERCEPTION_ROOT/.deps/lib/cmake/Ceres/CeresConfig.cmake" ]]; then
  echo 'Missing Ceres 2.1; run bash perception/visual_dynamic_perception/scripts/bootstrap_dependencies.sh' >&2
  exit 1
fi
export MAKEFLAGS="-j${MAKE_JOBS:-2}"
CMAKE_BUILD_PARALLEL_LEVEL="${MAKE_JOBS:-2}" colcon --log-base log/ws_perception build \
 --base-paths sensors/camera/orbbec_camera/orbbec_camera_msgs \
 sensors/camera/orbbec_camera/orbbec_camera sensors/camera/orbbec_camera/orbbec_description \
 localization/visual_slam/vins_fusion/camera_models localization/visual_slam/vins_fusion/vins \
 localization/visual_slam/open_vins/ov_core localization/visual_slam/open_vins/ov_init localization/visual_slam/open_vins/ov_msckf \
 perception/onboard_detector \
 --build-base build/ws_perception --install-base install/ws_perception \
 --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
 -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 \
 -DCeres_DIR="$VISUAL_PERCEPTION_ROOT/.deps/lib/cmake/Ceres" -DINSTALL_UDEV_RULES=OFF
