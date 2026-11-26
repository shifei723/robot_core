#!/usr/bin/env bash
# Local-only dependency bootstrap; no sudo, apt install or global environment changes.
set -euo pipefail
perception_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
perception_prefix="$perception_root/.deps"
mkdir -p "$perception_prefix/downloads" "$perception_prefix/src"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'ROS 2 Jazzy is required. See README.md system prerequisites.' >&2
  exit 1
fi
for perception_deb in ros-jazzy-foxglove-bridge ros-jazzy-rosx-introspection ros-jazzy-vision-msgs rapidjson-dev; do
  (
    cd "$perception_prefix/downloads"
    apt-get download "$perception_deb"
    for perception_archive in "${perception_deb}"_*.deb; do
      [[ -f "$perception_archive" ]] && dpkg-deb -x "$perception_archive" "$perception_prefix"
    done
  )
done
if [[ ! -f "$perception_prefix/lib/cmake/Ceres/CeresConfig.cmake" ]]; then
  if [[ ! -d "$perception_prefix/src/ceres-solver-2.1.0" ]]; then
    curl -fL --connect-timeout 15 --max-time 180 https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.1.0.tar.gz -o "$perception_prefix/downloads/ceres-2.1.0.tar.gz"
    tar -xzf "$perception_prefix/downloads/ceres-2.1.0.tar.gz" -C "$perception_prefix/src"
  fi
  cmake -S "$perception_prefix/src/ceres-solver-2.1.0" -B "$perception_prefix/ceres-build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$perception_prefix" \
    -DBUILD_SHARED_LIBS=ON -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DMINIGLOG=OFF
  cmake --build "$perception_prefix/ceres-build" --parallel "${MAKE_JOBS:-2}"
  cmake --install "$perception_prefix/ceres-build"
fi
bash "$perception_root/scripts/download_yolo_model.sh"
echo 'Local dependencies prepared. Build with: bash build.sh ws_perception'
