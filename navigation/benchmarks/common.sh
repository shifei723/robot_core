#!/usr/bin/env bash
# Shared runtime only. Algorithms are built directly from robot_core source.
BENCH_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ROBOT_CORE_ROOT=$(cd -- "$BENCH_ROOT/../.." && pwd)
BENCH_WS=/repo/navigation/benchmarks/.workspaces/jazzy
BENCH_IMAGE=${BENCH_IMAGE:-barn-jazzy:local}
bench_container() {
  local container_name=$1 domain=$2
  shift 2
  docker run --rm --name "$container_name" --network host --ipc private --shm-size 1g \
    --user "$(id -u):$(id -g)" -e HOME=/tmp/benchmark-home \
    -e ROS_DOMAIN_ID="$domain" -e GZ_PARTITION="$container_name" \
    -e BENCH_WS="$BENCH_WS" -e MPLCONFIGDIR=/tmp/matplotlib \
    -v "$ROBOT_CORE_ROOT:/repo" "$BENCH_IMAGE" "$@"
}
bench_require_build() {
  [[ -f "$BENCH_ROOT/.workspaces/jazzy/install/setup.bash" ]] || {
    echo '请先运行 navigation/benchmarks/build.sh'; return 2;
  }
}
