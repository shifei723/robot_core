#!/usr/bin/env bash
set -eo pipefail
# Dedicated DDS domain: never inherit the teleoperation/recording domain (42).
export ROS_DOMAIN_ID="${DYNAMIC_ROS_DOMAIN_ID:-36}"
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
source "$VISUAL_PERCEPTION_ROOT/scripts/dds_env.sh"
cd "$VISUAL_PERCEPTION_ROOT"
mkdir -p runtime/336l logs/live
# Reject accidental duplicate service, including the original robot_sense instance.
/usr/bin/python3 - "${FOXGLOVE_PORT:-9876}" <<'CHECK_PORT'
import socket,sys
with socket.socket() as probe:
    try:probe.bind(('0.0.0.0',int(sys.argv[1])))
    except OSError:raise SystemExit('Foxglove port already occupied; stop the previous dedicated perception service before switching.')
CHECK_PORT
if [[ ! -f "$ROBOT_CORE/install/ws_perception/setup.bash" ]]; then
  echo 'Build this stack first: bash build.sh ws_perception' >&2
  exit 1
fi
exec 9>runtime/336l/service.lock
if ! flock -n 9; then
  echo "The detection service is already running or still stopping; wait before restarting." >&2
  exit 1
fi
export YOLO_ASSIST="${YOLO_ASSIST:-1}"
export CAMERA_SERIAL="${CAMERA_SERIAL:-CPC87630003A}"
LOCALIZATION_MODE="${LOCALIZATION_MODE:-vins}"
if [[ "$LOCALIZATION_MODE" != vins && "$LOCALIZATION_MODE" != stationary && "$LOCALIZATION_MODE" != rgbd && "$LOCALIZATION_MODE" != openvins ]]; then
  echo "LOCALIZATION_MODE must be vins, openvins, rgbd or stationary" >&2
  exit 1
fi
# Never fall back to whichever USB camera happens to enumerate first.
found=false
for serial_file in /sys/bus/usb/devices/*/serial; do
  if [[ -r "$serial_file" ]] && [[ "$(cat "$serial_file")" == "$CAMERA_SERIAL" ]]; then
    found=true
    break
  fi
done
if [[ "$found" != true ]]; then
  echo "Required dynamic-detection camera $CAMERA_SERIAL is not connected." >&2
  exit 1
fi
if [[ "$YOLO_ASSIST" == 1 && ! -f "$VISUAL_PERCEPTION_ROOT/models/yolox_tiny.onnx" ]]; then
  echo "YOLO model missing; run: bash scripts/download_yolo_model.sh" >&2
  exit 1
fi
mkdir -p runtime/336l logs/live
DEPTH_REGISTRATION=false
if [[ "$LOCALIZATION_MODE" == rgbd ]]; then DEPTH_REGISTRATION=true; fi
pids=()
cleanup() {
  # Background jobs may inherit SIGINT as ignored; terminate our children too.
  for pid in "${pids[@]}"; do
    pkill -TERM -P "$pid" 2>/dev/null || true
    kill -TERM "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
ros2 launch orbbec_camera gemini_330_series.launch.py \
  camera_name:=camera serial_number:="$CAMERA_SERIAL" \
  depth_registration:="$DEPTH_REGISTRATION" align_mode:=SW \
  depth_width:=640 depth_height:=400 depth_fps:=30 \
  left_ir_width:=640 left_ir_height:=400 left_ir_fps:=30 enable_left_ir:=true \
  right_ir_width:=640 right_ir_height:=400 right_ir_fps:=30 enable_right_ir:=true \
  enable_color:=true color_width:=640 color_height:=480 color_fps:=30 \
  enable_accel:=true enable_gyro:=true enable_sync_output_accel_gyro:=true \
  enable_point_cloud:=true point_cloud_decimation_filter_factor:=2 > logs/live/camera.log 2>&1 &
pids+=("$!")
/usr/bin/python3 scripts/configure_camera.py --output "$VISUAL_PERCEPTION_ROOT/runtime/336l" --timeout 120 --recover-camera --localization-mode "$LOCALIZATION_MODE"
if [[ "$LOCALIZATION_MODE" == rgbd ]]; then
  /usr/bin/python3 scripts/rgbd_odometry.py --calibration "$VISUAL_PERCEPTION_ROOT/runtime/336l" > logs/live/vins.log 2>&1 &
  ODOM_TOPIC=/detection/odometry
elif [[ "$LOCALIZATION_MODE" == stationary ]]; then
  /usr/bin/python3 scripts/fixed_camera_odom.py > logs/live/vins.log 2>&1 &
  ODOM_TOPIC=/detection/odometry
elif [[ "$LOCALIZATION_MODE" == openvins ]]; then
  ros2 run ov_msckf run_subscribe_msckf "$VISUAL_PERCEPTION_ROOT/runtime/336l/openvins/estimator_config.yaml" --ros-args -r __ns:=/openvins -p publish_global_to_imu_tf:=false -p publish_calibration_tf:=false > logs/live/openvins.log 2>&1 &
  pids+=("$!")
  /usr/bin/python3 scripts/body_odometry.py --calibration "$VISUAL_PERCEPTION_ROOT/runtime/336l/calibration.json" --input-topic /openvins/odomimu --input-body-frame imu > logs/live/openvins_body.log 2>&1 &
  ODOM_TOPIC=/detection/odometry
else
  ros2 run vins vins_node "$VISUAL_PERCEPTION_ROOT/runtime/336l/vins.yaml" --ros-args -r __ns:=/vins > logs/live/vins.log 2>&1 &
  pids+=("$!")
  /usr/bin/python3 scripts/body_odometry.py --calibration "$VISUAL_PERCEPTION_ROOT/runtime/336l/calibration.json" > logs/live/vins_body.log 2>&1 &
  ODOM_TOPIC=/detection/odometry
fi
pids+=("$!")
if [[ "$YOLO_ASSIST" == 1 ]]; then
  /usr/bin/python3 scripts/yolo_semantics.py --calibration "$VISUAL_PERCEPTION_ROOT/runtime/336l/calibration.json" > logs/live/yolo.log 2>&1 &
  pids+=("$!")
fi
run_detector() {
  trap 'exit 143' TERM
  trap 'exit 130' INT
  local attempt status
  for attempt in 1 2 3; do
    if "${DETECTOR_BINARY:-$ROBOT_CORE/install/ws_perception/onboard_detector/lib/onboard_detector/detector_node}" --ros-args --params-file "$VISUAL_PERCEPTION_ROOT/runtime/336l/detector.yaml"; then
      return 0
    else
      status=$?
      echo "[detector] Exit $status on attempt $attempt/3; retrying in 2 seconds" >&2
    fi
    sleep 2
  done
  return "$status"
}
run_detector > logs/live/detector.log 2>&1 &
pids+=("$!")
/usr/bin/python3 scripts/odometry_tf.py --odom-topic "$ODOM_TOPIC" > logs/live/tf.log 2>&1 &
pids+=("$!")
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:="${FOXGLOVE_PORT:-9876}" -p address:=0.0.0.0 > logs/live/foxglove.log 2>&1 &
pids+=("$!")
echo "336L serial=$CAMERA_SERIAL + localization=$LOCALIZATION_MODE + dynamic detector started; ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "Foxglove: ws://localhost:${FOXGLOVE_PORT:-9876}  (RViz2 is not started)"
while true; do
  for pid in "${pids[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then echo "A process exited; inspect logs/live" >&2; exit 1; fi
  done
  sleep 2
done
