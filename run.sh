#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo './run.sh [input_source:=livox_mavros|livox_mavros_real] [rviz:=true] [launch arguments]'
  echo 'Real mode requires lidar_translation and lidar_quaternion. Start Livox and MAVROS separately.'
  exit 0
fi
source /opt/ros/noetic/setup.bash
MODE=livox_mavros
ARGS=()
for arg in "$@"; do
  case "$arg" in input_source:=*) MODE="${arg#input_source:=}" ;; *) ARGS+=("$arg") ;; esac
done
case "$MODE" in livox_mavros) LAUNCH=simulation.launch ;; livox_mavros_real) LAUNCH=real.launch ;; *) echo "Invalid input_source: $MODE" >&2; exit 1 ;; esac
if [[ -n "${LIVOX_DRIVER_SETUP:-}" ]]; then
  source "$LIVOX_DRIVER_SETUP" --extend
else
  for setup in "$(dirname "$ROOT")/livox_ros_driver2_ws/devel/setup.bash" "$HOME/auto/kufei_auto/uav_indoor/devel/setup.bash"; do
    [[ -f "$setup" ]] || continue
    source "$setup" --extend
    if rospack find livox_ros_driver2 >/dev/null 2>&1; then break; fi
  done
fi
WS="${TRACKER_WS:-$(dirname "$ROOT")/lidar_object_tracker_ws}"
[[ -f "$WS/devel/setup.bash" ]] || { echo "Run $ROOT/tools/build.sh first" >&2; exit 1; }
source "$WS/devel/setup.bash" --extend
[[ "$(realpath "$(rospack find lidar_object_tracker)")" == "$ROOT" ]] || { echo 'ROS package resolves to another checkout' >&2; exit 1; }
[[ -x "$WS/devel/lib/lidar_object_tracker/object_tracker" ]] || { echo "Run tools/build.sh first" >&2; exit 1; }
exec roslaunch lidar_object_tracker "$LAUNCH" "${ARGS[@]}"
