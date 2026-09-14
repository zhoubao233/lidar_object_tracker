#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
WS="${TRACKER_WS:-$(dirname -- "$ROOT")/lidar_object_tracker_ws}"
source /opt/ros/noetic/setup.bash
if [[ -n "${LIVOX_DRIVER_SETUP:-}" ]]; then
  source "$LIVOX_DRIVER_SETUP" --extend
else
  for setup in "$(dirname "$ROOT")/livox_ros_driver2_ws/devel/setup.bash" "$HOME/auto/kufei_auto/uav_indoor/devel/setup.bash"; do
    [[ -f "$setup" ]] || continue
    source "$setup" --extend
    if rospack find livox_ros_driver2 >/dev/null 2>&1; then break; fi
  done
fi
rospack find livox_ros_driver2 >/dev/null || { echo 'Set LIVOX_DRIVER_SETUP to a built driver workspace' >&2; exit 1; }
mkdir -p "$WS/src"
if [[ -e "$WS/src/lidar_object_tracker" || -L "$WS/src/lidar_object_tracker" ]]; then
  [[ "$(realpath "$WS/src/lidar_object_tracker")" == "$ROOT" ]] || { echo 'Workspace contains another lidar_object_tracker' >&2; exit 1; }
else
  ln -s "$ROOT" "$WS/src/lidar_object_tracker"
fi
catkin_make -C "$WS" -j2 -l2 "$@"
