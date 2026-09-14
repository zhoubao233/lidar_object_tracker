#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
WS="${TRACKER_WS:-$(dirname "$ROOT")/lidar_object_tracker_ws}"
"$ROOT/tools/build.sh" run_tests_lidar_object_tracker
source /opt/ros/noetic/setup.bash
catkin_test_results "$WS/build/test_results/lidar_object_tracker"
