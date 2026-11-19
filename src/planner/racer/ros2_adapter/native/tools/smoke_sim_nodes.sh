#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"

map_pid=""
cleanup() {
  if [[ -n "${map_pid}" ]] && kill -0 "${map_pid}" 2>/dev/null; then
    kill "${map_pid}" 2>/dev/null || true
    wait "${map_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

timeout 8 ros2 run map_generator map_pub "$(pwd)/src/map_generator/resource/pillar.pcd" >/tmp/racer_map_pub.log 2>&1 &
map_pid=$!

sleep 2
ros2 topic list --no-daemon | rg '/map_generator/global_cloud'
