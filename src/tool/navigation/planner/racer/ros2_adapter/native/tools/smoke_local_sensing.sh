#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"

map_pid=""
sensor_pid=""
cleanup() {
  if [[ -n "${sensor_pid}" ]] && kill -0 "${sensor_pid}" 2>/dev/null; then
    kill "${sensor_pid}" 2>/dev/null || true
    wait "${sensor_pid}" 2>/dev/null || true
  fi
  if [[ -n "${map_pid}" ]] && kill -0 "${map_pid}" 2>/dev/null; then
    kill "${map_pid}" 2>/dev/null || true
    wait "${map_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

ros2 run map_generator map_pub "$(pwd)/src/map_generator/resource/pillar.pcd" >/tmp/racer_map_pub.log 2>&1 &
map_pid=$!

ros2 run local_sensing_node pcl_render_node --ros-args \
  -p sensing_horizon:=5.0 \
  -p sensing_rate:=10.0 \
  -p estimation_rate:=10.0 \
  -p map.x_size:=35.0 \
  -p map.y_size:=35.0 \
  -p map.z_size:=3.5 \
  -p map.init_x:=0.0 \
  -p map.init_y:=0.0 \
  -p map.drone_id:=1 \
  -r __ns:=/ \
  >/tmp/racer_pcl_render.log 2>&1 &
sensor_pid=$!

sleep 4
ros2 topic list --no-daemon | rg '/pcl_render_node/depth|/pcl_render_node/sensor_pose'
