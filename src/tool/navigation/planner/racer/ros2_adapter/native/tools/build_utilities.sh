#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
colcon build --base-paths src --parallel-workers 1 --packages-select pose_utils poly_traj --cmake-args -DCMAKE_BUILD_TYPE=Release
