#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

export PATH=/usr/bin:/bin:/usr/sbin:/sbin:/opt/ros/humble/bin
export PYTHONNOUSERSITE=1
export COLCON_HOME="${COLCON_HOME:-/tmp/colcon_home}"
export COLCON_EXTENSION_BLOCKLIST="${COLCON_EXTENSION_BLOCKLIST:-colcon_core.shell.ros_domain_id}"
mkdir -p "${COLCON_HOME}"

/usr/bin/colcon build --base-paths src --parallel-workers 1 --packages-up-to exploration_manager \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3
