#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
cd /deps
sha256sum -c nlopt-debs.sha256
dpkg -i nlopt-debs/*.deb
export CMAKE_BUILD_PARALLEL_LEVEL=2
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=173
mkdir -p /tmp/racer-source /racer-run
cp -a /repo/src/tool/navigation/planner/racer/ros2_adapter/native/. /tmp/racer-source/
cd /racer-run
dpkg-query -W > packages.tsv
colcon --log-base /racer-run/colcon-log build --base-paths /tmp/racer-source/src \
  --build-base /racer-run/build --install-base /racer-run/install --executor sequential \
  --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
source /racer-run/install/setup.bash
ros2 interface show exploration_manager/msg/TrajectoryExecution
sha256sum install/exploration_manager/lib/exploration_manager/exploration_node \
  install/lkh_mtsp_solver/lib/lkh_mtsp_solver/mtsp_node install/lkh_mtsp_solver/bin/lkh3_solver > executable.sha256
