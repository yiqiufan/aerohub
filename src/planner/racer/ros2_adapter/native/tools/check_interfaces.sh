#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
colcon build --base-paths src --parallel-workers 1 --packages-select quadrotor_msgs bspline plan_env lkh_tsp_solver lkh_mtsp_solver exploration_manager --cmake-args -DCMAKE_BUILD_TYPE=Release
set +u
source install/setup.bash
set -u
ros2 interface show quadrotor_msgs/msg/PositionCommand >/tmp/position_command.interface
ros2 interface show bspline/msg/Bspline >/tmp/bspline.interface
ros2 interface show plan_env/msg/ChunkData >/tmp/chunk_data.interface
ros2 interface show exploration_manager/msg/DroneState >/tmp/drone_state.interface
ros2 interface show lkh_mtsp_solver/srv/SolveMTSP >/tmp/solve_mtsp.interface
