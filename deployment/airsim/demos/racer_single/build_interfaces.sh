#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
cd /interfaces
colcon --log-base /interfaces/log build --base-paths /repo/deployment/airsim/demos/racer_single/workstation_interfaces \
  --build-base /interfaces/build --install-base /interfaces/install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source /interfaces/install/local_setup.bash
python3 -c 'from as2_msgs.action import Land, Takeoff; from px4_msgs.msg import VehicleStatus; import rclpy; rclpy.init(); n=rclpy.create_node("racer_interface_load"); n.create_publisher(VehicleStatus,"/interface_check",1); print(VehicleStatus.MESSAGE_VERSION); n.destroy_node(); rclpy.shutdown()'
