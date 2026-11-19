cmake_minimum_required(VERSION 3.8)
project(odom_visualization)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(Armadillo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pose_utils REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(odom_visualization src/odom_visualization.cpp)
target_include_directories(odom_visualization PRIVATE
  ${ARMADILLO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(odom_visualization
  geometry_msgs
  nav_msgs
  pose_utils
  quadrotor_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)
target_link_libraries(odom_visualization ${ARMADILLO_LIBRARIES})

install(TARGETS odom_visualization DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes)

ament_package()
