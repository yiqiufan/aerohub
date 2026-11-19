cmake_minimum_required(VERSION 3.8)
project(poscmd_2_odom)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(poscmd_2_odom src/poscmd_2_odom.cpp)
target_include_directories(poscmd_2_odom PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(poscmd_2_odom nav_msgs quadrotor_msgs rclcpp tf2_geometry_msgs)

install(TARGETS poscmd_2_odom DESTINATION lib/${PROJECT_NAME})

ament_package()
