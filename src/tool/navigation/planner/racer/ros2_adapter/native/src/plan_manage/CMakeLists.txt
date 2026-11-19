cmake_minimum_required(VERSION 3.8)
project(plan_manage)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(active_perception REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(path_searching REQUIRED)
find_package(plan_env REQUIRED)
find_package(poly_traj REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(traj_utils REQUIRED)
find_package(visualization_msgs REQUIRED)
if(DEFINED ENV{CONDA_PREFIX})
  list(APPEND CMAKE_IGNORE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
endif()
set(Boost_NO_BOOST_CMAKE ON)
set(BOOST_ROOT "/usr")
set(BOOST_INCLUDEDIR "/usr/include")
set(BOOST_LIBRARYDIR "/usr/lib/x86_64-linux-gnu")
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(plan_manage_dependencies
  active_perception
  bspline
  bspline_opt
  geometry_msgs
  nav_msgs
  path_searching
  plan_env
  pcl_conversions
  poly_traj
  quadrotor_msgs
  rclcpp
  sensor_msgs
  std_msgs
  traj_utils
  visualization_msgs
)

add_library(plan_manage
  src/planner_manager.cpp
  src/planner_manager_dev.cpp
)
target_include_directories(plan_manage PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(plan_manage ${plan_manage_dependencies})

add_executable(fast_planner_node
  src/fast_planner_node.cpp
  src/kino_replan_fsm.cpp
  src/topo_replan_fsm.cpp
  test/local_explore_fsm.cpp
)
target_include_directories(fast_planner_node PRIVATE include ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(fast_planner_node ${plan_manage_dependencies})
target_link_libraries(fast_planner_node plan_manage)

add_executable(traj_server src/traj_server.cpp)
target_include_directories(traj_server PRIVATE include ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(traj_server
  active_perception
  bspline
  geometry_msgs
  nav_msgs
  poly_traj
  quadrotor_msgs
  rclcpp
  std_msgs
  traj_utils
  visualization_msgs
)

install(TARGETS plan_manage
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS fast_planner_node traj_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(plan_manage)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${plan_manage_dependencies})
ament_package()
