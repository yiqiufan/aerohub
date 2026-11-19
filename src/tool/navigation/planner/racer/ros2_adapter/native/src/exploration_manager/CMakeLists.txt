cmake_minimum_required(VERSION 3.8)
project(exploration_manager)

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
find_package(lkh_mtsp_solver REQUIRED)
find_package(lkh_tsp_solver REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(path_searching REQUIRED)
find_package(plan_env REQUIRED)
find_package(plan_manage REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
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

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DroneState.msg"
  "msg/PairOpt.msg"
  "msg/PairOptResponse.msg"
  "msg/HGrid.msg"
  "msg/GridTour.msg"
  "msg/TrajectoryExecution.msg"
  DEPENDENCIES geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(exploration_manager_dependencies
  active_perception
  bspline
  bspline_opt
  geometry_msgs
  lkh_mtsp_solver
  lkh_tsp_solver
  nav_msgs
  path_searching
  plan_env
  plan_manage
  pcl_conversions
  quadrotor_msgs
  rclcpp
  sensor_msgs
  std_msgs
  traj_utils
  visualization_msgs
)

add_library(exploration_manager_core STATIC
  src/fast_exploration_manager.cpp
)
target_include_directories(exploration_manager_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(exploration_manager_core ${exploration_manager_dependencies})

add_executable(exploration_node
  src/exploration_node.cpp
  src/fast_exploration_fsm.cpp
)
target_include_directories(exploration_node PRIVATE include ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(exploration_node ${exploration_manager_dependencies})
target_link_libraries(exploration_node exploration_manager_core "${cpp_typesupport_target}")

add_executable(ground_node
  src/ground_node.cpp
)
target_include_directories(ground_node PRIVATE include ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(ground_node ${exploration_manager_dependencies})
target_link_libraries(ground_node exploration_manager_core "${cpp_typesupport_target}")

install(TARGETS exploration_manager_core exploration_node ground_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${exploration_manager_dependencies})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
