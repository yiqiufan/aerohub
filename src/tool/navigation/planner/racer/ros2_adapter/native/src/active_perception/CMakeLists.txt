cmake_minimum_required(VERSION 3.8)
project(active_perception)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(bspline REQUIRED)
find_package(Eigen3 REQUIRED)
if(DEFINED ENV{CONDA_PREFIX})
  list(APPEND CMAKE_IGNORE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
endif()
set(Boost_NO_BOOST_CMAKE ON)
set(BOOST_ROOT "/usr")
set(BOOST_INCLUDEDIR "/usr/include")
set(BOOST_LIBRARYDIR "/usr/lib/x86_64-linux-gnu")
find_package(PCL REQUIRED)
find_package(path_searching REQUIRED)
find_package(plan_env REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(active_perception
  src/traj_visibility.cpp
  src/heading_planner.cpp
  src/frontier_finder.cpp
  src/graph_node.cpp
  src/perception_utils.cpp
  src/uniform_grid.cpp
  src/hgrid.cpp
)
target_include_directories(active_perception PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(active_perception
  bspline path_searching plan_env rclcpp sensor_msgs std_msgs visualization_msgs
)
target_link_libraries(active_perception ${PCL_LIBRARIES})

install(TARGETS active_perception
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(active_perception)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(bspline path_searching plan_env rclcpp sensor_msgs std_msgs visualization_msgs)
ament_package()
