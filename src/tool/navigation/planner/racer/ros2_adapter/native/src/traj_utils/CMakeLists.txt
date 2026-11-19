cmake_minimum_required(VERSION 3.8)
project(traj_utils)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
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
find_package(pcl_conversions REQUIRED)
find_package(poly_traj REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(traj_utils
  src/planning_visualization.cpp
)
target_include_directories(traj_utils PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(traj_utils
  bspline bspline_opt path_searching poly_traj rclcpp sensor_msgs std_msgs visualization_msgs
  pcl_conversions
)
target_link_libraries(traj_utils ${PCL_LIBRARIES})

add_executable(process_msg src/process_msg.cpp)
ament_target_dependencies(process_msg pcl_conversions rclcpp sensor_msgs)
target_link_libraries(process_msg ${PCL_LIBRARIES})

install(TARGETS traj_utils
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS process_msg
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(traj_utils)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  bspline bspline_opt path_searching pcl_conversions poly_traj rclcpp sensor_msgs std_msgs
  visualization_msgs
)
ament_package()
