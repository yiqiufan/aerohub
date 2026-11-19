cmake_minimum_required(VERSION 3.8)
project(local_sensing_node)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
if(DEFINED ENV{CONDA_PREFIX})
  list(APPEND CMAKE_IGNORE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
endif()
set(Boost_NO_BOOST_CMAKE ON)
set(BOOST_ROOT "/usr")
set(BOOST_INCLUDEDIR "/usr/include")
set(BOOST_LIBRARYDIR "/usr/lib/x86_64-linux-gnu")
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_render_node src/pointcloud_render_node.cpp)
target_include_directories(pcl_render_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(pcl_render_node
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2_ros
)
target_link_libraries(pcl_render_node ${PCL_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS pcl_render_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY params/ DESTINATION share/${PROJECT_NAME}/params)

ament_package()
