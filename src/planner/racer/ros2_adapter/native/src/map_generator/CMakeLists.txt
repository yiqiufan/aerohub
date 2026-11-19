cmake_minimum_required(VERSION 3.8)
project(map_generator)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
if(DEFINED ENV{CONDA_PREFIX})
  list(APPEND CMAKE_IGNORE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
endif()
set(Boost_NO_BOOST_CMAKE ON)
set(BOOST_ROOT "/usr")
set(BOOST_INCLUDEDIR "/usr/include")
set(BOOST_LIBRARYDIR "/usr/lib/x86_64-linux-gnu")
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(MAP_GENERATOR_DEPS
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
)

add_executable(map_recorder src/map_recorder.cpp)
ament_target_dependencies(map_recorder ${MAP_GENERATOR_DEPS})
target_link_libraries(map_recorder ${PCL_LIBRARIES})

add_executable(map_pub src/map_publisher.cpp)
ament_target_dependencies(map_pub ${MAP_GENERATOR_DEPS})
target_link_libraries(map_pub ${PCL_LIBRARIES})

add_executable(pilar_map src/random_pilar_map.cpp)
ament_target_dependencies(pilar_map ${MAP_GENERATOR_DEPS})
target_link_libraries(pilar_map ${PCL_LIBRARIES})

add_executable(click_map src/click_map.cpp)
ament_target_dependencies(click_map ${MAP_GENERATOR_DEPS})
target_link_libraries(click_map ${PCL_LIBRARIES})

install(TARGETS
  map_recorder
  map_pub
  pilar_map
  click_map
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY resource/ DESTINATION share/${PROJECT_NAME}/resource)

ament_package()
