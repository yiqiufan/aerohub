cmake_minimum_required(VERSION 3.8)
project(path_searching)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
if(DEFINED ENV{CONDA_PREFIX})
  list(APPEND CMAKE_IGNORE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
endif()
set(Boost_NO_BOOST_CMAKE ON)
set(BOOST_ROOT "/usr")
set(BOOST_INCLUDEDIR "/usr/include")
set(BOOST_LIBRARYDIR "/usr/lib/x86_64-linux-gnu")
find_package(PCL REQUIRED)
find_package(plan_env REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(path_searching
  src/kinodynamic_astar.cpp
  src/astar2.cpp
  src/topo_prm.cpp
)
target_include_directories(path_searching PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(path_searching plan_env rclcpp std_msgs visualization_msgs)
target_link_libraries(path_searching ${PCL_LIBRARIES})

install(TARGETS path_searching
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(path_searching)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(plan_env rclcpp std_msgs visualization_msgs)
ament_package()
