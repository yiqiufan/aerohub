cmake_minimum_required(VERSION 3.8)
project(poly_traj)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(poly_traj
  src/polynomial_traj.cpp
)
target_include_directories(poly_traj PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(poly_traj rclcpp std_msgs)

install(TARGETS poly_traj
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(poly_traj)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Eigen3 rclcpp std_msgs)
ament_package()
