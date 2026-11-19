cmake_minimum_required(VERSION 3.8)
project(bspline)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bspline.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(bspline_lib
  src/non_uniform_bspline.cpp
)
set_target_properties(bspline_lib PROPERTIES OUTPUT_NAME bspline)
target_include_directories(bspline_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(bspline_lib Eigen3 rclcpp)
target_link_libraries(bspline_lib "${cpp_typesupport_target}")

install(TARGETS bspline_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(bspline_lib)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Eigen3 rclcpp rosidl_default_runtime)
ament_package()
