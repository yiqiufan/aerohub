cmake_minimum_required(VERSION 3.8)
project(lkh_tsp_solver)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SolveTSP.srv"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

file(GLOB LKH_TSP_SRCS CONFIGURE_DEPENDS src/*.c)

add_library(lkh_tsp_solver_lib
  ${LKH_TSP_SRCS}
  src/lkh_interface.cpp
)
set_target_properties(lkh_tsp_solver_lib PROPERTIES OUTPUT_NAME lkh_tsp_solver)
target_include_directories(lkh_tsp_solver_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_compile_options(lkh_tsp_solver_lib PRIVATE -fcommon)
target_link_libraries(lkh_tsp_solver_lib m)

install(TARGETS lkh_tsp_solver_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY resource/ DESTINATION share/${PROJECT_NAME}/resource OPTIONAL)

ament_export_include_directories(include)
ament_export_libraries(lkh_tsp_solver_lib)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp rosidl_default_runtime)
ament_package()
