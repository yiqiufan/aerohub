# RACER ROS 2 Humble Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a ROS 2 Humble-native port of the original RACER lightweight simulation and exploration demo, preserving upstream algorithm behavior and producing five pillar benchmark runs.

**Architecture:** Keep `upstream_RACER/` as the reference checkout and create ROS2 packages under `src/`. Port interfaces and low-level utilities first, then LKH solver services, simulation/sensing nodes, planning libraries, exploration manager, launch/RViz, and benchmark automation. Every checkpoint must build and run focused smoke tests before moving to the next package group.

**Tech Stack:** ROS 2 Humble, `ament_cmake`, `rclcpp`, `rosidl_generate_interfaces`, C++14/C++17, Eigen3, PCL, OpenCV, NLopt 2.7.1, bundled RACER LKH3 code, Python launch files, RViz2.

---

## File Structure

Create and modify these top-level paths:

- `src/`: ROS2 packages copied and ported from upstream.
- `src/quadrotor_msgs/`: ROS2 message-only package for quadrotor commands and telemetry.
- `src/bspline/`: B-spline library plus `Bspline.msg`.
- `src/plan_env/`: SDF/occupancy map, map ROS wrapper, and swarm map messages.
- `src/lkh_tsp_solver/`: Original LKH TSP library and ROS2 service interface.
- `src/lkh_mtsp_solver/`: Original LKH3 MTSP/ACVRP library and ROS2 service node.
- `src/map_generator/`: PCD map publisher and map creation tools.
- `src/local_sensing/`: Original CPU pointcloud renderer used by the lightweight demo.
- `src/poscmd_2_odom/`: Original lightweight position-command-to-odometry simulator.
- `src/odom_visualization/`: Marker/path visualization for drone odometry.
- `src/path_searching/`, `src/active_perception/`, `src/bspline_opt/`, `src/traj_utils/`, `src/plan_manage/`, `src/exploration_manager/`: RACER planning and exploration packages.
- `launch/racer_pillar.launch.py`: Top-level ROS2 pillar simulation launch.
- `rviz/swarm.rviz`: Ported RViz2 config for the pillar demo.
- `tools/benchmark_pillar.py`: Five-run clean-state benchmark driver and metric collector.
- `benchmark_results/`: Generated CSV/log output ignored by git.

Keep these files untouched:

- `upstream_RACER/**`
- `docs/superpowers/specs/2026-05-22-racer-ros2-humble-port-design.md`

## Task 1: Workspace Baseline And Source Copy

**Files:**
- Modify: `.gitignore`
- Create: `src/`
- Create: `benchmark_results/`
- Copy from upstream into `src/`

- [ ] **Step 1: Extend ignored generated paths**

Add these lines to `.gitignore`:

```gitignore
benchmark_results/
*.bag/
*.db3
*.mcap
```

- [ ] **Step 2: Copy only simulation/exploration source packages**

Run:

```bash
mkdir -p src benchmark_results
cp -a upstream_RACER/uav_simulator/Utils/quadrotor_msgs src/quadrotor_msgs
cp -a upstream_RACER/swarm_exploration/bspline src/bspline
cp -a upstream_RACER/swarm_exploration/plan_env src/plan_env
cp -a upstream_RACER/swarm_exploration/path_searching src/path_searching
cp -a upstream_RACER/swarm_exploration/active_perception src/active_perception
cp -a upstream_RACER/swarm_exploration/bspline_opt src/bspline_opt
cp -a upstream_RACER/swarm_exploration/poly_traj src/poly_traj
cp -a upstream_RACER/swarm_exploration/traj_utils src/traj_utils
cp -a upstream_RACER/swarm_exploration/plan_manage src/plan_manage
cp -a upstream_RACER/swarm_exploration/exploration_manager src/exploration_manager
cp -a upstream_RACER/swarm_exploration/utils/lkh_tsp_solver src/lkh_tsp_solver
cp -a upstream_RACER/swarm_exploration/utils/lkh_mtsp_solver src/lkh_mtsp_solver
cp -a upstream_RACER/uav_simulator/map_generator src/map_generator
cp -a upstream_RACER/uav_simulator/local_sensing src/local_sensing
cp -a upstream_RACER/uav_simulator/poscmd_2_odom src/poscmd_2_odom
cp -a upstream_RACER/uav_simulator/Utils/odom_visualization src/odom_visualization
cp -a upstream_RACER/uav_simulator/Utils/pose_utils src/pose_utils
```

Expected: `find src -maxdepth 1 -mindepth 1 -type d | sort` lists the copied packages.

- [ ] **Step 3: Remove copied ROS1 build products**

Run:

```bash
find src -type d \( -name build -o -name devel -o -name .git \) -prune -exec rm -rf {} +
find src -type f \( -name '*.pyc' -o -name '*~' \) -delete
```

Expected: `find src -type d -name build -o -name devel` prints nothing.

- [ ] **Step 4: Commit baseline copy**

Run:

```bash
git add .gitignore src
git commit -m "chore: copy RACER simulation sources"
```

Expected: commit succeeds with copied source packages staged.

## Task 2: Port Interface Packages

**Files:**
- Modify: `src/quadrotor_msgs/package.xml`
- Replace: `src/quadrotor_msgs/CMakeLists.txt`
- Modify: `src/quadrotor_msgs/msg/*.msg`
- Modify: `src/bspline/package.xml`
- Replace: `src/bspline/CMakeLists.txt`
- Modify: `src/bspline/msg/Bspline.msg`
- Modify: `src/plan_env/package.xml`
- Replace: `src/plan_env/CMakeLists.txt`
- Modify: `src/plan_env/msg/*.msg`
- Modify: `src/exploration_manager/package.xml`
- Modify: `src/exploration_manager/CMakeLists.txt`
- Modify: `src/exploration_manager/msg/*.msg`
- Modify: `src/lkh_tsp_solver/package.xml`
- Modify: `src/lkh_tsp_solver/CMakeLists.txt`
- Modify: `src/lkh_mtsp_solver/package.xml`
- Modify: `src/lkh_mtsp_solver/CMakeLists.txt`

- [ ] **Step 1: Write interface syntax smoke test script**

Create `tools/check_interfaces.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
colcon build --packages-select quadrotor_msgs bspline plan_env lkh_tsp_solver lkh_mtsp_solver exploration_manager --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 interface show quadrotor_msgs/msg/PositionCommand >/tmp/position_command.interface
ros2 interface show bspline/msg/Bspline >/tmp/bspline.interface
ros2 interface show plan_env/msg/ChunkData >/tmp/chunk_data.interface
ros2 interface show exploration_manager/msg/DroneState >/tmp/drone_state.interface
ros2 interface show lkh_mtsp_solver/srv/SolveMTSP >/tmp/solve_mtsp.interface
```

Run:

```bash
chmod +x tools/check_interfaces.sh
tools/check_interfaces.sh
```

Expected before porting: FAIL during `colcon build` because packages still use `catkin`.

- [ ] **Step 2: Convert `quadrotor_msgs` message field names**

Replace every bare `Header header` in `src/quadrotor_msgs/msg/*.msg` with:

```text
std_msgs/Header header
```

Keep these package-qualified references unchanged:

```text
quadrotor_msgs/AuxCommand aux
nav_msgs/Odometry curodom
nav_msgs/Odometry kfodom
```

- [ ] **Step 3: Replace `src/quadrotor_msgs/package.xml`**

Use this package manifest:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>quadrotor_msgs</name>
  <version>0.0.0</version>
  <description>Message definitions used by the RACER lightweight simulator and trajectory server.</description>
  <maintainer email="codex@example.com">Codex</maintainer>
  <license>BSD-3-Clause</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <depend>builtin_interfaces</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>std_msgs</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 4: Replace `src/quadrotor_msgs/CMakeLists.txt`**

Use this CMake file:

```cmake
cmake_minimum_required(VERSION 3.8)
project(quadrotor_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AuxCommand.msg"
  "msg/Corrections.msg"
  "msg/Gains.msg"
  "msg/LQRTrajectory.msg"
  "msg/Odometry.msg"
  "msg/OutputData.msg"
  "msg/PolynomialTrajectory.msg"
  "msg/PositionCommand.msg"
  "msg/PPROutputData.msg"
  "msg/Serial.msg"
  "msg/SO3Command.msg"
  "msg/StatusData.msg"
  "msg/TRPYCommand.msg"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

- [ ] **Step 5: Convert `bspline` message dependency**

Confirm `src/bspline/msg/Bspline.msg` uses:

```text
builtin_interfaces/Time start_time
```

instead of:

```text
time start_time
```

- [ ] **Step 6: Replace `src/bspline/package.xml` for interface and library use**

Use:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>bspline</name>
  <version>0.0.0</version>
  <description>B-spline trajectory utilities and message interface for RACER.</description>
  <maintainer email="codex@example.com">Codex</maintainer>
  <license>BSD-3-Clause</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <depend>builtin_interfaces</depend>
  <depend>geometry_msgs</depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 7: Replace `src/bspline/CMakeLists.txt`**

Start with this CMake file; keep source filenames that exist in `src/bspline/src`:

```cmake
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

add_library(${PROJECT_NAME}
  src/non_uniform_bspline.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(${PROJECT_NAME} Eigen3 rclcpp)
target_link_libraries(${PROJECT_NAME} "${cpp_typesupport_target}")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Eigen3 rclcpp rosidl_default_runtime)
ament_package()
```

- [ ] **Step 8: Convert `plan_env` messages and manifest**

Replace `src/plan_env/package.xml` with:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>plan_env</name>
  <version>0.0.0</version>
  <description>Mapping environment and map-sharing interfaces for RACER.</description>
  <maintainer email="codex@example.com">Codex</maintainer>
  <license>BSD-3-Clause</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <depend>cv_bridge</depend>
  <depend>geometry_msgs</depend>
  <depend>message_filters</depend>
  <depend>nav_msgs</depend>
  <depend>pcl_conversions</depend>
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>visualization_msgs</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 9: Replace `src/plan_env/CMakeLists.txt` with an interface-first build**

Use this first version, then add library targets in Task 7:

```cmake
cmake_minimum_required(VERSION 3.8)
project(plan_env)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IdxList.msg"
  "msg/ChunkStamps.msg"
  "msg/ChunkData.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

- [ ] **Step 10: Convert exploration manager messages and manifest**

Replace message files that use `geometry_msgs/Point` with the same text; ROS2 accepts the package-qualified type. Replace `src/exploration_manager/package.xml` with a ROS2 manifest that includes:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>exploration_manager</name>
  <version>0.0.0</version>
  <description>RACER exploration manager, coordination messages, and exploration nodes.</description>
  <maintainer email="codex@example.com">Codex</maintainer>
  <license>BSD-3-Clause</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <depend>active_perception</depend>
  <depend>bspline</depend>
  <depend>bspline_opt</depend>
  <depend>geometry_msgs</depend>
  <depend>lkh_mtsp_solver</depend>
  <depend>lkh_tsp_solver</depend>
  <depend>path_searching</depend>
  <depend>plan_env</depend>
  <depend>plan_manage</depend>
  <depend>quadrotor_msgs</depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>traj_utils</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 11: Make exploration manager CMake generate messages only**

Use this first CMake version, then add executables in Task 10:

```cmake
cmake_minimum_required(VERSION 3.8)
project(exploration_manager)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DroneState.msg"
  "msg/PairOpt.msg"
  "msg/PairOptResponse.msg"
  "msg/HGrid.msg"
  "msg/GridTour.msg"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

- [ ] **Step 12: Convert LKH service package manifests and CMake service generation**

For `src/lkh_tsp_solver/package.xml`, use:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>lkh_tsp_solver</name>
  <version>0.0.0</version>
  <description>RACER bundled LKH TSP solver service.</description>
  <maintainer email="codex@example.com">Codex</maintainer>
  <license>BSD-3-Clause</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

For `src/lkh_mtsp_solver/package.xml`, use the same structure with `<name>lkh_mtsp_solver</name>` and description `RACER bundled LKH3 MTSP and ACVRP solver service.`

Use this interface-first CMake for `src/lkh_tsp_solver/CMakeLists.txt`:

```cmake
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

Use this interface-first CMake for `src/lkh_mtsp_solver/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(lkh_mtsp_solver)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SolveMTSP.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

- [ ] **Step 13: Run interface test and commit**

Run:

```bash
tools/check_interfaces.sh
```

Expected: PASS and all five `ros2 interface show` commands produce output.

Commit:

```bash
git add tools/check_interfaces.sh src/quadrotor_msgs src/bspline src/plan_env src/exploration_manager src/lkh_tsp_solver src/lkh_mtsp_solver
git commit -m "feat: port RACER ROS2 interfaces"
```

## Task 3: Port LKH Solver Libraries And Services

**Files:**
- Modify: `src/lkh_tsp_solver/CMakeLists.txt`
- Modify: `src/lkh_mtsp_solver/CMakeLists.txt`
- Modify: `src/lkh_mtsp_solver/src2/mtsp_node.cpp`

- [ ] **Step 1: Add LKH library build to `lkh_tsp_solver`**

Replace `src/lkh_tsp_solver/CMakeLists.txt` with:

```cmake
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

add_library(${PROJECT_NAME}
  ${LKH_TSP_SRCS}
  src/lkh_interface.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME} m)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY resource/ DESTINATION share/${PROJECT_NAME}/resource OPTIONAL)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp rosidl_default_runtime)
ament_package()
```

- [ ] **Step 2: Add LKH3 library and service node to `lkh_mtsp_solver`**

Replace `src/lkh_mtsp_solver/CMakeLists.txt` with:

```cmake
cmake_minimum_required(VERSION 3.8)
project(lkh_mtsp_solver)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SolveMTSP.srv"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

file(GLOB LKH_MTSP_SRCS CONFIGURE_DEPENDS src/*.c)

add_library(${PROJECT_NAME}
  ${LKH_MTSP_SRCS}
  src2/lkh3_interface.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME} m)

add_executable(mtsp_node
  src2/mtsp_node.cpp
)
ament_target_dependencies(mtsp_node rclcpp)
target_link_libraries(mtsp_node ${PROJECT_NAME} "${cpp_typesupport_target}" m)

install(TARGETS ${PROJECT_NAME} mtsp_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY resource/ DESTINATION share/${PROJECT_NAME}/resource)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp rosidl_default_runtime)
ament_package()
```

- [ ] **Step 3: Port `mtsp_node.cpp` service API**

Replace `src/lkh_mtsp_solver/src2/mtsp_node.cpp` with:

```cpp
#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <lkh_mtsp_solver/lkh3_interface.h>
#include <lkh_mtsp_solver/srv/solve_mtsp.hpp>

using SolveMTSP = lkh_mtsp_solver::srv::SolveMTSP;

namespace {
std::string mtsp_dir1;
std::string mtsp_dir2;
std::string mtsp_dir3;
int drone_id = 1;

void solve_callback(
    const std::shared_ptr<SolveMTSP::Request> request,
    std::shared_ptr<SolveMTSP::Response> response) {
  if (request->prob == 1) {
    solveMTSPWithLKH3(mtsp_dir1.c_str());
  } else if (request->prob == 2) {
    solveMTSPWithLKH3(mtsp_dir2.c_str());
  } else if (request->prob == 3) {
    const std::string cmd = "/usr/local/bin/LKH " + mtsp_dir3;
    std::system(cmd.c_str());
  }
  response->empty = 0;
}
}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mtsp_node");

  node->declare_parameter<std::string>("exploration.mtsp_dir", "null");
  node->declare_parameter<int>("exploration.drone_id", 1);
  node->declare_parameter<int>("exploration.problem_id", 1);

  const auto mtsp_dir = node->get_parameter("exploration.mtsp_dir").as_string();
  drone_id = node->get_parameter("exploration.drone_id").as_int();
  const int problem_id = node->get_parameter("exploration.problem_id").as_int();

  mtsp_dir1 = mtsp_dir + "/amtsp_" + std::to_string(drone_id) + ".par";
  mtsp_dir2 = mtsp_dir + "/amtsp2_" + std::to_string(drone_id) + ".par";
  mtsp_dir3 = mtsp_dir + "/amtsp3_" + std::to_string(drone_id) + ".par";

  std::string service_name;
  if (problem_id == 1) {
    service_name = "/solve_tsp_" + std::to_string(drone_id);
  } else {
    service_name = "/solve_acvrp_" + std::to_string(drone_id);
  }

  auto server = node->create_service<SolveMTSP>(service_name, solve_callback);
  RCLCPP_WARN(node->get_logger(), "MTSP server %d is ready.", drone_id);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 4: Build solver packages**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select lkh_tsp_solver lkh_mtsp_solver --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Expected: PASS.

- [ ] **Step 5: Service smoke test**

Run in terminal A:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run lkh_mtsp_solver mtsp_node --ros-args -p exploration.drone_id:=1 -p exploration.problem_id:=1 -p exploration.mtsp_dir:=$PWD/src/lkh_mtsp_solver/resource
```

Run in terminal B:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service list | rg '/solve_tsp_1'
```

Expected: `/solve_tsp_1` appears.

- [ ] **Step 6: Commit solver port**

Run:

```bash
git add src/lkh_tsp_solver src/lkh_mtsp_solver
git commit -m "feat: port RACER LKH solver services"
```

## Task 4: Port Core Utility Libraries

**Files:**
- Modify: `src/pose_utils/package.xml`
- Replace: `src/pose_utils/CMakeLists.txt`
- Modify only as required: `src/pose_utils/include/pose_utils.h`, `src/pose_utils/src/pose_utils.cpp`
- Modify: `src/poly_traj/package.xml`
- Replace: `src/poly_traj/CMakeLists.txt`

- [ ] **Step 1: Replace `pose_utils/package.xml`**

Use:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>pose_utils</name>
  <version>0.0.0</version>
  <description>Pose and geometry helper utilities used by RACER visualization.</description>
  <maintainer email="codex@example.com">Codex</maintainer>
  <license>BSD-3-Clause</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>geometry_msgs</depend>
  <depend>rclcpp</depend>
  <depend>tf2</depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Replace `pose_utils/CMakeLists.txt`**

Use:

```cmake
cmake_minimum_required(VERSION 3.8)
project(pose_utils)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)

add_library(${PROJECT_NAME}
  src/pose_utils.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${PROJECT_NAME} geometry_msgs rclcpp tf2)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(geometry_msgs rclcpp tf2)
ament_package()
```

- [ ] **Step 3: Convert ROS1 includes in pose utils only if compiler requires it**

If build reports `ros/ros.h` missing in pose utilities, replace:

```cpp
#include <ros/ros.h>
```

with:

```cpp
#include <rclcpp/rclcpp.hpp>
```

Do not alter math functions.

- [ ] **Step 4: Port `poly_traj` library**

Replace `src/poly_traj/package.xml` with:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>poly_traj</name>
  <version>0.0.0</version>
  <description>Polynomial trajectory library used by RACER trajectory utilities.</description>
  <maintainer email="codex@example.com">Codex</maintainer>
  <license>BSD-3-Clause</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>Eigen3</depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

Replace `src/poly_traj/CMakeLists.txt` with:

```cmake
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
```

- [ ] **Step 5: Build utility packages and commit**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select pose_utils poly_traj --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Expected: PASS.

Commit:

```bash
git add src/pose_utils src/poly_traj
git commit -m "feat: port RACER utility libraries"
```

## Task 5: Port Lightweight Simulation And Visualization Nodes

**Files:**
- Modify: `src/poscmd_2_odom/package.xml`
- Replace: `src/poscmd_2_odom/CMakeLists.txt`
- Modify: `src/poscmd_2_odom/src/poscmd_2_odom.cpp`
- Modify: `src/map_generator/package.xml`
- Replace: `src/map_generator/CMakeLists.txt`
- Modify: `src/map_generator/src/map_publisher.cpp`
- Modify: `src/map_generator/src/map_recorder.cpp`
- Modify: `src/map_generator/src/random_pilar_map.cpp`
- Modify: `src/map_generator/src/click_map.cpp`
- Modify: `src/odom_visualization/package.xml`
- Replace: `src/odom_visualization/CMakeLists.txt`
- Modify: `src/odom_visualization/src/odom_visualization.cpp`

- [ ] **Step 1: Write lightweight node smoke test**

Create `tools/smoke_sim_nodes.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 8 ros2 run map_generator map_pub "$(pwd)/src/map_generator/resource/pillar.pcd" >/tmp/racer_map_pub.log 2>&1 &
map_pid=$!
sleep 2
ros2 topic list | rg '/map_generator/global_cloud'
kill "${map_pid}" || true
```

Run:

```bash
chmod +x tools/smoke_sim_nodes.sh
tools/smoke_sim_nodes.sh
```

Expected before porting: FAIL because `map_generator` is not a ROS2 package yet.

- [ ] **Step 2: Replace `poscmd_2_odom` manifest and CMake**

Use a ROS2 manifest with `rclcpp`, `nav_msgs`, `quadrotor_msgs`, and `tf2_geometry_msgs`. Use this CMake shape:

```cmake
cmake_minimum_required(VERSION 3.8)
project(poscmd_2_odom)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(poscmd_2_odom src/poscmd_2_odom.cpp)
ament_target_dependencies(poscmd_2_odom nav_msgs quadrotor_msgs rclcpp tf2_geometry_msgs)

install(TARGETS poscmd_2_odom DESTINATION lib/${PROJECT_NAME})

ament_package()
```

- [ ] **Step 3: Port `poscmd_2_odom.cpp` API without changing dynamics**

Apply these replacements:

```text
ros::init -> rclcpp::init
ros::NodeHandle nh("~") -> auto node = std::make_shared<rclcpp::Node>("poscmd_2_odom")
nh.param("drone_id", drone_id, 0) -> node->declare_parameter<int>("drone_id", 0); drone_id = node->get_parameter("drone_id").as_int()
node.subscribe<quadrotor_msgs::PositionCommand> -> node->create_subscription<quadrotor_msgs::msg::PositionCommand>
node.advertise<nav_msgs::Odometry> -> node->create_publisher<nav_msgs::msg::Odometry>
ros::Timer -> rclcpp::TimerBase::SharedPtr
ros::Time::now() -> node->now()
ros::spin() -> rclcpp::spin(node)
```

Preserve the original command interpolation and odometry publication equations.

- [ ] **Step 4: Replace `map_generator` manifest and CMake**

Use dependencies `rclcpp`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `pcl_conversions`, `PCL`, and `Eigen3`. Install `resource/`.

The CMake target list must include:

```cmake
add_executable(map_recorder src/map_recorder.cpp)
add_executable(map_pub src/map_publisher.cpp)
add_executable(pilar_map src/random_pilar_map.cpp)
add_executable(click_map src/click_map.cpp)
```

Each target must link `${PCL_LIBRARIES}` and use `ament_target_dependencies(... rclcpp geometry_msgs nav_msgs sensor_msgs pcl_conversions)`.

- [ ] **Step 5: Port `map_publisher.cpp` with transient-local QoS**

Use this publishing pattern:

```cpp
auto node = std::make_shared<rclcpp::Node>("map_pub");
auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
auto cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_generator/global_cloud", qos);
```

Keep the original PCD loading path, cloud conversion, frame id, and 5 Hz publish loop.

- [ ] **Step 6: Port `odom_visualization` manifest and CMake**

Use dependencies `rclcpp`, `sensor_msgs`, `nav_msgs`, `visualization_msgs`, `geometry_msgs`, `tf2`, `tf2_geometry_msgs`, and `pose_utils`. Build one executable:

```cmake
add_executable(odom_visualization src/odom_visualization.cpp)
ament_target_dependencies(odom_visualization
  geometry_msgs nav_msgs pose_utils rclcpp sensor_msgs tf2 tf2_geometry_msgs visualization_msgs
)
```

- [ ] **Step 7: Build simulation node group**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select quadrotor_msgs pose_utils poscmd_2_odom map_generator odom_visualization --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Expected: PASS.

- [ ] **Step 8: Run smoke test and commit**

Run:

```bash
tools/smoke_sim_nodes.sh
```

Expected: PASS and `/map_generator/global_cloud` is visible.

Commit:

```bash
git add tools/smoke_sim_nodes.sh src/poscmd_2_odom src/map_generator src/odom_visualization
git commit -m "feat: port RACER lightweight simulation nodes"
```

## Task 6: Port Local Sensing Renderer

**Files:**
- Modify: `src/local_sensing/package.xml`
- Replace: `src/local_sensing/CMakeLists.txt`
- Modify: `src/local_sensing/src/pointcloud_render_node.cpp`
- Install: `src/local_sensing/params/camera.yaml`

- [ ] **Step 1: Write local sensing smoke launch**

Create `tools/smoke_local_sensing.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run map_generator map_pub "$(pwd)/src/map_generator/resource/pillar.pcd" >/tmp/racer_map_pub.log 2>&1 &
map_pid=$!
ros2 run local_sensing_node pcl_render_node --ros-args \
  -p sensing_horizon:=5.0 \
  -p sensing_rate:=10.0 \
  -p estimation_rate:=10.0 \
  -p map.x_size:=35.0 \
  -p map.y_size:=35.0 \
  -p map.z_size:=3.5 \
  -p map.init_x:=0.0 \
  -p map.init_y:=0.0 \
  -p map.drone_id:=1 \
  -r __ns:=/ \
  >/tmp/racer_pcl_render.log 2>&1 &
sensor_pid=$!
sleep 4
ros2 topic list | rg '/pcl_render_node/depth|/pcl_render_node/sensor_pose'
kill "${sensor_pid}" "${map_pid}" || true
```

Expected before porting: FAIL because `local_sensing_node` is not buildable.

- [ ] **Step 2: Replace local sensing manifest**

Use package name `local_sensing_node` to match upstream launch references. Dependencies:

```xml
<depend>cv_bridge</depend>
<depend>geometry_msgs</depend>
<depend>image_transport</depend>
<depend>nav_msgs</depend>
<depend>pcl_conversions</depend>
<depend>quadrotor_msgs</depend>
<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
<depend>tf2_ros</depend>
```

- [ ] **Step 3: Replace local sensing CMake for CPU renderer**

Use the original CPU path from upstream `ENABLE_CUDA false`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(local_sensing_node)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(pcl_render_node src/pointcloud_render_node.cpp)
target_include_directories(pcl_render_node PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(pcl_render_node
  cv_bridge geometry_msgs image_transport nav_msgs pcl_conversions quadrotor_msgs rclcpp sensor_msgs tf2_ros
)
target_link_libraries(pcl_render_node ${PCL_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS pcl_render_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY params/ DESTINATION share/${PROJECT_NAME}/params)

ament_package()
```

- [ ] **Step 4: Port `pointcloud_render_node.cpp` ROS API**

Apply these exact type migrations:

```text
geometry_msgs::PoseStamped -> geometry_msgs::msg::PoseStamped
geometry_msgs::TransformStamped -> geometry_msgs::msg::TransformStamped
nav_msgs::Odometry -> nav_msgs::msg::Odometry
nav_msgs::Path -> nav_msgs::msg::Path
sensor_msgs::Image -> sensor_msgs::msg::Image
sensor_msgs::Imu -> sensor_msgs::msg::Imu
sensor_msgs::PointCloud2 -> sensor_msgs::msg::PointCloud2
sensor_msgs::CameraInfo -> sensor_msgs::msg::CameraInfo
ros::Publisher -> rclcpp::Publisher<T>::SharedPtr
ros::Subscriber -> rclcpp::Subscription<T>::SharedPtr
ros::Timer -> rclcpp::TimerBase::SharedPtr
ros::Time -> rclcpp::Time
```

Preserve camera intrinsics, rendered cloud generation, depth image encoding, and odometry-to-camera transform math.

- [ ] **Step 5: Use transient-local subscription for global map**

Subscribe to `/map_generator/global_cloud` with:

```cpp
auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
global_map_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/map_generator/global_cloud", map_qos, rcvGlobalPointCloudCallBack);
```

- [ ] **Step 6: Build and smoke test local sensing**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select local_sensing_node map_generator quadrotor_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release
tools/smoke_local_sensing.sh
```

Expected: build passes and topics `/pcl_render_node/depth` and `/pcl_render_node/sensor_pose` appear.

- [ ] **Step 7: Commit local sensing port**

Run:

```bash
git add tools/smoke_local_sensing.sh src/local_sensing
git commit -m "feat: port RACER local sensing renderer"
```

## Task 7: Port Planning Libraries

**Files:**
- Modify: `src/path_searching/package.xml`, `src/path_searching/CMakeLists.txt`, source/header files
- Modify: `src/active_perception/package.xml`, `src/active_perception/CMakeLists.txt`, source/header files
- Modify: `src/bspline_opt/package.xml`, `src/bspline_opt/CMakeLists.txt`, source/header files
- Modify: `src/traj_utils/package.xml`, `src/traj_utils/CMakeLists.txt`, source/header files
- Modify: `src/plan_env/CMakeLists.txt`, `src/plan_env/src/*.cpp`, `src/plan_env/include/**/*.h`

- [ ] **Step 1: Write planning library build test**

Create `tools/build_planning_libs.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  quadrotor_msgs bspline plan_env path_searching active_perception bspline_opt poly_traj traj_utils \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Expected before porting: FAIL on `catkin` and ROS1 API references.

- [ ] **Step 2: Port each package manifest to `ament_cmake`**

For each package, use package format 3 and replace `roscpp`/`rospy` with `rclcpp`. Keep package-to-package dependencies from upstream:

```text
path_searching: rclcpp, std_msgs, plan_env
active_perception: rclcpp, std_msgs, plan_env, bspline, path_searching
bspline_opt: rclcpp, std_msgs, plan_env, active_perception
traj_utils: rclcpp, std_msgs, bspline, bspline_opt, path_searching, poly_traj, visualization_msgs
plan_env: rclcpp, std_msgs, visualization_msgs, cv_bridge, message_filters, sensor_msgs, nav_msgs, geometry_msgs, pcl_conversions
```

- [ ] **Step 3: Port CMake library targets**

For each package CMake:

```cmake
cmake_minimum_required(VERSION 3.8)
project(PACKAGE_NAME)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)
find_package(ament_cmake REQUIRED)
```

Then add the original `add_library` source list, `target_include_directories` with `include`, `ament_target_dependencies`, `install(TARGETS ...)`, `install(DIRECTORY include/ DESTINATION include)`, and `ament_export_*` calls.

- [ ] **Step 4: Port ROS API references in headers and libraries**

Use `rg -n "ros::|ros/|ROS_|NodeHandle|getParam|advertise|subscribe|createTimer|serviceClient|message_filters" src/path_searching src/active_perception src/bspline_opt src/traj_utils src/plan_env`.

For each match, apply this mapping:

```text
#include <ros/ros.h> -> #include <rclcpp/rclcpp.hpp>
ROS_INFO -> RCLCPP_INFO(logger_, ...)
ROS_WARN -> RCLCPP_WARN(logger_, ...)
ROS_ERROR -> RCLCPP_ERROR(logger_, ...)
ros::Time::now() -> node_->now()
ros::Duration(x).sleep() -> rclcpp::sleep_for(std::chrono::duration<double>(x))
ros::NodeHandle& nh -> const rclcpp::Node::SharedPtr& node
nh.param("a/b", value, default) -> node->declare_parameter<T>("a.b", default); value = node->get_parameter("a.b").get_value<T>()
nh.getParam("a/b", value) -> node->get_parameter("a.b", value)
```

Use package-local logger declarations in classes that did not previously keep a node:

```cpp
rclcpp::Logger logger_{rclcpp::get_logger("package_name")};
```

- [ ] **Step 5: Preserve original parameter names in launch-compatible dotted form**

For each ROS1 parameter key with slash form, declare the ROS2 parameter with dots:

```text
sdf_map/resolution -> sdf_map.resolution
map_ros/depth_filter_maxdist -> map_ros.depth_filter_maxdist
frontier/cluster_min -> frontier.cluster_min
optimization/max_vel -> optimization.max_vel
```

The launch files in Task 11 must use the same dotted names.

- [ ] **Step 6: Restore `plan_env` CMake library target**

After message generation works, extend `src/plan_env/CMakeLists.txt` with:

```cmake
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(plan_env
  src/sdf_map.cpp
  src/obj_predictor.cpp
  src/edt_environment.cpp
  src/raycast.cpp
  src/map_ros.cpp
  src/multi_map_manager.cpp
)
target_include_directories(plan_env PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(plan_env
  cv_bridge message_filters nav_msgs pcl_conversions rclcpp sensor_msgs std_msgs visualization_msgs
)
target_link_libraries(plan_env ${PCL_LIBRARIES} ${OpenCV_LIBS} "${cpp_typesupport_target}")
```

- [ ] **Step 7: Build planning libraries**

Run:

```bash
chmod +x tools/build_planning_libs.sh
tools/build_planning_libs.sh
```

Expected: PASS.

- [ ] **Step 8: Commit planning libraries**

Run:

```bash
git add tools/build_planning_libs.sh src/path_searching src/active_perception src/bspline_opt src/traj_utils src/plan_env src/bspline
git commit -m "feat: port RACER planning libraries"
```

## Task 8: Port Trajectory Server And Planner Manager Nodes

**Files:**
- Modify: `src/plan_manage/package.xml`
- Replace: `src/plan_manage/CMakeLists.txt`
- Modify: `src/plan_manage/src/traj_server.cpp`
- Modify: `src/plan_manage/src/fast_planner_node.cpp`
- Modify: `src/plan_manage/src/planner_manager.cpp`
- Modify headers under `src/plan_manage/include/plan_manage/`

- [ ] **Step 1: Write trajectory server build test**

Create `tools/build_plan_manage.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
colcon build --packages-select plan_manage --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Expected before porting: FAIL on `catkin` or ROS1 APIs.

- [ ] **Step 2: Replace `plan_manage` manifest dependencies**

Use package format 3 with dependencies:

```text
active_perception
bspline
bspline_opt
geometry_msgs
nav_msgs
path_searching
plan_env
poly_traj
quadrotor_msgs
rclcpp
std_msgs
traj_utils
visualization_msgs
```

- [ ] **Step 3: Replace CMake targets**

Build these targets:

```cmake
add_library(plan_manage
  src/planner_manager.cpp
)
add_executable(traj_server src/traj_server.cpp)
add_executable(fast_planner_node src/fast_planner_node.cpp)
```

Link `plan_manage` against the planning libraries. Link `traj_server` against `bspline`, `quadrotor_msgs`, `nav_msgs`, `visualization_msgs`, and `rclcpp`.

- [ ] **Step 4: Port `traj_server.cpp` callbacks and timers**

Use ROS2 message names:

```text
bspline::Bspline -> bspline::msg::Bspline
quadrotor_msgs::PositionCommand -> quadrotor_msgs::msg::PositionCommand
nav_msgs::Odometry -> nav_msgs::msg::Odometry
visualization_msgs::Marker -> visualization_msgs::msg::Marker
```

Use these timer periods:

```cpp
auto cmd_timer = node->create_wall_timer(std::chrono::milliseconds(10), cmdCallback);
auto vis_timer = node->create_wall_timer(std::chrono::milliseconds(250), visCallback);
```

Preserve original trajectory sampling, command publication, yaw handling, and path length visualization.

- [ ] **Step 5: Build `plan_manage`**

Run:

```bash
chmod +x tools/build_plan_manage.sh
tools/build_plan_manage.sh
```

Expected: PASS.

- [ ] **Step 6: Commit plan manager port**

Run:

```bash
git add tools/build_plan_manage.sh src/plan_manage
git commit -m "feat: port RACER plan manager"
```

## Task 9: Port Exploration Manager

**Files:**
- Modify: `src/exploration_manager/CMakeLists.txt`
- Modify: `src/exploration_manager/src/exploration_node.cpp`
- Modify: `src/exploration_manager/src/fast_exploration_fsm.cpp`
- Modify: `src/exploration_manager/src/fast_exploration_manager.cpp`
- Modify: `src/exploration_manager/src/ground_node.cpp`
- Modify headers under `src/exploration_manager/include/exploration_manager/`

- [ ] **Step 1: Write exploration manager build test**

Create `tools/build_exploration_manager.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
colcon build --packages-select exploration_manager --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Expected before porting: FAIL because only interfaces are buildable.

- [ ] **Step 2: Restore exploration manager executables in CMake**

Extend `src/exploration_manager/CMakeLists.txt`:

```cmake
find_package(active_perception REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(lkh_mtsp_solver REQUIRED)
find_package(lkh_tsp_solver REQUIRED)
find_package(path_searching REQUIRED)
find_package(PCL REQUIRED)
find_package(plan_env REQUIRED)
find_package(plan_manage REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(traj_utils REQUIRED)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(exploration_node
  src/exploration_node.cpp
  src/fast_exploration_fsm.cpp
  src/fast_exploration_manager.cpp
)
add_executable(ground_node
  src/ground_node.cpp
  src/fast_exploration_manager.cpp
)
```

Both executables must link `cpp_typesupport_target` and depend on all listed packages.

- [ ] **Step 3: Port exploration node startup**

Replace ROS1 node setup with:

```cpp
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("exploration_node");
  fast_planner::FastExplorationFSM fsm;
  fsm.init(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

The upstream namespace and class are `fast_planner::FastExplorationFSM`.

- [ ] **Step 4: Port FSM init signatures**

Change initialization methods that accept ROS1 handles:

```cpp
void init(ros::NodeHandle& nh);
```

to:

```cpp
void init(const rclcpp::Node::SharedPtr& node);
```

Store the node as:

```cpp
rclcpp::Node::SharedPtr node_;
rclcpp::Logger logger_{rclcpp::get_logger("exploration_manager")};
```

- [ ] **Step 5: Port coordination messages and services**

Use these ROS2 types:

```text
exploration_manager::DroneState -> exploration_manager::msg::DroneState
exploration_manager::PairOpt -> exploration_manager::msg::PairOpt
exploration_manager::PairOptResponse -> exploration_manager::msg::PairOptResponse
exploration_manager::HGrid -> exploration_manager::msg::HGrid
exploration_manager::GridTour -> exploration_manager::msg::GridTour
lkh_mtsp_solver::SolveMTSP -> lkh_mtsp_solver::srv::SolveMTSP
```

Preserve original topic names:

```text
/swarm_expl/drone_state
/swarm_expl/pair_opt
/swarm_expl/pair_opt_res
/swarm_expl/grid_tour
/swarm_expl/hgrid
```

- [ ] **Step 6: Port start trigger**

Keep the original RViz `2D Nav Goal` behavior by subscribing to:

```text
/move_base_simple/goal
```

with type:

```cpp
geometry_msgs::msg::PoseStamped
```

Do not introduce a new trigger algorithm. The benchmark script may publish the same message to the same topic.

- [ ] **Step 7: Preserve pairwise coordination and hgrid logic**

Verify these method groups compile without algorithmic edits:

```text
hgrid update and subdivision
pairwise request/response scheduling
CVRP/ACVRP file generation
frontier information structure update
coverage path extraction
viewpoint refinement
minimum-time B-spline trajectory generation
swarm trajectory sharing
```

Only ROS type names, node/timer/service APIs, parameter access, and logging calls may change.

- [ ] **Step 8: Build exploration manager**

Run:

```bash
chmod +x tools/build_exploration_manager.sh
tools/build_exploration_manager.sh
```

Expected: PASS.

- [ ] **Step 9: Commit exploration manager port**

Run:

```bash
git add tools/build_exploration_manager.sh src/exploration_manager
git commit -m "feat: port RACER exploration manager"
```

## Task 10: ROS2 Launch And RViz2 Configuration

**Files:**
- Create: `launch/racer_pillar.launch.py`
- Create: `launch/single_drone_exploration.launch.py`
- Create: `launch/rviz.launch.py`
- Create or modify: `rviz/swarm.rviz`

- [ ] **Step 1: Create top-level launch file**

Create `launch/racer_pillar.launch.py` with launch arguments:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_size_x = LaunchConfiguration("map_size_x")
    map_size_y = LaunchConfiguration("map_size_y")
    map_size_z = LaunchConfiguration("map_size_z")
    drone_num = LaunchConfiguration("drone_num")

    single = PathJoinSubstitution([
        FindPackageShare("racer_ros2_bringup"),
        "launch",
        "single_drone_exploration.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("map_size_x", default_value="35.0"),
        DeclareLaunchArgument("map_size_y", default_value="35.0"),
        DeclareLaunchArgument("map_size_z", default_value="3.5"),
        DeclareLaunchArgument("drone_num", default_value="4"),
        Node(
            package="map_generator",
            executable="map_pub",
            name="map_pub",
            output="screen",
            arguments=[PathJoinSubstitution([
                FindPackageShare("map_generator"),
                "resource",
                "pillar.pcd",
            ])],
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(single), launch_arguments={
            "drone_id": "1", "init_x": "0", "init_y": "0", "init_z": "1",
            "drone_num": drone_num, "map_size_x": map_size_x, "map_size_y": map_size_y,
            "map_size_z": map_size_z,
        }.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(single), launch_arguments={
            "drone_id": "2", "init_x": "0", "init_y": "-1", "init_z": "1",
            "drone_num": drone_num, "map_size_x": map_size_x, "map_size_y": map_size_y,
            "map_size_z": map_size_z,
        }.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(single), launch_arguments={
            "drone_id": "3", "init_x": "1", "init_y": "-1", "init_z": "1",
            "drone_num": drone_num, "map_size_x": map_size_x, "map_size_y": map_size_y,
            "map_size_z": map_size_z,
        }.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(single), launch_arguments={
            "drone_id": "4", "init_x": "1", "init_y": "0", "init_z": "1",
            "drone_num": drone_num, "map_size_x": map_size_x, "map_size_y": map_size_y,
            "map_size_z": map_size_z,
        }.items()),
    ])
```

If a package named `racer_ros2_bringup` does not exist yet, create it with `ament_cmake` and install `launch/` and `rviz/`.

- [ ] **Step 2: Create per-drone launch file**

Create `launch/single_drone_exploration.launch.py` that launches exactly these nodes for one drone:

```text
exploration_manager/exploration_node
plan_manage/traj_server
lkh_mtsp_solver/mtsp_node with problem_id=1
lkh_mtsp_solver/mtsp_node with problem_id=2
poscmd_2_odom/poscmd_2_odom
odom_visualization/odom_visualization
local_sensing_node/pcl_render_node
```

Use the original remap shape:

```text
/odom_world -> /state_ukf/odom_<drone_id>
/planning/bspline -> /planning/bspline_<drone_id>
/planning/replan -> /planning/replan_<drone_id>
/planning/new -> /planning/new_<drone_id>
/position_cmd -> /planning/pos_cmd_<drone_id>
/pcl_render_node/depth -> /pcl_render_node/depth_<drone_id>
/pcl_render_node/sensor_pose -> /pcl_render_node/sensor_pose_<drone_id>
/pcl_render_node/cloud -> /pcl_render_node/cloud_<drone_id>
```

- [ ] **Step 3: Use original pillar parameters**

In the `exploration_node` parameter dictionary, include these exact benchmark values:

```python
"manager.max_vel": 1.5,
"search.max_vel": 1.5,
"optimization.max_vel": 1.5,
"bspline.limit_vel": 1.5,
"manager.max_yawdot": 0.9,
"heading_planner.max_yaw_rate": 0.9,
"sdf_map.box_min_x": -7.0,
"sdf_map.box_min_y": -15.0,
"sdf_map.box_min_z": 0.0,
"sdf_map.box_max_x": 7.0,
"sdf_map.box_max_y": 15.0,
"sdf_map.box_max_z": 1.7,
```

All other parameters should be copied from `upstream_RACER/swarm_exploration/exploration_manager/launch/single_drone_planner.xml` with slash names converted to dotted names.

- [ ] **Step 4: Create RViz2 launch**

Create `launch/rviz.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([
                FindPackageShare("racer_ros2_bringup"),
                "rviz",
                "swarm.rviz",
            ])],
        )
    ])
```

- [ ] **Step 5: Convert RViz config**

Copy `upstream_RACER/swarm_exploration/exploration_manager/config/swarm.rviz` to `rviz/swarm.rviz`. Replace ROS1 display class names with RViz2 display names where needed:

```text
rviz/PointCloud2 -> rviz_default_plugins/PointCloud2
rviz/Marker -> rviz_default_plugins/Marker
rviz/MarkerArray -> rviz_default_plugins/MarkerArray
rviz/Odometry -> rviz_default_plugins/Odometry
rviz/Path -> rviz_default_plugins/Path
```

Set fixed frame to:

```text
world
```

- [ ] **Step 6: Build bringup and launch smoke test**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to exploration_manager plan_manage map_generator local_sensing_node poscmd_2_odom odom_visualization --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
timeout 20 ros2 launch racer_ros2_bringup racer_pillar.launch.py
```

Expected: process starts all nodes and exits from `timeout` without an immediate launch exception.

- [ ] **Step 7: Commit launch and RViz**

Run:

```bash
git add launch rviz src/racer_ros2_bringup
git commit -m "feat: add ROS2 RACER pillar bringup"
```

## Task 11: Functional Pillar Run

**Files:**
- Modify: the specific ported package files reported by runtime errors, limited to ROS2 compatibility issues listed in Step 3.
- Create: `tools/run_pillar_once.sh`

- [ ] **Step 1: Create one-run script**

Create `tools/run_pillar_once.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true
timeout 120 ros2 launch racer_ros2_bringup racer_pillar.launch.py
```

- [ ] **Step 2: Launch and trigger exploration manually**

Run:

```bash
chmod +x tools/run_pillar_once.sh
tools/run_pillar_once.sh
```

In a second terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped "{header: {frame_id: world}, pose: {position: {x: 0.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"
```

Expected: drones start exploration, publish `/planning/bspline_*`, `/planning/pos_cmd_*`, `/sdf_map/occupancy_*`, and `/planning_vis/trajectory_*`.

- [ ] **Step 3: Fix runtime-only ROS2 issues without changing algorithms**

For runtime issues, limit fixes to:

```text
QoS incompatibility
missing parameter declaration
topic remap mismatch
resource path mismatch
service availability wait
timer callback lifetime
message timestamp conversion
```

Do not change planner scoring, hgrid allocation, TSP/ACVRP problem generation, frontier gain, trajectory optimization costs, or completion thresholds.

- [ ] **Step 4: Commit functional run fixes**

Run:

```bash
git add src launch rviz tools/run_pillar_once.sh
git commit -m "fix: stabilize ROS2 pillar exploration run"
```

## Task 12: Benchmark Automation

**Files:**
- Create: `tools/benchmark_pillar.py`
- Create: `tools/benchmark_pillar.sh`
- Create: `benchmark_results/.gitkeep`

- [ ] **Step 1: Create benchmark shell wrapper**

Create `tools/benchmark_pillar.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/benchmark_pillar.py --runs 5 --timeout-sec 120 --output benchmark_results/pillar_runs.csv
```

- [ ] **Step 2: Create benchmark Python skeleton**

Create `tools/benchmark_pillar.py` with this structure:

```python
#!/usr/bin/env python3
import argparse
import csv
import math
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


@dataclass
class DronePath:
    last: Optional[Tuple[float, float, float]] = None
    length: float = 0.0

    def add(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        current = (p.x, p.y, p.z)
        if self.last is not None:
            dx = current[0] - self.last[0]
            dy = current[1] - self.last[1]
            dz = current[2] - self.last[2]
            self.length += math.sqrt(dx * dx + dy * dy + dz * dz)
        self.last = current


@dataclass
class LaunchLog:
    lines: "queue.Queue[str]" = field(default_factory=queue.Queue)
    finished_drones: set = field(default_factory=set)

    def feed(self, line: str) -> None:
        self.lines.put(line)
        match = re.search(r"Drone\\s+(\\d+)\\s+from\\s+\\S+\\s+to\\s+FINISH", line)
        if match:
            self.finished_drones.add(int(match.group(1)))

    def team_finished(self, drone_num: int) -> bool:
        return all(drone_id in self.finished_drones for drone_id in range(1, drone_num + 1))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--timeout-sec", type=float, default=120.0)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


def start_launch(log: LaunchLog) -> subprocess.Popen:
    env = os.environ.copy()
    proc = subprocess.Popen(
        ["ros2", "launch", "racer_ros2_bringup", "racer_pillar.launch.py"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
        env=env,
    )
    def reader() -> None:
        assert proc.stdout is not None
        for line in proc.stdout:
            log.feed(line.rstrip())
    threading.Thread(target=reader, daemon=True).start()
    return proc


def stop_launch(proc: subprocess.Popen) -> None:
    if proc.poll() is None:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=10)


def trigger_start(node) -> None:
    pub = node.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    msg = PoseStamped()
    msg.header.frame_id = "world"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x = 0.0
    msg.pose.position.y = 0.0
    msg.pose.position.z = 1.0
    msg.pose.orientation.w = 1.0
    pub.publish(msg)


def run_once(run_id: int, timeout_sec: float) -> Tuple[float, float]:
    subprocess.run(["ros2", "daemon", "stop"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["ros2", "daemon", "start"], check=True, stdout=subprocess.DEVNULL)
    log = LaunchLog()
    proc = start_launch(log)
    rclpy.init(args=None)
    node = rclpy.create_node(f"pillar_benchmark_{run_id}")
    paths: Dict[int, DronePath] = {i: DronePath() for i in range(1, 5)}
    subscriptions = []
    for drone_id in paths:
        topic = f"/state_ukf/odom_{drone_id}"
        subscriptions.append(node.create_subscription(
            Odometry,
            topic,
            lambda msg, did=drone_id: paths[did].add(msg),
            50,
        ))
    try:
        trigger_start(node)
        start_time = time.monotonic()
        deadline = start_time + timeout_sec
        completed_time = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if log.team_finished(4):
                completed_time = time.monotonic()
                break
        if completed_time is None:
            raise TimeoutError(f"run {run_id} did not complete within {timeout_sec} seconds")
        elapsed = completed_time - start_time
        total_length = sum(path.length for path in paths.values())
        return elapsed, total_length
    finally:
        node.destroy_node()
        rclpy.shutdown()
        stop_launch(proc)


def main() -> int:
    args = parse_args()
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    rows = []
    for run_id in range(1, args.runs + 1):
        elapsed, total_length = run_once(run_id, args.timeout_sec)
        rows.append({"run": run_id, "exploration_time_sec": elapsed, "total_path_length_m": total_length})
        print(f"run={run_id} exploration_time_sec={elapsed:.3f} total_path_length_m={total_length:.3f}", flush=True)
    with open(args.output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["run", "exploration_time_sec", "total_path_length_m"])
        writer.writeheader()
        writer.writerows(rows)
    avg_time = sum(row["exploration_time_sec"] for row in rows) / len(rows)
    avg_length = sum(row["total_path_length_m"] for row in rows) / len(rows)
    print(f"average_exploration_time_sec={avg_time:.3f}")
    print(f"average_total_path_length_m={avg_length:.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 3: Verify completion detection uses RACER FSM transition logs**

Confirm that the ROS2 port preserves the upstream FSM transition log emitted by `FastExplorationFSM::transitState`:

```cpp
RCLCPP_INFO_STREAM(
    node_->get_logger(),
    "[" + pos_call + "]: Drone " << getId()
    << " from " + fd_->state_str_[pre_s]
    << " to " + fd_->state_str_[int(new_state)]);
```

The benchmark script parses `Drone <id> from <state> to FINISH` and succeeds only when drones `1`, `2`, `3`, and `4` have reached `FINISH`.

- [ ] **Step 4: Run one benchmark trial**

Run:

```bash
chmod +x tools/benchmark_pillar.py tools/benchmark_pillar.sh
python3 tools/benchmark_pillar.py --runs 1 --timeout-sec 120 --output benchmark_results/pillar_smoke.csv
```

Expected: CSV contains one row with exploration time and total path length.

- [ ] **Step 5: Commit benchmark automation**

Run:

```bash
touch benchmark_results/.gitkeep
git add tools/benchmark_pillar.py tools/benchmark_pillar.sh benchmark_results/.gitkeep
git commit -m "feat: add RACER pillar benchmark automation"
```

## Task 13: Final Verification And Five Clean Runs

**Files:**
- Generated: `benchmark_results/pillar_runs.csv`
- Generated: `benchmark_results/pillar_summary.txt`
- Modify: final bug fixes only if verification reveals ROS2 port errors.

- [ ] **Step 1: Run full build**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Expected: PASS.

- [ ] **Step 2: Run focused smoke tests**

Run:

```bash
tools/check_interfaces.sh
tools/smoke_sim_nodes.sh
tools/smoke_local_sensing.sh
tools/build_planning_libs.sh
tools/build_plan_manage.sh
tools/build_exploration_manager.sh
```

Expected: all PASS.

- [ ] **Step 3: Verify RViz2 config loads**

Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 20 ros2 launch racer_ros2_bringup rviz.launch.py
```

Expected: RViz2 starts and exits from `timeout` without config parse errors.

- [ ] **Step 4: Run five clean-state pillar benchmarks**

Run:

```bash
tools/benchmark_pillar.sh | tee benchmark_results/pillar_summary.txt
```

Expected:

```text
benchmark_results/pillar_runs.csv contains 5 rows
average_exploration_time_sec is around 40.0
average_total_path_length_m is in the 170-180 range
```

- [ ] **Step 5: Inspect final git diff**

Run:

```bash
git status --short
git diff --stat HEAD
```

Expected: only intentional source, launch, RViz, tools, and generated benchmark result files are changed.

- [ ] **Step 6: Commit final verification artifacts**

Run:

```bash
git add benchmark_results/pillar_runs.csv benchmark_results/pillar_summary.txt
git commit -m "test: record RACER pillar benchmark runs"
```

- [ ] **Step 7: Final report**

Report:

```text
ROS 2 distro: Humble
Velocity cap: 1.5 m/s
Angular-rate cap: 0.9 rad/s
Run 1: exploration_time_sec=<value>, total_path_length_m=<value>
Run 2: exploration_time_sec=<value>, total_path_length_m=<value>
Run 3: exploration_time_sec=<value>, total_path_length_m=<value>
Run 4: exploration_time_sec=<value>, total_path_length_m=<value>
Run 5: exploration_time_sec=<value>, total_path_length_m=<value>
Average: exploration_time_sec=<value>, total_path_length_m=<value>
Verification commands run: <commands>
Known deviations from upstream: <none unless explicitly discovered>
```

## Self-Review

Spec coverage:

- ROS 2 Humble target: covered by all build and launch commands sourcing `/opt/ros/humble/setup.bash`.
- Original lightweight simulation: covered by Tasks 5, 6, 10, and 11.
- Original algorithms only: covered by Task 7 and Task 9 constraints that limit changes to ROS2 API/build/runtime compatibility.
- Pillar benchmark with velocity and angular-rate caps: covered by Task 10 parameter values and Task 13 report.
- RViz2 visualization: covered by Task 10 and Task 13.
- Five clean-state runs: covered by Tasks 12 and 13.

Placeholder scan:

- No unresolved markers.
- No unfinished implementation sections.
- The only runtime-discovered item is completion signal wiring in Task 12; the allowed sources are explicit and must be resolved before final benchmark success.

Type consistency:

- ROS2 message namespaces use `package::msg::Type`.
- ROS2 service namespaces use `package::srv::Type`.
- Parameter names use dotted ROS2 names in code and launch.
