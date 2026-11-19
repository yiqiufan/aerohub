# RACER ROS 2 Humble Port Design

## Objective

Port the original RACER simulation and exploration stack to ROS 2 Humble while preserving the upstream algorithmic behavior. The port must keep the original lightweight simulation path and avoid adding algorithms that are not present in the upstream RACER implementation.

Success criteria:

- Original RACER simulation and exploration features work in ROS 2 Humble.
- The pillar-area demo runs with max velocity `1.5 m/s` and angular-rate cap `0.9 rad/s`.
- The pillar-area benchmark completes around `40.0 s` with total path length around `170-180 m`.
- RViz2 visualization works for maps, frontiers/unknown voxels, viewpoints, FoV, trajectories, and drone state.
- After migration, the pillar-area benchmark is run five times from clean state and reports exploration time and total path length for each run.

## Scope

The port targets ROS 2 Humble only.

Included:

- RACER exploration/planning packages used by the original lightweight simulation demo.
- Original PCD map generation and pillar/office environment selection.
- Original local sensing renderer path.
- Original lightweight `poscmd_2_odom` simulation.
- Original trajectory server and B-spline command flow.
- Original hgrid decomposition, pairwise coordination, CVRP/TSP coverage planning, frontier extraction, viewpoint refinement, and trajectory optimization code.
- ROS2-native launch files and RViz2 visualization.
- Benchmark automation for the pillar-area demo.

Excluded:

- ROS1 compatibility bridge as the main runtime.
- Gazebo, PX4, or alternate simulation backends.
- New exploration, planning, allocation, mapping, or control algorithms.
- Real-world and hardware-facing launch paths from upstream RACER unless they are required by the simulation/exploration demo.
- ROS1 RViz custom plugin migration unless a simulation/exploration visualization feature cannot be represented with standard RViz2 displays.

## Package Strategy

Keep `upstream_RACER/` as an untouched reference checkout. Create ROS2 port packages alongside it in the workspace.

Port the packages required for the original simulation and exploration demo:

- `quadrotor_msgs`
- `bspline`
- `plan_env`
- `path_searching`
- `bspline_opt`
- `active_perception`
- `traj_utils`
- `plan_manage`
- `exploration_manager`
- `lkh_tsp_solver`
- `lkh_mtsp_solver`
- `map_generator`
- `local_sensing`
- `poscmd_2_odom`
- `odom_visualization`
- small utility packages only where required by these packages

Packages not needed by the lightweight demo, such as full SO3 dynamics, disturbance generation, and ROS1 RViz plugin tooling, remain out of the first port unless dependency tracing proves they are required.

## Runtime Architecture

The ROS2 demo preserves the original lightweight simulation pipeline.

`map_generator` publishes the selected PCD environment, with `pillar.pcd` as the default quick-start scene, on `/map_generator/global_cloud`.

For each drone:

- `poscmd_2_odom` simulates odometry from planner position commands.
- `local_sensing` renders local depth/cloud observations from the global PCD map and drone odometry.
- `exploration_manager` runs RACER's FSM, frontier extraction, hgrid decomposition, pairwise coordination, coverage-path planning, and trajectory planning.
- `lkh_mtsp_solver` provides original TSP/ACVRP solve services using RACER's bundled LKH3 code path.
- `plan_manage/traj_server` converts B-spline trajectories into position commands and publishes path/trajectory visualization.

Shared coordination, mapping, and trajectory topics keep the original names where practical. ROS2 launch remaps replace ROS1 XML remaps. Latched ROS1 publishers, such as static/global map publication, become ROS2 publishers with transient-local QoS.

RViz2 uses a ported `swarm.rviz` configuration with standard RViz2 displays for point clouds, occupancy/unknown markers, frontiers, viewpoints, FoV, trajectories, path/command markers, and drone odometry.

## Porting Rules

Algorithm code stays structurally close to upstream. Acceptable changes are limited to:

- ROS API migration from `roscpp` to `rclcpp`.
- `catkin` to `ament_cmake`.
- ROS1 messages/services to ROS2 `rosidl_generate_interfaces`.
- ROS1 parameters to ROS2 declared parameters.
- ROS1 launch XML to ROS2 Python launch or ROS2 XML where appropriate.
- ROS1 time, duration, rate, timer, service, publisher, subscriber, and logging API replacements.
- Compatibility updates for ROS2 dependencies, QoS, and C++ standard requirements.

Parameter behavior starts from upstream defaults. The pillar benchmark keeps max velocity `1.5 m/s`. Angular-rate-related limits are audited and configured so the benchmark uses `0.9 rad/s` as the requested cap without adding new behavior.

The LKH behavior should remain original. The host has NLopt 2.7.1 available. `LKH` is not currently on `PATH`, so the port should prefer the bundled LKH3 library/service path already present in RACER. External `/usr/local/bin/LKH` is required only for original code paths that explicitly invoke it.

## Testing And Benchmarking

Verification sequence:

1. Build all ported packages with `colcon build` on ROS 2 Humble.
2. Run node and topic smoke tests for the pillar demo.
3. Launch RViz2 with the ported config and verify displays load.
4. Run one functional pillar exploration to validate completion and metric capture.
5. Run five clean-state pillar benchmarks.

Benchmark definition:

- Exploration time starts when the automated start trigger is accepted and ends when RACER reports completion for the team.
- Total path length is the sum of integrated odometry path lengths across all drones, matching the paper's total movement distance style.
- Clean state means no stale demo processes, no stale ROS2 daemon state affecting discovery, and no stale solver/log files from the previous run.

Benchmark automation will launch the pillar demo, trigger exploration reproducibly, record completion and odometry, compute exploration time/path length, and write per-run CSV rows plus a summary.

The expected target is aligned with the RACER paper's Pillar result for the full method under ideal communication: roughly `38.5 s` average exploration time and `171.3 m` average total path length.

## Risks

- ROS2 timing and QoS differences can change coordination and completion timing if not matched carefully.
- Parameter declaration and namespace differences can silently alter behavior if ported launch files do not preserve original names.
- RViz2 may not support every original ROS1 RViz plugin display directly; standard displays should be used unless a missing visualization is essential.
- The upstream stack has many ROS API touch points, so the port should proceed package by package with build and smoke-test checkpoints.

## References

- Upstream RACER quick start: `https://github.com/Robotics-STAR-Lab/RACER#quick-start`
- RACER paper: `https://arxiv.org/pdf/2209.08533`
