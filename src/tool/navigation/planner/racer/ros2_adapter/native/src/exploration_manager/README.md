# RACER ROS 2 Humble Pillar Demo

This package contains the ROS 2 Humble launch entry point for the RACER
multi-robot exploration demo in the original lightweight simulation
environment. The default demo uses the pillar map, four simulated drones,
RACER's original exploration/planning pipeline, and RViz2 visualization.

The ROS 1 upstream checkout under `upstream_RACER/` is kept as reference code.
Run and build the ROS 2 port from `src/`; do not ask `colcon` to build the
whole repository root unless you intentionally exclude the upstream reference
tree.

## What the Pillar Demo Starts

`racer_pillar.launch.py` starts these components:

- `map_generator/map_pub` publishes `pillar.pcd`.
- Four `exploration_manager/exploration_node` instances run RACER.
- Four `plan_manage/traj_server` instances execute planned B-spline
  trajectories.
- Four `poscmd_2_odom` instances provide the lightweight simulated odometry
  loop used by the original demo.
- Four `local_sensing_node/pcl_render_node` instances render local depth/cloud
  observations from the static pillar map.
- Four `odom_visualization` instances publish drone visualization markers.
- RViz2 opens with `config/swarm.rviz` unless `rviz:=false` is passed.

The configured limits for the pillar launch are:

- Maximum velocity: `1.5 m/s`
- Maximum acceleration: `1.0 m/s^2`
- Maximum yaw/angular rate: `0.9 rad/s`
- SDF resolution: `0.1 m`
- Exploration box: `x=[-7, 7]`, `y=[-15, 15]`, `z=[0, 1.7]`

## Requirements

- ROS 2 Humble.
- A working `colcon` build environment.
- The package dependencies used by this workspace already installed.
- A graphical desktop or X server for RViz2.

All commands below are run from the repository root. No command depends on a
machine-specific absolute workspace path.

## Build

Use `--base-paths src` so `colcon` only discovers the ROS 2 packages and does
not collide with duplicate ROS 1 package names in `upstream_RACER/`.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src
source install/setup.bash
```

If you changed only `plan_env` or `exploration_manager`, a smaller rebuild is
usually enough:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src \
  --packages-select plan_env exploration_manager
source install/setup.bash
```

## Start the Interactive Demo

Open a terminal at the repository root:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch exploration_manager racer_pillar.launch.py
```

This starts the full pillar simulation and RViz2. The launch file uses the
RViz2 config at `src/exploration_manager/config/swarm.rviz`.

If RViz2 has GLX/rendering problems, keep the same launch and force software
rendering from the shell:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch exploration_manager racer_pillar.launch.py \
  rviz_gl_vendor:=mesa
```

To run the demo without RViz2:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch exploration_manager racer_pillar.launch.py rviz:=false
```

## Trigger Exploration

The launch waits for a goal on `/move_base_simple/goal`. You can trigger it in
RViz2 or from the command line.

### Option 1: RViz2

1. Wait until RViz2 shows the pillar map/grid and the left display tree is
   loaded.
2. Select the `2D Goal Pose` tool in the RViz2 toolbar.
3. Click near the center of the map and drag to set a heading.
4. The four drones should leave `IDLE`, plan, and begin exploring.

### Option 2: Terminal Trigger

Open a second terminal at the repository root:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 10 ros2 topic pub -r 10 /move_base_simple/goal \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"
```

The repeated publish is intentional. It gives all exploration nodes enough time
to receive the same start goal.

## What to Watch in RViz2

The default RViz2 config visualizes the same RACER surfaces used by the demo:

- `/map_generator/global_cloud`: pillar map.
- `/sdf_map/occupancy_all_<id>` and `/sdf_map/occupancy_local_<id>`:
  per-drone occupied map.
- `/planning_vis/frontier_<id>`: frontier clusters.
- `/planning_vis/viewpoints_<id>`: candidate viewpoints.
- `/planning_vis/trajectory_<id>` and `/planning/travel_traj_<id>`:
  planned/executed trajectories.
- `/planning/position_cmd_vis_<id>`: command path visualization.
- `/odom_visualization/robot_<id>` and related odometry displays:
  drone pose markers.

When exploration finishes, each exploration node logs `No grid`. The trajectory
servers also print a summary line:

```text
Drone <id>, [<time>,<length>,<velocity>,<energy>], [time, length, vel, energy]
```

For the pillar scene, a typical completed run finishes in roughly `40-60 s`.
Path length varies by run because the original RACER planner and map update
timing are not deterministic.

## Run a Clean Benchmark

The benchmark wrapper repeatedly starts a clean pillar launch, publishes the
same goal, waits for completion, and writes CSV/log artifacts under
`benchmark_results/`.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
tools/benchmark_pillar.sh --runs 10 --timeout-sec 180 \
  --output benchmark_results/pillar_runs_10.csv \
  --log-dir benchmark_results/logs_10
```

The CSV contains:

- `exploration_time_sec`
- `total_path_length_m`
- `wall_time_sec`
- `completion_mode`
- `coverage_percent`
- `coverage_min_percent`
- `coverage_max_percent`
- `coverage_drone_<id>_percent`

The coverage columns are measurement-only. They do not change RACER planning.
Coverage is computed as known SDF voxels divided by total voxels inside the
configured exploration box.

## Launch Arguments

Useful arguments for `racer_pillar.launch.py`:

```bash
rviz:=true|false
rviz_gl_vendor:=mesa
coverage_metrics:=true|false
coverage_report_interval_sec:=1.0
```

Examples:

```bash
# Full demo with RViz2
ros2 launch exploration_manager racer_pillar.launch.py

# Headless demo
ros2 launch exploration_manager racer_pillar.launch.py rviz:=false

# Demo with coverage log lines
ros2 launch exploration_manager racer_pillar.launch.py \
  coverage_metrics:=true coverage_report_interval_sec:=1.0
```

## Troubleshooting

### `Duplicate package names not supported`

Build only the ROS 2 source tree:

```bash
colcon build --symlink-install --base-paths src
```

This avoids duplicate package names from `upstream_RACER/`.

### RViz2 opens but the scene is blank

Check that the launch is still running and topics are available:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic list | grep -E "map_generator|sdf_map|planning_vis|odom_visualization"
```

If topics exist but RViz2 does not render, restart with software rendering:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch exploration_manager racer_pillar.launch.py \
  rviz_gl_vendor:=mesa
```

### Exploration does not start

Publish the goal again from a second terminal:

```bash
timeout 10 ros2 topic pub -r 10 /move_base_simple/goal \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"
```

Then watch the launch terminal for state transitions from `IDLE` to planning
states.

### Stop a run

Press `Ctrl-C` in the launch terminal. For a new clean run, start the launch
again from a fresh terminal after sourcing `install/setup.bash`.
