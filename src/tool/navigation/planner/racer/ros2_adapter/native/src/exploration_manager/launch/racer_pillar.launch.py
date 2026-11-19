from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


MAP_SIZE_X = 35.0
MAP_SIZE_Y = 35.0
MAP_SIZE_Z = 3.5
DRONE_NUM = 5
MAX_VEL = 1.5
MAX_ACC = 1.0
MAX_YAW_RATE = 0.9


def planner_params(drone_id, coverage_metrics, coverage_report_interval_sec, show_grid_text):
    return {
        "sdf_map.resolution": 0.1,
        "sdf_map.map_size_x": MAP_SIZE_X,
        "sdf_map.map_size_y": MAP_SIZE_Y,
        "sdf_map.map_size_z": MAP_SIZE_Z,
        "sdf_map.obstacles_inflation": 0.199,
        "sdf_map.local_bound_inflate": 0.5,
        "sdf_map.local_map_margin": 50,
        "sdf_map.ground_height": -1.0,
        "sdf_map.default_dist": 0.5,
        "sdf_map.p_hit": 0.65,
        "sdf_map.p_miss": 0.35,
        "sdf_map.p_min": 0.12,
        "sdf_map.p_max": 0.90,
        "sdf_map.p_occ": 0.80,
        "sdf_map.min_ray_length": 0.5,
        "sdf_map.max_ray_length": 4.5,
        "sdf_map.virtual_ceil_height": -10.0,
        "sdf_map.optimistic": True,
        "sdf_map.signed_dist": False,
        "sdf_map.box_min_x": -7.0,
        "sdf_map.box_min_y": -15.0,
        "sdf_map.box_min_z": 0.0,
        "sdf_map.box_max_x": 7.0,
        "sdf_map.box_max_y": 15.0,
        "sdf_map.box_max_z": 1.7,
        "sdf_map.no_drone_1": False,
        "map_ros.cx": 324.0879821777344,
        "map_ros.cy": 239.10362243652344,
        "map_ros.fx": 385.69793701171875,
        "map_ros.fy": 385.69793701171875,
        "map_ros.depth_filter_maxdist": 4.6,
        "map_ros.depth_filter_mindist": 0.2,
        "map_ros.depth_filter_margin": 2,
        "map_ros.k_depth_scaling_factor": 1000.0,
        "map_ros.skip_pixel": 2,
        "map_ros.esdf_slice_height": 0.5,
        "map_ros.visualization_truncate_height": 10.09,
        "map_ros.visualization_truncate_low": -2.0,
        "map_ros.show_occ_time": False,
        "map_ros.show_esdf_time": False,
        "map_ros.show_all_map": True,
        "map_ros.report_coverage": ParameterValue(coverage_metrics, value_type=bool),
        "map_ros.coverage_report_interval": ParameterValue(
            coverage_report_interval_sec, value_type=float),
        "map_ros.frame_id": "world",
        "visualization.show_grid_text": ParameterValue(show_grid_text, value_type=bool),
        "fsm.thresh_replan1": 0.2,
        "fsm.thresh_replan2": 0.2,
        "fsm.thresh_replan3": 1.5,
        "fsm.replan_time": 0.2,
        "fsm.sync_interval": 0.2,
        "fsm.wait_delete_duration": 0.05,
        "fsm.gain_thresh": 10,
        "fsm.attempt_interval": 0.1,
        "fsm.pair_opt_interval": 0.5,
        "partitioning.min_unknown": 4000,
        "partitioning.min_frontier": 100,
        "partitioning.min_free": 3000,
        "partitioning.consistent_cost": -5.0,
        "partitioning.consistent_cost2": 8.0,
        "partitioning.w_unknown": 0.0,
        "partitioning.grid_size": 5.0,
        "partitioning.use_swarm_tf": True,
        "exploration.refine_local": True,
        "exploration.refined_num": 7,
        "exploration.refined_radius": 5.0,
        "exploration.max_decay": 0.8,
        "exploration.top_view_num": 15,
        "exploration.vm": MAX_VEL,
        "exploration.am": MAX_ACC,
        "exploration.yd": MAX_YAW_RATE,
        "exploration.ydd": 90.0 * 3.1415926 / 180.0,
        "exploration.w_dir": 1.5,
        "exploration.tsp_dir": PathJoinSubstitution([FindPackageShare("lkh_tsp_solver"), "resource"]),
        "exploration.mtsp_dir": PathJoinSubstitution([FindPackageShare("lkh_mtsp_solver"), "resource"]),
        "exploration.drone_num": DRONE_NUM,
        "exploration.drone_id": drone_id,
        "exploration.init_plan_num": 2,
        "frontier.cluster_min": 100,
        "frontier.cluster_size_xy": 2.0,
        "frontier.cluster_size_z": 10.0,
        "frontier.min_candidate_dist": 0.5,
        "frontier.min_candidate_clearance": 0.21,
        "frontier.candidate_dphi": 15.0 * 3.1415926 / 180.0,
        "frontier.candidate_rnum": 3,
        "frontier.candidate_rmin": 1.0,
        "frontier.candidate_rmax": 1.5,
        "frontier.down_sample": 3,
        "frontier.min_visib_num": 30,
        "frontier.min_view_finish_fraction": 0.2,
        "perception_utils.top_angle": 0.56125,
        "perception_utils.left_angle": 0.69222,
        "perception_utils.right_angle": 0.68901,
        "perception_utils.max_dist": 4.5,
        "perception_utils.vis_dist": 1.0,
        "heading_planner.yaw_diff": 30.0 * 3.1415926 / 180.0,
        "heading_planner.half_vert_num": 5,
        "heading_planner.lambda1": 2.0,
        "heading_planner.lambda2": 1.0,
        "heading_planner.max_yaw_rate": 10.0 * 3.1415926 / 180.0,
        "heading_planner.w": 20000.0,
        "heading_planner.weight_type": 1,
        "manager.max_vel": MAX_VEL,
        "manager.max_acc": MAX_ACC,
        "manager.max_jerk": 4.0,
        "manager.dynamic_environment": 0,
        "manager.local_segment_length": 6.0,
        "manager.clearance_threshold": 0.2,
        "manager.control_points_distance": 0.5,
        "manager.use_geometric_path": True,
        "manager.use_kinodynamic_path": True,
        "manager.use_topo_path": False,
        "manager.use_optimization": True,
        "manager.use_active_perception": True,
        "manager.min_time": True,
        "manager.relax_time1": 0.3,
        "manager.relax_time2": 1.5,
        "manager.max_yawdot": MAX_YAW_RATE,
        "search.max_tau": 0.8,
        "search.init_max_tau": 1.0,
        "search.max_vel": MAX_VEL,
        "search.vel_margin": 0.25,
        "search.max_acc": MAX_ACC,
        "search.w_time": 10.0,
        "search.horizon": 5.0,
        "search.lambda_heu": 10.0,
        "search.resolution_astar": 0.025,
        "search.time_resolution": 0.8,
        "search.margin": 0.2,
        "search.allocate_num": 100000,
        "search.check_num": 10,
        "search.optimistic": False,
        "astar.lambda_heu": 10000.0,
        "astar.resolution_astar": 0.3,
        "astar.allocate_num": 1000000,
        "astar.max_search_time": 0.001,
        "optimization.ld_smooth": 5.0,
        "optimization.ld_dist": 10.0,
        "optimization.ld_feasi": 2.0,
        "optimization.ld_start": 100.0,
        "optimization.ld_end": 0.5,
        "optimization.ld_guide": 1.5,
        "optimization.ld_waypt": 7.0,
        "optimization.ld_view": 0.0,
        "optimization.ld_time": 1.5,
        "optimization.ld_swarm": 5.0,
        "optimization.swarm_safe_dist": 1.0,
        "optimization.dist0": 0.7,
        "optimization.max_vel": MAX_VEL,
        "optimization.max_acc": MAX_ACC,
        "optimization.algorithm1": 15,
        "optimization.algorithm2": 11,
        "optimization.max_iteration_num1": 2,
        "optimization.max_iteration_num2": 2000,
        "optimization.max_iteration_num3": 200,
        "optimization.max_iteration_num4": 200,
        "optimization.max_iteration_time1": 0.0001,
        "optimization.max_iteration_time2": 0.005,
        "optimization.max_iteration_time3": 0.003,
        "optimization.max_iteration_time4": 0.003,
        "bspline.limit_vel": MAX_VEL,
        "bspline.limit_acc": MAX_ACC,
        "bspline.limit_ratio": 1.1,
    }


def drone_nodes(
    drone_id,
    init_x,
    init_y,
    coverage_metrics,
    coverage_report_interval_sec,
    show_grid_text,
):
    odom_topic = f"/state_ukf/odom_{drone_id}"
    use_external = PythonExpression([
        "'", LaunchConfiguration("external_execution_vehicle"), "' == '", str(drone_id), "'"
    ])
    legacy_execution = LaunchConfigurationNotEquals("external_execution_vehicle", str(drone_id))
    return [
        Node(
            package="exploration_manager",
            executable="exploration_node",
            name=f"exploration_node_{drone_id}",
            output="screen",
            parameters=[planner_params(
                drone_id, coverage_metrics, coverage_report_interval_sec, show_grid_text), {
                    "external_execution.enabled": ParameterValue(use_external, value_type=bool),
                    "external_execution.lead": ParameterValue(
                        LaunchConfiguration("external_execution_lead"), value_type=float),
                    "external_execution.stop_reserve": ParameterValue(
                        LaunchConfiguration("external_execution_stop_reserve"), value_type=float),
                }],
            remappings=[
                ("/odom_world", odom_topic),
                ("/map_ros/pose", f"/pcl_render_node/sensor_pose_{drone_id}"),
                ("/map_ros/depth", f"/pcl_render_node/depth_{drone_id}"),
                ("/map_ros/cloud", f"/pcl_render_node/cloud_{drone_id}"),
                ("/planning/replan", f"/planning/replan_{drone_id}"),
                ("/planning/new", f"/planning/new_{drone_id}"),
                ("/planning/bspline", f"/planning/bspline_{drone_id}"),
                ("/swarm_expl/drone_state_send", "/swarm_expl/drone_state"),
                ("/swarm_expl/drone_state_recv", "/swarm_expl/drone_state"),
                ("/swarm_expl/pair_opt_send", "/swarm_expl/pair_opt"),
                ("/swarm_expl/pair_opt_recv", "/swarm_expl/pair_opt"),
                ("/swarm_expl/pair_opt_res_send", "/swarm_expl/pair_opt_res"),
                ("/swarm_expl/pair_opt_res_recv", "/swarm_expl/pair_opt_res"),
                ("/swarm_expl/grid_tour_send", "/swarm_expl/grid_tour"),
                ("/swarm_expl/hgrid_send", "/swarm_expl/hgrid"),
                ("/multi_map_manager/chunk_stamps_send", "/multi_map_manager/chunk_stamps"),
                ("/multi_map_manager/chunk_data_send", "/multi_map_manager/chunk_data"),
                ("/multi_map_manager/chunk_stamps_recv", "/multi_map_manager/chunk_stamps"),
                ("/multi_map_manager/chunk_data_recv", "/multi_map_manager/chunk_data"),
                ("/planning/swarm_traj_recv", "/planning/swarm_traj"),
                ("/planning/swarm_traj_send", "/planning/swarm_traj"),
                ("/planning_vis/trajectory", f"/planning_vis/trajectory_{drone_id}"),
                ("/planning_vis/frontier", f"/planning_vis/frontier_{drone_id}"),
                ("/planning_vis/viewpoints", f"/planning_vis/viewpoints_{drone_id}"),
                ("/sdf_map/occupancy_all", f"/sdf_map/occupancy_all_{drone_id}"),
                ("/sdf_map/occupancy_local", f"/sdf_map/occupancy_local_{drone_id}"),
                ("/sdf_map/occupancy_local_inflate", f"/sdf_map/occupancy_local_inflate_{drone_id}"),
                ("/sdf_map/unknown", f"/sdf_map/unknown_{drone_id}"),
                ("/sdf_map/update_range", f"/sdf_map/update_range_{drone_id}"),
                ("/sdf_map/basecoor", f"/swarm_sim_tf/basecoor_{drone_id}"),
            ],
        ),
        Node(
            package="plan_manage",
            executable="traj_server",
            name=f"traj_server_{drone_id}",
            condition=legacy_execution,
            output="screen",
            parameters=[{
                "traj_server.time_forward": 1.5,
                "traj_server.pub_traj_id": 4,
                "traj_server.drone_id": drone_id,
                "traj_server.drone_num": DRONE_NUM,
                "fsm.replan_time": 0.2,
                "perception_utils.top_angle": 0.56125,
                "perception_utils.left_angle": 0.69222,
                "perception_utils.right_angle": 0.68901,
                "perception_utils.max_dist": 4.5,
                "perception_utils.vis_dist": 1.0,
            }],
            remappings=[
                ("/odom_world", odom_topic),
                ("/planning/bspline", f"/planning/bspline_{drone_id}"),
                ("planning/bspline", f"/planning/bspline_{drone_id}"),
                ("/planning/replan", f"/planning/replan_{drone_id}"),
                ("planning/replan", f"/planning/replan_{drone_id}"),
                ("/planning/new", f"/planning/new_{drone_id}"),
                ("planning/new", f"/planning/new_{drone_id}"),
                ("/position_cmd", f"/planning/pos_cmd_{drone_id}"),
                ("planning/position_cmd_vis", f"/planning/position_cmd_vis_{drone_id}"),
                ("planning/travel_traj", f"/planning/travel_traj_{drone_id}"),
            ],
        ),
        Node(
            package="lkh_mtsp_solver",
            executable="mtsp_node",
            name=f"tsp_solver_{drone_id}",
            output="screen",
            parameters=[{
                "exploration.drone_id": drone_id,
                "exploration.mtsp_dir": PathJoinSubstitution(
                    [FindPackageShare("lkh_mtsp_solver"), "resource"]),
                "exploration.problem_id": 1,
            }],
        ),
        Node(
            package="lkh_mtsp_solver",
            executable="mtsp_node",
            name=f"acvrp_solver_{drone_id}",
            output="screen",
            parameters=[{
                "exploration.drone_id": drone_id,
                "exploration.mtsp_dir": PathJoinSubstitution(
                    [FindPackageShare("lkh_mtsp_solver"), "resource"]),
                "exploration.problem_id": 2,
            }],
        ),
        Node(
            package="poscmd_2_odom",
            executable="poscmd_2_odom",
            name=f"poscmd_2_odom_{drone_id}",
            condition=legacy_execution,
            output="screen",
            parameters=[{
                "drone_id": drone_id,
                "init_x": float(init_x),
                "init_y": float(init_y),
                "init_z": 1.0,
            }],
            remappings=[
                ("~/command", f"/planning/pos_cmd_{drone_id}"),
                ("~/odometry", odom_topic),
            ],
        ),
        Node(
            package="odom_visualization",
            executable="odom_visualization",
            name=f"odom_visualization_{drone_id}",
            output="screen",
            parameters=[{
                "drone_id": drone_id,
                "drone_num": DRONE_NUM,
                "color.a": 1.0,
                "color.r": 0.0,
                "color.g": 0.0,
                "color.b": 1.0,
                "covariance_scale": 100.0,
                "robot_scale": 1.0,
            }],
            remappings=[("~/odom", odom_topic)],
        ),
        Node(
            package="local_sensing_node",
            executable="pcl_render_node",
            name=f"pcl_render_node_{drone_id}",
            output="screen",
            parameters=[{
                "cam_width": 640,
                "cam_height": 480,
                "cam_fx": 387.229,
                "cam_fy": 387.229,
                "cam_cx": 321.046,
                "cam_cy": 243.449,
                "sensing_horizon": 5.0,
                "sensing_rate": 10.0,
                "estimation_rate": 10.0,
                "map.x_size": MAP_SIZE_X,
                "map.y_size": MAP_SIZE_Y,
                "map.z_size": MAP_SIZE_Z,
                "map.init_x": float(init_x),
                "map.init_y": float(init_y),
                "map.drone_id": drone_id,
            }],
            remappings=[
                ("~/odometry", odom_topic),
                ("/pcl_render_node/depth", f"/pcl_render_node/depth_{drone_id}"),
                ("/pcl_render_node/sensor_pose", f"/pcl_render_node/sensor_pose_{drone_id}"),
                ("/pcl_render_node/odom", f"/pcl_render_node/odom_{drone_id}"),
            ],
        ),
    ]


def generate_launch_description():
    rviz_enabled = LaunchConfiguration("rviz")
    rviz_gl_vendor = LaunchConfiguration("rviz_gl_vendor")
    coverage_metrics = LaunchConfiguration("coverage_metrics")
    coverage_report_interval_sec = LaunchConfiguration("coverage_report_interval_sec")
    show_grid_text = LaunchConfiguration("show_grid_text")
    map_file = PathJoinSubstitution([FindPackageShare("map_generator"), "resource", "pillar.pcd"])
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("exploration_manager"), "config", "swarm.rviz"])

    actions = [
        DeclareLaunchArgument("external_execution_vehicle", default_value="0", choices=["0", "1", "2", "3", "4"],
                              description="One RACER vehicle delegated to the external execution adapter."),
        DeclareLaunchArgument("external_execution_lead", default_value="0.75",
                              description="Future RACER handoff target in seconds for external execution."),
        DeclareLaunchArgument("external_execution_stop_reserve", default_value="0.35",
                              description="Time reserved for stopping before an unconfirmed handoff."),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz2 with the original RACER swarm visualization config.",
        ),
        DeclareLaunchArgument(
            "rviz_gl_vendor",
            default_value="mesa",
            description="GLX vendor used by RViz2. Use mesa when the NVIDIA GLX path is unavailable.",
        ),
        DeclareLaunchArgument(
            "coverage_metrics",
            default_value="false",
            description="Log measurement-only SDF map coverage percentages.",
        ),
        DeclareLaunchArgument(
            "coverage_report_interval_sec",
            default_value="1.0",
            description="Seconds between measurement-only SDF map coverage log lines.",
        ),
        DeclareLaunchArgument(
            "show_grid_text",
            default_value="false",
            description="Show HGrid debug text labels in RViz2.",
        ),
        Node(
            package="map_generator",
            executable="map_pub",
            name="map_pub",
            output="screen",
            arguments=[map_file],
        ),
    ]

    for drone_id, init_x, init_y in [
        (1, 0.0, 0.0),
        (2, 0.0, -1.0),
        (3, 1.0, -1.0),
        (4, 1.0, 0.0),
    ]:
        actions.extend(
            drone_nodes(drone_id, init_x, init_y, coverage_metrics,
                        coverage_report_interval_sec, show_grid_text))

    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config],
            additional_env={"__GLX_VENDOR_LIBRARY_NAME": rviz_gl_vendor},
            condition=IfCondition(rviz_enabled),
        )
    )

    return LaunchDescription(actions)
