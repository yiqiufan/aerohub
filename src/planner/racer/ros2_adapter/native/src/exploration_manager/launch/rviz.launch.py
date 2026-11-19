from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_gl_vendor = LaunchConfiguration("rviz_gl_vendor")

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz_gl_vendor",
            default_value="mesa",
            description="GLX vendor used by RViz2. Use mesa when the NVIDIA GLX path is unavailable.",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [FindPackageShare("exploration_manager"), "config", "swarm.rviz"])
            ],
            additional_env={"__GLX_VENDOR_LIBRARY_NAME": rviz_gl_vendor},
        )
    ])
