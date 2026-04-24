"""Launch Gazebo Harmonic with the warehouse world."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("delivery_robot_sim")
    world_file = PathJoinSubstitution([pkg_share, "worlds", "warehouse_v2.sdf"])

    paused = LaunchConfiguration("paused")
    verbose = LaunchConfiguration("verbose")

    gazebo_paused = ExecuteProcess(
        cmd=["gz", "sim", world_file],
        output="screen",
        condition=IfCondition(
            PythonExpression([paused, " == 'true' and ", verbose, " == 'false'"])
        ),
    )

    gazebo_running = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen",
        condition=IfCondition(
            PythonExpression([paused, " == 'false' and ", verbose, " == 'false'"])
        ),
    )

    gazebo_verbose_paused = ExecuteProcess(
        cmd=["gz", "sim", "-v", "4", world_file],
        output="screen",
        condition=IfCondition(
            PythonExpression([paused, " == 'true' and ", verbose, " == 'true'"])
        ),
    )

    gazebo_verbose_running = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-v", "4", world_file],
        output="screen",
        condition=IfCondition(
            PythonExpression([paused, " == 'false' and ", verbose, " == 'true'"])
        ),
    )

    bridge = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ros_gz_bridge",
            "parameter_bridge",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "paused",
                default_value="false",
                description="Start Gazebo paused.",
            ),
            DeclareLaunchArgument(
                "verbose",
                default_value="false",
                description="Enable verbose Gazebo logging.",
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=pkg_share,
            ),
            gazebo_paused,
            gazebo_running,
            gazebo_verbose_paused,
            gazebo_verbose_running,
            bridge,
        ]
    )
