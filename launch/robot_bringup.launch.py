"""
launch/robot_bringup.launch.py
Launches robot_state_publisher, gz bridge, SLAM Toolbox, RViz2.
Updated for full URDF with diff drive + LiDAR + IMU.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG_NAME = 'delivery_robot_sim'


def generate_launch_description():
    pkg_dir = get_package_share_directory(PKG_NAME)
    urdf_file = os.path.join(pkg_dir, 'urdf', 'delivery_robot.urdf.xacro')
    use_sim_time = {'use_sim_time': True}

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # ── 1. Robot state publisher ──────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            use_sim_time,
        ]
    )

    # ── 2. Gazebo ↔ ROS bridge ────────────────────────────
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        output='screen',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # LiDAR — Gazebo publishes as gz LaserScan
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Odometry
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # TF from diff drive plugin (odom→base_footprint)
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # cmd_vel ROS → Gazebo
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # IMU
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Joint states
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        parameters=[use_sim_time]
    )

    # ── 3. SLAM Toolbox ───────────────────────────────────
    slam_params = os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml')

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, use_sim_time],
        remappings=[('/scan', '/scan')]
    )

    # ── 4. RViz2 ─────────────────────────────────────────
    rviz_config = os.path.join(pkg_dir, 'config', 'slam_mapping.rviz')
    rviz_args = ['-d', rviz_config] if os.path.exists(rviz_config) else []

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args,
        parameters=[use_sim_time]
    )

    return LaunchDescription([
        robot_state_publisher,
        gz_bridge,
        TimerAction(period=2.0, actions=[slam_toolbox]),
        TimerAction(period=3.0, actions=[rviz]),
    ])
