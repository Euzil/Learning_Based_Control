"""
Simulation launch file for isaac policies.
"""

import os

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory


def policy_launch_setup(context, *args, **kwargs):
    robot_description_pkg = LaunchConfiguration('robot_description_pkg').perform(context)


    # --------------------------------------------------------------------------
    # t265
    # Runs the visual odometry

    odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments=[
            ('camera_name', 't265'),
            ('device_type', 't265'),
            ('enable_pose', 'true'),         # Ensures the odometry/pose data is published
            ('enable_fisheye1', 'false'),    # Set to 'true' if you want the raw left camera image
            ('enable_fisheye2', 'false'),    # Set to 'true' if you want the raw right camera image
        ]
    )
    

    # --------------------------------------------------------------------------
    # pm01_ros_bridge
    # Sets up ros topics of odmetry, sensors, tf tree, etc.
    
    pm01_ros_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pm01_ros_bridge'),
                'launch',
                'pm01_ros_bridge.launch.py'
            ])
        ]),
        launch_arguments=[
            ('robot_description_pkg', robot_description_pkg),
        ]
    )

    return [
        odometry,
        pm01_ros_bridge,
    ]


def generate_launch_description():
    return LaunchDescription([
        # Declare the argument so it can be overwritten
        DeclareLaunchArgument(
            'robot_description_pkg', 
            default_value=get_package_share_directory("pm01_description"),
            description='Robot Package'
        ),
        OpaqueFunction(function=policy_launch_setup)
    ])