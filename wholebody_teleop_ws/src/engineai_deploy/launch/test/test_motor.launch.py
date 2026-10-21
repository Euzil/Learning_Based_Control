"""
Test individual motion controls
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
    config_file = LaunchConfiguration('config_file').perform(context)
    
    # policy
    policy_node = Node(
        package='engineai_deploy',
        executable='test_motors_node',
        name='test_motors_node',
        output='screen',
        emulate_tty=True,
        arguments=args,
        parameters=[
            {'config': config_file},
            {'dry_run': False},
            {'use_sim_time': False},
        ]
    )

    return [
        policy_node,
    ]

def generate_launch_description():
    return LaunchDescription([
        # Declare the argument so it can be overwritten
        DeclareLaunchArgument(
            'config_file', 
            default_value=os.path.join(get_package_share_directory("pm01_description"), "configs", "config.yaml"),
            description='Robot YAML config file'
        ),
        OpaqueFunction(function=policy_launch_setup)
    ])