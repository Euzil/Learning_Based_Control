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
    
    config_file = os.path.join(robot_description_pkg, "configs", "real.yaml")
    
    # --------------------------------------------------------------------------
    # engineai_deploy
    # Runs the robot policy 
    
    policy_node = Node(
        package='engineai_deploy',
        executable='multi_policy_runner_node',
        name='policy_node',
        output='screen',
        emulate_tty=True,
        arguments=args,
        parameters=[
            {'config': config_file},
            {'tasks': ['pm01_amp_flat_walk', 'pm01_amp_flat_run']},
            {'models': ['20260602_141905.onnx', '20260520_145236.onnx']},
            {'default_task': 'pm01_amp_flat_walk'},
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
            'robot_description_pkg', 
            default_value=get_package_share_directory("pm01_description"),
            description='Robot Package'
        ),
        OpaqueFunction(function=policy_launch_setup)
    ])