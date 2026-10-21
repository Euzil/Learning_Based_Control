"""
Simulation launch file for isaac policies.
"""

import os

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory


def policy_launch_setup(context, *args, **kwargs):
    robot_description_pkg = LaunchConfiguration('robot_description_pkg').perform(context)
    urdf_file = LaunchConfiguration('urdf_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # build paths
    config_file = os.path.join(robot_description_pkg, "configs", "pm01.yaml")
    
 
    # --------------------------------------------------------------------------
    # policy
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
            {'models': ['2026-09-27_22-30-26.onnx', '2026-09-24_11-03-04.onnx']},
            {'default_task': 'pm01_amp_flat_walk'},
            {'dry_run': False},
            {'use_sim_time': use_sim_time},
        ]
    )

    # --------------------------------------------------------------------------
    # simulation
    simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mujoco_simulation'),
                'launch',
                'mujoco_simulator.launch.py'
            ])
        ]),
        launch_arguments=[
            ('config', config_file),
        ]
    )
    
    # --------------------------------------------------------------------------
    # skin
    skin_features = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('skin_feature_generator'),
                'launch',
                'skin_feature_generator.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # --------------------------------------------------------------------------
    # visualization
    # vis_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('engineai_deploy'),
    #             'launch',
    #             'vis.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'robot_description_pkg': robot_description_pkg, 
    #         'urdf_file': urdf_file,
    #         'use_sim_time': use_sim_time
    #     }.items()
    # )
    
    return [
        policy_node,
        skin_features,
        simulator_launch,
        # vis_launch,
    ]

def generate_launch_description():
    return LaunchDescription([
        # Declare the arguments
        DeclareLaunchArgument(
            'robot_description_pkg', 
            default_value=get_package_share_directory("pm01_description"),
            description='Robot Package'
        ),
        DeclareLaunchArgument(
            'urdf_file', 
            default_value='pm01/pm01_cylinder_collision.urdf',
            description='Robot URDF'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        OpaqueFunction(function=policy_launch_setup)
    ])