import glob
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    config = LaunchConfiguration('config').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    # simulation
    skin_feature_generator_node = Node(
        package='skin_feature_generator',
        executable='skin_feature_generator_node',
        name='skin_feature_generator',
        output='screen',
        emulate_tty=True,
        arguments=[config],
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )
    
    return [
        skin_feature_generator_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', 
            default_value=os.path.join(get_package_share_directory("skin_feature_generator"), "configs", "config.yaml"),
            description='Robot YAML config file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        OpaqueFunction(function=launch_setup)
    ])