import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory

def vis_launch_setup(context, *args, **kwargs):
    robot_description_pkg = LaunchConfiguration('robot_description_pkg').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_config_path = os.path.join(robot_description_pkg, 'rviz', 'rviz.rviz')

    # --------------------------------------------------------------------------
    # skin visualizer
    
    # skin_config_path = os.path.join(
    #     get_package_share_directory('skin_visualizer'), 'configs', 'config.yaml')
    # skin_node = Node(
    #     package='skin_visualizer',
    #     executable='visualizer_node',
    #     name='visualizer_node',
    #     output='screen',
    #     emulate_tty=True,
    #     arguments=[skin_config_path] + list(args),
    #     parameters=[
    #         {'use_sim_time': use_sim_time},
    #     ]
    # )
    
    # --------------------------------------------------------------------------
    # rviz visualization
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Return the LaunchDescription
    return [
        # skin_node,
        rviz_node,
    ]

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'robot_description_pkg', 
            default_value=get_package_share_directory("pm01_description"),
            description='Robot Package'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        OpaqueFunction(function=vis_launch_setup)
    ])