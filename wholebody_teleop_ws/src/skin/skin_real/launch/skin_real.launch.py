import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory


def skin_launch_setup(context, *args, **kwargs):
    config = LaunchConfiguration('config').perform(context)

    # simulation
    skin_real_node = Node(
        package='skin_real',
        executable='skin_real_node',
        name='skin_real_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'config': config}],
    )
    return [skin_real_node]


def generate_launch_description():
    return LaunchDescription([
        # Declare the argument so it can be overwritten
        DeclareLaunchArgument(
            'config', 
            default_value=os.path.join(get_package_share_directory("skin_real"), "configs", "config.yaml"),
            description='Robot YAML config file'
        ),
        OpaqueFunction(function=skin_launch_setup)
    ])