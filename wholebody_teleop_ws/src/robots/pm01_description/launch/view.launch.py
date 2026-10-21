import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    """
    Launch a gui to move urdf
    """
    
    pkg_share = get_package_share_directory('pm01_description')
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'pm01_cylinder_collision.urdf')    
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'pm01.rviz')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation'
    )

    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file_path]), 
        value_type=str
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        use_sim_time_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])