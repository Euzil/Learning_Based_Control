import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory

def vis_launch_setup(context, *args, **kwargs):
    robot_description_pkg = LaunchConfiguration('robot_description_pkg').perform(context)
    urdf_file = LaunchConfiguration('urdf_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    # build paths
    config_file = os.path.join(robot_description_pkg, "configs", "config.yaml")
    urdf_file_path = os.path.join(robot_description_pkg, 'urdf', urdf_file)
    rviz_config_path = os.path.join(robot_description_pkg, 'rviz', 'rviz.rviz')
    
    # Read urdf into str
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()
    
    # --------------------------------------------------------------------------
    # robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'publish_frequency': 50.0,
            'use_sim_time': use_sim_time
        }]
    )
    
    # fake joint state publisher (fake robot)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )
    
    
    # --------------------------------------------------------------------------
    # skin visualizer
    skin_config_path = os.path.join(
        get_package_share_directory('skin_visualizer'), 'configs', 'config.yaml')
    skin_node = Node(
        package='skin_visualizer',
        executable='visualizer_node',
        name='visualizer_node',
        output='screen',
        emulate_tty=True,
        arguments=[skin_config_path] + list(args),
        parameters=[
            {'use_sim_time': use_sim_time},
        ]
    )
    
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
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        skin_node,
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
            'urdf_file', 
            default_value='pm01_cylinder_collision.urdf',
            description='Robot URDF'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        OpaqueFunction(function=vis_launch_setup)
    ])