import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction

def pm01_ros_bridge_setup(context, *args, **kwargs):
    robot_description_pkg = LaunchConfiguration('robot_description_pkg').perform(context)

    config_file = os.path.join(robot_description_pkg, "configs", "pm01.yaml")
    urdf_file_path = os.path.join(robot_description_pkg, 'urdf', "pm01/pm01_cylinder_collision.urdf")
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()
    
    #---------------------------------------------------------------------------
    # pm01_ros_bridge
    
    pm01_ros_bridge_node = Node(
        package="pm01_ros_bridge",
        executable="pm01_ros_bridge_node",
        name="pm01_ros_bridge",
        namespace="pm01",
        parameters=[{
            "config_file": config_file
        }],
        output="screen",
    )
    
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
        }],
        remappings=[
            ('/joint_states', '/pm01/joint_states')
        ]
    )
    
    # --------------------------------------------------------------------------
    # robot odometry
    
    odom_frame_transformer = Node(
        package="odom_frame_transformer",
        executable="odom_frame_transformer_node",
        name="odom_frame_transformer",
        parameters=[{
            "input_topic":     "/t265/pose/sample",
            "output_topic":    "/pm01/odom",
            "camera_frame":    "tracking_camera_link",
            "reference_frame": "base_link",
        }],
        output="screen",
    )

    # --------------------------------------------------------------------------
    # static TF: world -> odom  (z offset 0.8 m)

    world_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_odom_static_tf",
        arguments=["0.1", "0", "0.8", "0", "0", "0", "world", "odom"],
        output="screen",
    )

    # Return the LaunchDescription
    return [
        pm01_ros_bridge_node,
        robot_state_publisher_node,
        odom_frame_transformer,
        world_to_odom_tf,
    ]


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'robot_description_pkg', 
            default_value=get_package_share_directory("pm01_description"),
            description='Robot Package'
        ),
        OpaqueFunction(function=pm01_ros_bridge_setup)
    ])