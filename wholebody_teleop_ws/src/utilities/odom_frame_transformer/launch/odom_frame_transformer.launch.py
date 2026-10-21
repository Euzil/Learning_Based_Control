from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("input_topic",     default_value="odom"),
        DeclareLaunchArgument("output_topic",    default_value="odom_transformed"),
        DeclareLaunchArgument("camera_frame",    default_value="camera"),
        DeclareLaunchArgument("reference_frame", default_value="map"),

        Node(
            package="odom_frame_transformer",
            executable="odom_frame_transformer_node",
            name="odom_frame_transformer",
            parameters=[{
                "input_topic":     LaunchConfiguration("input_topic"),
                "output_topic":    LaunchConfiguration("output_topic"),
                "camera_frame":    LaunchConfiguration("camera_frame"),
                "reference_frame": LaunchConfiguration("reference_frame"),
            }],
            output="screen",
        ),
    ])
