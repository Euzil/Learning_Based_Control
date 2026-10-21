"""Launch realsense2_camera node specifically for the T265."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    # Define the parameters for the T265
    t265_parameters = {
        'camera_name': 't265',
        'device_type': 't265',
        'enable_pose': 'true',         # Ensures the odometry/pose data is published
        'enable_fisheye1': 'false',    # Set to 'true' if you want the raw left camera image
        'enable_fisheye2': 'false',    # Set to 'true' if you want the raw right camera image
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            ]),
            launch_arguments=t265_parameters.items(),
        )
    ])