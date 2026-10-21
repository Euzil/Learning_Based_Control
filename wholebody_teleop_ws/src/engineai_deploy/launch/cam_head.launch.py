import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. Find the official realsense2_camera package
    realsense_dir = get_package_share_directory('realsense2_camera')
    rs_launch_file = os.path.join(realsense_dir, 'launch', 'rs_launch.py')

    target_serial_no = '_243222076591'

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch_file),
        launch_arguments={
            'serial_no': target_serial_no,
            'camera_name': 'head', 
            'initial_reset': 'true',
            'rgb_camera.color_profile': '848x480x30',
            'auto_exposure_priority': 'false',
            # Disable the depth stream
            'enable_depth': 'false',
            # Disable IR streams
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            # Optional: Disable the IMU
            'enable_accel': 'false',
            'enable_gyro': 'false',
        }.items()
    )

    return LaunchDescription([
        realsense_node
    ])