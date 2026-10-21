import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. Find the official realsense2_camera package
    realsense_dir = get_package_share_directory('realsense2_camera')
    rs_launch_file = os.path.join(realsense_dir, 'launch', 'rs_launch.py')

    target_serial_no = '_243122070645'

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch_file),
        launch_arguments={
            'serial_no': target_serial_no,
            'camera_name': 'torso', 
            # 'rgb_camera.color_profile': '848x480x30',
            # 'depth_module.depth_profile': '848x480x30',
        }.items()
    )

    return LaunchDescription([
        realsense_node
    ])