"""
Simulation launch file for isaac policies.
"""

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    def load_policy_launch_setup(context, *args, **kwargs):
        args = []
        policy_node = Node(
            package='engineai_deploy',
            executable='load_policy_node',
            name='load_policy_node',
            output='screen',
            emulate_tty=True,
            arguments=args,
            parameters=[
                {'task': 'pm01_amp_flat_run'},      # pm01_wbc
                {'model': '20260520_145236.onnx'},  # 20260704_002329.onnx
                {'dry_run': False},
                {'use_sim_time': True},
            ]
        )
        return [policy_node]
    load_policy_launch = OpaqueFunction(function=load_policy_launch_setup)

    return LaunchDescription([
        load_policy_launch
    ])
