import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory


def _find_joystick():
    """Find the joystick"""
    import re
    for dev in sorted(os.listdir('/dev/input')):
        if not dev.startswith('js'):
            continue
        try:
            with open(f'/sys/class/input/{dev}/device/uevent') as f:
                uevent = f.read()
            m = re.search(r'^KEY=([0-9a-f ]+)', uevent, re.M)
            if not m:
                continue
            total = 0
            for word in m.group(1).strip().split():
                total = (total << 64) | int(word, 16)
            # count buttons in BTN_GAMEPAD range (0x130-0x13F); real gamepads have >= 4
            gamepad_buttons = bin((total >> 0x130) & 0xFFFF).count('1')
            if gamepad_buttons >= 4:
                return f'/dev/input/{dev}'
        except OSError:
            pass
    return None


def mujoco_launch_setup(context, *args, **kwargs):
    config = LaunchConfiguration('config').perform(context)
    joy_dev = _find_joystick() or '/dev/input/js0'

    # simulation
    mujoco_node = Node(
        package='mujoco_simulation',
        executable='mujoco_simulator',
        name='mujoco_simulator',
        output='screen',
        emulate_tty=True,
        arguments=[config],
        parameters=[
            {'use_sim_time': True},
        ]
    )
    
    # joystick
    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_publisher_node',
        output='screen',
        parameters=[{
            'dev': joy_dev,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0 
        }]
    )

    return [
        mujoco_node,
        joy_node
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', 
            default_value=os.path.join(get_package_share_directory("pm01_description"), "configs", "config.yaml"),
            description='Robot YAML config file'
        ),
        OpaqueFunction(function=mujoco_launch_setup)
    ])