#!/usr/bin/env bash
# Restore the complete video and retargeting environment inside the container.
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$workspace/scripts/setup_quest_video.sh"
/home/devel/quest-video-venv/bin/python -m pip install -r "$workspace/scripts/requirements-quest-teleop.txt"
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
/home/devel/quest-video-venv/bin/python -c 'import mujoco, numpy, yaml, rclpy, aiortc, aiohttp; from interface_protocol.msg import JointCommand; print("Quest teleop dependencies ready.")'
