#!/usr/bin/env bash
# Run inside the development container as its normal user.
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if ! dpkg-query -W -f='${Status}' python3.10-venv 2>/dev/null | grep -q 'install ok installed'; then
    if [[ "$EUID" -eq 0 ]]; then
        apt-get update
        apt-get install -y python3.10-venv
    else
        sudo apt-get update
        sudo apt-get install -y python3.10-venv
    fi
fi
python3 -m venv --system-site-packages /home/devel/quest-video-venv
/home/devel/quest-video-venv/bin/python -m pip install -r "$workspace/scripts/requirements-quest-video.txt"
source /opt/ros/humble/setup.bash
/home/devel/quest-video-venv/bin/python -c 'import rclpy, aiortc, aiohttp, numpy; print("Video dependencies ready.")'
