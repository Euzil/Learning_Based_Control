#!/usr/bin/env bash
# Display-only simulation: no policy or robot command publisher is started.
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"
export ROS_LOCALHOST_ONLY=1
video_python="${QUEST_VIDEO_PYTHON:-/home/devel/quest-video-venv/bin/python}"
if ! "$video_python" -c 'import aiortc, aiohttp, rclpy' >/dev/null 2>&1; then
    echo "Video dependencies missing. See QUEST_SIM_VIDEO.md." >&2
    exit 1
fi
sim_pid=""
video_pid=""
cleanup() {
    trap - EXIT INT TERM
    for pid in "$sim_pid" "$video_pid"; do
        if [ -n "$pid" ]; then kill -TERM "$pid" 2>/dev/null || true; fi
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
# Run the installed executable directly so cleanup owns the actual process.
"$workspace/install/mujoco_simulation/lib/mujoco_simulation/mujoco_simulator" \
    "$workspace/src/robots/pm01_description/configs/pm01.yaml" &
sim_pid=$!
"$video_python" "$workspace/scripts/quest_video_server.py" "$@" &
video_pid=$!
wait -n "$sim_pid" "$video_pid"
