#!/usr/bin/env bash
# Fixed-base upper-body tracking with the existing robot camera stream.
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}" ROS_LOCALHOST_ONLY=1
video_python="${QUEST_VIDEO_PYTHON:-/home/devel/quest-video-venv/bin/python}"
if ! "$video_python" -c 'import mujoco, numpy, yaml, rclpy, aiortc, aiohttp'; then
    echo "Teleop dependencies missing. Run inside this container: bash \"$workspace/scripts/setup_quest_teleop.sh\"" >&2
    exit 1
fi
runtime_dir="$(mktemp -d)"
pids=()
cleanup() {
    trap - EXIT INT TERM
    for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
    # WebRTC shutdown can wait on a peer; bound cleanup of our own children.
    for _ in {1..30}; do
        alive=0
        for pid in "${pids[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then alive=1; fi
        done
        if [ "$alive" -eq 0 ]; then break; fi
        sleep 0.1
    done
    for pid in "${pids[@]}"; do kill -KILL "$pid" 2>/dev/null || true; done
    wait 2>/dev/null || true
    rm -rf "$runtime_dir"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
"$video_python" "$workspace/scripts/prepare_quest_teleop.py" "$runtime_dir"
# Fail on invalid personal calibration before starting simulation/video processes.
"$video_python" "$workspace/scripts/quest_retarget.py" --config "$runtime_dir/config.yaml" "$@" --check-profile
bridge_args=()
video_args=()
for arg in "$@"; do
    if [[ "$arg" == "--check-profile" ]]; then exit 0; fi
    if [[ "$arg" == "--calibrate" || "$arg" == --calibrate=* ]]; then
        # Per-launch marker, never the output profile (which may already exist).
        bridge_args=(--tracking-ready-file "$runtime_dir/tracking-ready")
        video_args=(--wait-for-tracking "$runtime_dir/tracking-ready")
    fi
done
"$workspace/install/mujoco_simulation/lib/mujoco_simulation/mujoco_simulator" "$runtime_dir/config.yaml" &
pids+=("$!")
"$video_python" "$workspace/scripts/quest_retarget.py" --config "$runtime_dir/config.yaml" "$@" "${bridge_args[@]}" &
pids+=("$!")
"$video_python" "$workspace/scripts/quest_video_server.py" "${video_args[@]}" &
pids+=("$!")
wait -n "${pids[@]}"
