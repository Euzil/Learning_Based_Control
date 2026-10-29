#!/usr/bin/env bash
# Quest tracking: fixed base, Walk/Run overrides, or WBC references.
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}" ROS_LOCALHOST_ONLY=1
video_python="${QUEST_VIDEO_PYTHON:-/home/devel/quest-video-venv/bin/python}"
policy_mode=""
model_path=""
policy_velocity=(0 0 0)
velocity_set=0
retarget_args=()
select_mode() {
    if [ -n "$policy_mode" ] && [ "$policy_mode" != "$1" ]; then
        echo 'Choose only one of --walk, --run, --wbc per launch.' >&2; exit 2
    fi
    policy_mode="$1"
}
while [ "$#" -gt 0 ]; do
    case "$1" in
        --walk|--run|--wbc) select_mode "${1#--}"; shift ;;
        --walk-model|--run-model|--wbc-model)
            mode="${1#--}"; select_mode "${mode%-model}"
            model_path="${2:?Model option needs a path}"; shift 2 ;;
        --model) model_path="${2:?--model needs a path}"; shift 2 ;;
        --walk-velocity|--run-velocity|--wbc-velocity|--velocity)
            if [ "$1" != '--velocity' ]; then
                mode="${1#--}"; select_mode "${mode%-velocity}"
            fi
            if [ "$#" -lt 4 ]; then echo 'Velocity needs VX VY YAW_RATE' >&2; exit 2; fi
            policy_velocity=("$2" "$3" "$4"); velocity_set=1; shift 4 ;;
        --help|-h)
            cat <<'HELP'
Usage: bash scripts/run_quest_teleop.sh [--walk|--run|--wbc] [options]
No mode: original fixed-base Quest tracking.
--walk / --run       Locomotion policy with Quest upper-body overrides.
--wbc               WBC policy with Quest references as policy inputs.
--model PATH        Select ONNX file for the chosen mode.
--walk-model PATH / --run-model PATH / --wbc-model PATH
                    Select mode and ONNX file together (existing aliases).
--velocity VX VY YAW_RATE   Default 0 0 0; mode-specific aliases also accepted.
--body-profile PATH, --port PORT, --check-profile and other Quest options are forwarded.
HELP
            exit 0 ;;
        *) retarget_args+=("$1"); shift ;;
    esac
done
if { [ -n "$model_path" ] || [ "$velocity_set" -eq 1 ]; } && [ -z "$policy_mode" ]; then
    echo '--model/--velocity requires --walk, --run or --wbc.' >&2; exit 2
fi
if [ -z "$model_path" ]; then
    case "$policy_mode" in
        walk) model_path="$workspace/models/pm01_amp_flat_walk/2026-09-27_22-30-26.onnx" ;;
        run) model_path="$workspace/models/pm01_amp_flat_run/2026-09-24_11-03-04.onnx" ;;
        wbc) model_path="$workspace/models/pm01_wbc/20260721_211647.onnx" ;;
    esac
fi
set -- "${retarget_args[@]}"
if ! "$video_python" -c 'import mujoco, numpy, yaml, rclpy, aiortc, aiohttp'; then
    echo "Teleop dependencies missing. Run inside this container: bash \"$workspace/scripts/setup_quest_teleop.sh\"" >&2
    exit 1
fi
prepare_args=()
if [ -n "$policy_mode" ]; then
    "$video_python" -c 'import onnxruntime' || {
        echo 'Install scripts/requirements-quest-teleop.txt to enable policies.' >&2; exit 1;
    }
    prepare_args=("--$policy_mode-model" "$model_path")
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
"$video_python" "$workspace/scripts/prepare_quest_teleop.py" "$runtime_dir" "${prepare_args[@]}"
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
if [ -n "$policy_mode" ]; then
    "$video_python" "$workspace/scripts/quest_policy.py" --config "$runtime_dir/config.yaml" \
        --velocity "${policy_velocity[@]}" &
    pids+=("$!")
fi
"$workspace/install/mujoco_simulation/lib/mujoco_simulation/mujoco_simulator" "$runtime_dir/config.yaml" &
pids+=("$!")
"$video_python" "$workspace/scripts/quest_retarget.py" --config "$runtime_dir/config.yaml" "$@" "${bridge_args[@]}" &
pids+=("$!")
"$video_python" "$workspace/scripts/quest_video_server.py" "${video_args[@]}" &
pids+=("$!")
wait -n "${pids[@]}"
