#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(dirname "$(realpath "$0")")
recording=${1:-"${SCRIPT_DIR}/../hand-tracking-streamer/recordings/test_01.npz"}
recording=$(realpath "${recording}")

cd "${SCRIPT_DIR}/deploy_real"

python xrobot_teleop_to_robot_w_hand.py \
    --robot unitree_g1 \
    --actual_human_height 1.6 \
    --redis_ip localhost \
    --target_fps 100 \
    --measure_fps 1 \
    --offline_hts_recording "${recording}" \
    --offline_hts_loop \
    "${@:2}"
