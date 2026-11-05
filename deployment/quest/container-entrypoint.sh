#!/usr/bin/env bash
set -eo pipefail
workspace=/workspaces/Learning_Based_Control/wholebody_teleop_ws
source /opt/ros/humble/setup.bash
cd "$workspace"
action="${1:-run}"
if [ "$#" -gt 0 ]; then shift; fi
case "$action" in
  build)
    # Build only the simulation and its ROS message/plugin dependencies.
    export CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS:-2}"
    colcon build --symlink-install --packages-up-to mujoco_simulation pm01_description \
      --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash
    "$QUEST_VIDEO_PYTHON" -c 'from interface_protocol.msg import JointCommand; import rclpy, mujoco, aiortc, onnxruntime; print("Build and imports OK")'
    ;;
  receiver)
    exec python3 ../hand-tracking-streamer/scripts/sockets.py \
      --protocol tcp --host "${TRACKING_HOST:-0.0.0.0}" --port "${TRACKING_PORT:-8000}" \
      --forward-port "${FORWARD_PORT:-9010}" --tally "$@"
    ;;
  run)
    test -f install/setup.bash || { echo 'First run: docker compose run --rm build' >&2; exit 1; }
    : "${DISPLAY:?Configure the server NVIDIA Xorg display in .env}"
    : "${XAUTHORITY:?Mount the Xorg authorization file}"
    test -r "$XAUTHORITY" || { echo 'Cannot read XAUTHORITY' >&2; exit 1; }
    info="$(glxinfo -B)"
    echo "$info"
    grep -q 'OpenGL vendor string: NVIDIA' <<< "$info" || {
      echo 'Expected NVIDIA OpenGL. Check the server headless Xorg setup.' >&2; exit 1;
    }
    source install/setup.bash
    exec bash scripts/run_quest_teleop.sh "$@"
    ;;
  *) exec "$action" "$@" ;;
esac
