#!/usr/bin/env bash
# Run on the desktop host after login or container recreation.
# Share only this desktop's X11 cookie, without opening X access to all clients.
set -euo pipefail
container="${1:-wholebody_teleop_ws_devcont}"
: "${DISPLAY:?Run this script from the desktop session}"
command -v xauth >/dev/null
cookie="$(mktemp)"
trap 'rm -f "$cookie"' EXIT
chmod 600 "$cookie"
xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$cookie" nmerge -
if [ ! -s "$cookie" ]; then
    echo 'No X11 authorization cookie found for the current display.' >&2
    exit 1
fi
docker cp "$cookie" "$container:/tmp/quest-display.Xauthority" >/dev/null
docker exec "$container" sh -c 'chown devel:devel /tmp/quest-display.Xauthority && chmod 600 /tmp/quest-display.Xauthority'
echo "Desktop rendering authorized for $container on $DISPLAY."
