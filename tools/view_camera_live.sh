#!/usr/bin/env bash
# view_camera_live.sh — open rqt_image_view on the rover's perception topics
# while the stack is running, with the window forwarded back to your laptop.
#
# WHY THIS EXISTS
#   camera_node already publishes /camera/debug_view (BGR + overlay) and
#   /camera/mask_view (binary mask) at all times. They're invisible because the
#   ROS 2 container runs headless (no X) and on --network rover1_default (DDS is
#   private to the bridge net, so you can't subscribe from the laptop directly).
#   This script starts a *second, throwaway* container on the SAME Docker
#   network and ROS domain, with the X socket mounted, and runs the viewer
#   there — so it sees the topics and its window can reach your screen.
#
# PREREQUISITES
#   1. Laptop (Linux): connect with X forwarding:
#          ssh -X giacomo@<uno-q-ip>          (use -Y if -X is refused)
#   2. The rover stack must already be running (`docker ps` shows a container
#      from sancho_rover:latest).
#   3. Once per UNO Q login, allow local containers to use the X server:
#          xhost +local:
#
# USAGE  (run this ON the UNO Q, inside the `ssh -X` session)
#   ./tools/view_camera_live.sh                  # /camera/debug_view (default)
#   ./tools/view_camera_live.sh /camera/mask_view
#
# The rover keeps running untouched; closing the window or Ctrl+C only stops
# this viewer container (--rm cleans it up).
set -euo pipefail

TOPIC="${1:-/camera/debug_view}"
NETWORK="${SANCHO_NETWORK:-rover1_default}"   # override if you renamed the app
IMAGE="sancho_rover:latest"

# DISPLAY is set by ssh -X (typically localhost:10.0). For the container to
# reach the same X client we forward DISPLAY and the host's X socket.
if [[ -z "${DISPLAY:-}" ]]; then
    echo "ERROR: \$DISPLAY is empty — did you connect with 'ssh -X'?" >&2
    exit 1
fi

echo "Viewer: topic=${TOPIC}  network=${NETWORK}  DISPLAY=${DISPLAY}"
echo "(Close the window or Ctrl+C to stop; the rover stack is unaffected.)"

# A fresh container on the rover's DDS network. We override the image's
# ENTRYPOINT (which would launch the whole stack) and run only the viewer.
docker run --rm -it \
    --network "${NETWORK}" \
    --env DISPLAY \
    --env ROS_DOMAIN_ID \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
    --volume "${HOME}/.Xauthority:/root/.Xauthority:ro" \
    --entrypoint /bin/bash \
    "${IMAGE}" \
    -c "source /opt/ros/jazzy/setup.bash \
        && source /ros2_ws/install/setup.bash \
        && ros2 run rqt_image_view rqt_image_view ${TOPIC}"
