# SANCHO deployment image — runs on the QRB2210 MPU of the Arduino UNO Q.
#
# Build (any host arch, builds natively for the host platform):
#     docker build -f docker/Dockerfile -t sancho_rover:latest .
#
# Cross-build for the rover (arm64) from an x86_64 laptop:
#     docker buildx build --platform linux/arm64 \
#         -f docker/Dockerfile -t sancho_rover:latest --load .
#
# Run on the rover:
#     docker run --rm -it \
#         --device /dev/video0 \
#         --device /dev/ttyACM0 \
#         --network host \
#         sancho_rover:latest
#
# At startup the entrypoint launches the full ROS 2 stack
# (camera_node + controller_node + motor_bridge_node) with parameters
# from sancho_bringup/config/sancho_params.yaml.
#
# Not used during development on the laptop — there we build and run natively.

FROM ros:jazzy-ros-base-noble

# ── system dependencies ──────────────────────────────────────────────────────
# python3-opencv : used by camera_node and sim_node
# python3-numpy  : transitive but pinned explicitly to avoid pip pulling its own
# python3-pip    : needed for the Arduino Bridge Python client (next step)
# v4l-utils      : v4l2-ctl, handy for camera diagnostics inside the container
# rqt-image-view : live viewer for /camera/debug_view & /camera/mask_view over
#                  X forwarding (see docs/deployment.md §5.5). Pulls in the Qt/X
#                  client libs absent from ros-base; ~adds a few hundred MB but
#                  only used interactively, never on the autonomous path.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-opencv \
        python3-numpy \
        python3-pip \
        v4l-utils \
        ros-jazzy-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

# ── Python dependencies ───────────────────────────────────────────────────────
# onnxruntime: inferenza YOLOv8 instance segmentation locale (camera_node)
RUN pip3 install --no-cache-dir --break-system-packages onnxruntime

# arduino-bridge: usato da motor_bridge_node per chiamare set_motors() sull'MCU.
# Se non disponibile il nodo cade in dry_run automaticamente.
RUN pip3 install --no-cache-dir --break-system-packages arduino-bridge || \
    echo "WARNING: arduino-bridge not installed; motor_bridge_node will run in dry_run."

# ── workspace build ──────────────────────────────────────────────────────────
WORKDIR /ros2_ws
COPY ros2_ws/src ./src

# Resolve declared deps from package.xml and build all four packages.
RUN . /opt/ros/jazzy/setup.sh && \
    apt-get update && \
    rosdep update --rosdistro jazzy && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/* && \
    colcon build --symlink-install

# ── entrypoint ───────────────────────────────────────────────────────────────
# Source the ROS 2 underlay + our workspace overlay, then launch the full stack.
# Any extra args on `docker run image_name <args>` are forwarded to ros2 launch
# as launch arguments, e.g. `docker run ... sancho_rover:latest wheel_separation:=0.45`
# overrides the wheel_separation parameter for that run only.
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && exec ros2 launch sancho_bringup trail_test_launch.py \"$@\"", "--"]
