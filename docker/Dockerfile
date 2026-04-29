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
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-opencv \
        python3-numpy \
        python3-pip \
        v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# ── Arduino Bridge Python client ─────────────────────────────────────────────
# Used by motor_bridge_node to call set_motors() on the MCU over the internal
# UART. If this install fails the node falls back to dry_run mode and logs
# the failure — the rest of the stack still works (controller, camera, sim).
# TODO: confirm the exact package name once the Arduino UNO Q SDK is finalised.
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
ENTRYPOINT ["/bin/bash", "-c", \
    "source /opt/ros/jazzy/setup.bash && \
     source /ros2_ws/install/setup.bash && \
     ros2 launch sancho_bringup sancho_launch.py"]
