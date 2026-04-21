# Deployment image for the QRB2210 (arm64) on the Arduino UNO Q.
# Not used during development — build and run natively on the host for dev work.
FROM ros:jazzy-ros-base-noble

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-opencv \
        python3-pip \
        ros-jazzy-launch-ros \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY ros2_ws/ .

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --packages-select sancho_rover

ENTRYPOINT ["/bin/bash", "-c", \
    "source /opt/ros/jazzy/setup.bash && \
     source /ros2_ws/install/setup.bash && \
     ros2 launch sancho_rover sancho.launch.py"]
