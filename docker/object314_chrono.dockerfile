FROM kirillhit/project_chrono:10.0.0

RUN apt update && apt install -y build-essential cmake

WORKDIR /workspace

# ROS interfaces used by the Object314 simulator and the NMPC client.
COPY third_party/vehicle_nmpc/ros_packages /workspace/ros2_ws/src

RUN . /opt/ros/jazzy/setup.sh \
    && cd /workspace/ros2_ws \
    && colcon build --symlink-install --packages-select vehicle_nmpc_interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release

# Object314 Chrono model and executables.
COPY src /workspace/object314/src

RUN . /opt/ros/jazzy/setup.sh \
    && . /workspace/ros2_ws/install/setup.sh \
    && cmake -S /workspace/object314/src -B /workspace/object314/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${CHRONO_INSTALL_DIR}" \
    && cmake --build /workspace/object314/build --target install --parallel "$(nproc)"

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
