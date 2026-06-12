# Chrono::ROS dependencies.

ENV ROS_WORKSPACE_DIR="/opt/chrono/ros2_ws"
ENV CHRONO_ROS_INTERFACES_DIR="${ROS_WORKSPACE_DIR}/src/chrono_ros_interfaces"

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        ros-${ROS_DISTRO}-ament-index-cpp \
        ros-${ROS_DISTRO}-geometry-msgs \
        ros-${ROS_DISTRO}-rcl-interfaces \
        ros-${ROS_DISTRO}-rclcpp \
        ros-${ROS_DISTRO}-sensor-msgs \
        ros-${ROS_DISTRO}-std-msgs \
        ros-${ROS_DISTRO}-tf2-msgs \
        ros-${ROS_DISTRO}-tf2-ros && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p ${CHRONO_ROS_INTERFACES_DIR} && \
    git clone --depth 1 https://github.com/projectchrono/chrono_ros_interfaces.git ${CHRONO_ROS_INTERFACES_DIR} && \
    cd ${ROS_WORKSPACE_DIR} && \
    . ${ROS_INSTALL_PREFIX}/setup.sh && \
    colcon build --symlink-install --packages-select chrono_ros_interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release

ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_ROS=ON \
    -Dchrono_ros_interfaces_DIR=${ROS_WORKSPACE_DIR}/install/chrono_ros_interfaces/share/chrono_ros_interfaces/cmake"
ENV PRE_BUILD_SCRIPTS="${PRE_BUILD_SCRIPTS} . ${ROS_INSTALL_PREFIX}/setup.sh && . ${ROS_WORKSPACE_DIR}/install/setup.sh &&"
