# Install ROS 2 Jazzy following the official Ubuntu deb packages flow.

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        ca-certificates \
        locales \
        curl \
        software-properties-common && \
    rm -rf /var/lib/apt/lists/*

RUN add-apt-repository universe

RUN ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install --no-install-recommends -y \
        python3-colcon-common-extensions \
        ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        ros-${ROS_DISTRO}-ros-base && \
    rm -rf /var/lib/apt/lists/*
