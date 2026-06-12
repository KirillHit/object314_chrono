# syntax = devthefuture/dockerfile-x
# The INCLUDE directive is provided by the devthefuture/dockerfile-x project

FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV CHRONO_VERSION="10.0.0"
ENV CUDA_VERSION="12-8"
ENV ROS_DISTRO="jazzy"
ENV CHRONO_REPO="https://github.com/projectchrono/chrono.git"
ENV CHRONO_DIR="/opt/chrono/src"
ENV CHRONO_INSTALL_DIR="/opt/chrono/install"
ENV PACKAGE_DIR="/opt/chrono/packages"
ENV ROS_INSTALL_PREFIX="/opt/ros/${ROS_DISTRO}"
ENV WORKSPACE_DIR="/workspace"

RUN apt-get update && \
    apt-get upgrade -y && \
    rm -rf /var/lib/apt/lists/*

WORKDIR ${WORKSPACE_DIR}

INCLUDE ./python.dockerfile
INCLUDE ./cuda.dockerfile
INCLUDE ./ros.dockerfile
INCLUDE ./chrono.dockerfile

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
