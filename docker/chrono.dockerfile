FROM ubuntu:24.04

# ----------------- Base system -----------------
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y \
    build-essential cmake git wget curl unzip pkg-config ninja-build cmake-curses-gui \
    libglu1-mesa-dev freeglut3-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev \
    libxi-dev libglew-dev libglm-dev libboost-all-dev openmpi-bin libopenmpi-dev \
    libomp-dev liburdfdom-dev libhdf5-dev libirrlicht-dev swig libglfw3-dev libeigen3-dev \
    python3 python3-pip python3-dev python3-numpy

# ----------------- CUDA -----------------
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && \
    apt-get update && \
    apt-get -y install cuda-toolkit-12-8
ENV PATH=/usr/local/cuda-12.8/bin:$PATH
ENV LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64

# ----------------- Blaze -----------------
RUN git clone --branch v3.8.2 --depth 1 https://bitbucket.org/blaze-lib/blaze.git /opt/blaze
ENV BLAZE_DIR=/opt/blaze

# ----------------- OptiX -----------------
COPY NVIDIA-OptiX-SDK-7.7.0-linux64-x86_64.sh /tmp/
RUN mkdir -p /opt/optix && \
    chmod +x /tmp/NVIDIA-OptiX-SDK-7.7.0-linux64-x86_64.sh && \
    /tmp/NVIDIA-OptiX-SDK-7.7.0-linux64-x86_64.sh --skip-license --prefix=/opt/optix
ENV OPTIX_INSTALL_DIR=/opt/optix

# ----------------- ROS 2 Jazzy -----------------
RUN apt-get install -y software-properties-common curl gnupg lsb-release && \
    add-apt-repository universe && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb
RUN apt-get update && apt-get upgrade -y && apt-get install -y ros-dev-tools ros-jazzy-ros-base
ENV ROS_DISTRO=jazzy
ENV ROS2_WS=/opt/ros2_ws

# ----------------- Chrono ROS interfaces -----------------
WORKDIR ${ROS2_WS}/src
RUN git clone --branch main --depth 1 https://github.com/projectchrono/chrono_ros_interfaces.git
WORKDIR ${ROS2_WS}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
ENV ROS2_WS_INSTALL=${ROS2_WS}/install

# ----------------- Spectra -----------------
RUN git clone --branch develop https://github.com/yixuan/spectra.git /opt/spectra
ENV SPECTRA_DIR=/opt/spectra

# ----------------- Eigen -----------------
RUN git clone --branch 5.0.1 --depth 1 https://gitlab.com/libeigen/eigen.git /opt/eigen
RUN mkdir -p /opt/eigen/build && cd /opt/eigen/build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && \
    make -j$(nproc) install
ENV EIGEN3_INCLUDE_DIR=/usr/local/include/eigen3

# ----------------- OpenCRG -----------------
RUN git clone --branch v1.1.2 --depth 1 https://github.com/hlrs-vis/opencrg.git /opt/OpenCRG

WORKDIR /opt/OpenCRG
RUN sed -i 's/-ansi/-std=c99 -fPIC/' makefile && \
    mkdir -p obj lib && \
    make -f makefile && \
    mkdir -p /opt/OpenCRG/install/include /opt/OpenCRG/install/lib && \
    cp -r inc/* /opt/OpenCRG/install/include/ && \
    cp lib/libOpenCRG.*.a /opt/OpenCRG/install/lib/

ENV OPENCRG_INSTALL_DIR=/opt/OpenCRG/install

# ----------------- Chrono -----------------
WORKDIR /opt/chrono
RUN git clone --branch main --depth 1 https://github.com/projectchrono/chrono.git .
RUN git submodule update --init --recursive

# ----------------- Build VSG -----------------
WORKDIR /opt/chrono/contrib/build-scripts/linux
RUN chmod +x buildVSG.sh && ./buildVSG.sh /opt/vsg
RUN apt install -y vulkan-tools

ENV VSG_INSTALL_DIR=/opt/vsg
ENV CMAKE_PREFIX_PATH=$VSG_INSTALL_DIR:$CMAKE_PREFIX_PATH
ENV LD_LIBRARY_PATH=/opt/vsg/lib:$LD_LIBRARY_PATH

# ----------------- Build Chrono -----------------

WORKDIR /opt/chrono/build
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . ${ROS2_WS_INSTALL}/setup.sh && \
    cmake .. \
        -G Ninja \
        -DCH_ENABLE_MODULE_IRRLICHT:BOOL=ON \
        -DCH_ENABLE_MODULE_VSG:BOOL=ON \
        -DCH_ENABLE_MODULE_VEHICLE:BOOL=ON \
        -DCH_ENABLE_MODULE_POSTPROCESS:BOOL=ON \
        -DCH_ENABLE_MODULE_MULTICORE:BOOL=ON \
        -DCH_ENABLE_MODULE_FSI:BOOL=ON \
        -DCH_ENABLE_MODULE_FSI_SPH:BOOL=ON \
        -DCH_ENABLE_MODULE_FSI_TDPF:BOOL=ON \
        -DCH_ENABLE_MODULE_DEM:BOOL=ON \
        -DCH_ENABLE_MODULE_PARDISO_MKL:BOOL=OFF \
        -DCH_ENABLE_MODULE_CASCADE:BOOL=OFF \
        -DCH_ENABLE_MODULE_SENSOR:BOOL=ON \
        -DCH_ENABLE_MODULE_MODAL:BOOL=ON \
        -DCH_ENABLE_MODULE_MATLAB:BOOL= OFF \
        -DCH_ENABLE_MODULE_CSHARP:BOOL=ON \
        -DCH_ENABLE_MODULE_PYTHON:BOOL=ON \
        -DCH_ENABLE_MODULE_SYNCHRONO:BOOL=OFF \
        -DCH_ENABLE_MODULE_ROS:BOOL=ON \
        -DBUILD_BENCHMARKING:BOOL=ON \
        -DBUILD_TESTING:BOOL=ON \
        -DCH_ENABLE_OPENCRG:BOOL=ON \
        -DCH_USE_SENSOR_NVRTC:BOOL=ON \
        -DEIGEN3_INCLUDE_DIR:PATH=/usr/include/eigen3 \
        -DIrrlicht_ROOT:PATH=/usr \
        -Dblaze_INCLUDE_DIR:PATH=${BLAZE_DIR} \
        -DOptiX_INSTALL_DIR:PATH=${OPTIX_INSTALL_DIR} \
        -Dspectra_INCLUDE_DIR:PATH=${SPECTRA_DIR}/include \
        -DEIGEN3_INCLUDE_DIR:PATH=${EIGEN3_INCLUDE_DIR} \
        -DOpenCRG_INCLUDE_DIR:PATH=${OPENCRG_INSTALL_DIR}/include \
        -DOpenCRG_LIBRARY:PATH=${OPENCRG_INSTALL_DIR}/lib/libOpenCRG.1.1.2.a \
        -Dvsg_DIR:PATH=${VSG_INSTALL_DIR}/lib/cmake/vsg \
        -DvsgImGui_DIR:PATH=${VSG_INSTALL_DIR}/lib/cmake/vsgImGui \
        -DvsgXchange_DIR:PATH=${VSG_INSTALL_DIR}/lib/cmake/vsgXchange \
        -DCMAKE_BUILD_TYPE=Release

RUN ninja -j8 && ninja install

CMD ["/bin/bash"]
