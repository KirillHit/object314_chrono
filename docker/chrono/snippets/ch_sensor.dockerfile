# Chrono::Sensor dependencies.

ARG OPTIX_SCRIPT

RUN if [ -z "${OPTIX_SCRIPT}" ]; then \
        echo "OPTIX_SCRIPT must be set to install Chrono::Sensor. Put the NVIDIA OptiX installer under docker/chrono/data and pass --build-arg OPTIX_SCRIPT=data/<installer>.sh"; exit 1; \
    fi

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        freeglut3-dev \
        libglew-dev \
        libglfw3-dev \
        libglu1-mesa-dev \
        libx11-dev \
        libxcursor-dev \
        libxi-dev \
        libxinerama-dev \
        libxrandr-dev \
        libxxf86vm-dev \
        xorg-dev && \
    rm -rf /var/lib/apt/lists/*

COPY ${OPTIX_SCRIPT} /tmp/optix.sh
RUN chmod +x /tmp/optix.sh && \
    mkdir -p ${PACKAGE_DIR}/optix && \
    /tmp/optix.sh --prefix=${PACKAGE_DIR}/optix --skip-license && \
    rm /tmp/optix.sh

ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_SENSOR=ON \
    -DCH_USE_SENSOR_NVRTC=ON \
    -DOptiX_INSTALL_DIR=${PACKAGE_DIR}/optix \
    -DOptiX_INCLUDE_DIR=${PACKAGE_DIR}/optix/include \
    -DNUMPY_INCLUDE_DIR=$(python3 -c 'import numpy; print(numpy.get_include())')"
