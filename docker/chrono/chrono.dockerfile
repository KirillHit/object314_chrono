# Install Chrono 10.0.0 and selected modules.

RUN mkdir -p ${PACKAGE_DIR}

ENV CMAKE_OPTIONS=""
ENV PRE_BUILD_SCRIPTS=""

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        build-essential \
        cmake \
        git \
        libeigen3-dev \
        ninja-build \
        pkg-config \
        unzip \
        wget && \
    rm -rf /var/lib/apt/lists/*

RUN git clone --recursive -b ${CHRONO_VERSION} ${CHRONO_REPO} ${CHRONO_DIR}

INCLUDE ./snippets/ch_vsg.dockerfile
INCLUDE ./snippets/ch_irrlicht.dockerfile
INCLUDE ./snippets/ch_vehicle.dockerfile
INCLUDE ./snippets/ch_parser.dockerfile
INCLUDE ./snippets/ch_multicore.dockerfile
INCLUDE ./snippets/ch_fsi.dockerfile
INCLUDE ./snippets/ch_dem.dockerfile
INCLUDE ./snippets/ch_sensor.dockerfile
INCLUDE ./snippets/ch_modal.dockerfile
INCLUDE ./snippets/ch_postprocess.dockerfile
INCLUDE ./snippets/ch_csharp.dockerfile
INCLUDE ./snippets/ch_python.dockerfile
INCLUDE ./snippets/ch_synchrono.dockerfile
INCLUDE ./snippets/ch_ros.dockerfile

RUN ${PRE_BUILD_SCRIPTS} true && \
    eval "_CMAKE_OPTIONS=\"${CMAKE_OPTIONS}\"" && \
    mkdir -p ${CHRONO_DIR}/build && \
    cd ${CHRONO_DIR}/build && \
    cmake .. -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_DEMOS=OFF \
        -DBUILD_BENCHMARKING=ON \
        -DBUILD_TESTING=ON \
        -DCMAKE_INSTALL_PREFIX=${CHRONO_INSTALL_DIR} \
        -DCMAKE_LIBRARY_PATH=$(find /usr/local/cuda -type d -name stubs | head -n 1) \
        -DEigen3_DIR=/usr/lib/cmake/eigen3 \
        -DNUMPY_INCLUDE_DIR=$(python3 -c 'import numpy; print(numpy.get_include())') \
        -DCH_ENABLE_MODULE_PARDISO_MKL=OFF \
        -DCH_ENABLE_MODULE_CASCADE=OFF \
        -DCH_ENABLE_MODULE_MATLAB=OFF \
        ${_CMAKE_OPTIONS} && \
    ninja -j$(nproc) && \
    ninja install

ENV Chrono_DIR="${CHRONO_INSTALL_DIR}/lib/cmake/Chrono"
ENV LD_LIBRARY_PATH="${CHRONO_INSTALL_DIR}/lib:${LD_LIBRARY_PATH}"
