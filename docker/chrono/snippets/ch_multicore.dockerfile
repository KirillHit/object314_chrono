# Chrono::Multicore support.

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        libblas-dev \
        liblapack-dev \
        libomp-dev && \
    rm -rf /var/lib/apt/lists/*

RUN cd ${CHRONO_DIR}/contrib/build-scripts/linux && \
    bash -e buildBlaze.sh ${PACKAGE_DIR}/blaze

ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_MULTICORE=ON \
    -Dblaze_INCLUDE_DIR=${PACKAGE_DIR}/blaze/include"
