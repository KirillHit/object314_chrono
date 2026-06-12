# Chrono::Vehicle dependencies.

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        libopenmpi-dev \
        openmpi-bin && \
    rm -rf /var/lib/apt/lists/*

RUN cd ${CHRONO_DIR}/contrib/build-scripts/linux && \
    bash buildOpenCRG.sh ${PACKAGE_DIR}/opencrg

ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_VEHICLE=ON \
    -DCH_ENABLE_OPENCRG=ON \
    -DOpenCRG_INCLUDE_DIR=${PACKAGE_DIR}/opencrg/include \
    -DOpenCRG_LIBRARY=${PACKAGE_DIR}/opencrg/lib/libOpenCRG.1.1.2.a"
