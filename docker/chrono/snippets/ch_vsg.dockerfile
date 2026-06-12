# Chrono::VSG dependencies.

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        libxcb1-dev \
        libvulkan-dev \
        vulkan-tools \
        wget && \
    rm -rf /var/lib/apt/lists/*

RUN cd ${CHRONO_DIR}/contrib/build-scripts/linux && \
    bash -e buildVSG.sh ${PACKAGE_DIR}/vsg

ENV CMAKE_PREFIX_PATH="${PACKAGE_DIR}/vsg"
ENV LD_LIBRARY_PATH="${PACKAGE_DIR}/vsg/lib"
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_VSG=ON \
    -Dvsg_DIR=${PACKAGE_DIR}/vsg/lib/cmake/vsg \
    -DvsgImGui_DIR=${PACKAGE_DIR}/vsg/lib/cmake/vsgImGui \
    -DvsgXchange_DIR=${PACKAGE_DIR}/vsg/lib/cmake/vsgXchange"
