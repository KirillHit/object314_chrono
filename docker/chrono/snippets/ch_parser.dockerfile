# Chrono::Parser dependencies.

RUN cd ${CHRONO_DIR}/contrib/build-scripts/linux && \
    bash buildURDF.sh ${PACKAGE_DIR}/urdf

ENV LD_LIBRARY_PATH="${PACKAGE_DIR}/urdf/lib:${LD_LIBRARY_PATH}"
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_PARSERS=ON \
    -Durdfdom_DIR=${PACKAGE_DIR}/urdf/lib/urdfdom/cmake \
    -Durdfdom_headers_DIR=${PACKAGE_DIR}/urdf/lib/urdfdom_headers/cmake \
    -Dconsole_bridge_DIR=${PACKAGE_DIR}/urdf/lib/console_bridge/cmake \
    -Dtinyxml2_DIR=${PACKAGE_DIR}/urdf/CMake"
