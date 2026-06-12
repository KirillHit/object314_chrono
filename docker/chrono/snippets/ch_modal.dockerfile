# Chrono::Modal dependencies.

RUN cd ${CHRONO_DIR}/contrib/build-scripts/linux && \
    bash buildSpectra.sh ${PACKAGE_DIR}/spectra

ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_MODAL=ON \
    -Dspectra_INCLUDE_DIR=${PACKAGE_DIR}/spectra/include"
