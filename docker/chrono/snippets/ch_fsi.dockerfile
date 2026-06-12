# Chrono::FSI support.

RUN apt-get update && \
    apt-get install --no-install-recommends -y libhdf5-dev && \
    rm -rf /var/lib/apt/lists/*

ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_HDF5=ON \
    -DCH_ENABLE_MODULE_FSI=ON \
    -DCH_ENABLE_MODULE_FSI_SPH=ON \
    -DCH_ENABLE_MODULE_FSI_TDPF=ON"
