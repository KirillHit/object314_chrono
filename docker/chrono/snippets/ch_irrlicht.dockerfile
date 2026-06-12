# Chrono::Irrlicht dependencies.

RUN apt-get update && \
    apt-get install --no-install-recommends -y libirrlicht-dev && \
    rm -rf /var/lib/apt/lists/*

ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_IRRLICHT=ON \
    -DIrrlicht_ROOT=/usr"
