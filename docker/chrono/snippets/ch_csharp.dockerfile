# Chrono C# bindings.

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        mono-devel \
        swig && \
    rm -rf /var/lib/apt/lists/*

ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_CSHARP=ON"
