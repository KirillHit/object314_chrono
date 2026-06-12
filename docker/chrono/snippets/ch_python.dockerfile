# PyChrono support.

RUN apt-get update && \
    apt-get install --no-install-recommends -y swig && \
    rm -rf /var/lib/apt/lists/*

ENV PYTHONPATH="${CHRONO_INSTALL_DIR}/share/chrono/python:${PYTHONPATH}"
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_PYTHON=ON"
