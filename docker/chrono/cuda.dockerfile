# Install CUDA 12.8 from the NVIDIA Ubuntu 24.04 repository.

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        ca-certificates \
        gnupg \
        lsb-release \
        software-properties-common \
        wget && \
    rm -rf /var/lib/apt/lists/*

RUN DISTRO=$(lsb_release -is | tr '[:upper:]' '[:lower:]') && \
    VERSION=$(lsb_release -rs | tr -d '.') && \
    ARCH=$(dpkg --print-architecture | sed 's/amd64/x86_64/') && \
    wget -q https://developer.download.nvidia.com/compute/cuda/repos/${DISTRO}${VERSION}/${ARCH}/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && \
    rm cuda-keyring_1.1-1_all.deb

RUN apt-get update && \
    apt-get install --no-install-recommends -y cuda-toolkit-${CUDA_VERSION} && \
    rm -rf /var/lib/apt/lists/*

ENV PATH="/usr/local/cuda/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:/usr/local/cuda/lib:${LD_LIBRARY_PATH}"
