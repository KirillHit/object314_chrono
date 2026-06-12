# Configure the Python environment used by Chrono and local ROS tooling.

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        python3 \
        python3-dev \
        python3-pip \
        python3-venv && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m venv env
ENV PATH="${WORKSPACE_DIR}/env/bin:${PATH}"

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
