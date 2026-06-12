FROM kirillhit/project_chrono:10.0.0

RUN apt update && apt install -y build-essential cmake

WORKDIR /workspace
COPY ../src .

RUN mkdir -p build \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build --target install --parallel $(nproc)
